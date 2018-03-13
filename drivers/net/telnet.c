/****************************************************************************
 * drivers/net/telnet.c
 *
 *   Copyright (C) 2007, 2009, 2011-2013, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This is a leverage of similar logic from uIP which has a compatible BSD
 * license:
 *
 *   Author: Adam Dunkels <adam@sics.se>
 *   Copyright (c) 2003, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute, NuttX nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <semaphore.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>
#include <nuttx/net/telnet.h>

#ifdef CONFIG_NETDEV_TELNET

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_TELNET_RXBUFFER_SIZE
#  define CONFIG_TELNET_RXBUFFER_SIZE 256
#endif

#ifndef CONFIG_TELNET_TXBUFFER_SIZE
#  define CONFIG_TELNET_TXBUFFER_SIZE 256
#endif

/* Telnet protocol stuff ****************************************************/

#define ISO_nl       0x0a
#define ISO_cr       0x0d

#define TELNET_SGA   0x03  /* Suppress Go Ahead */
#define TELNET_ECHO  0x01

#define TELNET_IAC   255
#define TELNET_WILL  251
#define TELNET_WONT  252
#define TELNET_DO    253
#define TELNET_DONT  254

/* Device stuff *************************************************************/

#define TELNETD_DEVFMT "/dev/telnet%d"

/****************************************************************************
 * Private Types
 ****************************************************************************/
/* The state of the telnet parser */

enum telnet_state_e
{
  STATE_NORMAL = 0,
  STATE_IAC,
  STATE_WILL,
  STATE_WONT,
  STATE_DO,
  STATE_DONT
};

/* This structure describes the internal state of the driver */

struct telnet_dev_s
{
  sem_t              td_exclsem; /* Enforces mutually exclusive access */
  uint8_t            td_state;   /* (See telnet_state_e) */
  uint8_t            td_pending; /* Number of valid, pending bytes in the rxbuffer */
  uint8_t            td_offset;  /* Offset to the valid, pending bytes in the rxbuffer */
  uint8_t            td_crefs;   /* The number of open references to the session */
  int                td_minor;   /* Minor device number */
  FAR struct socket  td_psock;   /* A clone of the internal socket structure */
  char td_rxbuffer[CONFIG_TELNET_RXBUFFER_SIZE];
  char td_txbuffer[CONFIG_TELNET_TXBUFFER_SIZE];
};

/* This structure  contains global information visable to all telnet driver
 * instances.
 */

struct telnet_common_s
{
  sem_t              tc_exclsem; /* Enforces exclusive access to 'minor' */
  uint16_t           tc_minor;   /* The next minor number to use */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* Support functions */

#ifdef CONFIG_TELNET_DUMPBUFFER
static inline void telnet_dumpbuffer(FAR const char *msg,
                 FAR const char *buffer, unsigned int nbytes);
#else
# define telnet_dumpbuffer(msg,buffer,nbytes)
#endif
static void    telnet_getchar(FAR struct telnet_dev_s *priv, uint8_t ch,
                 FAR char *dest, int *nread);
static ssize_t telnet_receive(FAR struct telnet_dev_s *priv,
                 FAR const char *src, size_t srclen, FAR char *dest,
                 size_t destlen);
static bool    telnet_putchar(FAR struct telnet_dev_s *priv, uint8_t ch,
                 int *nwritten);
static void    telnet_sendopt(FAR struct telnet_dev_s *priv, uint8_t option,
                 uint8_t value);

/* Telnet character driver methods */

static int     telnet_open(FAR struct file *filep);
static int     telnet_close(FAR struct file *filep);
static ssize_t telnet_read(FAR struct file *, FAR char *, size_t);
static ssize_t telnet_write(FAR struct file *, FAR const char *, size_t);

/* Telnet session creation */

static int     telnet_session(FAR struct telnet_session_s *session);

/* Telnet factory driver methods */

static ssize_t factory_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t factory_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     common_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_telnet_fops =
{
  telnet_open,   /* open */
  telnet_close,  /* close */
  telnet_read,   /* read */
  telnet_write,  /* write */
  0,             /* seek */
  common_ioctl   /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0            /* poll */
#endif
};

static const struct file_operations g_factory_fops =
{
  0,             /* open */
  0,             /* close */
  factory_read,  /* read */
  factory_write, /* write */
  0,             /* seek */
  common_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0            /* poll */
#endif
};

/* Global information shared amongst telnet driver instanaces. */

static struct telnet_common_s g_telnet_common =
{
  SEM_INITIALIZER(1),
  0
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: telnet_dumpbuffer
 *
 * Description:
 *   Dump a buffer of data (debug only)
 *
 ****************************************************************************/

#ifdef CONFIG_TELNET_DUMPBUFFER
static inline void telnet_dumpbuffer(FAR const char *msg,
                                     FAR const char *buffer,
                                     unsigned int nbytes)
{
  /* CONFIG_DEBUG_FEATURES, CONFIG_DEBUG_INFO, and CONFIG_DEBUG_NET have to be
  * defined or the following does nothing.
  */

  ninfodumpbuffer(msg, (FAR const uint8_t*)buffer, nbytes);
}
#endif

/****************************************************************************
 * Name: telnet_getchar
 *
 * Description:
 *   Get another character for the user received buffer from the RX buffer
 *
 ****************************************************************************/

static void telnet_getchar(FAR struct telnet_dev_s *priv, uint8_t ch,
                           FAR char *dest, int *nread)
{
  register int index;

#ifndef CONFIG_TELNET_CHARACTER_MODE
  /* Ignore carriage returns */

  if (ch != ISO_cr)
#endif
    {
      /* Add all other characters to the destination buffer */

      index = *nread;
      dest[index++] = ch;
      *nread = index;
    }
}

/****************************************************************************
 * Name: telnet_receive
 *
 * Description:
 *   Process a received Telnet buffer
 *
 ****************************************************************************/

static ssize_t telnet_receive(FAR struct telnet_dev_s *priv, FAR const char *src,
                              size_t srclen, FAR char *dest, size_t destlen)
{
  int nread;
  uint8_t ch;

  ninfo("srclen: %d destlen: %d\n", srclen, destlen);

  for (nread = 0; srclen > 0 && nread < destlen; srclen--)
    {
      ch = *src++;
      ninfo("ch=%02x state=%d\n", ch, priv->td_state);

      switch (priv->td_state)
        {
          case STATE_IAC:
            if (ch == TELNET_IAC)
              {
                telnet_getchar(priv, ch, dest, &nread);
                priv->td_state = STATE_NORMAL;
             }
            else
              {
                switch (ch)
                  {
                    case TELNET_WILL:
                      priv->td_state = STATE_WILL;
                      break;

                    case TELNET_WONT:
                      priv->td_state = STATE_WONT;
                      break;

                    case TELNET_DO:
                      priv->td_state = STATE_DO;
                      break;

                    case TELNET_DONT:
                      priv->td_state = STATE_DONT;
                      break;

                    default:
                      priv->td_state = STATE_NORMAL;
                      break;
                  }
              }
            break;

          case STATE_WILL:
            /* Reply with a DONT */

            telnet_sendopt(priv, TELNET_DONT, ch);
            priv->td_state = STATE_NORMAL;
            break;

          case STATE_WONT:
            /* Reply with a DONT */

            telnet_sendopt(priv, TELNET_DONT, ch);
            priv->td_state = STATE_NORMAL;
            break;

          case STATE_DO:
#ifdef CONFIG_TELNET_CHARACTER_MODE
            if (ch == TELNET_SGA)
              {
                /* If it received 'Suppress Go Ahead', reply with a WILL */

                telnet_sendopt(priv, TELNET_WILL, ch);

                /* Also, send 'WILL ECHO' */

                telnet_sendopt(priv, TELNET_WILL, TELNET_ECHO);
              }
            else if (ch == TELNET_ECHO)
              {
                /* If it received 'ECHO', then do nothing */
              }
            else
              {
                /* Reply with a WONT */

                telnet_sendopt(priv, TELNET_WONT, ch);
              }
#else
            /* Reply with a WONT */

            telnet_sendopt(priv, TELNET_WONT, ch);
#endif
            priv->td_state = STATE_NORMAL;
            break;

          case STATE_DONT:
            /* Reply with a WONT */

            telnet_sendopt(priv, TELNET_WONT, ch);
            priv->td_state = STATE_NORMAL;
            break;

          case STATE_NORMAL:
            if (ch == TELNET_IAC)
              {
                priv->td_state = STATE_IAC;
              }
            else
              {
                telnet_getchar(priv, ch, dest, &nread);
              }
            break;
        }
    }

  /* We get here if (1) all of the received bytes have been processed, or
   * (2) if the user's buffer has become full.
   */

  if (srclen > 0)
    {
      /* Remember where we left off.  These bytes will be returned the next
       * time that telnet_read() is called.
       */

      priv->td_pending = srclen;
      priv->td_offset = (src - priv->td_rxbuffer);
    }
  else
    {
      /* All of the received bytes were consumed */

      priv->td_pending = 0;
      priv->td_offset  = 0;
    }

  return nread;
}

/****************************************************************************
 * Name: telnet_putchar
 *
 * Description:
 *   Put another character from the user buffer to the TX buffer.
 *
 ****************************************************************************/

static bool telnet_putchar(FAR struct telnet_dev_s *priv, uint8_t ch,
                           int *nread)
{
  register int index;
  bool ret = false;

  /* Ignore carriage returns (we will put these in automatically as necesary) */

  if (ch != ISO_cr)
    {
      /* Add all other characters to the destination buffer */

      index = *nread;
      priv->td_txbuffer[index++] = ch;

      /* Check for line feeds */

      if (ch == ISO_nl)
        {
          /* Now add the carriage return */

          priv->td_txbuffer[index++] = ISO_cr;
          priv->td_txbuffer[index++] = '\0';

          /* End of line */

          ret = true;
        }

      *nread = index;
    }

  return ret;
}

/****************************************************************************
 * Name: telnet_sendopt
 *
 * Description:
 *   Send the telnet option bytes
 *
 ****************************************************************************/

static void telnet_sendopt(FAR struct telnet_dev_s *priv, uint8_t option,
                           uint8_t value)
{
  uint8_t optbuf[4];
  int ret;

  optbuf[0] = TELNET_IAC;
  optbuf[1] = option;
  optbuf[2] = value;
  optbuf[3] = 0;

  telnet_dumpbuffer("Send optbuf", optbuf, 4);

  ret = psock_send(&priv->td_psock, optbuf, 4, 0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to send TELNET_IAC: %d\n", ret);
    }
}

/****************************************************************************
 * Name: telnet_open
 ****************************************************************************/

static int telnet_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct telnet_dev_s *priv = inode->i_private;
  int tmp;
  int ret;

  ninfo("td_crefs: %d\n", priv->td_crefs);

  /* O_NONBLOCK is not supported */

  if (filep->f_oflags & O_NONBLOCK)
    {
      ret = -ENOSYS;
      goto errout;
    }

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&priv->td_exclsem);
  if (ret < 0)
    {
      nerr("ERROR: nxsem_wait failed: %d\n", ret);
      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
      goto errout;
    }

  /* Increment the count of references to the device.  If this the first
   * time that the driver has been opened for this device, then initialize
   * the device.
   */

  tmp = priv->td_crefs + 1;
  if (tmp > 255)
    {
      /* More than 255 opens; uint8_t would overflow to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* Save the new open count on success */

  priv->td_crefs = tmp;
  ret = OK;

errout_with_sem:
  nxsem_post(&priv->td_exclsem);

errout:
  return ret;
}

/****************************************************************************
 * Name: telnet_close
 ****************************************************************************/

static int telnet_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct telnet_dev_s *priv = inode->i_private;
  FAR char *devpath;
  int ret;

  ninfo("td_crefs: %d\n", priv->td_crefs);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&priv->td_exclsem);
  if (ret < 0)
    {
      nerr("ERROR: nxsem_wait failed: %d\n", ret);
      DEBUGASSERT(ret == -EINTR || ret == -ECANCELED);
      goto errout;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (priv->td_crefs > 1)
    {
      /* Just decrement the reference count and release the semaphore */

      priv->td_crefs--;
      nxsem_post(&priv->td_exclsem);
    }
  else
    {
      /* Re-create the path to the driver. */

      sched_lock();
      ret = asprintf(&devpath, TELNETD_DEVFMT, priv->td_minor);
      if (ret < 0)
        {
          nerr("ERROR: Failed to allocate the driver path\n");
        }
      else
        {
          /* Un-register the character driver */

          ret = unregister_driver(devpath);
          if (ret < 0)
            {
              /* NOTE: a return value of -EBUSY is not an error, it simply
               * means that the Telnet driver is busy now and cannot be
               * registered now because there are other sessions using the
               * connection.  The driver will be properly unregistered when
               * the final session terminates.
               */

              if (ret != -EBUSY)
                {
                  nerr("ERROR: Failed to unregister the driver %s: %d\n",
                       devpath, ret);
                }
            }

          free(devpath);
        }

      /* Close the socket */

      psock_close(&priv->td_psock);

      /* Release the driver memory.  What if there are threads waiting on
       * td_exclsem?  They will never be awakened!  How could this happen?
       * crefs == 1 so there are no other open references to the driver.
       * But this could have if someone were trying to re-open the driver
       * after every other thread has closed it.  That really should not
       * happen in the intended usage model.
       */

      DEBUGASSERT(priv->td_exclsem.semcount == 0);
      nxsem_destroy(&priv->td_exclsem);
      free(priv);
      sched_unlock();
    }

  ret = OK;

errout:
  return ret;
}

/****************************************************************************
 * Name: telnet_read
 ****************************************************************************/

static ssize_t telnet_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct telnet_dev_s *priv = inode->i_private;
  ssize_t ret;

  ninfo("len: %d\n", len);

  /* First, handle the case where there are still valid bytes left in the
   * I/O buffer from the last time that read was called.  NOTE:  Much of
   * what we read may be protocol stuff and may not correspond to user
   * data.  Hence we need the loop and we need may need to call psock_recv()
   * multiple times in order to get data that the client is interested in.
   */

  do
    {
      if (priv->td_pending > 0)
        {
          /* Process the buffered telnet data */

          FAR const char *src = &priv->td_rxbuffer[priv->td_offset];
          ret = telnet_receive(priv, src, priv->td_pending, buffer, len);
        }

      /* Read a buffer of data from the telnet client */

      else
        {
          ret = psock_recv(&priv->td_psock, priv->td_rxbuffer,
                          CONFIG_TELNET_RXBUFFER_SIZE, 0);

          /* Did we receive anything? */

          if (ret > 0)
            {
              /* Yes.. Process the newly received telnet data */

              telnet_dumpbuffer("Received buffer", priv->td_rxbuffer, ret);
              ret = telnet_receive(priv, priv->td_rxbuffer, ret, buffer, len);
           }

          /* Otherwise the peer closed the connection (ret == 0) or an error
           * occurred (ret < 0).
           */

          else
            {
              break;
            }
        }
    }
  while (ret == 0);

  /* Returned Value:
   *
   * ret > 0:  The number of characters copied into the user buffer by
   *           telnet_receive().
   * ret <= 0: Loss of connection or error events reported by recv().
   */

  return ret;
}

/****************************************************************************
 * Name: telnet_write
 ****************************************************************************/

static ssize_t telnet_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct telnet_dev_s *priv = inode->i_private;
  FAR const char *src = buffer;
  ssize_t nsent;
  ssize_t ret;
  int ncopied;
  char ch;
  bool eol;

  ninfo("len: %d\n", len);

  /* Process each character from the user buffer */

  for (nsent = 0, ncopied = 0; nsent < len; nsent++)
    {
      /* Get the next character from the user buffer */

      ch = *src++;

      /* Add the character to the TX buffer */

      eol = telnet_putchar(priv, ch, &ncopied);

      /* Was that the end of a line? Or is the buffer too full to hold the
       * next largest character sequence ("\r\n\0")?
       */

      if (eol || ncopied > CONFIG_TELNET_TXBUFFER_SIZE-3)
        {
          /* Yes... send the data now */

          ret = psock_send(&priv->td_psock, priv->td_txbuffer, ncopied, 0);
          if (ret < 0)
            {
              nerr("ERROR: psock_send failed '%s': %d\n", priv->td_txbuffer, ret);
              return ret;
            }

          /* Reset the index to the beginning of the TX buffer. */

          ncopied = 0;
        }
    }

  /* Send anything remaining in the TX buffer */

  if (ncopied > 0)
    {
      ret = psock_send(&priv->td_psock, priv->td_txbuffer, ncopied, 0);
      if (ret < 0)
        {
          nerr("ERROR: psock_send failed '%s': %d\n", priv->td_txbuffer, ret);
          return ret;
        }
    }

  /* Notice that we don't actually return the number of bytes sent, but
   * rather, the number of bytes that the caller asked us to send.  We may
   * have sent more bytes (because of CR-LF expansion and because of NULL
   * termination). But it confuses some logic if you report that you sent
   * more than you were requested to.
   */

  return len;
}

/****************************************************************************
 * Name: telnet_session
 *
 * Description:
 *   Create a character driver to "wrap" the telnet session.  This function
 *   will select and return a unique path for the new telnet device.
 *
 * Input Parameters:
 *   session - On input, contains the socket descriptor that represents the
 *   new telnet connection.  On output, it holds the path to the new Telnet driver.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int telnet_session(FAR struct telnet_session_s *session)
{
  FAR struct telnet_dev_s *priv;
  FAR struct socket *psock;
  struct stat statbuf;
  uint16_t start;
  int ret;

  /* Allocate instance data for this driver */

  priv = (FAR struct telnet_dev_s*)malloc(sizeof(struct telnet_dev_s));
  if (!priv)
    {
      nerr("ERROR: Failed to allocate the driver data structure\n");
      return -ENOMEM;
    }

  /* Initialize the allocated driver instance */

  nxsem_init(&priv->td_exclsem, 0, 1);

  priv->td_state   = STATE_NORMAL;
  priv->td_crefs   = 0;
  priv->td_pending = 0;
  priv->td_offset  = 0;

  /* Clone the internal socket structure.  We do this so that it will be
   * independent of threads and of socket descriptors (the original socket
   * instance resided in the daemon's task group`).
   */

  psock = sockfd_socket(session->ts_sd);
  if (!psock)
    {
      nerr("ERROR: Failed to convert sd=%d to a socket structure\n", session->ts_sd);
      ret = -EINVAL;
      goto errout_with_dev;
    }

  ret = net_clone(psock, &priv->td_psock);
  if (ret < 0)
    {
      nerr("ERROR: net_clone failed: %d\n", ret);
      goto errout_with_dev;
    }

  /* Allocate a unique minor device number of the telnet drvier.
   * Get exclusive access to the minor counter.
   */

  do
    {
      ret = nxsem_wait(&g_telnet_common.tc_exclsem);
      if (ret < 0 && ret != -EINTR)
        {
          nerr("ERROR: nxsem_wait failed: %d\n", ret);
          goto errout_with_clone;
        }
    }
  while (ret == -EINTR);

  /* Loop until the device name is verified to be unique. */

  start = g_telnet_common.tc_minor;
  do
    {
      /* Get the next candiate minor number */

      priv->td_minor = g_telnet_common.tc_minor;
      g_telnet_common.tc_minor++;

      snprintf(session->ts_devpath, TELNET_DEVPATH_MAX, TELNETD_DEVFMT,
               priv->td_minor);

      ret = stat(session->ts_devpath, &statbuf);
      DEBUGASSERT(ret >= 0 || get_errno() == ENOENT);
    }
  while (ret >= 0 && start != g_telnet_common.tc_minor);

  if (ret >= 0)
    {
      nerr("ERROR: Too many sessions\n");
      ret = -ENFILE;
      goto errout_with_semaphore;
    }

  /* Register the driver */

  ret = register_driver(session->ts_devpath, &g_telnet_fops, 0666, priv);
  if (ret < 0)
    {
      nerr("ERROR: Failed to register the driver %s: %d\n",
           session->ts_devpath, ret);
      goto errout_with_semaphore;
    }

  /* Close the original psoock (keeping the clone) */

  psock_close(psock);

  /* Return the path to the new telnet driver */

  nxsem_post(&g_telnet_common.tc_exclsem);
  return OK;

errout_with_semaphore:
  nxsem_post(&g_telnet_common.tc_exclsem);

errout_with_clone:
  psock_close(&priv->td_psock);

errout_with_dev:
  free(priv);
  return ret;
}

/****************************************************************************
 * Name: factory_read
 ****************************************************************************/

static ssize_t factory_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  return 0; /* Return EOF */
}

/****************************************************************************
 * Name: factory_write
 ****************************************************************************/

static ssize_t factory_write(FAR struct file *filep, FAR const char *buffer,
                             size_t len)
{
  return len; /* Say that everything was written */
}

/****************************************************************************
 * Name: common_ioctl
 ****************************************************************************/

static int common_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret;

  switch (cmd)
    {
    /* Command:      SIOCTELNET
     * Description:  Create a Telnet sessions.
     * Argument:     A pointer to a write-able instance of struct
     *               telnet_session_s.
     * Dependencies: CONFIG_NETDEV_TELNET
     */

    case SIOCTELNET:
      {
         FAR struct telnet_session_s *session =
           (FAR struct telnet_session_s *)((uintptr_t)arg);

        if (session == NULL)
          {
            ret = -EINVAL;
          }
        else
          {
            ret = telnet_session(session);
          }
      }
      break;

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: telnet_initialize
 *
 * Description:
 *   Create the Telnet factory at /dev/telnet.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int telnet_initialize(void)
{
  return register_driver("/dev/telnet", &g_factory_fops, 0666, NULL);
}

#endif /* CONFIG_NETDEV_TELNET */
