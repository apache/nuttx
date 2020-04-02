/****************************************************************************
 * drivers/net/telnet.c
 *
 *   Copyright (C) 2007, 2009, 2011-2013, 2017, 2019, 2020 Gregory Nutt. All
 *     rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This derives remotely from some Telnet logic from uIP which has a
 * compatible BSD license:
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
 * 3. Neither the name of the Institute, NuttX nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

#include <assert.h>
#include <stdio.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/semaphore.h>
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

#ifndef CONFIG_TELNET_MAXLCLIENTS
#  define CONFIG_TELNET_MAXLCLIENTS 8
#endif

#ifndef CONFIG_TELNET_IOTHREAD_PRIORITY
#  define CONFIG_TELNET_IOTHREAD_PRIORITY 100
#endif

#ifndef CONFIG_TELNET_IOTHREAD_STACKSIZE
#  define CONFIG_TELNET_IOTHREAD_STACKSIZE 1024
#endif

#undef HAVE_SIGNALS
#if defined(CONFIG_TTY_SIGINT) || defined(CONFIG_TTY_SIGSTP)
#  define HAVE_SIGNALS
#endif

/* Telnet protocol stuff ****************************************************/

#define TELNET_NL             0x0a
#define TELNET_CR             0x0d

/* Telnet commands */

#define TELNET_ECHO           1
#define TELNET_SGA            3     /* Suppress Go Ahead */
#define TELNET_NAWS           31    /* Negotiate about window size */

/* Telnet control */

#define TELNET_IAC            255
#define TELNET_WILL           251
#define TELNET_WONT           252
#define TELNET_DO             253
#define TELNET_DONT           254
#define TELNET_SB             250
#define TELNET_SE             240

/* Device stuff *************************************************************/

#define TELNET_DEVFMT         "/dev/telnet%d"

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
  STATE_DONT,
  STATE_SB,
  STATE_SB_NAWS,
  STATE_SE
};

/* This structure describes the internal state of the driver */

struct telnet_dev_s
{
  sem_t             td_exclsem;   /* Enforces mutually exclusive access */
  sem_t             td_iosem;     /* I/O thread will notify that data is available */
  uint8_t           td_state;     /* (See telnet_state_e) */
  uint8_t           td_pending;   /* Number of valid, pending bytes in the rxbuffer */
  uint8_t           td_offset;    /* Offset to the valid, pending bytes in the rxbuffer */
  uint8_t           td_crefs;     /* The number of open references to the session */
  uint8_t           td_minor;     /* Minor device number */
#ifdef CONFIG_TELNET_SUPPORT_NAWS
  uint16_t          td_rows;      /* Number of NAWS rows */
  uint16_t          td_cols;      /* Number of NAWS cols */
  int               td_sb_count;  /* Count of TELNET_SB bytes received */
#endif
#ifdef HAVE_SIGNALS
  pid_t             td_pid;
#endif
  struct pollfd     td_fds;
  FAR struct socket td_psock;     /* A clone of the internal socket structure */
  char td_rxbuffer[CONFIG_TELNET_RXBUFFER_SIZE];
  char td_txbuffer[CONFIG_TELNET_TXBUFFER_SIZE];
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
static int     telnet_io_main(int argc, FAR char** argv);

/* Telnet character driver methods */

static int     telnet_open(FAR struct file *filep);
static int     telnet_close(FAR struct file *filep);
static ssize_t telnet_read(FAR struct file *filep, FAR char *buffer,
                 size_t len);
static ssize_t telnet_write(FAR struct file *filep, FAR const char *buffer,
                 size_t len);
static int     telnet_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);
static int     telnet_poll(FAR struct file *filep, FAR struct pollfd *fds,
                 bool setup);

/* Telnet session creation */

static int     telnet_session(FAR struct telnet_session_s *session);

/* Telnet factory driver methods */

static ssize_t factory_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t factory_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     factory_ioctl(FAR struct file *filep, int cmd,
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
  NULL,          /* seek */
  telnet_ioctl,  /* ioctl */
  telnet_poll    /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

static const struct file_operations g_factory_fops =
{
  NULL,          /* open */
  NULL,          /* close */
  factory_read,  /* read */
  factory_write, /* write */
  NULL,          /* seek */
  factory_ioctl, /* ioctl */
  NULL           /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

/* This is an global data set of all of all active Telnet drivers.  This
 * additional logic in included to handle killing of task via control
 * characters received via Telenet (via Ctrl-C SIGINT, in particular).
 */

static pid_t                g_telnet_io_kthread;
static struct telnet_dev_s *g_telnet_clients[CONFIG_TELNET_MAXLCLIENTS];
static sem_t                g_iosem       = SEM_INITIALIZER(0);
static sem_t                g_clients_sem = SEM_INITIALIZER(1);

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
  /* CONFIG_DEBUG_FEATURES, CONFIG_DEBUG_INFO, and CONFIG_DEBUG_NET have to
   * be defined or the following does nothing.
   */

  ninfodumpbuffer(msg, (FAR const uint8_t *)buffer, nbytes);
}
#endif

/****************************************************************************
 * Name: telnet_check_ctrlchar
 *
 * Description:
 *   Check if an incoming control character should generate a signal.
 *
 ****************************************************************************/

#ifdef HAVE_SIGNALS
static void telnet_check_ctrlchar(FAR struct telnet_dev_s *priv,
                                  FAR char *buffer, size_t len)
{
  int signo = 0;

  for (; priv->td_pid >= 0 && len > 0; buffer++, len--)
    {
#ifdef CONFIG_TTY_SIGINT
      /* Is this the special character that will generate the SIGINT signal? */

      if (*buffer == CONFIG_TTY_SIGINT_CHAR)
        {
          /* Yes.. note that the kill is needed and do not put the character
           * into the Rx buffer.  It should not be read as normal data.
           */

          signo = SIGINT;
          break;
        }
#endif

#ifdef CONFIG_TTY_SIGSTP
      /* Is this the special character that will generate the SIGSTP signal? */

      if (*buffer == CONFIG_TTY_SIGSTP_CHAR)
        {
          /* Note that the kill is needed and do not put the character
           * into the Rx buffer.  It should not be read as normal data.
           */

          signo = SIGSTP;
#ifndef CONFIG_TTY_SIGINT
          break;
#endif
        }
#endif
    }

  /* Send the signal if necessary */

  if (signo != 0)
    {
      nxsig_kill(priv->td_pid, signo);
    }
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

  if (ch != TELNET_CR)
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

static ssize_t telnet_receive(FAR struct telnet_dev_s *priv,
                              FAR const char *src, size_t srclen,
                              FAR char *dest, size_t destlen)
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

#ifdef CONFIG_TELNET_SUPPORT_NAWS
                    case TELNET_SB:
                      priv->td_state = STATE_SB;
                      priv->td_sb_count = 0;
                      break;

                    case TELNET_SE:
                      priv->td_state = STATE_NORMAL;
                      break;
#endif

                    default:
                      priv->td_state = STATE_NORMAL;
                      break;
                  }
              }
            break;

          case STATE_WILL:
#ifdef CONFIG_TELNET_SUPPORT_NAWS
            /* For NAWS, Reply with a DO */

            if (ch == TELNET_NAWS)
              {
                telnet_sendopt(priv, TELNET_DO, ch);
              }

            /* Reply with a DON'T */

            else
#endif
              {
                telnet_sendopt(priv, TELNET_DONT, ch);
                ninfo("Suppress: 0x%02X (%d)\n", ch, ch);
              }

            priv->td_state = STATE_NORMAL;
            break;

          case STATE_WONT:
            telnet_sendopt(priv, TELNET_DONT, ch);
            priv->td_state = STATE_NORMAL;
            break;

          case STATE_DO:
#ifdef CONFIG_TELNET_CHARACTER_MODE
            if (ch == TELNET_SGA || ch == TELNET_ECHO)
              {
                /* If it received 'ECHO' or 'Suppress Go Ahead', then do nothing */
              }
            else
              {
                /* Reply with a WONT */

                telnet_sendopt(priv, TELNET_WONT, ch);
                ninfo("WONT: 0x%02X\n", ch);
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

#ifdef CONFIG_TELNET_SUPPORT_NAWS
          /* Handle Telnet Sub negotiation request */

          case STATE_SB:
            switch (ch)
              {
                case TELNET_NAWS:
                  priv->td_state = STATE_SB_NAWS;
                  break;

                default:
                  priv->td_state = STATE_NORMAL;
                  break;
              }
            break;

          /* Handle NAWS sub-option negotiation */

          case STATE_SB_NAWS:

            /* Update cols / rows based on received byte count */

            switch (priv->td_sb_count)
              {
                case 0:
                  priv->td_cols = (priv->td_cols & 0x00ff) | (ch << 8);
                  break;

                case 1:
                  priv->td_cols = (priv->td_cols & 0xff00) | ch;
                  break;

                case 2:
                  priv->td_rows = (priv->td_rows & 0x00ff) | (ch << 8);
                  break;

                case 3:
                  priv->td_rows = (priv->td_rows & 0xff00) | ch;
                  ninfo("NAWS: %d,%d", priv->td_cols, priv->td_rows);
                  break;
              }

            /* Increment SB count and switch to NORMAL when complete */

            if (++priv->td_sb_count == 4)
              {
                priv->td_state = STATE_NORMAL;
              }

            break;
#endif
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

  /* Ignore carriage returns (we will put these in automatically as necessary) */

  if (ch != TELNET_CR)
    {
      /* Add all other characters to the destination buffer */

      index = *nread;
      priv->td_txbuffer[index++] = ch;

      /* Check for line feeds */

      if (ch == TELNET_NL)
        {
          /* Now add the carriage return */

          priv->td_txbuffer[index++] = TELNET_CR;

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
  uint8_t optbuf[3];
  int ret;

  optbuf[0] = TELNET_IAC;
  optbuf[1] = option;
  optbuf[2] = value;

  telnet_dumpbuffer("Send optbuf", optbuf, 3);

  ret = psock_send(&priv->td_psock, optbuf, 3, 0);
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

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&priv->td_exclsem);
  if (ret < 0)
    {
      nerr("ERROR: nxsem_wait failed: %d\n", ret);
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
  int i;

  ninfo("td_crefs: %d\n", priv->td_crefs);

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&priv->td_exclsem);
  if (ret < 0)
    {
      nerr("ERROR: nxsem_wait failed: %d\n", ret);
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

      ret = asprintf(&devpath, TELNET_DEVFMT, priv->td_minor);
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
              else
                {
                  ret = OK;
                }
            }

          kmm_free(devpath);
        }

      /* Remove ourself from the clients list */

      nxsem_wait(&g_clients_sem);
      for (i = 0; i < CONFIG_TELNET_MAXLCLIENTS; i++)
        {
          if (g_telnet_clients[i] == priv)
            {
              g_telnet_clients[i] = NULL;
              break;
            }
        }

      /* If the socket is still polling */

      if (priv->td_fds.events)
        {
          /* Tear down the poll */

          psock_poll(&priv->td_psock, &priv->td_fds, FALSE);
          priv->td_fds.events = 0;
        }

      nxsem_post(&g_clients_sem);

      /* Notify the I/O thread that a client was removed */

      nxsem_post(&g_iosem);

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
      kmm_free(priv);
    }

errout:
  return ret;
}

/****************************************************************************
 * Name: telnet_read
 ****************************************************************************/

static ssize_t telnet_read(FAR struct file *filep, FAR char *buffer,
                           size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct telnet_dev_s *priv = inode->i_private;
  ssize_t nread = 0;
  int ret;

  ninfo("len: %d\n", len);

  /* First, handle the case where there are still valid bytes left in the
   * I/O buffer from the last time that read was called.  NOTE:  Much of
   * what we read may be protocol stuff and may not correspond to user
   * data.  Hence we need the loop and we need may need to wait for data
   * multiple times in order to get data that the client is interested in.
   */

  do
    {
      FAR const char *src;

      if (priv->td_pending == 0)
        {
          /* poll fds.revents contains last poll status in case of error */

          if ((priv->td_fds.revents & (POLLHUP | POLLERR)) != 0)
            {
              return -EPIPE;
            }

          if (filep->f_oflags & O_NONBLOCK)
            {
              return -EAGAIN;
            }

          /* Wait for new data, interrupt, or thread cancellation */

          ret = nxsem_wait(&priv->td_iosem);
          if (ret < 0)
            {
              nerr("ERROR: nxsem_wait failed: %d\n", ret);
              return (ssize_t)ret;
            }

          continue;
        }

      /* Take exclusive access to data buffer */

      ret = nxsem_wait(&priv->td_exclsem);
      if (ret < 0)
        {
          nerr("ERROR: nxsem_wait failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Process the buffered telnet data */

      src = &priv->td_rxbuffer[priv->td_offset];
      nread = telnet_receive(priv, src, priv->td_pending, buffer, len);

      nxsem_post(&priv->td_exclsem);
    }
  while (nread == 0);

  /* Returned Value:
   *
   * nread > 0:  The number of characters copied into the user buffer by
   *             telnet_receive().
   * nread <= 0: Loss of connection or error events reported by recv().
   */

  return nread;
}

/****************************************************************************
 * Name: telnet_write
 ****************************************************************************/

static ssize_t telnet_write(FAR struct file *filep, FAR const char *buffer,
                            size_t len)
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
       * next largest character sequence ("\r\n")?
       */

      if (eol || ncopied > CONFIG_TELNET_TXBUFFER_SIZE - 2)
        {
          /* Yes... send the data now */

          ret = psock_send(&priv->td_psock, priv->td_txbuffer, ncopied, 0);
          if (ret < 0)
            {
              nerr("ERROR: psock_send failed '%s': %d\n",
                   priv->td_txbuffer, ret);
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
          nerr("ERROR: psock_send failed '%s': %d\n",
               priv->td_txbuffer, ret);
          return ret;
        }
    }

  /* Notice that we don't actually return the number of bytes sent, but
   * rather, the number of bytes that the caller asked us to send.  We may
   * have sent more bytes (because of CR-LF expansion). But it confuses
   * some logic if you report that you sent more than you were requested to.
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
 *   new telnet connection.  On output, it holds the path to the new Telnet
 *   driver.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int telnet_session(FAR struct telnet_session_s *session)
{
  FAR struct telnet_dev_s *priv;
  FAR struct socket *psock;
  int ret;

  /* Allocate instance data for this driver */

  priv = (FAR struct telnet_dev_s *)kmm_zalloc(sizeof(struct telnet_dev_s));
  if (!priv)
    {
      nerr("ERROR: Failed to allocate the driver data structure\n");
      return -ENOMEM;
    }

  /* Initialize the allocated driver instance */

  nxsem_init(&priv->td_exclsem, 0, 1);
  nxsem_init(&priv->td_iosem, 0, 0);

  /* td_iosem is used for signaling and, hence, must not participate in
   * priority inheritance.
   */

  nxsem_setprotocol(&priv->td_iosem, SEM_PRIO_NONE);

  priv->td_state     = STATE_NORMAL;
  priv->td_crefs     = 0;
  priv->td_minor     = 0;
  priv->td_pending   = 0;
  priv->td_offset    = 0;
#ifdef HAVE_SIGNALS
  priv->td_pid       = -1;
#endif
#ifdef CONFIG_TELNET_SUPPORT_NAWS
  priv->td_rows      = 25;
  priv->td_cols      = 80;
  priv->td_sb_count  = 0;
#endif

  /* Clone the internal socket structure.  We do this so that it will be
   * independent of threads and of socket descriptors (the original socket
   * instance resided in the daemon's task group`).
   */

  psock = sockfd_socket(session->ts_sd);
  if (!psock)
    {
      nerr("ERROR: Failed to convert sd=%d to a socket structure\n",
           session->ts_sd);
      ret = -EINVAL;
      goto errout_with_dev;
    }

  ret = psock_dup2(psock, &priv->td_psock);
  if (ret < 0)
    {
      nerr("ERROR: psock_dup2 failed: %d\n", ret);
      goto errout_with_dev;
    }

  /* Allocate a unique minor device number of the telnet driver.
   * Get exclusive access to the minor counter.
   */

  ret = nxsem_wait_uninterruptible(&g_clients_sem);
  if (ret < 0)
    {
      nerr("ERROR: nxsem_wait failed: %d\n", ret);
      goto errout_with_clone;
    }

  /* Loop until the device name is verified to be unique. */

  while (priv->td_minor < CONFIG_TELNET_MAXLCLIENTS)
    {
      if (g_telnet_clients[priv->td_minor] == NULL)
        {
          snprintf(session->ts_devpath, TELNET_DEVPATH_MAX,
                   TELNET_DEVFMT, priv->td_minor);
          break;
        }

      priv->td_minor++;
    }

  if (priv->td_minor >= CONFIG_TELNET_MAXLCLIENTS)
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

#ifdef CONFIG_TELNET_SUPPORT_NAWS
  telnet_sendopt(priv, TELNET_DO, TELNET_NAWS);
#endif

#ifdef CONFIG_TELNET_CHARACTER_MODE
  telnet_sendopt(priv, TELNET_WILL, TELNET_SGA);
  telnet_sendopt(priv, TELNET_WILL, TELNET_ECHO);
#endif

  /* Has the I/O thread been started? */

  if (g_telnet_io_kthread == (pid_t)0)
    {
      /* g_iosem is used for signaling and, hence, must not participate in
       * priority inheritance.
       */

      nxsem_setprotocol(&g_iosem, SEM_PRIO_NONE);

      /* Start the I/O thread */

      g_telnet_io_kthread =
        kthread_create("telnet_io", CONFIG_TELNET_IOTHREAD_PRIORITY,
                       CONFIG_TELNET_IOTHREAD_STACKSIZE, telnet_io_main,
                       NULL);
    }

  /* Save ourself in the list of Telnet client threads */

  g_telnet_clients[priv->td_minor] = priv;
  nxsem_post(&g_clients_sem);
  nxsem_post(&g_iosem);

  return OK;

errout_with_semaphore:
  nxsem_post(&g_clients_sem);

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
 * Name: telnet_ioctl
 ****************************************************************************/

static int telnet_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct telnet_dev_s *priv = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
#ifdef HAVE_SIGNALS
      /* Make the given terminal the controlling terminal of the calling process */

    case TIOCSCTTY:
      {
        /* Check if the ISIG flag is set in the termios c_lflag to enable
         * this feature.  This flag is set automatically for a serial console
         * device.
         */

        /* Save the PID of the recipient of the SIGINT signal. */

        priv->td_pid = (pid_t)arg;
        DEBUGASSERT((unsigned long)(priv->td_pid) == arg);
      }
      break;
#endif

#ifdef CONFIG_TELNET_SUPPORT_NAWS
      case TIOCGWINSZ:
        {
          FAR struct winsize *pw = (FAR struct winsize *)((uintptr_t)arg);

          /* Get row/col from the private data */

          pw->ws_row = priv->td_rows;
          pw->ws_col = priv->td_cols;
        }
      break;
#endif

    default:
      ret = -ENOTTY;
      break;
    }

  UNUSED(priv);  /* Avoid warning if not used */
  return ret;
}

/****************************************************************************
 * Name: telnet_poll
 *
 * Description:
 *   The standard poll() operation redirects operations on socket descriptors
 *   to this function.
 *
 * Input Parameters:
 *   fd    - The socket descriptor of interest
 *   fds   - The structure describing the events to be monitored, OR NULL if
 *           this is a request to stop monitoring events.
 *   setup - true: Setup up the poll; false: Teardown the poll
 *
 * Returned Value:
 *  0: Success; Negated errno on failure
 *
 ****************************************************************************/

static int telnet_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct telnet_dev_s *priv = inode->i_private;
  FAR struct socket *psock;

  DEBUGASSERT(fds != NULL);

  /* Get the underlying socket structure and verify that the sockfd
   * corresponds to valid, allocated socket
   */

  psock = &priv->td_psock;
  if (!psock || psock->s_crefs <= 0)
    {
      return -EBADF;
    }

  /* Test if we have cached data waiting to be read */

  if (priv->td_pending > 0)
    {
      /* Yes.. then signal the poll logic */

      fds->revents |= (POLLRDNORM & fds->events);
      if (fds->revents)
        {
          nxsem_post(fds->sem);
        }
    }

  /* Then let psock_poll() do the heavy lifting */

  return psock_poll(psock, fds, setup);
}

/****************************************************************************
 * Name: telnet_io_main
 ****************************************************************************/

static int telnet_io_main(int argc, FAR char** argv)
{
  FAR struct telnet_dev_s *priv;
  FAR char *buffer;
  int i;
  int ret;

  while (1)
    {
      nxsem_reset(&g_iosem, 0);

      /* Poll each client in the g_telnet_clients[] array. */

      nxsem_wait(&g_clients_sem);
      for (i = 0; i < CONFIG_TELNET_MAXLCLIENTS; i++)
        {
          priv = g_telnet_clients[i];
          if (priv != NULL && !(priv->td_fds.revents & (POLLHUP | POLLERR)))
            {
              priv->td_fds.sem     = &g_iosem;
              priv->td_fds.events  = POLLIN | POLLHUP | POLLERR;
              priv->td_fds.revents = 0;

              psock_poll(&priv->td_psock, &priv->td_fds, TRUE);
            }
        }

      nxsem_post(&g_clients_sem);

      /* Wait for any Telnet connect/disconnect events
       * to include/remove client sockets from polling
       */

      nxsem_wait(&g_iosem);

      /* Revisit each client in the g_telnet_clients[] array */

      nxsem_wait(&g_clients_sem);
      for (i = 0; i < CONFIG_TELNET_MAXLCLIENTS; i++)
        {
          priv = g_telnet_clients[i];

          /* If poll was setup previously (events != 0) */

          if (priv != NULL && priv->td_fds.events)
            {
              /* Check for a pending poll() */

              if (priv->td_fds.revents & POLLIN)
                {
                  if (priv->td_pending < CONFIG_TELNET_RXBUFFER_SIZE)
                    {
                      /* Take exclusive access to data buffer */

                      nxsem_wait(&priv->td_exclsem);
                      buffer = priv->td_rxbuffer + priv->td_pending +
                               priv->td_offset;

                      ret = psock_recv(&priv->td_psock, buffer,
                                       CONFIG_TELNET_RXBUFFER_SIZE -
                                       priv->td_pending - priv->td_offset,
                                       0);

                      priv->td_pending += ret;
                      nxsem_post(&priv->td_exclsem);

                      /* Notify the client thread that data is available */

                      nxsem_post(&priv->td_iosem);

#ifdef HAVE_SIGNALS
                      /* Check if any of the received characters is a
                       * control that should generate a signal.
                       */

                      telnet_check_ctrlchar(priv, buffer, ret);
#endif
                    }
                }

              /* Tear it down */

              psock_poll(&priv->td_psock, &priv->td_fds, FALSE);
              priv->td_fds.events = 0;

              /* POLLHUP (or POLLERR) indicates that this session has
               * terminated.
               */

              if (priv->td_fds.revents & (POLLHUP | POLLERR))
                {
                  /* notify the client thread */

                  nxsem_post(&priv->td_iosem);
                }
            }
        }

      nxsem_post(&g_clients_sem);
    }

  return 0;
}

/****************************************************************************
 * Name: factory_ioctl
 ****************************************************************************/

static int factory_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int ret = OK;

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
