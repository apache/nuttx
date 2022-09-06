/****************************************************************************
 * drivers/serial/pty.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdbool.h>
#include <unistd.h>
#include <sched.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/fs/fs.h>
#include <nuttx/serial/pty.h>

#include "pty.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum number of threads than can be waiting for POLL events */

#ifndef CONFIG_DEV_PTY_NPOLLWAITERS
#  define CONFIG_DEV_PTY_NPOLLWAITERS 2
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pty_poll_s
{
  FAR void *src;
  FAR void *sink;
};

/* This device structure describes on memory of the PTY device pair */

struct pty_devpair_s;
struct pty_dev_s
{
  FAR struct pty_devpair_s *pd_devpair;
  struct file pd_src;           /* Provides data to read() method (pipe output) */
  struct file pd_sink;          /* Accepts data from write() method (pipe input) */
  bool pd_master;               /* True: this is the master */
#ifdef CONFIG_SERIAL_TERMIOS
  tcflag_t pd_iflag;            /* Terminal input modes */
#endif
  tcflag_t pd_oflag;            /* Terminal output modes */
  struct pty_poll_s pd_poll[CONFIG_DEV_PTY_NPOLLWAITERS];
};

/* This structure describes the pipe pair */

struct pty_devpair_s
{
  struct pty_dev_s pp_master;   /* Maseter device */
  struct pty_dev_s pp_slave;    /* Slave device */

  bool pp_susv1;                /* SUSv1 or BSD style */
  bool pp_locked;               /* Slave is locked */
  bool pp_unlinked;             /* File has been unlinked */
  uint8_t pp_minor;             /* Minor device number */
  uint16_t pp_nopen;            /* Open file count */
  sem_t pp_slavesem;            /* Slave lock semaphore */
  mutex_t pp_lock;              /* Mutual exclusion */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void    pty_destroy(FAR struct pty_devpair_s *devpair);
static int     pty_pipe(FAR struct pty_devpair_s *devpair);
static int     pty_open(FAR struct file *filep);
static int     pty_close(FAR struct file *filep);
static ssize_t pty_read(FAR struct file *filep, FAR char *buffer,
                        size_t buflen);
static ssize_t pty_write(FAR struct file *filep, FAR const char *buffer,
                         size_t buflen);
static int     pty_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int     pty_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     pty_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_pty_fops =
{
  pty_open,      /* open */
  pty_close,     /* close */
  pty_read,      /* read */
  pty_write,     /* write */
  NULL,          /* seek */
  pty_ioctl,     /* ioctl */
  pty_poll       /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , pty_unlink   /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pty_destroy
 ****************************************************************************/

static void pty_destroy(FAR struct pty_devpair_s *devpair)
{
  char devname[16];

  if (devpair->pp_susv1)
    {
      /* Free this minor number so that it can be reused */

      ptmx_minor_free(devpair->pp_minor);

      /* Un-register the slave device */

      snprintf(devname, 16, "/dev/pts/%d", devpair->pp_minor);
    }
  else
    {
      /* Un-register the master device (/dev/ptyN may have already been
       * unlinked).
       */

      snprintf(devname, 16, "/dev/pty%d", (int)devpair->pp_minor);
      unregister_driver(devname);

      /* Un-register the slave device */

      snprintf(devname, 16, "/dev/ttyp%d", devpair->pp_minor);
    }

  unregister_driver(devname);

  /* And free the device structure */

  nxmutex_destroy(&devpair->pp_lock);
  nxsem_destroy(&devpair->pp_slavesem);
  kmm_free(devpair);
}

/****************************************************************************
 * Name: pty_pipe
 ****************************************************************************/

static int pty_pipe(FAR struct pty_devpair_s *devpair)
{
  FAR struct file *pipe_a[2];
  FAR struct file *pipe_b[2];
  int ret;

  /* Create two pipes:
   *
   *   pipe_a:  Master source, slave sink (TX, slave-to-master)
   *   pipe_b:  Master sink, slave source (RX, master-to-slave)
   */

  pipe_a[0] = &devpair->pp_master.pd_src;
  pipe_a[1] = &devpair->pp_slave.pd_sink;

  ret = file_pipe(pipe_a, CONFIG_PSEUDOTERM_TXBUFSIZE, 0);
  if (ret < 0)
    {
      return ret;
    }

  pipe_b[0] = &devpair->pp_slave.pd_src;
  pipe_b[1] = &devpair->pp_master.pd_sink;

  ret = file_pipe(pipe_b, CONFIG_PSEUDOTERM_RXBUFSIZE, 0);
  if (ret < 0)
    {
      file_close(pipe_a[0]);
      file_close(pipe_a[1]);
    }

  return ret;
}

/****************************************************************************
 * Name: pty_open
 ****************************************************************************/

static int pty_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct pty_dev_s *dev;
  FAR struct pty_devpair_s *devpair;
  int ret = OK;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode   = filep->f_inode;
  dev     = inode->i_private;
  DEBUGASSERT(dev != NULL && dev->pd_devpair != NULL);
  devpair = dev->pd_devpair;

  /* Get exclusive access to the device structure */

  ret = nxmutex_lock(&devpair->pp_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait if this is an attempt to open the slave device and the slave
   * device is locked.
   */

  if (!dev->pd_master)
    {
      /* Slave... Check if the slave driver is locked. */

      while (devpair->pp_locked)
        {
          /* Release the exclusive access before wait */

          nxmutex_unlock(&devpair->pp_lock);

          /* Wait until unlocked.
           * We will also most certainly suspend here.
           */

          ret = nxsem_wait(&devpair->pp_slavesem);
          if (ret < 0)
            {
              return ret;
            }

          /* Restore the semaphore count */

          DEBUGVERIFY(nxsem_post(&devpair->pp_slavesem));

          /* Get exclusive access to the device structure.  This might also
           * cause suspension.
           */

          ret = nxmutex_lock(&devpair->pp_lock);
          if (ret < 0)
            {
              return ret;
            }
        }
    }

  /* If one side of the driver has been unlinked, then refuse further
   * opens.
   */

  if (devpair->pp_unlinked)
    {
      ret = -EIDRM;
    }
  else
    {
      /* First open? */

      if (devpair->pp_nopen == 0)
        {
          /* Yes, create the internal pipe */

          ret = pty_pipe(devpair);
        }

      /* Increment the count of open references on the driver */

      if (ret >= 0)
        {
          devpair->pp_nopen++;
          DEBUGASSERT(devpair->pp_nopen > 0);
        }
    }

  nxmutex_unlock(&devpair->pp_lock);
  return ret;
}

/****************************************************************************
 * Name: pty_open
 ****************************************************************************/

static int pty_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct pty_dev_s *dev;
  FAR struct pty_devpair_s *devpair;
  int ret;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode   = filep->f_inode;
  dev     = inode->i_private;
  DEBUGASSERT(dev != NULL && dev->pd_devpair != NULL);
  devpair = dev->pd_devpair;

  /* Get exclusive access */

  ret = nxmutex_lock(&devpair->pp_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Check if the decremented inode reference count would go to zero */

  if (inode->i_crefs == 1)
    {
      /* Did the (single) master just close its reference? */

      if (dev->pd_master && devpair->pp_susv1)
        {
          /* Yes, then we are essentially unlinked and when all of the
           * slaves close there references, then the PTY should be
           * destroyed.
           */

          devpair->pp_unlinked = true;
        }

      /* Close the contained file structures */

      file_close(&dev->pd_src);
      file_close(&dev->pd_sink);
    }

  /* Is this the last open reference?  If so, was the driver previously
   * unlinked?
   */

  DEBUGASSERT(devpair->pp_nopen > 0);
  if (devpair->pp_nopen <= 1 && devpair->pp_unlinked)
    {
      /* Yes.. Free the device pair now (without freeing the semaphore) */

      pty_destroy(devpair);
      return OK;
    }
  else
    {
      /* Otherwise just decrement the open count */

      devpair->pp_nopen--;
    }

  nxmutex_unlock(&devpair->pp_lock);
  return OK;
}

/****************************************************************************
 * Name: pty_read
 ****************************************************************************/

static ssize_t pty_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode *inode;
  FAR struct pty_dev_s *dev;
  ssize_t ntotal;
#ifdef CONFIG_SERIAL_TERMIOS
  ssize_t i;
  ssize_t j;
  char ch;
#endif

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  dev   = inode->i_private;
  DEBUGASSERT(dev != NULL);

#ifdef CONFIG_SERIAL_TERMIOS
  /* Do input processing if any is enabled
   *
   * Specifically not handled:
   *
   *   All of the local modes; echo, line editing, etc.
   *   Anything to do with break or parity errors.
   *   ISTRIP     - We should be 8-bit clean.
   *   IUCLC      - Not Posix
   *   IXON/OXOFF - No xon/xoff flow control.
   */

  if (dev->pd_iflag & (INLCR | IGNCR | ICRNL))
    {
      while ((ntotal = file_read(&dev->pd_src, buffer, len)) > 0)
        {
          for (i = j = 0; i < ntotal; i++)
            {
              /* Perform input processing */

              ch = buffer[i];

              /* \n -> \r or \r -> \n translation? */

              if (ch == '\n' && (dev->pd_iflag & INLCR) != 0)
                {
                  ch = '\r';
                }
              else if (ch == '\r' && (dev->pd_iflag & ICRNL) != 0)
                {
                  ch = '\n';
                }

              /* Discarding \r ?  Print character if (1) character is not \r
               * or if (2) we were not asked to ignore \r.
               */

              if (ch != '\r' || (dev->pd_iflag & IGNCR) == 0)
                {
                  buffer[j++] = ch;
                }
            }

          /* Is the buffer not empty after process? */

          if (j != 0)
            {
              /* Yes, we are done. */

              ntotal = j;
              break;
            }
        }
    }
  else
#endif
    {
      /* NOTE: the source pipe will block if no data is available in
       * the pipe.   Otherwise, it will return data from the pipe.  If
       * there are fewer than 'len' bytes in the, it will return with
       * ntotal < len.
       *
       * REVISIT: Should not block if the oflags include O_NONBLOCK.
       * How would we ripple the O_NONBLOCK characteristic to the
       * contained source pipe? file_fcntl()?  Or FIONREAD?  See the
       * TODO comment at the top of this file.
       */

      ntotal = file_read(&dev->pd_src, buffer, len);
    }

  return ntotal;
}

/****************************************************************************
 * Name: pty_write
 ****************************************************************************/

static ssize_t pty_write(FAR struct file *filep,
                         FAR const char *buffer, size_t len)
{
  FAR struct inode *inode;
  FAR struct pty_dev_s *dev;
  ssize_t ntotal;
  ssize_t nwritten;
  size_t i;
  char ch;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  dev   = inode->i_private;
  DEBUGASSERT(dev != NULL);

  /* Do output post-processing */

  if ((dev->pd_oflag & OPOST) != 0)
    {
      /* We will transfer one byte at a time, making the appropriae
       * translations.  Specifically not handled:
       *
       *   OXTABS - primarily a full-screen terminal optimisation
       *   ONOEOT - Unix interoperability hack
       *   OLCUC  - Not specified by POSIX
       *   ONOCR  - low-speed interactive optimisation
       */

      ntotal = 0;
      for (i = 0; i < len; i++)
        {
          ch = *buffer++;

          /* Mapping CR to NL? */

          if (ch == '\r' && (dev->pd_oflag & OCRNL) != 0)
            {
              ch = '\n';
            }

          /* Are we interested in newline processing? */

          if ((ch == '\n') && (dev->pd_oflag & (ONLCR | ONLRET)) != 0)
            {
              char cr = '\r';

              /* Transfer the carriage return.  This will block if the
               * sink pipe is full.
               *
               * REVISIT: Should not block if the oflags include O_NONBLOCK.
               * How would we ripple the O_NONBLOCK characteristic to the
               * contained sink pipe?  file_fcntl()?  Or FIONSPACE?  See the
               * TODO comment at the top of this file.
               *
               * NOTE: The newline is not included in total number of bytes
               * written.  Otherwise, we would return more than the
               * requested number of bytes.
               */

              nwritten = file_write(&dev->pd_sink, &cr, 1);
              if (nwritten < 0)
                {
                  ntotal = nwritten;
                  break;
                }
            }

          /* Transfer the (possibly translated) character..  This will block
           * if the sink pipe is full
           *
           * REVISIT: Should not block if the oflags include O_NONBLOCK.
           * How would we ripple the O_NONBLOCK characteristic to the
           * contained sink pipe?  file_fcntl()?  Or FIONSPACe?  See the
           * TODO comment at the top of this file.
           */

          nwritten = file_write(&dev->pd_sink, &ch, 1);
          if (nwritten < 0)
            {
              ntotal = nwritten;
              break;
            }

          /* Update the count of bytes transferred */

          ntotal++;
        }
    }
  else
    {
      /* Write the 'len' bytes to the sink pipe.  This will block until all
       * 'len' bytes have been written to the pipe.
       *
       * REVISIT: Should not block if the oflags include O_NONBLOCK.
       * How would we ripple the O_NONBLOCK characteristic to the
       * contained sink pipe?  file_fcntl()?  Or FIONSPACE?  See the
       * TODO comment at the top of this file.
       */

      ntotal = file_write(&dev->pd_sink, buffer, len);
    }

  return ntotal;
}

/****************************************************************************
 * Name: pty_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the PWM work is done.
 *
 ****************************************************************************/

static int pty_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode;
  FAR struct pty_dev_s *dev;
  FAR struct pty_devpair_s *devpair;
  int ret;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode   = filep->f_inode;
  dev     = inode->i_private;
  DEBUGASSERT(dev != NULL && dev->pd_devpair != NULL);
  devpair = dev->pd_devpair;

  /* Get exclusive access */

  ret = nxmutex_lock(&devpair->pp_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle IOCTL commands */

  switch (cmd)
    {
      /* PTY IOCTL commands would be handled here */

      case TIOCGPTN:    /* Get Pty Number (of pty-mux device): FAR int* */
        {
          FAR int *ptyno = (FAR int *)((uintptr_t)arg);
          if (ptyno == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              *ptyno = devpair->pp_minor;
              ret = OK;
            }
        }
        break;

      case TIOCSPTLCK:  /* Lock/unlock Pty: int */
        {
          if (arg == 0)
            {
              if (devpair->pp_locked)
                {
                  /* Release any waiting threads */

                  ret = nxsem_post(&devpair->pp_slavesem);
                  if (ret >= 0)
                    {
                      devpair->pp_locked = false;
                    }
                }
            }
          else if (!devpair->pp_locked)
            {
              /* Locking */

              ret = nxsem_wait(&devpair->pp_slavesem);
              if (ret >= 0)
                {
                  devpair->pp_locked = true;
                }
            }
        }
        break;

      case TIOCGPTLCK:  /* Get Pty lock state: FAR int* */
        {
          FAR int *ptr = (FAR int *)((uintptr_t)arg);
          if (ptr == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              *ptr = devpair->pp_locked;
              ret = OK;
            }
        }
        break;

#ifdef CONFIG_SERIAL_TERMIOS
      case TCGETS:
        {
          FAR struct termios *termiosp = (FAR struct termios *)arg;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          /* And update with flags from this layer */

          termiosp->c_iflag = dev->pd_iflag;
          termiosp->c_oflag = dev->pd_oflag;
          termiosp->c_lflag = 0;
          ret = OK;
        }
        break;

      case TCSETS:
        {
          FAR struct termios *termiosp = (FAR struct termios *)arg;

          if (!termiosp)
            {
              ret = -EINVAL;
              break;
            }

          /* Update the flags we keep at this layer */

          dev->pd_iflag = termiosp->c_iflag;
          dev->pd_oflag = termiosp->c_oflag;
          ret = OK;
        }
        break;
#endif

      /* Get the number of bytes that are immediately available for reading
       * from the source pipe.
       */

      case FIONREAD:
        {
          ret = file_ioctl(&dev->pd_src, cmd, arg);
        }
        break;

      /* Get the number of bytes waiting in the sink pipe (FIONWRITE) or the
       * number of unused bytes in the sink pipe (FIONSPACE).
       */

      case FIONWRITE:
      case FIONSPACE:
        {
          ret = file_ioctl(&dev->pd_sink, cmd, arg);
        }
        break;

      case FIONBIO:
        {
          ret = file_ioctl(&dev->pd_src, cmd, arg);
          if (ret >= 0)
            {
              ret = file_ioctl(&dev->pd_sink, cmd, arg);
            }
        }
        break;

      /* Any unrecognized IOCTL commands will be passed to the contained
       * pipe driver.
       *
       * REVISIT:  We know for a fact that the pipe driver only supports
       * FIONREAD, FIONWRITE, FIONSPACE and PIPEIOC_POLICY.  The first two
       * are handled above and PIPEIOC_POLICY should not be managed by
       * applications -- it can break the PTY!
       */

      default:
        {
#if 0
          ret = file_ioctl(&dev->pd_src, cmd, arg);
          if (ret >= 0 || ret == -ENOTTY)
            {
              ret = file_ioctl(&dev->pd_sink, cmd, arg);
            }
#else
          ret = -ENOTTY;
#endif
        }
        break;
    }

  nxmutex_unlock(&devpair->pp_lock);
  return ret;
}

/****************************************************************************
 * Name: pty_poll
 ****************************************************************************/

static int pty_poll(FAR struct file *filep, FAR struct pollfd *fds,
                    bool setup)
{
  FAR struct inode *inode;
  FAR struct pty_dev_s *dev;
  FAR struct pty_devpair_s *devpair;
  FAR struct pty_poll_s *pollp = NULL;
  int ret;
  int i;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode   = filep->f_inode;
  dev     = inode->i_private;
  devpair = dev->pd_devpair;

  ret = nxmutex_lock(&devpair->pp_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      for (i = 0; i < CONFIG_DEV_PTY_NPOLLWAITERS; i++)
        {
          if (dev->pd_poll[i].src == NULL && dev->pd_poll[i].sink == NULL)
            {
              pollp = &dev->pd_poll[i];
              break;
            }
        }

      if (i >= CONFIG_DEV_PTY_NPOLLWAITERS)
        {
          ret = -EBUSY;
          goto errout;
        }
    }
  else
    {
      pollp = (FAR struct pty_poll_s *)fds->priv;
    }

  /* POLLIN: Data may be read without blocking. */

  if ((fds->events & POLLIN) != 0)
    {
      fds->priv = pollp->src;
      ret = file_poll(&dev->pd_src, fds, setup);
      if (ret < 0)
        {
          goto errout;
        }

      pollp->src = fds->priv;
    }

  /* POLLOUT: Normal data may be written without blocking. */

  if ((fds->events & POLLOUT) != 0)
    {
      fds->priv = pollp->sink;
      ret = file_poll(&dev->pd_sink, fds, setup);
      if (ret < 0)
        {
          if (pollp->src)
            {
              fds->priv = pollp->src;
              file_poll(&dev->pd_src, fds, false);
              pollp->src = NULL;
            }

          goto errout;
        }

      pollp->sink = fds->priv;
    }

  if (setup)
    {
      fds->priv = pollp;
    }

errout:
  nxmutex_unlock(&devpair->pp_lock);
  return ret;
}

/****************************************************************************
 * Name: pty_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int pty_unlink(FAR struct inode *inode)
{
  FAR struct pty_dev_s *dev;
  FAR struct pty_devpair_s *devpair;
  int ret;

  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  dev     = inode->i_private;
  devpair = dev->pd_devpair;
  DEBUGASSERT(dev->pd_devpair != NULL);

  /* Get exclusive access */

  ret = nxmutex_lock(&devpair->pp_lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Indicate that the driver has been unlinked */

  devpair->pp_unlinked = true;

  /* If there are no further open references to the driver, then commit
   * Hara-Kiri now.
   */

  if (devpair->pp_nopen == 0)
    {
      pty_destroy(devpair);
      return OK;
    }

  nxmutex_unlock(&devpair->pp_lock);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pty_register2
 *
 * Description:
 *   Create and register PTY master and slave devices.  The slave side of
 *   the interface is always locked initially.  The master must call
 *   unlockpt() before the slave device can be opened.
 *
 * Input Parameters:
 *   minor - The number that qualifies the naming of the created devices.
 *   susv1 - select SUSv1 or BSD behaviour
 *
 * Returned Value:
 *   0 is returned on success; otherwise, the negative error code return
 *   appropriately.
 *
 ****************************************************************************/

int pty_register2(int minor, bool susv1)
{
  FAR struct pty_devpair_s *devpair;
  char devname[16];
  int ret;

  /* Allocate a device instance */

  devpair = kmm_zalloc(sizeof(struct pty_devpair_s));
  if (devpair == NULL)
    {
      return -ENOMEM;
    }

  /* Initialize semaphores & mutex */

  nxsem_init(&devpair->pp_slavesem, 0, 0);
  nxmutex_init(&devpair->pp_lock);

  /* The pp_slavesem semaphore is used for signaling and, hence, should not
   * have priority inheritance enabled.
   */

  nxsem_set_protocol(&devpair->pp_slavesem, SEM_PRIO_NONE);

  devpair->pp_susv1             = susv1;
  devpair->pp_minor             = minor;
  devpair->pp_locked            = true;
  devpair->pp_master.pd_devpair = devpair;
  devpair->pp_master.pd_master  = true;
  devpair->pp_slave.pd_devpair  = devpair;
  devpair->pp_slave.pd_oflag    = OPOST | ONLCR;

  /* Register the master device
   *
   * BSD style (deprecated): /dev/ptyN
   * SUSv1 style: Master: /dev/ptmx (multiplexor, see ptmx.c)
   *
   * Where N is the minor number
   */

  snprintf(devname, 16, "/dev/pty%d", minor);

  ret = register_driver(devname, &g_pty_fops, 0666, &devpair->pp_master);
  if (ret < 0)
    {
      goto errout_with_devpair;
    }

  /* Register the slave device
   *
   * BSD style (deprecated): /dev/ttypN
   * SUSv1 style: /dev/pts/N
   *
   * Where N is the minor number
   */

  if (susv1)
    {
      snprintf(devname, 16, "/dev/pts/%d", minor);
    }
  else
    {
      snprintf(devname, 16, "/dev/ttyp%d", minor);
    }

  ret = register_driver(devname, &g_pty_fops, 0666, &devpair->pp_slave);
  if (ret < 0)
    {
      goto errout_with_master;
    }

  return OK;

errout_with_master:
  snprintf(devname, 16, "/dev/pty%d", minor);
  unregister_driver(devname);

errout_with_devpair:
  nxmutex_destroy(&devpair->pp_lock);
  nxsem_destroy(&devpair->pp_slavesem);
  kmm_free(devpair);
  return ret;
}

/****************************************************************************
 * Name: pty_register
 *
 * Description:
 *   Create and register PTY master and slave devices.  The master device
 *   will be registered at /dev/ptyN and slave at /dev/ttypN where N is
 *   the provided minor number.
 *
 *   The slave side of the interface is always locked initially.  The
 *   master must call unlockpt() before the slave device can be opened.
 *
 * Input Parameters:
 *   minor - The number that qualifies the naming of the created devices.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int pty_register(int minor)
{
  return pty_register2(minor, false);
}
