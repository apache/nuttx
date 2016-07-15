/****************************************************************************
 * drivers/serial/pty.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include <semaphore.h>
#include <stdio.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/serial/pty.h>

#include "pty.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     pty_open(FAR struct file *filep);
static int     pty_close(FAR struct file *filep);
static ssize_t pty_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t pty_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     pty_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
#ifndef CONFIG_DISABLE_POLL
static int     pty_poll(FAR struct file *filep, FAR struct pollfd *fds,
                 bool setup);
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     pty_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This device structure describes on memory of the PTY device pair */

struct pty_devpair_s;
struct pty_dev_s
{
  FAR struct pty_devpair_s *pd_devpair;
  struct file pd_src;           /* Provides data to read() method (pipe output) */
  struct file pd_sink;          /* Accepts data from write() method (pipe input) */
  bool pd_master;               /* True: this is the master */
};

/* This structure describes the pipe pair */

struct pty_devpair_s
{
  struct pty_dev_s pp_master;   /* Maseter device */
  struct pty_dev_s pp_slave;    /* Slave device */

  bool pp_locked;               /* Slave is locked */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool pp_unlinked;             /* File has been unlinked */
  uint8_t pp_minor;             /* Minor device number */
  uint16_t pp_nopen;            /* Open file count */
#endif
  sem_t pp_slavesem;            /* Slave lock semaphore */
  sem_t pp_exclsem;             /* Mutual exclusion */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations pty_fops =
{
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  pty_open,      /* open */
  pty_close,     /* close */
#else
  NULL,          /* open */
  NULL,          /* close */
#endif
  pty_read,      /* read */
  pty_write,     /* write */
  NULL,          /* seek */
  pty_ioctl      /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , pty_poll     /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , pty_unlink   /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pty_semtake
 ****************************************************************************/

static void pty_semtake(FAR struct pty_devpair_s *devpair)
{
  while (sem_wait(&devpair->pp_exclsem) < 0)
    {
      DEBUGASSERT(errno == EINTR);
    }
}

/****************************************************************************
 * Name: pty_semgive
 ****************************************************************************/

#define pty_semgive(c) sem_post(&(c)->pp_exclsem)

/****************************************************************************
 * Name: pty_destroy
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static void pty_destroy(FAR struct pty_devpair_s *devpair)
{
  char devname[16];

  /* Un-register the slave device */

#ifdef CONFIG_PSEUDOTERM_BSD
  snprintf(devname, 16, "/dev/ttyp%d", devpair->pp_minor);
#else
  snprintf(devname, 16, "/dev/pts/%d", devpair->pp_minor);
#endif
  (void)unregister_driver(devname);

  /* Un-register the master device (/dev/ptyN may have already been unlinked) */

  snprintf(devname, 16, "/dev/pty%d", (int)devpair->pp_minor);
  (void)unregister_driver(devname);

  /* Close the contained file structures */

  (void)file_close_detached(&devpair->pp_master.pd_src);
  (void)file_close_detached(&devpair->pp_master.pd_sink);
  (void)file_close_detached(&devpair->pp_slave.pd_src);
  (void)file_close_detached(&devpair->pp_slave.pd_sink);

#ifdef CONFIG_PSEUDOTERM_SUSV1
  /* Free this minor number so that it can be reused */

  ptmx_minor_free(devpair->pp_minor);
#endif

  /* And free the device structure */

  sem_destroy(&devpair->pp_exclsem);
  kmm_free(devpair);
}
#endif

/****************************************************************************
 * Name: pty_open
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int pty_open(FAR struct file *filep)
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

  /* Wait if this is an attempt to open the slave device and the slave
   * device is locked.
   */

  if (!dev->pd_master)
    {
      /* Slave... Check if the slave driver is locked.  We need to lock the
       * scheduler while we are running to prevent asyncrhonous modification
       * of pp_locked by pty_ioctl().
       */

      sched_lock();
      while (devpair->pp_locked)
        {
          /* Wait until unlocked.  We will also most certainly suspend here. */

          sem_wait(&devpair->pp_slavesem);

          /* Get exclusive access to the device structure.  This might also
           * cause suspension.
           */

          pty_semtake(devpair);

          /* Check again in case something happened asynchronously while we
           * were suspended.
           */

          if (devpair->pp_locked)
           {
             /* This cannot suspend because we have the scheduler locked.
              * So pp_locked cannot change asyncrhonously between this test
              * and the redundant test at the top of the loop.
              */

             pty_semgive(devpair);
           }
        }

      sched_unlock();
    }
  else
    {
       /* Master ... Get exclusive access to the device structure */

       pty_semtake(devpair);
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
      /* Increment the count of open references on the driver */

      devpair->pp_nopen++;
      DEBUGASSERT(devpair->pp_nopen > 0);

      ret = OK;
    }

  pty_semgive(devpair);
  return ret;
}
#endif

/****************************************************************************
 * Name: pty_open
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int pty_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct pty_dev_s *dev;
  FAR struct pty_devpair_s *devpair;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode     = filep->f_inode;
  dev       = inode->i_private;
  DEBUGASSERT(dev != NULL && dev->pd_devpair != NULL);
  devpair   = dev->pd_devpair;

  /* Get exclusive access */

  pty_semtake(devpair);

#ifdef CONFIG_PSEUDOTERM_SUSV1
  /* Did the (single) master just close its reference? */

  if (dev->pd_master)
    {
      /* Yes, then we are essentially unlinked and when all of the
       * slaves close there references, then the PTY should be
       * destroyed.
       */

      devpair->pp_unlinked = true;
    }
#endif

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

  pty_semgive(devpair);
  return OK;
}
#endif

/****************************************************************************
 * Name: pty_read
 ****************************************************************************/

static ssize_t pty_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
  FAR struct inode *inode;
  FAR struct pty_dev_s *dev;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  dev   = inode->i_private;
  DEBUGASSERT(dev != NULL);

  return file_read(&dev->pd_src, buffer, len);
}

/****************************************************************************
 * Name: pty_write
 ****************************************************************************/

static ssize_t pty_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
  FAR struct inode *inode;
  FAR struct pty_dev_s *dev;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;
  dev   = inode->i_private;
  DEBUGASSERT(dev != NULL);

  return file_write(&dev->pd_sink, buffer, len);
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

  pty_semtake(devpair);

  /* Handle IOCTL commands */

  switch (cmd)
    {
      /* PTY IOCTL commands would be handled here */

      case TIOCGPTN:    /* Get Pty Number (of pty-mux device): FAR int* */
        {
#ifdef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
          ret = -ENOSYS;
#else
          FAR int *ptyno = (FAR int *)((uintptr_t)arg);
          if (ptyno == NULL)
            {
              ret = -EINVAL;
            }
          else
            {
              *ptyno = (int)devpair->pp_minor;
              ret = OK;
            }
#endif
        }
        break;

      case TIOCSPTLCK:  /* Lock/unlock Pty: int */
        {
          if (arg == 0)
            {
               int sval;

               /* Unlocking */

               sched_lock();
               devpair->pp_locked = false;

               /* Release any waiting threads */

               do
                 {
                   DEBUGVERIFY(sem_getvalue(&devpair->pp_slavesem, &sval));
                   if (sval < 0)
                     {
                       sem_post(&devpair->pp_slavesem);
                     }
                 }
               while (sval < 0);

               sched_unlock();
               ret = OK;
            }
          else
            {
              /* Locking */

               devpair->pp_locked = true;
               ret = OK;
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
              *ptr = (int)devpair->pp_locked;
              ret = OK;
            }
        }
        break;

      /* Any unrecognized IOCTL commands will be passed to the contained
       * pipe driver.
       */

      default:
        {
          ret = file_ioctl(&dev->pd_src, cmd, arg);
          if (ret >= 0 || ret == -ENOTTY)
            {
              ret = file_ioctl(&dev->pd_sink, cmd, arg);
            }
        }
        break;
    }

  pty_semgive(devpair);
  return ret;
}

/****************************************************************************
 * Name: pty_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
/* REVISIT: There is a file_poll() function, but it does not work in this
 * context so the logic is implemented locally as my_file_poll().
 */

static my_file_poll(FAR struct file *filep, FAR struct pollfd *fds, bool setup)
{
  FAR struct inode *inode;
  int ret = -ENOSYS;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode = filep->f_inode;

  /* Is a driver registered? Does it support the poll method?
   * If not, return -ENOSYS
   */

  if (inode != NULL && inode->u.i_ops != NULL && inode->u.i_ops->poll != NULL)
    {
      /* Yes, then setup the poll */

      ret = (int)inode->u.i_ops->poll(filep, fds, setup);
    }

  return ret;
}

static int pty_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode *inode;
  FAR struct pty_dev_s *dev;
  int ret = -ENOSYS;

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);
  inode   = filep->f_inode;
  dev     = inode->i_private;

  /* REVISIT: If both POLLIN and POLLOUT are set, might the following logic
   * fail?  Could we not get POLLIN on the sink file and POLLOUT on the source
   * file?
   *
   * REVISIT: There is a file_poll() function, but it does not work in this
   * context so the logic is implemented locally as my_file_poll().
   */

  /* POLLIN: Data other than high-priority data may be read without blocking. */

  if ((fds->events & POLLIN) != 0)
    {
      ret = my_file_poll(dev->pd_src, fds, setup);
    }

  if (ret >= OK || ret == -ENOTTY)
    {
      /* POLLOUT: Normal data may be written without blocking. */

      if ((fds->events & POLLOUT) != 0)
        {
          ret = my_file_poll(dev->pd_sink, fds, setup);
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: pty_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int pty_unlink(FAR struct inode *inode)
{
  FAR struct pty_dev_s *dev;
  FAR struct pty_devpair_s *devpair;

  DEBUGASSERT(inode != NULL && inode->i_private != NULL);
  dev       = inode->i_private;
  devpair   = dev->pd_devpair;
  DEBUGASSERT(dev->pd_devpair != NULL);

  /* Get exclusive access */

  pty_semtake(devpair);

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

  pty_semgive(devpair);
  return OK;
}
#endif
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pty_register
 *
 * Description:
 *   Create and register PTY master and slave devices.  The slave side of
 *   the interface is always locked initially.  The master must call
 *   unlockpt() before the slave device can be opened.
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
  FAR struct pty_devpair_s *devpair;
  int pipe_a[2];
  int pipe_b[2];
  char devname[16];
  int ret;

  /* Allocate a device instance */

  devpair = kmm_zalloc(sizeof(struct pty_devpair_s));
  if (devpair == NULL)
    {
      return -ENOMEM;
    }

  sem_init(&devpair->pp_slavesem, 0, 0);
  sem_init(&devpair->pp_exclsem, 0, 1);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  devpair->pp_minor             = minor;
#endif
  devpair->pp_locked            = true;
  devpair->pp_master.pd_devpair = devpair;
  devpair->pp_master.pd_master  = true;
  devpair->pp_slave.pd_devpair  = devpair;

  /* Create two pipes */

  ret = pipe(pipe_a);
  if (ret < 0)
    {
      goto errout_with_devpair;
    }

  ret = pipe(pipe_b);
  if (ret < 0)
    {
      goto errout_with_pipea;
    }

  /* Detach the pipe file descriptors (closing them in the process)
   *
   *  fd[0] is for reading;
   *  fd[1] is for writing.
   */

  ret = file_detach(pipe_a[0], &devpair->pp_master.pd_src);
  if (ret < 0)
    {
      goto errout_with_pipeb;
    }

  pipe_a[0] = -1;

  ret = file_detach(pipe_a[1], &devpair->pp_slave.pd_sink);
  if (ret < 0)
    {
      goto errout_with_pipeb;
    }

  pipe_a[1] = -1;

  ret = file_detach(pipe_b[0], &devpair->pp_slave.pd_src);
  if (ret < 0)
    {
      goto errout_with_pipeb;
    }

  pipe_b[0] = -1;

  ret = file_detach(pipe_b[1], &devpair->pp_master.pd_sink);
  if (ret < 0)
    {
      goto errout_with_pipeb;
    }

  pipe_b[1] = -1;

  /* Register the slave device
   *
   * BSD style (deprecated): /dev/ttypN
   * SUSv1 style:  /dev/pts/N
   *
   * Where N is the minor number
   */

#ifdef CONFIG_PSEUDOTERM_BSD
  snprintf(devname, 16, "/dev/ttyp%d", minor);
#else
  snprintf(devname, 16, "/dev/pts/%d", minor);
#endif

  ret = register_driver(devname, &pty_fops, 0666, &devpair->pp_slave);
  if (ret < 0)
    {
      goto errout_with_pipeb;
    }

  /* Register the master device
   *
   * BSD style (deprecated):  /dev/ptyN
   * SUSv1 style: Master: /dev/ptmx (see ptmx.c)
   *
   * Where N is the minor number
   */

  snprintf(devname, 16, "/dev/pty%d", minor);

  ret = register_driver(devname, &pty_fops, 0666, &devpair->pp_master);
  if (ret < 0)
    {
      goto errout_with_slave;
    }

  return OK;

errout_with_slave:
#ifdef CONFIG_PSEUDOTERM_BSD
  snprintf(devname, 16, "/dev/ttyp%d", minor);
#else
  snprintf(devname, 16, "/dev/pts/%d", minor);
#endif
  (void)unregister_driver(devname);

errout_with_pipeb:
  if (pipe_b[0] >= 0)
    {
      close(pipe_b[0]);
    }
  else
    {
      (void)file_close_detached(&devpair->pp_master.pd_src);
    }

  if (pipe_b[1] >= 0)
    {
      close(pipe_b[1]);
    }
  else
    {
      (void)file_close_detached(&devpair->pp_slave.pd_sink);
    }

errout_with_pipea:
  if (pipe_a[0] >= 0)
    {
      close(pipe_a[0]);
    }
  else
    {
      (void)file_close_detached(&devpair->pp_slave.pd_src);
    }

  if (pipe_a[1] >= 0)
    {
      close(pipe_a[1]);
    }
  else
    {
      (void)file_close_detached(&devpair->pp_master.pd_sink);
    }

errout_with_devpair:
   sem_destroy(&devpair->pp_exclsem);
   sem_destroy(&devpair->pp_slavesem);
   kmm_free(devpair);
   return ret;
}
