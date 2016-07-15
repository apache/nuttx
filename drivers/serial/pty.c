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
#include <stdbool.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

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
  FAR struct pty_common_s *pd_devpair;
  struct file pd_src;           /* Provides data to read() method (pipe output) */
  struct file pd_sink;          /* Accepts data from write() method (pipe input) */
};

/* This structure describes the pipe pair */

struct pty_devpair_s
{
  struct pty_dev_s pp_ptyp;     /* /dev/ptypN device */    
  struct pty_dev_s pp_ttyp;     /* /dev/ttypN device */    

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  uint8_t pp_minor;             /* Minor device number */
  uint16_t pp_nopen;            /* Open file count */
  sem_t pp_exclsem;             /* Mutual exclusion */
  bool pp_unlinked;             /* File has been unlinked */
#endif
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

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static void pty_semtake(FAR struct pty_common_s *devpair)
{
  while (sem_wait(&devpair->pp_exclsem) < 0)
    {
      DEBUGASSERT(errno == EINTR);
    }
}
#endif

/****************************************************************************
 * Name: pty_semgive
 ****************************************************************************/

#define pty_semgive(c) sem_post(&(c)->pp_exclsem)

/****************************************************************************
 * Name: pty_destroy
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static void pty_destroy(FAR struct pty_common_s *devpair)
{
  char devname[16];

  /* Un-register /dev/ptypN */

  snprintf(devname, 16, "/dev/pp_ptyp%d", (int)devpair->pp_minor);
  (void)unregister_driver(devname);
  
  /* Un-register /dev/ptypN */

  snprintf(devname, 16, "/dev/ttyp%d", (int)devpair->pp_minor);
  (void)unregister_driver(devname);

  /* Close the contained file structures */

  (void)file_close_detached(&devpair->pp_ptyp.pd_src);
  (void)file_close_detached(&devpair->pp_ptyp.pd_sink);
  (void)file_close_detached(&devpair->pp_ttyp.pd_src);
  (void)file_close_detached(&devpair->pp_ttyp.pd_sink);

  /* And free the device structure */

  sem_destroy(&devpair->pp_exclsem);
  kmm_free(upper);
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
  FAR struct pty_common_s *devpair;
  int ret;

  DEBUGASSERT(filep != NULL && file->f_inode != NULL);
  inode   = filep->f_inode;
  dev     = inode->i_private;
  DEBUGASSERT(dev != NULL && dev->pd_devpair != NULL);
  devpair = dev->pd_devpair;

  /* Get exclusive access */

  pty_semtake(devpair);

  /* If one side of the driver has been unlinked, then refuse further
   * opens.
   */

  if (cmd->pp_unlinked)
    {
      ret = -EIDRAM
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
  FAR struct pty_common_s *devpair;

  DEBUGASSERT(filep != NULL && file->f_inode != NULL);
  inode     = filep->f_inode;
  dev       = inode->i_private;
  DEBUGASSERT(dev != NULL && dev->pd_devpair != NULL);
  devpair   = dev->pd_devpair;

  /* Get exclusive access */

  pty_semtake(devpair);

  /* Is this the last open reference? */

  DEBUGASSERT(devpair->pp_nopen > 0);
  if (devpair->pp_nopen <= 1 && devpair->pp_unlinked)
    {
      /* Free the device pair now (without freeing the semaphore) */

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

  DEBUGASSERT(filep != NULL && file->f_inode != NULL);
  inode = filep->f_inode;
  dev   = inode->i_private;
  DEBUGASSERT(dev != NULL);

  return file_read(&dev->src, buffer, len);
}

/****************************************************************************
 * Name: pty_write
 ****************************************************************************/

static ssize_t pty_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
  FAR struct inode *inode;
  FAR struct pty_dev_s *dev;

  DEBUGASSERT(filep != NULL && file->f_inode != NULL);
  inode = filep->f_inode;
  dev   = inode->i_private;
  DEBUGASSERT(dev != NULL);

  return file_write(&dev->src, buffer, len);
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
  int ret;

  DEBUGASSERT(filep != NULL && file->f_inode != NULL);
  inode = filep->f_inode;
  dev   = inode->i_private;
  DEBUGASSERT(dev != NULL);

  /* Handle IOCTL commands */

  switch (cmd)
    {
      /* PTY IOCTL commands would be handled here */
      /* There aren't any yet */

      /* Any unrecognized IOCTL commands will be passed to the contained
       * pipe driver.
       */

      default:
        {
          ret = file_ioctl(dev->pd_src, cmd, arg);
          if (ret >= 0 || ret == -ENOTTY)
            {
              ret = file_ioctl(dev->pd_sink, cmd, arg);
            }
        }
        break;
    }

  sem_post(&upper->exclsem);
  return ret;
}

/****************************************************************************
 * Name: pty_poll
 ****************************************************************************/

#ifndef CONFIG_DISABLE_POLL
static int pty_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
#warning Missing logic
}
#endif

/****************************************************************************
 * Name: pty_unlink
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int pty_unlink(FAR struct inode *inode)
{
  FAR struct inode *inode;
  FAR struct pty_dev_s *dev;
  FAR struct pty_common_s *devpair;

  DEBUGASSERT(filep != NULL && file->f_inode != NULL);
  inode     = filep->f_inode;
  dev       = inode->i_private;
  DEBUGASSERT(dev != NULL && dev->pd_devpair != NULL);
  devpair   = dev->pd_devpair;

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
 * Input Parameters:
 *   
 * Description:
 *   Register /dev/ttypN and /dev/ptpN where N=minor number
 *
 ****************************************************************************/

int pty_register(int minor)
{
  FAR struct pty_devpair_s *devpair;
  int pipe_a[2];
  int pipe_b[2];
  char devname[16];

  /* Allocate a device instance */

  devpair = kmm_zalloc(sizeof(struct pty_devpair_s));
  if (devpair == NULL)
    {
      return -ENOMEM;
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  sem_init(&devpair->pp_exclsem, 0, 1);
  devpair->pp_minor           = minor;
#endif
  devpair->pp_ptyp.pd_devpair = devpair;
  devpair->pp_ttyp.pd_devpair = devpair;

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

  ret = file_detach(pipe_a[0], &devpair->pp_ptyp.pd_src);
  if (ret < 0)
    {
      goto errout_with_pipeb;
    }

  pipe_a[0] = -1;

  ret = file_detach(pipe_a[1], &devpair->pp_ttyp.pd_sink);
  if (ret < 0)
    {
      goto errout_with_pipeb;
    }

  pipe_a[1] = -1;

  ret = file_detach(pipe_b[0], &devpair->pp_ttyp.pd_src);
  if (ret < 0)
    {
      goto errout_with_pipeb;
    }

  pipe_b[0] = -1;

  ret = file_detach(pipe_b[1], &devpair->pp_ptyp.pd_sink);
  if (ret < 0)
    {
      goto errout_with_pipeb;
    }

  pipe_b[1] = -1;

  /* Register /dev/ptypN */

  snprintf(devname, 16, "/dev/pts/%d", minor);

  ret = register_driver(devname, &pty_fops, 0666, &devpair->pp_ptyp);
  if (ret < 0)
    {
      goto errout_with_pipeb;
    }
  
  /* Register /dev/ptypN */

  snprintf(devname, 16, "/dev/ttyp%d", minor);

  ret = register_driver(devname, &pty_fops, 0666, &devpair->pp_ttyp);
  if (ret < 0)
    {
      goto errout_with_ptyp;
    }

  return OK;

errout_with_ptyp:
  snprintf(devname, 16, "/dev/ptyp%d", minor);
  (void)unregister_driver(devname)

errout_with_pipeb:
  if (pipe_b[0] >= 0)
    {
      close(pipe_b[0]);
    }
  else
    {
      (void)file_close_detached(&devpair->pp_ptyp.pd_src);
    }

  if (pipe_b[1] >= 0)
    {
      close(pipe_b[1]);
    }
  else
    {
      (void)file_close_detached(&devpair->pp_ttyp.pd_sink);
    }

errout_with_pipea:
  if (pipe_a[0] >= 0)
    {
      close(pipe_a[0]);
    }
  else
    {
      (void)file_close_detached(&devpair->pp_ttyp.pd_src);
    }

  if (pipe_a[1] >= 0)
    {
      close(pipe_a[1]);
    }
  else
    {
      (void)file_close_detached(&devpair->pp_ptyp.pd_sink);
    }

errout_with_devpair:
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
   sem_destroy(&devpair->pp_exclsem);
#endif
   kmm_free(devpair);
   return ret;
}
