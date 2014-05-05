/****************************************************************************
 * drivers/timer.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Bob Doiron
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <semaphore.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/timer.h>

#ifdef CONFIG_TIMER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Debug ********************************************************************/
/* Non-standard debug that may be enabled just for testing the timer driver */

#ifdef CONFIG_DEBUG_TIMER
#  define tmrdbg    dbg
#  define tmrvdbg   vdbg
#  define tmrlldbg  lldbg
#  define tmrllvdbg llvdbg
#else
#  define tmrdbg(x...)
#  define tmrvdbg(x...)
#  define tmrlldbg(x...)
#  define tmrllvdbg(x...)
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct timer_upperhalf_s
{
  uint8_t   crefs;    /* The number of times the device has been opened */
//sem_t     exclsem;  /* Supports mutual exclusion */
  FAR char *path;     /* Registration path */

  /* The contained lower-half driver */
 
  FAR struct timer_lowerhalf_s *lower;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     timer_open(FAR struct file *filep);
static int     timer_close(FAR struct file *filep);
static ssize_t timer_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t timer_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     timer_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_timerops =
{
  timer_open,  /* open */
  timer_close, /* close */
  timer_read,  /* read */
  timer_write, /* write */
  0,           /* seek */
  timer_ioctl  /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  , 0          /* poll */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: timer_open
 *
 * Description:
 *   This function is called whenever the timer device is opened.
 *
 ************************************************************************************/

static int timer_open(FAR struct file *filep)
{
  FAR struct inode                *inode = filep->f_inode;
  FAR struct timer_upperhalf_s *upper = inode->i_private;
  uint8_t                          tmp;
  int                              ret;

  tmrvdbg("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = 1; //sem_wait(&upper->exclsem);
  if (ret < 0)
    {
      ret = -errno;
      goto errout;
    }

  /* Increment the count of references to the device.  If this the first
   * time that the driver has been opened for this device, then initialize
   * the device.
   */

  tmp = upper->crefs + 1;
  if (tmp == 0)
    {
      /* More than 255 opens; uint8_t overflows to zero */

      ret = -EMFILE;
      goto errout_with_sem;
    }

  /* Save the new open count */

  upper->crefs = tmp;
  ret = OK;

errout_with_sem:
//  sem_post(&upper->exclsem);
  
errout:
  return ret;
}

/************************************************************************************
 * Name: timer_close
 *
 * Description:
 *   This function is called when the timer device is closed.
 *
 ************************************************************************************/

static int timer_close(FAR struct file *filep)
{
  FAR struct inode                *inode = filep->f_inode;
  FAR struct timer_upperhalf_s *upper = inode->i_private;
  int                              ret;

  tmrvdbg("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = 1; //sem_wait(&upper->exclsem);
  if (ret < 0)
    {
      ret = -errno;
      goto errout;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (upper->crefs > 0)
    {
      upper->crefs--;
    }

  //sem_post(&upper->exclsem);
  ret = OK;
  
errout:
  return ret;
}

/************************************************************************************
 * Name: timer_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ************************************************************************************/

static ssize_t timer_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/************************************************************************************
 * Name: timer_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
 *
 ************************************************************************************/

static ssize_t timer_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return 0;
}

/************************************************************************************
 * Name: timer_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the timer work is
 *   done.
 *   
 ************************************************************************************/

static int timer_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode             *inode = filep->f_inode;
  FAR struct timer_upperhalf_s *upper = inode->i_private;
  FAR struct timer_lowerhalf_s *lower = upper->lower;
  int                           ret;

  tmrvdbg("cmd: %d arg: %ld\n", cmd, arg);
  DEBUGASSERT(upper && lower);

  /* Get exclusive access to the device structures */

  ret = 1; //sem_wait(&upper->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Handle built-in ioctl commands */

  switch (cmd)
    {
    /* cmd:         TCIOC_START
     * Description: Start the timer
     * Argument:    Ignored
     */

    case TCIOC_START:
      {
        /* Start the timer, resetting the time to the current timeout */

        if(lower->ops->start)
          {
            ret = lower->ops->start(lower);
          }
        else
          {
            ret = -ENOSYS;
          }
      }
      break;

    /* cmd:         TCIOC_STOP
     * Description: Stop the timer
     * Argument:    Ignored
     */

    case TCIOC_STOP:
      {
        /* Stop the timer */

        if(lower->ops->start)
          {
            ret = lower->ops->stop(lower);
          }
        else
          {
            ret = -ENOSYS;
          }
      }
      break;

    /* cmd:         TCIOC_GETSTATUS
     * Description: Get the status of the timer.
     * Argument:    A writeable pointer to struct timer_status_s.
     */

    case TCIOC_GETSTATUS:
      {
        FAR struct timer_status_s *status;

        /* Get the current timer status */

        if (lower->ops->getstatus) /* Optional */
          {
            status = (FAR struct timer_status_s *)((uintptr_t)arg);
            if (status)
              {
                ret = lower->ops->getstatus(lower, status);
              }
            else
              {
                ret = -EINVAL;
              }
          }
        else
          {
            ret = -ENOSYS;
          }
      }
      break;

    /* cmd:         TCIOC_SETTIMEOUT
     * Description: Reset the timeout to this value
     * Argument:    A 32-bit timeout value in microseconds.
     *
     * TODO: pass pointer to uint64 ns? Need to determine if these timers
     * are 16 or 32 bit...
     */

    case TCIOC_SETTIMEOUT:
      {
        /* Set a new timeout value (and reset the timer) */

        if (lower->ops->settimeout) /* Optional */
          {
            ret = lower->ops->settimeout(lower, (uint32_t)arg);
          }
        else
          {
            ret = -ENOSYS;
          }
      }
      break;

    /* cmd:         TCIOC_SETHANDLER
     * Description: Call this handler on timeout
     * Argument:    A pointer to struct timer_sethandler_s.
     *
     * NOTE: This ioctl cannot be support in the kernel build mode. In that
     * case direct callbacks from kernel space into user space is forbidden.
     */

#ifndef CONFIG_NUTTX_KERNEL
    case TCIOC_SETHANDLER:
      {
        FAR struct timer_sethandler_s *sethandler;

        /* Don't reset on timer timeout; instead, call this user
         * provider timeout handler.  NOTE:  Providing handler==NULL will
         * restore the reset behavior.
         */

        if (lower->ops->sethandler) /* Optional */
          {
            sethandler = (FAR struct timer_sethandler_s *)((uintptr_t)arg);
            if (sethandler)
              {
                sethandler->oldhandler =
                  lower->ops->sethandler(lower, sethandler->newhandler);
                ret = OK;
              }
            else
              {
                ret = -EINVAL;
              }
          }
        else
          {
            ret = -ENOSYS;
          }
      }
      break;
#endif

    /* Any unrecognized IOCTL commands might be platform-specific ioctl commands */

    default:
      {
        tmrvdbg("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);

        /* An ioctl commands that are not recognized by the "upper-half"
         * driver are forwarded to the lower half driver through this
         * method.
         */

        if (lower->ops->ioctl) /* Optional */
          {
            ret = lower->ops->ioctl(lower, cmd, arg);
          }
        else
          {
            ret = -ENOSYS;
          }
      }
      break;
    }

  //sem_post(&upper->exclsem);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_register
 *
 * Description:
 *   This function binds an instance of a "lower half" timer driver with the
 *   "upper half" timer device and registers that device so that can be used
 *   by application code.
 *
 *   When this function is called, the "lower half" driver should be in the
 *   disabled state (as if the stop() method had already been called).
 *
 * Input parameters:
 *   dev path - The full path to the driver to be registers in the NuttX
 *     pseudo-filesystem.  The recommended convention is to name all timer
 *     drivers as "/dev/tc0", "/dev/tc1", etc.  where the driver
 *     path differs only in the "minor" number at the end of the device name.
 *   lower - A pointer to an instance of lower half timer driver.  This
 *     instance is bound to the timer driver and must persists as long as
 *     the driver persists.
 *
 * Returned Value:
 *   On success, a non-NULL handle is returned to the caller.  In the event
 *   of any failure, a NULL value is returned.
 *
 ****************************************************************************/

FAR void *timer_register(FAR const char *path,
                         FAR struct timer_lowerhalf_s *lower)
{
  FAR struct timer_upperhalf_s *upper;
  int ret;

  DEBUGASSERT(path && lower);
  tmrvdbg("Entry: path=%s\n", path);

  /* Allocate the upper-half data structure */

  upper = (FAR struct timer_upperhalf_s *)
    kzalloc(sizeof(struct timer_upperhalf_s));
  if (!upper)
    {
      tmrdbg("Upper half allocation failed\n");
      goto errout;
    }

  /* Initialize the timer device structure (it was already zeroed
   * by kzalloc()).
   */

  //sem_init(&upper->exclsem, 0, 1);
  upper->lower = lower;

  /* Copy the registration path */

  upper->path = strdup(path);
  if (!upper->path)
    {
      tmrdbg("Path allocation failed\n");
      goto errout_with_upper;
    }

  /* Register the timer device */

  ret = register_driver(path, &g_timerops, 0666, upper);
  if (ret < 0)
    {
      tmrdbg("register_driver failed: %d\n", ret);
      goto errout_with_path;
    }

  return (FAR void *)upper;

errout_with_path:
  kfree(upper->path);

errout_with_upper:
  //sem_destroy(&upper->exclsem);
  kfree(upper);

errout:
  return NULL;
}

/****************************************************************************
 * Name: timer_unregister
 *
 * Description:
 *   This function can be called to disable and unregister the timer
 *   device driver.
 *
 * Input parameters:
 *   handle - This is the handle that was returned by timer_register()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void timer_unregister(FAR void *handle)
{
  FAR struct timer_upperhalf_s *upper;
  FAR struct timer_lowerhalf_s *lower;

  /* Recover the pointer to the upper-half driver state */

  upper = (FAR struct timer_upperhalf_s *)handle;
  lower = upper->lower;
  DEBUGASSERT(upper && lower);

  tmrvdbg("Unregistering: %s\n", upper->path);

  /* Disable the timer */

  DEBUGASSERT(lower->ops->stop); /* Required */
  (void)lower->ops->stop(lower);

  /* Unregister the timer device */

  (void)unregister_driver(upper->path);

  /* Then free all of the driver resources */

  kfree(upper->path);
  //sem_destroy(&upper->exclsem);
  kfree(upper);
}

#endif /* CONFIG_TIMER */
