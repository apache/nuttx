/****************************************************************************
 * drivers/timers/timer.c
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <nuttx/timers/timer.h>

#ifdef CONFIG_TIMER

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct timer_upperhalf_s
{
  mutex_t lock;            /* Supports mutual exclusion */
  uint8_t crefs;           /* The number of times the device has been opened */
  FAR char *path;          /* Registration path */

  /* The contained signal info */

  struct timer_notify_s notify;
  struct sigwork_s work;

  /* The contained lower-half driver */

  FAR struct timer_lowerhalf_s *lower;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool    timer_notifier(FAR uint32_t *next_interval_us, FAR void *arg);
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
  NULL,        /* seek */
  timer_ioctl, /* ioctl */
  NULL         /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL       /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_notifier
 *
 * Description:
 *   Notify the application via a signal when the timer interrupt occurs
 *
 * REVISIT: This function prototype is insufficient to support signaling
 *
 ****************************************************************************/

static bool timer_notifier(FAR uint32_t *next_interval_us, FAR void *arg)
{
  FAR struct timer_upperhalf_s *upper = (FAR struct timer_upperhalf_s *)arg;
  FAR struct timer_notify_s *notify = &upper->notify;

  DEBUGASSERT(upper != NULL);

  /* Signal the waiter.. if there is one */

  nxsig_notification(notify->pid, &notify->event, SI_QUEUE, &upper->work);

  return notify->periodic;
}

/****************************************************************************
 * Name: timer_open
 *
 * Description:
 *   This function is called whenever the timer device is opened.
 *
 ****************************************************************************/

static int timer_open(FAR struct file *filep)
{
  FAR struct inode             *inode = filep->f_inode;
  FAR struct timer_upperhalf_s *upper = inode->i_private;
  uint8_t                       tmp;
  int                           ret;

  tmrinfo("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
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
      goto errout_with_lock;
    }

  /* Save the new open count */

  upper->crefs = tmp;
  ret = OK;

errout_with_lock:
  nxmutex_unlock(&upper->lock);

errout:
  return ret;
}

/****************************************************************************
 * Name: timer_close
 *
 * Description:
 *   This function is called when the timer device is closed.
 *
 ****************************************************************************/

static int timer_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct timer_upperhalf_s *upper = inode->i_private;
  int ret;

  tmrinfo("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (upper->crefs > 0)
    {
      upper->crefs--;
    }

  nxmutex_unlock(&upper->lock);
  return OK;
}

/****************************************************************************
 * Name: timer_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t timer_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/****************************************************************************
 * Name: timer_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t timer_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  return 0;
}

/****************************************************************************
 * Name: timer_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the timer work is
 *   done.
 *
 ****************************************************************************/

static int timer_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode             *inode = filep->f_inode;
  FAR struct timer_upperhalf_s *upper;
  FAR struct timer_lowerhalf_s *lower;
  int                           ret;

  tmrinfo("cmd: %d arg: %ld\n", cmd, arg);
  upper = inode->i_private;
  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&upper->lock);
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

        ret = TIMER_START(lower);
      }
      break;

    /* cmd:         TCIOC_STOP
     * Description: Stop the timer
     * Argument:    Ignored
     */

    case TCIOC_STOP:
      {
        /* Stop the timer */

        ret = TIMER_STOP(lower);
        nxsig_cancel_notification(&upper->work);
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

        status = (FAR struct timer_status_s *)((uintptr_t)arg);
        if (status)
          {
            ret = TIMER_GETSTATUS(lower, status);
          }
        else
          {
            ret = -EINVAL;
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

        ret = TIMER_SETTIMEOUT(lower, (uint32_t)arg);
      }
      break;

    /* cmd:         TCIOC_NOTIFICATION
     * Description: Notify application via a signal when the timer expires.
     * Argument:    signal information
     */

    case TCIOC_NOTIFICATION:
      {
        FAR struct timer_notify_s *notify =
          (FAR struct timer_notify_s *)((uintptr_t)arg);

        if (notify != NULL)
          {
            memcpy(&upper->notify, notify, sizeof(*notify));
            ret = timer_setcallback((FAR void *)upper, timer_notifier,
                                    (FAR void *)upper);
          }
        else
          {
            ret = -EINVAL;
          }
      }
      break;

    /* cmd:         TCIOC_MAXTIMEOUT
     * Description: Get the maximum supported timeout value
     * Argument:    A 32-bit timeout value in microseconds.
     */

    case TCIOC_MAXTIMEOUT:
      {
        /*  Get the maximum supported timeout value */

        ret = TIMER_MAXTIMEOUT(lower, (FAR uint32_t *)arg);
      }
      break;

    /* Any unrecognized IOCTL commands might be platform-specific ioctl
     * commands
     */

    default:
      {
        tmrinfo("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);

        /* An ioctl commands that are not recognized by the "upper-half"
         * driver are forwarded to the lower half driver through this
         * method.
         */

        ret = TIMER_IOCTL(lower, cmd, arg);
      }
      break;
    }

  nxmutex_unlock(&upper->lock);
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
 * Input Parameters:
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
  tmrinfo("Entry: path=%s\n", path);

  /* Allocate the upper-half data structure */

  upper = (FAR struct timer_upperhalf_s *)
    kmm_zalloc(sizeof(struct timer_upperhalf_s));
  if (!upper)
    {
      tmrerr("ERROR: Upper half allocation failed\n");
      goto errout;
    }

  /* Initialize the timer device structure (it was already zeroed
   * by kmm_zalloc()).
   */

  upper->lower = lower;
  nxmutex_init(&upper->lock);

  /* Copy the registration path */

  upper->path = strdup(path);
  if (!upper->path)
    {
      tmrerr("ERROR: Path allocation failed\n");
      goto errout_with_upper;
    }

  /* Register the timer device */

  ret = register_driver(path, &g_timerops, 0666, upper);
  if (ret < 0)
    {
      tmrerr("ERROR: register_driver failed: %d\n", ret);
      goto errout_with_path;
    }

  return (FAR void *)upper;

errout_with_path:
  kmm_free(upper->path);

errout_with_upper:
  nxmutex_destroy(&upper->lock);
  kmm_free(upper);

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
 * Input Parameters:
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
  DEBUGASSERT(upper != NULL && upper->lower != NULL);
  lower = upper->lower;

  tmrinfo("Unregistering: %s\n", upper->path);

  /* Disable the timer */

  TIMER_STOP(lower);
  nxsig_cancel_notification(&upper->work);

  /* Unregister the timer device */

  unregister_driver(upper->path);

  /* Then free all of the driver resources */

  nxmutex_destroy(&upper->lock);
  kmm_free(upper->path);
  kmm_free(upper);
}

/****************************************************************************
 * Name: timer_setcallback
 *
 * Description:
 *   This function can be called to add a callback into driver-related code
 *   to handle timer expirations.  This is a strictly OS internal interface
 *   and may NOT be used by application code.
 *
 * Input Parameters:
 *   handle   - This is the handle that was returned by timer_register()
 *   callback - The new timer interrupt callback
 *   arg      - Argument to be provided with the callback
 *
 * Returned Value:
 *   Zero (OK), if the callback was successfully set, or -ENOSYS if the lower
 *   half driver does not support the operation.
 *
 ****************************************************************************/

int timer_setcallback(FAR void *handle, tccb_t callback, FAR void *arg)
{
  FAR struct timer_upperhalf_s *upper;
  FAR struct timer_lowerhalf_s *lower;

  /* Recover the pointer to the upper-half driver state */

  upper = (FAR struct timer_upperhalf_s *)handle;
  DEBUGASSERT(upper != NULL && upper->lower != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower->ops != NULL);

  /* Check if the lower half driver supports the setcallback method */

  if (lower->ops->setcallback != NULL) /* Optional */
    {
      /* Yes.. Defer the handler attachment to the lower half driver */

      lower->ops->setcallback(lower, callback, arg);
      return OK;
    }

  return -ENOSYS;
}

#endif /* CONFIG_TIMER */
