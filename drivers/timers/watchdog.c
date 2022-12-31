/****************************************************************************
 * drivers/timers/watchdog.c
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

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/panic_notifier.h>
#include <nuttx/power/pm.h>
#include <nuttx/mutex.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/timers/oneshot.h>
#include <nuttx/timers/timer.h>
#include <nuttx/timers/watchdog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_WATCHDOG_AUTOMONITOR
#  define WATCHDOG_AUTOMONITOR_TIMEOUT_MSEC \
     (CONFIG_WATCHDOG_AUTOMONITOR_TIMEOUT * MSEC_PER_SEC)
#  if !defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_CAPTURE) && \
      !defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_IDLE)
#    if CONFIG_WATCHDOG_AUTOMONITOR_PING_INTERVAL == 0
#      define WATCHDOG_AUTOMONITOR_PING_INTERVAL \
         (CONFIG_WATCHDOG_AUTOMONITOR_TIMEOUT / 2)
#    else
#      define WATCHDOG_AUTOMONITOR_PING_INTERVAL \
         CONFIG_WATCHDOG_AUTOMONITOR_PING_INTERVAL
#    endif
#    define WATCHDOG_AUTOMONITOR_PING_INTERVAL_MSEC \
       (WATCHDOG_AUTOMONITOR_PING_INTERVAL * MSEC_PER_SEC)
#    define WATCHDOG_AUTOMONITOR_PING_INTERVAL_TICK \
       SEC2TICK(WATCHDOG_AUTOMONITOR_PING_INTERVAL)
#  endif
#endif

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct watchdog_upperhalf_s
{
  /* When a crash occurs, stop the watchdog */

  struct notifier_block nb;
#ifdef CONFIG_WATCHDOG_AUTOMONITOR
#  if defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_ONESHOT)
  FAR struct oneshot_lowerhalf_s *oneshot;
#  elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_TIMER)
  FAR struct timer_lowerhalf_s   *timer;
#  elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_WDOG)
  struct wdog_s                   wdog;
#  elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_WORKER)
  struct work_s                   work;
#  elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_IDLE)
  struct pm_callback_s            idle;
#  endif
  bool                            monitor;
#endif

  uint8_t   crefs;    /* The number of times the device has been opened */
  mutex_t   lock;     /* Supports mutual exclusion */
  FAR char *path;     /* Registration path */

  /* The contained lower-half driver */

  FAR struct watchdog_lowerhalf_s *lower;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     wdog_open(FAR struct file *filep);
static int     wdog_close(FAR struct file *filep);
static ssize_t wdog_read(FAR struct file *filep, FAR char *buffer,
                 size_t buflen);
static ssize_t wdog_write(FAR struct file *filep, FAR const char *buffer,
                 size_t buflen);
static int     wdog_ioctl(FAR struct file *filep, int cmd,
                 unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_wdogops =
{
  wdog_open,  /* open */
  wdog_close, /* close */
  wdog_read,  /* read */
  wdog_write, /* write */
  NULL,       /* seek */
  wdog_ioctl, /* ioctl */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_CAPTURE)
static int watchdog_automonitor_capture(int irq, FAR void *context,
                                        FAR void *arg)
{
  FAR struct watchdog_upperhalf_s *upper = arg;
  FAR struct watchdog_lowerhalf_s *lower = upper->lower;

  if (upper->monitor)
    {
      lower->ops->keepalive(lower);
    }

  return 0;
}
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_ONESHOT)
static void
watchdog_automonitor_oneshot(FAR struct oneshot_lowerhalf_s *oneshot,
                             FAR void *arg)
{
  FAR struct watchdog_upperhalf_s *upper = arg;
  FAR struct watchdog_lowerhalf_s *lower = upper->lower;

  if (upper->monitor)
    {
      struct timespec ts =
      {
        WATCHDOG_AUTOMONITOR_PING_INTERVAL, 0
      };

      lower->ops->keepalive(lower);
      ONESHOT_START(oneshot, watchdog_automonitor_oneshot, upper, &ts);
    }
}
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_TIMER)
static bool watchdog_automonitor_timer(FAR uint32_t *next_interval_us,
                                       FAR void *arg)
{
  FAR struct watchdog_upperhalf_s *upper = arg;
  FAR struct watchdog_lowerhalf_s *lower = upper->lower;

  if (upper->monitor)
    {
      lower->ops->keepalive(lower);
    }

  return upper->monitor;
}
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_WDOG)
static void watchdog_automonitor_wdog(wdparm_t arg)
{
  FAR struct watchdog_upperhalf_s *upper = (FAR void *)arg;
  FAR struct watchdog_lowerhalf_s *lower = upper->lower;

  if (upper->monitor)
    {
      lower->ops->keepalive(lower);
      wd_start(&upper->wdog, WATCHDOG_AUTOMONITOR_PING_INTERVAL_TICK,
               watchdog_automonitor_wdog, (wdparm_t)upper);
    }
}
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_WORKER)
static void watchdog_automonitor_worker(FAR void *arg)
{
  FAR struct watchdog_upperhalf_s *upper = arg;
  FAR struct watchdog_lowerhalf_s *lower = upper->lower;

  if (upper->monitor)
    {
      lower->ops->keepalive(lower);
      work_queue(LPWORK, &upper->work, watchdog_automonitor_worker,
                 upper, WATCHDOG_AUTOMONITOR_PING_INTERVAL_TICK);
    }
}
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_IDLE)
static void watchdog_automonitor_idle(FAR struct pm_callback_s *cb,
                                      int domain, enum pm_state_e pmstate)
{
  FAR struct watchdog_upperhalf_s *upper = (FAR void *)cb;
  FAR struct watchdog_lowerhalf_s *lower = upper->lower;

  if (domain == PM_IDLE_DOMAIN &&
      pmstate != PM_RESTORE && upper->monitor)
    {
      lower->ops->keepalive(lower);
    }
}
#endif

#ifdef CONFIG_WATCHDOG_AUTOMONITOR
#  if defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_ONESHOT)
static void
watchdog_automonitor_start(FAR struct watchdog_upperhalf_s *upper,
                           FAR struct oneshot_lowerhalf_s *oneshot)
#  elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_TIMER)
static void
watchdog_automonitor_start(FAR struct watchdog_upperhalf_s *upper,
                           FAR struct timer_lowerhalf_s *timer)
#  else
static void
watchdog_automonitor_start(FAR struct watchdog_upperhalf_s *upper)
#  endif
{
  FAR struct watchdog_lowerhalf_s *lower = upper->lower;

  if (!upper->monitor)
    {
#  if defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_CAPTURE)
      lower->ops->capture(lower, watchdog_automonitor_capture);
#  elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_ONESHOT)
      struct timespec ts =
      {
        WATCHDOG_AUTOMONITOR_PING_INTERVAL, 0
      };

      upper->oneshot = oneshot;
      ONESHOT_START(oneshot, watchdog_automonitor_oneshot, upper, &ts);
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_TIMER)
      upper->timer = timer;
      timer->ops->setcallback(timer, watchdog_automonitor_timer, upper);
      timer->ops->settimeout(timer, WATCHDOG_AUTOMONITOR_PING_INTERVAL_MSEC);
      timer->ops->start(timer);
#  elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_WDOG)
      wd_start(&upper->wdog, WATCHDOG_AUTOMONITOR_PING_INTERVAL_TICK,
               watchdog_automonitor_wdog, (wdparm_t)upper);
#  elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_WORKER)
      work_queue(LPWORK, &upper->work, watchdog_automonitor_worker,
                 upper, WATCHDOG_AUTOMONITOR_PING_INTERVAL_TICK);
#  elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_IDLE)
      upper->idle.notify = watchdog_automonitor_idle;
      pm_register(&upper->idle);
#  endif
      upper->monitor = true;
      if (lower->ops->settimeout)
        {
          lower->ops->settimeout(lower, WATCHDOG_AUTOMONITOR_TIMEOUT_MSEC);
        }

      lower->ops->start(lower);
    }
}

static void watchdog_automonitor_stop(FAR struct watchdog_upperhalf_s *upper)
{
  FAR struct watchdog_lowerhalf_s *lower = upper->lower;

  if (upper->monitor)
    {
      upper->monitor = false;
      lower->ops->stop(lower);
#  if defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_CAPTURE)
      lower->ops->capture(lower, NULL);
#  elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_ONESHOT)
      ONESHOT_CANCEL(upper->oneshot, NULL);
#  elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_TIMER)
      upper->timer->ops->stop(upper->timer);
#  elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_WDOG)
      wd_cancel(&upper->wdog);
#  elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_WORKER)
      work_cancel(LPWORK, &upper->work);
#  elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_IDLE)
      pm_unregister(&upper->idle);
#  endif
    }
}
#endif

static int wdog_notifier(FAR struct notifier_block *nb, unsigned long action,
                         FAR void *data)
{
  FAR struct watchdog_upperhalf_s *upper =
                                       (FAR struct watchdog_upperhalf_s *)nb;

  if (action == PANIC_KERNEL)
    {
#ifdef CONFIG_WATCHDOG_AUTOMONITOR
      watchdog_automonitor_stop(upper);
#else
      return upper->lower->ops->stop(upper->lower);
#endif
    }

  return 0;
}

/****************************************************************************
 * Name: wdog_open
 *
 * Description:
 *   This function is called whenever the watchdog timer device is opened.
 *
 ****************************************************************************/

static int wdog_open(FAR struct file *filep)
{
  FAR struct inode                *inode = filep->f_inode;
  FAR struct watchdog_upperhalf_s *upper = inode->i_private;
  uint8_t                          tmp;
  int                              ret;

  wdinfo("crefs: %d\n", upper->crefs);

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
 * Name: wdog_close
 *
 * Description:
 *   This function is called when the watchdog timer device is closed.
 *
 ****************************************************************************/

static int wdog_close(FAR struct file *filep)
{
  FAR struct inode                *inode = filep->f_inode;
  FAR struct watchdog_upperhalf_s *upper = inode->i_private;
  int                              ret;

  wdinfo("crefs: %d\n", upper->crefs);

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&upper->lock);
  if (ret < 0)
    {
      goto errout;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (upper->crefs > 0)
    {
      upper->crefs--;
    }

  nxmutex_unlock(&upper->lock);
  ret = OK;

errout:
  return ret;
}

/****************************************************************************
 * Name: wdog_read
 *
 * Description:
 *   A dummy read method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t wdog_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  /* Return zero -- usually meaning end-of-file */

  return 0;
}

/****************************************************************************
 * Name: wdog_write
 *
 * Description:
 *   A dummy write method.  This is provided only to satisfy the VFS layer.
 *
 ****************************************************************************/

static ssize_t wdog_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen)
{
  FAR struct inode                *inode = filep->f_inode;
  FAR struct watchdog_upperhalf_s *upper;
  FAR struct watchdog_lowerhalf_s *lower;
  int err = 0;
  int i;

  upper = inode->i_private;
  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  nxmutex_lock(&upper->lock);

  for (i = 0; i < buflen; i++)
    {
      if (buffer[i] == 'V')
        {
#ifdef CONFIG_WATCHDOG_AUTOMONITOR
          watchdog_automonitor_stop(upper);
#else
          err = lower->ops->stop(lower);
#endif
          break;
        }
    }

  if (i == buflen)
    {
      err = lower->ops->keepalive(lower);
    }

  nxmutex_unlock(&upper->lock);

  if (err < 0)
    {
      return err;
    }

  return buflen;
}

/****************************************************************************
 * Name: wdog_ioctl
 *
 * Description:
 *   The standard ioctl method.  This is where ALL of the watchdog timer
 *   work is done.
 *
 ****************************************************************************/

static int wdog_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode                *inode = filep->f_inode;
  FAR struct watchdog_upperhalf_s *upper;
  FAR struct watchdog_lowerhalf_s *lower;
  int                              ret;

  wdinfo("cmd: %d arg: %ld\n", cmd, arg);
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
    /* cmd:         WDIOC_START
     * Description: Start the watchdog timer
     * Argument:    Ignored
     */

    case WDIOC_START:
      {
#ifdef CONFIG_WATCHDOG_AUTOMONITOR
        watchdog_automonitor_stop(upper);
#endif

        /* Start the watchdog timer, resetting the time to the current
         * timeout
         */

        DEBUGASSERT(lower->ops->start); /* Required */
        ret = lower->ops->start(lower);
      }
      break;

    /* cmd:         WDIOC_STOP
     * Description: Stop the watchdog timer
     * Argument:    Ignored
     */

    case WDIOC_STOP:
      {
        /* Stop the watchdog timer */

        DEBUGASSERT(lower->ops->stop); /* Required */
        ret = lower->ops->stop(lower);
      }
      break;

    /* cmd:         WDIOC_GETSTATUS
     * Description: Get the status of the watchdog timer.
     * Argument:    A writeable pointer to struct watchdog_status_s.
     */

    case WDIOC_GETSTATUS:
      {
        FAR struct watchdog_status_s *status;

        /* Get the current watchdog timer status */

        if (lower->ops->getstatus) /* Optional */
          {
            status = (FAR struct watchdog_status_s *)((uintptr_t)arg);
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

    /* cmd:         WDIOC_SETTIMEOUT
     * Description: Reset the watchdog timeout to this value
     * Argument:    A 32-bit timeout value in milliseconds.
     */

    case WDIOC_SETTIMEOUT:
      {
        /* Set a new timeout value (and reset the watchdog timer) */

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

    /* cmd:         WDIOC_CAPTURE
     * Description: Do not reset.  Instead, called this handler.
     * Argument:    A pointer to struct watchdog_capture_s.
     */

    case WDIOC_CAPTURE:
      {
        FAR struct watchdog_capture_s *capture;

        /* Don't reset on watchdog timer timeout; instead, call this user
         * provider timeout handler.  NOTE:  Providing handler==NULL will
         * restore the reset behavior.
         */

        if (lower->ops->capture) /* Optional */
          {
            capture = (FAR struct watchdog_capture_s *)((uintptr_t)arg);
            if (capture)
              {
                capture->oldhandler =
                  lower->ops->capture(lower, capture->newhandler);
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

    /* cmd:         WDIOC_KEEPALIVE
     * Description: Reset the watchdog timer ("ping", "pet the dog")
     * Argument:    Argument: Ignored
     */

    case WDIOC_KEEPALIVE:
      {
        /* Reset the watchdog timer to the current timeout value, prevent
         * any imminent watchdog timeouts.  This is sometimes referred as
         * "pinging" the watchdog timer or "petting the dog".
         */

        if (lower->ops->keepalive) /* Optional */
          {
            ret = lower->ops->keepalive(lower);
          }
        else
          {
            ret = -ENOSYS;
          }
      }
      break;

    /* Any unrecognized IOCTL commands might be platform-specific ioctl
     * commands
     */

    default:
      {
        wdinfo("Forwarding unrecognized cmd: %d arg: %ld\n", cmd, arg);

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
            ret = -ENOTTY;
          }
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
 * Name: watchdog_register
 *
 * Description:
 *   This function binds an instance of a "lower half" watchdog driver with
 *   the "upper half" watchdog device and registers that device so that can
 *   be usedby application code.
 *
 *   When this function is called, the "lower half" driver should be in the
 *   disabled state (as if the stop() method had already been called).
 *
 * Input Parameters:
 *   dev path - The full path to the driver to be registers in the NuttX
 *     pseudo-filesystem.  The recommended convention is to name all
 *     watchdogdrivers as "/dev/watchdog0", "/dev/watchdog1", etc.  where
 *     the driverpath differs only in the "minor" number at the end of the
 *     device name.
 *   lower - A pointer to an instance of lower half watchdog driver.  This
 *     instance is bound to the watchdog driver and must persists as long as
 *     the driver persists.
 *
 * Returned Value:
 *   On success, a non-NULL handle is returned to the caller.  In the event
 *   of any failure, a NULL value is returned.
 *
 ****************************************************************************/

#if defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_ONESHOT)
FAR void *watchdog_register(FAR const char *path,
                            FAR struct watchdog_lowerhalf_s *lower,
                            FAR struct oneshot_lowerhalf_s *oneshot)
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_TIMER)
FAR void *watchdog_register(FAR const char *path,
                            FAR struct watchdog_lowerhalf_s *lower,
                            FAR struct timer_lowerhalf_s *timer)
#else
FAR void *watchdog_register(FAR const char *path,
                            FAR struct watchdog_lowerhalf_s *lower)
#endif
{
  FAR struct watchdog_upperhalf_s *upper;
  int ret;

  DEBUGASSERT(path && lower);
  wdinfo("Entry: path=%s\n", path);

  /* Allocate the upper-half data structure */

  upper = (FAR struct watchdog_upperhalf_s *)
    kmm_zalloc(sizeof(struct watchdog_upperhalf_s));
  if (!upper)
    {
      wderr("Upper half allocation failed\n");
      goto errout;
    }

  /* Initialize the watchdog timer device structure (it was already zeroed
   * by kmm_zalloc()).
   */

  nxmutex_init(&upper->lock);
  upper->lower = lower;

  /* Copy the registration path */

  upper->path = strdup(path);
  if (!upper->path)
    {
      wderr("Path allocation failed\n");
      goto errout_with_upper;
    }

  /* Register the watchdog timer device */

  ret = register_driver(path, &g_wdogops, 0666, upper);
  if (ret < 0)
    {
      wderr("register_driver failed: %d\n", ret);
      goto errout_with_path;
    }

#if defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_ONESHOT)
  watchdog_automonitor_start(upper, oneshot);
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_TIMER)
  watchdog_automonitor_start(upper, timer);
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR)
  watchdog_automonitor_start(upper);
#endif

  upper->nb.notifier_call = wdog_notifier;
  panic_notifier_chain_register(&upper->nb);

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
 * Name: watchdog_unregister
 *
 * Description:
 *   This function can be called to disable and unregister the watchdog
 *   device driver.
 *
 * Input Parameters:
 *   handle - This is the handle that was returned by watchdog_register()
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void watchdog_unregister(FAR void *handle)
{
  FAR struct watchdog_upperhalf_s *upper;
  FAR struct watchdog_lowerhalf_s *lower;

  /* Recover the pointer to the upper-half driver state */

  upper = (FAR struct watchdog_upperhalf_s *)handle;
  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  wdinfo("Unregistering: %s\n", upper->path);

#ifdef CONFIG_WATCHDOG_AUTOMONITOR
  watchdog_automonitor_stop(upper);
#endif

  /* Disable the watchdog timer */

  DEBUGASSERT(lower->ops->stop); /* Required */
  lower->ops->stop(lower);

  /* Unregister the watchdog timer device */

  unregister_driver(upper->path);
  panic_notifier_chain_unregister(&upper->nb);

  /* Then free all of the driver resources */

  kmm_free(upper->path);
  nxmutex_destroy(&upper->lock);
  kmm_free(upper);
}
