/****************************************************************************
 * drivers/timers/watchdog.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/nuttx.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <nuttx/debug.h>

#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/lib/lib.h>
#include <nuttx/list.h>
#include <nuttx/panic_notifier.h>
#include <nuttx/power/pm.h>
#include <nuttx/mutex.h>
#include <nuttx/spinlock.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/timers/oneshot.h>
#include <nuttx/timers/timer.h>
#include <nuttx/timers/watchdog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_WATCHDOG_AUTOMONITOR
#  define WATCHDOG_AUTOMONITOR_TIMEOUT \
     (CONFIG_WATCHDOG_AUTOMONITOR_TIMEOUT)
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
       (WATCHDOG_AUTOMONITOR_PING_INTERVAL)
#    define WATCHDOG_AUTOMONITOR_PING_INTERVAL_TICK \
       MSEC2TICK(WATCHDOG_AUTOMONITOR_PING_INTERVAL)
#  endif
#endif

#if defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_ONESHOT)
#  define WATCHDOG_NOTIFIER_ACTION WATCHDOG_KEEPALIVE_BY_ONESHOT
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_TIMER)
#  define WATCHDOG_NOTIFIER_ACTION WATCHDOG_KEEPALIVE_BY_TIMER
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_WDOG)
#  define WATCHDOG_NOTIFIER_ACTION WATCHDOG_KEEPALIVE_BY_WDOG
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_WORKER)
#  define WATCHDOG_NOTIFIER_ACTION WATCHDOG_KEEPALIVE_BY_WORKER
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_CAPTURE)
#  define WATCHDOG_NOTIFIER_ACTION WATCHDOG_KEEPALIVE_BY_CAPTURE
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_IDLE)
#  define WATCHDOG_NOTIFIER_ACTION WATCHDOG_KEEPALIVE_BY_IDLE
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_WATCHDOG_AUTOMONITOR_TASKS
/* The supervision state of one open() of the device, a channel.  It is
 * allocated by open() and kept in the list of the upper half.  close()
 * releases it only if it is disarmed: an armed channel stays in the list and
 * is never fed again, so it times out, which is what happens when the task
 * that owns it exits or crashes.
 */

struct watchdog_user_s
{
  struct list_node node;                            /* Entry in the list */
  pid_t            pid;                             /* Task that opened it */
  char             name[CONFIG_TASK_NAME_SIZE + 1]; /* Its name */
  clock_t          timeout;                         /* Timeout in ticks */
  clock_t          deadline;                        /* Tick it times out */
  bool             armed;                           /* Is it supervised */
};
#endif

/* This structure describes the state of the upper half driver */

struct watchdog_upperhalf_s
{
#ifdef CONFIG_WATCHDOG_PANIC_NOTIFIER
  /* When a crash occurs, stop the watchdog */

  struct notifier_block nb;
#endif
#ifdef CONFIG_WATCHDOG_AUTOMONITOR
#  if defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_CAPTURE)
  FAR struct watchdog_upperhalf_s *capture_next;
#  endif
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
#  ifdef CONFIG_WATCHDOG_AUTOMONITOR_TASKS
  spinlock_t                      slock;  /* Protects the channels */
  struct list_node                users;  /* All the channels */
#  endif
#endif

  mutex_t   lock;     /* Supports mutual exclusion */
  FAR char *path;     /* Registration path */

  /* The contained lower-half driver */

  FAR struct watchdog_lowerhalf_s *lower;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static ssize_t wdog_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen);

#ifdef CONFIG_WATCHDOG_AUTOMONITOR_TASKS
static int     watchdog_channel_open(FAR struct file *filep);
static int     watchdog_channel_close(FAR struct file *filep);
static ssize_t watchdog_channel_write(FAR struct file *filep,
                                      FAR const char *buffer,
                                      size_t buflen);
static int     watchdog_channel_ioctl(FAR struct file *filep, int cmd,
                                      unsigned long arg);
#else
static ssize_t wdog_write(FAR struct file *filep, FAR const char *buffer,
                          size_t buflen);
static int     wdog_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_wdogops =
{
#ifdef CONFIG_WATCHDOG_AUTOMONITOR_TASKS
  watchdog_channel_open,  /* open */
  watchdog_channel_close, /* close */
  wdog_read,              /* read */
  watchdog_channel_write, /* write */
  NULL,                   /* seek */
  watchdog_channel_ioctl, /* ioctl */
#else
  NULL,       /* open */
  NULL,       /* close */
  wdog_read,  /* read */
  wdog_write, /* write */
  NULL,       /* seek */
  wdog_ioctl, /* ioctl */
#endif
};

#ifdef CONFIG_WATCHDOG_TIMEOUT_NOTIFIER
static ATOMIC_NOTIFIER_HEAD(g_watchdog_notifier_list);
#endif

#ifdef CONFIG_WATCHDOG_AUTOMONITOR_BY_CAPTURE
/* Active capture automonitor instances.  Lower halves that pass an argument
 * can identify their upper-half directly or through the lower-half pointer.
 * A NULL argument is accepted only when this list contains one instance.
 */

static FAR struct watchdog_upperhalf_s *g_watchdog_capture_list;
static spinlock_t g_watchdog_capture_lock = SP_UNLOCKED;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_WATCHDOG_AUTOMONITOR
#ifdef CONFIG_WATCHDOG_AUTOMONITOR_TASKS
/* Has an armed channel timed out?  Called by the keepalive triggers, some of
 * which run in interrupt context.
 */

static bool watchdog_channel_expired(FAR struct watchdog_upperhalf_s *upper)
{
  FAR struct watchdog_user_s *user;
  bool expired = false;
  irqstate_t flags;
  clock_t now;

  flags = spin_lock_irqsave(&upper->slock);
  now = clock_systime_ticks();

  list_for_every_entry(&upper->users, user, struct watchdog_user_s, node)
    {
      if (user->armed && clock_compare(user->deadline, now))
        {
          _alert("%s: timeout of task %d (%s), %ju ms\n", upper->path,
                 user->pid, user->name, (uintmax_t)TICK2MSEC(user->timeout));
          expired = true;
        }
    }

  spin_unlock_irqrestore(&upper->slock, flags);
  return expired;
}
#endif

/* Feed the lower half.  With the task supervision, only while no armed
 * channel timed out: the hardware watchdog then resets the system.
 */

static void watchdog_automonitor_keepalive(
                                    FAR struct watchdog_upperhalf_s *upper)
{
#ifdef CONFIG_WATCHDOG_AUTOMONITOR_TASKS
  if (watchdog_channel_expired(upper))
    {
#  ifdef CONFIG_WATCHDOG_AUTOMONITOR_TASKS_PANIC
      PANIC();
#  endif

      return;
    }
#endif

  upper->lower->ops->keepalive(upper->lower);
}

#endif

#ifdef CONFIG_WATCHDOG_AUTOMONITOR_BY_CAPTURE
static void watchdog_capture_add(FAR struct watchdog_upperhalf_s *upper)
{
  irqstate_t flags = spin_lock_irqsave(&g_watchdog_capture_lock);

  upper->capture_next = g_watchdog_capture_list;
  g_watchdog_capture_list = upper;
  spin_unlock_irqrestore(&g_watchdog_capture_lock, flags);
}

static void watchdog_capture_remove(FAR struct watchdog_upperhalf_s *upper)
{
  FAR struct watchdog_upperhalf_s **cursor;
  irqstate_t flags = spin_lock_irqsave(&g_watchdog_capture_lock);

  cursor = &g_watchdog_capture_list;
  while (*cursor != NULL)
    {
      if (*cursor == upper)
        {
          *cursor = upper->capture_next;
          upper->capture_next = NULL;
          break;
        }

      cursor = &(*cursor)->capture_next;
    }

  spin_unlock_irqrestore(&g_watchdog_capture_lock, flags);
}

static FAR struct watchdog_upperhalf_s *
watchdog_capture_find(FAR void *arg)
{
  FAR struct watchdog_upperhalf_s *upper;
  FAR struct watchdog_upperhalf_s *match = NULL;
  irqstate_t flags = spin_lock_irqsave(&g_watchdog_capture_lock);

  if (arg == NULL)
    {
      for (upper = g_watchdog_capture_list; upper != NULL;
           upper = upper->capture_next)
        {
          if (match != NULL)
            {
              /* More than one NULL-context instance cannot be identified. */

              match = NULL;
              break;
            }

          match = upper;
        }
    }
  else
    {
      for (upper = g_watchdog_capture_list; upper != NULL;
           upper = upper->capture_next)
        {
          if (arg == upper || arg == upper->lower)
            {
              match = upper;
              break;
            }
        }
    }

  spin_unlock_irqrestore(&g_watchdog_capture_lock, flags);
  return match;
}

static int watchdog_automonitor_capture(int irq, FAR void *context,
                                        FAR void *arg)
{
  FAR struct watchdog_upperhalf_s *upper = watchdog_capture_find(arg);

  /* A stop operation can race with a pending interrupt.  Do not dereference
   * a removed association after automonitor has been stopped.
   */

  if (upper == NULL)
    {
      return 0;
    }

  if (upper->monitor)
    {
      /* Reload the hardware watchdog before dispatching optional
       * notifications.  The EWI-to-reset interval is short, and notifier
       * callbacks must not delay the keepalive operation.
       */

      watchdog_automonitor_keepalive(upper);

#ifdef CONFIG_WATCHDOG_TIMEOUT_NOTIFIER
      /* The capture callback is entered from the watchdog timeout
       * interrupt.  Notifier callbacks must be safe in interrupt context.
       */

      watchdog_automonitor_timeout();
#endif
    }

  return 0;
}

#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_ONESHOT)
static void
watchdog_automonitor_oneshot(FAR struct oneshot_lowerhalf_s *oneshot,
                             FAR void *arg)
{
  FAR struct watchdog_upperhalf_s *upper = arg;

  if (upper->monitor)
    {
      struct timespec ts =
      {
        WATCHDOG_AUTOMONITOR_PING_INTERVAL, 0
      };

      watchdog_automonitor_keepalive(upper);
      ONESHOT_START(oneshot, &ts);
    }
}
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_TIMER)
static bool watchdog_automonitor_timer(FAR uint32_t *next_interval_us,
                                       FAR void *arg)
{
  FAR struct watchdog_upperhalf_s *upper = arg;

  if (upper->monitor)
    {
      watchdog_automonitor_keepalive(upper);
    }

  return upper->monitor;
}
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_WDOG)
static void watchdog_automonitor_wdog(wdparm_t arg)
{
  FAR struct watchdog_upperhalf_s *upper = (FAR void *)arg;

  if (upper->monitor)
    {
      watchdog_automonitor_keepalive(upper);
      wd_start(&upper->wdog, WATCHDOG_AUTOMONITOR_PING_INTERVAL_TICK,
               watchdog_automonitor_wdog, (wdparm_t)upper);
    }
}
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_WORKER)
static void watchdog_automonitor_worker(FAR void *arg)
{
  FAR struct watchdog_upperhalf_s *upper = arg;

  if (upper->monitor)
    {
      watchdog_automonitor_keepalive(upper);
      work_queue(LPWORK, &upper->work, watchdog_automonitor_worker,
                 upper, WATCHDOG_AUTOMONITOR_PING_INTERVAL_TICK);
    }
}
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_IDLE)
static void watchdog_automonitor_idle(FAR struct pm_callback_s *cb,
                                      int domain, enum pm_state_e pmstate)
{
  FAR struct watchdog_upperhalf_s *upper;

  upper = container_of(cb, struct watchdog_upperhalf_s, idle);

  if (domain == PM_IDLE_DOMAIN &&
      pmstate != PM_RESTORE && upper->monitor)
    {
      watchdog_automonitor_keepalive(upper);
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
      watchdog_capture_add(upper);
      lower->ops->capture(lower, watchdog_automonitor_capture);
#  elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_ONESHOT)
      struct timespec ts =
      {
        WATCHDOG_AUTOMONITOR_PING_INTERVAL, 0
      };

      upper->oneshot = oneshot;
      ONESHOT_START(oneshot, &ts);
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
          lower->ops->settimeout(lower, WATCHDOG_AUTOMONITOR_TIMEOUT);
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
      watchdog_capture_remove(upper);
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

#ifdef CONFIG_WATCHDOG_PANIC_NOTIFIER
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
#endif

/****************************************************************************
 * Name: wdog_read
 *
 * Description:
 *   Return the current status of the watchdog timer. Mimics what an `ioctl`
 *   with `WDIOC_GETSTATUS` would do. This is useful for userspace to get the
 *   current status of the watchdog timer by just reading from the device.
 *
 * Returns:
 *   The number of bytes read on success, or a negated errno value on failure
 *
 ****************************************************************************/

static ssize_t wdog_read(FAR struct file *filep, FAR char *buffer,
                         size_t buflen)
{
  /* simulate a "get status" on the userspace */

  ssize_t ret;
  FAR struct watchdog_status_s status;

  /* the f_pos is not used along this driver.
   * Use it as a marker to know if the status has been read already.
   */

  if (filep->f_pos)
    {
      filep->f_pos = 0;
      return 0;
    }

  ret = g_wdogops.ioctl(filep, WDIOC_GETSTATUS, (uintptr_t)&status);
  if (ret < 0)
    {
      return ret;
    }

  /* Copy the watchdog status into the user buffer */

  ret = snprintf(
          buffer,
          buflen,
          "Status       :\n"
          "  flags      : 0x%08" PRIx32 "\n"
          "  timeout    : %" PRId32 "\n"
          "  timeleft   : %" PRId32 "\n",
          status.flags, status.timeout, status.timeleft
        );
  if (ret < 0)
    {
      return ret;
    }

  if (ret >= buflen)
    {
      return -ENOSPC;
    }

  filep->f_pos++;
  return ret;
}

#ifdef CONFIG_WATCHDOG_AUTOMONITOR_TASKS
/* Has an armed channel timed out?  Such a channel stays so, even if the
 * monitor did not notice it yet, otherwise a task that is late by less than
 * the keepalive interval would escape by feeding, disarming or arming the
 * channel again.  The lock must be held.
 */

static bool watchdog_channel_late(FAR struct watchdog_user_s *user)
{
  return user->armed && clock_compare(user->deadline, clock_systime_ticks());
}

/* Arm a channel, its timeout starts now.  The lock must be held. */

static void watchdog_channel_arm(FAR struct watchdog_user_s *user)
{
  if (!watchdog_channel_late(user))
    {
      user->armed    = true;
      user->deadline = clock_systime_ticks() + user->timeout;
    }
}

/* Disarm a channel.  The lock must be held. */

static void watchdog_channel_disarm(FAR struct watchdog_user_s *user)
{
  if (!watchdog_channel_late(user))
    {
      user->armed = false;
    }
}

/* Restart the timeout of a channel, if it is armed.  The lock must be
 * held.
 */

static void watchdog_channel_feed(FAR struct watchdog_user_s *user)
{
  if (user->armed && !watchdog_channel_late(user))
    {
      user->deadline = clock_systime_ticks() + user->timeout;
    }
}

/* Each open() creates a channel */

static int watchdog_channel_open(FAR struct file *filep)
{
  FAR struct watchdog_upperhalf_s *upper = filep->f_inode->i_private;
  FAR struct tcb_s *tcb = nxsched_self();
  FAR struct watchdog_user_s *user;
  irqstate_t flags;

  user = kmm_zalloc(sizeof(struct watchdog_user_s));
  if (user == NULL)
    {
      return -ENOMEM;
    }

  user->pid     = tcb->pid;
  user->timeout = MSEC2TICK(CONFIG_WATCHDOG_AUTOMONITOR_TASKS_TIMEOUT);
  strlcpy(user->name, get_task_name(tcb), sizeof(user->name));

  flags = spin_lock_irqsave(&upper->slock);
  list_add_tail(&upper->users, &user->node);
  watchdog_channel_arm(user);
  spin_unlock_irqrestore(&upper->slock, flags);

  filep->f_priv = user;
  return OK;
}

/* An armed channel stays in the list and times out, see the structure */

static int watchdog_channel_close(FAR struct file *filep)
{
  FAR struct watchdog_upperhalf_s *upper = filep->f_inode->i_private;
  FAR struct watchdog_user_s *user = filep->f_priv;
  irqstate_t flags;
  bool release;

  flags = spin_lock_irqsave(&upper->slock);
  release = !user->armed;
  if (release)
    {
      list_delete(&user->node);
    }

  spin_unlock_irqrestore(&upper->slock, flags);

  if (release)
    {
      kmm_free(user);
    }

  return OK;
}

/* Feed the channel, or disarm it with a 'V' */

static ssize_t watchdog_channel_write(FAR struct file *filep,
                                      FAR const char *buffer,
                                      size_t buflen)
{
  FAR struct watchdog_upperhalf_s *upper = filep->f_inode->i_private;
  FAR struct watchdog_user_s *user = filep->f_priv;
  irqstate_t flags;

  flags = spin_lock_irqsave(&upper->slock);

#ifdef CONFIG_WATCHDOG_MAGIC_V
  if (memchr(buffer, 'V', buflen) != NULL)
    {
      watchdog_channel_disarm(user);
    }
  else
#endif
    {
      watchdog_channel_feed(user);
    }

  spin_unlock_irqrestore(&upper->slock, flags);
  return buflen;
}

/* The commands act on the channel, never on the lower half, so that no task
 * can stop the hardware watchdog, change its timeout or replace its reset
 * with a capture handler.
 */

static int watchdog_channel_ioctl(FAR struct file *filep, int cmd,
                                  unsigned long arg)
{
  FAR struct watchdog_upperhalf_s *upper = filep->f_inode->i_private;
  FAR struct watchdog_user_s *user = filep->f_priv;
  FAR struct watchdog_status_s *status;
  irqstate_t flags;
  clock_t now;
  int ret = OK;

  switch (cmd)
    {
      case WDIOC_START:
        flags = spin_lock_irqsave(&upper->slock);
        watchdog_channel_arm(user);
        spin_unlock_irqrestore(&upper->slock, flags);
        break;

      case WDIOC_STOP:
        flags = spin_lock_irqsave(&upper->slock);
        watchdog_channel_disarm(user);
        spin_unlock_irqrestore(&upper->slock, flags);
        break;

      case WDIOC_KEEPALIVE:
        flags = spin_lock_irqsave(&upper->slock);
        watchdog_channel_feed(user);
        spin_unlock_irqrestore(&upper->slock, flags);
        break;

      case WDIOC_SETTIMEOUT:
        if (arg == 0)
          {
            ret = -EINVAL;
            break;
          }

        flags = spin_lock_irqsave(&upper->slock);
        user->timeout = MSEC2TICK(arg);
        watchdog_channel_feed(user);
        spin_unlock_irqrestore(&upper->slock, flags);
        break;

      case WDIOC_GETSTATUS:
        status = (FAR struct watchdog_status_s *)((uintptr_t)arg);
        if (status == NULL)
          {
            ret = -EINVAL;
            break;
          }

        flags = spin_lock_irqsave(&upper->slock);
        now = clock_systime_ticks();
        status->flags    = user->armed ? WDFLAGS_ACTIVE | WDFLAGS_RESET : 0;
        status->timeout  = TICK2MSEC(user->timeout);
        status->timeleft = user->armed && !clock_compare(user->deadline, now)
                           ? TICK2MSEC(user->deadline - now) : 0;
        spin_unlock_irqrestore(&upper->slock, flags);
        break;

      case WDIOC_CAPTURE:
        ret = -ENOSYS;
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

#else
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
#ifdef CONFIG_WATCHDOG_MAGIC_V
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

#endif

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
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_WATCHDOG_TIMEOUT_NOTIFIER
/****************************************************************************
 * Name:  watchdog_notifier_chain_register
 *
 * Description:
 *   Add notifier to the watchdog notifier chain
 *
 * Input Parameters:
 *    nb - New entry in notifier chain
 *
 ****************************************************************************/

void watchdog_notifier_chain_register(FAR struct notifier_block *nb)
{
  atomic_notifier_chain_register(&g_watchdog_notifier_list, nb);
}

/****************************************************************************
 * Name:  watchdog_notifier_chain_unregister
 *
 * Description:
 *   Remove notifier from the watchdog notifier chain
 *
 * Input Parameters:
 *    nb - Entry to remove from notifier chain
 *
 ****************************************************************************/

void watchdog_notifier_chain_unregister(FAR struct notifier_block *nb)
{
  atomic_notifier_chain_unregister(&g_watchdog_notifier_list, nb);
}

/****************************************************************************
 * Name: watchdog_automonitor_timeout
 *
 * Description:
 *   This function can be called in the watchdog timeout interrupt handler.
 *   If so, callbacks on the watchdog timer notify chain are called when the
 *   watchdog timer times out.
 *
 ****************************************************************************/

void watchdog_automonitor_timeout(void)
{
  /* The action identifies the automonitor keepalive mechanism selected at
   * build time.  There is no source-specific payload for this event.
   */

  atomic_notifier_call_chain(&g_watchdog_notifier_list,
                             WATCHDOG_NOTIFIER_ACTION, NULL);
}
#endif /* CONFIG_WATCHDOG_TIMEOUT_NOTIFIER */

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
#ifdef CONFIG_WATCHDOG_AUTOMONITOR_TASKS
  spin_lock_init(&upper->slock);
  list_initialize(&upper->users);
#endif

  /* Copy the registration path */

  upper->path = strdup(path);
  if (!upper->path)
    {
      wderr("Path allocation failed\n");
      goto errout_with_upper;
    }

  /* Register the watchdog timer device */

  ret = register_driver(path, &g_wdogops, 0600, upper);
  if (ret < 0)
    {
      wderr("register_driver failed: %d\n", ret);
      goto errout_with_path;
    }

#if defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_ONESHOT)
  upper->oneshot->callback = watchdog_automonitor_oneshot;
  upper->oneshot->arg      = upper;
  watchdog_automonitor_start(upper, oneshot);
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_TIMER)
  watchdog_automonitor_start(upper, timer);
#elif defined(CONFIG_WATCHDOG_AUTOMONITOR)
  watchdog_automonitor_start(upper);
#endif

#ifdef CONFIG_WATCHDOG_PANIC_NOTIFIER
  upper->nb.notifier_call = wdog_notifier;
  panic_notifier_chain_register(&upper->nb);
#endif

  return (FAR void *)upper;

errout_with_path:
  lib_free(upper->path);

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
#ifdef CONFIG_WATCHDOG_AUTOMONITOR_TASKS
  FAR struct watchdog_user_s *user;
#endif

  /* Recover the pointer to the upper-half driver state */

  upper = (FAR struct watchdog_upperhalf_s *)handle;
  DEBUGASSERT(upper != NULL);
  lower = upper->lower;
  DEBUGASSERT(lower != NULL);

  wdinfo("Unregistering: %s\n", upper->path);

#ifdef CONFIG_WATCHDOG_AUTOMONITOR
  watchdog_automonitor_stop(upper);
#  ifdef CONFIG_WATCHDOG_AUTOMONITOR_BY_WORKER
  work_cancel_sync(LPWORK, &upper->work);
#  endif
#endif

#ifdef CONFIG_WATCHDOG_AUTOMONITOR_TASKS
  while ((user = list_remove_head_type(&upper->users, struct watchdog_user_s,
                                       node)) != NULL)
    {
      kmm_free(user);
    }
#endif

  /* Disable the watchdog timer */

  DEBUGASSERT(lower->ops->stop); /* Required */
  lower->ops->stop(lower);

  /* Unregister the watchdog timer device */

  unregister_driver(upper->path);
#ifdef CONFIG_WATCHDOG_PANIC_NOTIFIER
  panic_notifier_chain_unregister(&upper->nb);
#endif

  /* Then free all of the driver resources */

  lib_free(upper->path);
  nxmutex_destroy(&upper->lock);
  kmm_free(upper);
}
