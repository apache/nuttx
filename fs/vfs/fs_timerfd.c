/****************************************************************************
 * fs/vfs/fs_timerfd.c
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
#include <stdio.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <fcntl.h>

#include <debug.h>

#include <nuttx/wdog.h>
#include <nuttx/mutex.h>

#include <sys/ioctl.h>
#include <sys/timerfd.h>

#include "clock/clock.h"
#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct timerfd_waiter_sem_s
{
  sem_t sem;
  FAR struct timerfd_waiter_sem_s *next;
} timerfd_waiter_sem_t;

/* This structure describes the internal state of the driver */

struct timerfd_priv_s
{
  mutex_t                   lock;    /* Enforces device exclusive access */
  FAR timerfd_waiter_sem_t *rdsems;  /* List of blocking readers */
  int                       clock;   /* Clock to use as the timing base */
  int                       delay;   /* If non-zero, used to reset repetitive
                                      * timers */
  struct wdog_s             wdog;    /* The watchdog that provides the timing */
  timerfd_t                 counter; /* timerfd counter */
  uint8_t                   crefs;   /* References counts on timerfd (max: 255) */

  /* The following is a list if poll structures of threads waiting for
   * driver events.
   */

#ifdef CONFIG_TIMER_FD_POLL
  FAR struct pollfd *fds[CONFIG_TIMER_FD_NPOLLWAITERS];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int timerfd_open(FAR struct file *filep);
static int timerfd_close(FAR struct file *filep);

static ssize_t timerfd_read(FAR struct file *filep, FAR char *buffer,
                            size_t len);
#ifdef CONFIG_TIMER_FD_POLL
static int timerfd_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);
#endif

static int timerfd_blocking_io(FAR struct timerfd_priv_s *dev,
                               FAR timerfd_waiter_sem_t  *sem,
                               FAR timerfd_waiter_sem_t **slist);

static FAR struct timerfd_priv_s *timerfd_allocdev(void);
static void timerfd_destroy(FAR struct timerfd_priv_s *dev);

static void timerfd_timeout(wdparm_t arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_timerfd_fops =
{
  timerfd_open,  /* open */
  timerfd_close, /* close */
  timerfd_read,  /* read */
  NULL,          /* write */
  NULL,          /* seek */
  NULL,          /* ioctl */
#ifdef CONFIG_TIMER_FD_POLL
  timerfd_poll   /* poll */
#else
  NULL           /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL         /* unlink */
#endif
};

static struct inode g_timerfd_inode =
{
  NULL,                   /* i_parent */
  NULL,                   /* i_peer */
  NULL,                   /* i_child */
  1,                      /* i_crefs */
  FSNODEFLAG_TYPE_DRIVER, /* i_flags */
  {
    &g_timerfd_fops       /* u */
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR struct timerfd_priv_s *timerfd_allocdev(void)
{
  FAR struct timerfd_priv_s *dev;

  dev = (FAR struct timerfd_priv_s *)
    kmm_zalloc(sizeof(struct timerfd_priv_s));
  if (dev)
    {
      /* Initialize the private structure */

      nxmutex_init(&dev->lock);
      nxmutex_lock(&dev->lock);
    }

  return dev;
}

static void timerfd_destroy(FAR struct timerfd_priv_s *dev)
{
  wd_cancel(&dev->wdog);
  nxmutex_unlock(&dev->lock);
  nxmutex_destroy(&dev->lock);
  kmm_free(dev);
}

static int timerfd_open(FAR struct file *filep)
{
  FAR struct timerfd_priv_s *priv = filep->f_priv;
  int ret;

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->crefs >= 255)
    {
      /* More than 255 opens; uint8_t would overflow to zero */

      ret = -EMFILE;
    }
  else
    {
      /* Save the new open count on success */

      priv->crefs += 1;
      ret = OK;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

static int timerfd_close(FAR struct file *filep)
{
  FAR struct timerfd_priv_s *priv = filep->f_priv;
  int ret;

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Decrement the references to the driver.  If the reference count will
   * decrement to 0, then uninitialize the driver.
   */

  if (priv->crefs > 1)
    {
      /* Just decrement the reference count and release the semaphore */

      priv->crefs--;
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  /* Re-create the path to the driver. */

  finfo("destroy\n");

  timerfd_destroy(priv);
  return OK;
}

static int timerfd_blocking_io(FAR struct timerfd_priv_s *dev,
                               FAR  timerfd_waiter_sem_t *sem,
                               FAR timerfd_waiter_sem_t **slist)
{
  int ret;

  sem->next = *slist;
  *slist = sem;

  /* Wait for timerfd to notify */

  ret = nxsem_wait(&sem->sem);
  if (ret < 0)
    {
      FAR timerfd_waiter_sem_t *cur_sem;

      cur_sem = *slist;
      if (cur_sem == sem)
        {
          *slist = sem->next;
        }
      else
        {
          while (cur_sem)
            {
              if (cur_sem->next == sem)
                {
                  cur_sem->next = sem->next;
                  break;
                }
            }
        }
    }

  return ret;
}

static ssize_t timerfd_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  FAR struct timerfd_priv_s *dev = filep->f_priv;
  irqstate_t intflags;
  ssize_t ret;

  if (len < sizeof(timerfd_t) || buffer == NULL)
    {
      return -EINVAL;
    }

  /* Ensure that interrupts are disabled and we do not lose counts
   * if expiration occurs after read, but before setting counter
   * to zero
   */

  intflags = enter_critical_section();

  /* Wait for an incoming event */

  if (dev->counter == 0)
    {
      timerfd_waiter_sem_t sem;

      if (filep->f_oflags & O_NONBLOCK)
        {
          leave_critical_section(intflags);
          return -EAGAIN;
        }

      nxsem_init(&sem.sem, 0, 0);
      do
        {
          ret = timerfd_blocking_io(dev, &sem, &dev->rdsems);
          if (ret < 0)
            {
              leave_critical_section(intflags);
              nxsem_destroy(&sem.sem);
              return ret;
            }
        }
      while (dev->counter == 0);

      nxsem_destroy(&sem.sem);
    }

  *(FAR timerfd_t *)buffer = dev->counter;
  dev->counter = 0;

  leave_critical_section(intflags);

  return sizeof(timerfd_t);
}

#ifdef CONFIG_TIMER_FD_POLL
static int timerfd_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct timerfd_priv_s *dev = filep->f_priv;
  irqstate_t intflags;
  int ret = OK;
  int i;

  intflags = enter_critical_section();
  if (!setup)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

      /* Remove all memory of the poll setup */

      *slot     = NULL;
      fds->priv = NULL;
      goto out;
    }

  /* This is a request to set up the poll. Find an available
   * slot for the poll structure reference
   */

  for (i = 0; i < CONFIG_TIMER_FD_NPOLLWAITERS; i++)
    {
      /* Find an available slot */

      if (!dev->fds[i])
        {
          /* Bind the poll structure and this slot */

          dev->fds[i] = fds;
          fds->priv   = &dev->fds[i];
          break;
        }
    }

  if (i >= CONFIG_TIMER_FD_NPOLLWAITERS)
    {
      fds->priv = NULL;
      ret       = -EBUSY;
      goto out;
    }

  /* Notify the POLLIN event if the counter is not zero */

  if (dev->counter > 0)
    {
#ifdef CONFIG_TIMER_FD_POLL
      poll_notify(dev->fds, CONFIG_TIMER_FD_NPOLLWAITERS, POLLIN);
#endif
    }

out:
  leave_critical_section(intflags);
  return ret;
}
#endif

static void timerfd_timeout(wdparm_t arg)
{
  FAR struct timerfd_priv_s *dev = (FAR struct timerfd_priv_s *)arg;
  FAR timerfd_waiter_sem_t *cur_sem;
  irqstate_t intflags;

  /* Disable interrupts to ensure that expiration counter is accessed
   * atomically
   */

  intflags = enter_critical_section();

  /* Increment timer expiration counter */

  dev->counter++;

  /* If this is a repetitive timer, then restart the watchdog */

  if (dev->delay)
    {
      wd_start(&dev->wdog, dev->delay, timerfd_timeout, arg);
    }

#ifdef CONFIG_TIMER_FD_POLL
  /* Notify all poll/select waiters */

  poll_notify(dev->fds, CONFIG_TIMER_FD_NPOLLWAITERS, POLLIN);
#endif

  /* Notify all of the waiting readers */

  cur_sem = dev->rdsems;
  while (cur_sem != NULL)
    {
      nxsem_post(&cur_sem->sem);
      cur_sem = cur_sem->next;
    }

  dev->rdsems = NULL;

  leave_critical_section(intflags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int timerfd_create(int clockid, int flags)
{
  FAR struct timerfd_priv_s *new_dev;
  int new_fd;
  int ret;

  /* Sanity checks. */

  if ((clockid != CLOCK_REALTIME &&
       clockid != CLOCK_MONOTONIC &&
       clockid != CLOCK_BOOTTIME) ||
      (flags & ~(TFD_NONBLOCK | TFD_CLOEXEC)))
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Allocate instance data for this driver */

  new_dev = timerfd_allocdev();
  if (new_dev == NULL)
    {
      /* Failed to allocate new device */

      ret = -ENOMEM;
      goto errout;
    }

  /* Initialize the timer instance */

  new_dev->clock = clockid;
  new_fd = file_allocate(&g_timerfd_inode, O_RDONLY | flags,
                         0, new_dev, 0, true);
  if (new_fd < 0)
    {
      ret = new_fd;
      goto errout_with_dev;
    }

  /* Device is ready for use */

  nxmutex_unlock(&new_dev->lock);

  return new_fd;

errout_with_dev:
  timerfd_destroy(new_dev);
errout:
  set_errno(-ret);
  return ERROR;
}

int timerfd_settime(int fd, int flags,
                    FAR const struct itimerspec *new_value,
                    FAR struct itimerspec *old_value)
{
  FAR struct timerfd_priv_s *dev;
  FAR struct file *filep;
  irqstate_t intflags;
  sclock_t delay;
  int ret;

  /* Some sanity checks */

  if (!new_value)
    {
      ret = -EFAULT;
      goto errout;
    }

  if ((flags & ~TFD_TIMER_ABSTIME) != 0)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* Get file pointer by file descriptor */

  ret = fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      goto errout;
    }

  /* Check fd come from us */

  if (!filep->f_inode || filep->f_inode->u.i_ops != &g_timerfd_fops)
    {
      ret = -EINVAL;
      goto errout;
    }

  dev = (FAR struct timerfd_priv_s *)filep->f_priv;

  /* Disable interrupts here to ensure that expiration counter is accessed
   * atomicaly.
   */

  intflags = enter_critical_section();

  if (old_value)
    {
      /* Get the number of ticks before the underlying watchdog expires */

      delay = wd_gettime(&dev->wdog);

      /* Convert that to a struct timespec and return it */

      clock_ticks2time(delay, &old_value->it_value);
      clock_ticks2time(dev->delay, &old_value->it_interval);
    }

  /* Disarm the timer (in case the timer was already armed when
   * timerfd_settime() is called).
   */

  wd_cancel(&dev->wdog);

  /* Clear expiration counter */

  dev->counter = 0;

  /* If the it_value member of value is zero, the timer will not be
   * re-armed
   */

  if (new_value->it_value.tv_sec <= 0 && new_value->it_value.tv_nsec <= 0)
    {
      leave_critical_section(intflags);
      return OK;
    }

  /* Setup up any repetitive timer */

  clock_time2ticks(&new_value->it_interval, &delay);
  dev->delay = delay;

  /* We need to disable timer interrupts through the following section so
   * that the system timer is stable.
   */

  /* Check if abstime is selected */

  if ((flags & TFD_TIMER_ABSTIME) != 0)
    {
      /* Calculate a delay corresponding to the absolute time in 'value' */

      clock_abstime2ticks(dev->clock, &new_value->it_value, &delay);
    }
  else
    {
      /* Calculate a delay assuming that 'value' holds the relative time
       * to wait.  We have internal knowledge that clock_time2ticks always
       * returns success.
       */

      clock_time2ticks(&new_value->it_value, &delay);
    }

  /* If the time is in the past or now, then set up the next interval
   * instead (assuming a repetitive timer).
   */

  if (delay <= 0)
    {
      delay = dev->delay;
    }

  /* Then start the watchdog */

  ret = wd_start(&dev->wdog, delay, timerfd_timeout, (wdparm_t)dev);
  if (ret < 0)
    {
      leave_critical_section(intflags);
      goto errout;
    }

  leave_critical_section(intflags);
  return OK;

errout:
  set_errno(-ret);
  return ERROR;
}

int timerfd_gettime(int fd, FAR struct itimerspec *curr_value)
{
  FAR struct timerfd_priv_s *dev;
  FAR struct file *filep;
  sclock_t ticks;
  int ret;

  /* Some sanity checks */

  if (!curr_value)
    {
      ret = -EFAULT;
      goto errout;
    }

  /* Get file pointer by file descriptor */

  ret = fs_getfilep(fd, &filep);
  if (ret < 0)
    {
      goto errout;
    }

  /* Check fd come from us */

  if (!filep->f_inode || filep->f_inode->u.i_ops != &g_timerfd_fops)
    {
      ret = -EINVAL;
      goto errout;
    }

  dev = (FAR struct timerfd_priv_s *)filep->f_priv;

  /* Get the number of ticks before the underlying watchdog expires */

  ticks = wd_gettime(&dev->wdog);

  /* Convert that to a struct timespec and return it */

  clock_ticks2time(ticks, &curr_value->it_value);
  clock_ticks2time(dev->delay, &curr_value->it_interval);
  return OK;

errout:
  set_errno(-ret);
  return ERROR;
}
