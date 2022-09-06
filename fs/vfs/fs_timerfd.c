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
#include <nuttx/wqueue.h>
#include <nuttx/spinlock.h>
#include <nuttx/mutex.h>

#include <sys/ioctl.h>
#include <sys/timerfd.h>

#include "clock/clock.h"
#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIMER_FD_WORK LPWORK

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct timerfd_waiter_sem_s
{
  sem_t sem;
  struct timerfd_waiter_sem_s *next;
} timerfd_waiter_sem_t;

/* This structure describes the internal state of the driver */

struct timerfd_priv_s
{
  mutex_t       lock;           /* Enforces device exclusive access */
  timerfd_waiter_sem_t *rdsems; /* List of blocking readers */
  int           clock;          /* Clock to use as the timing base */
  int           delay;          /* If non-zero, used to reset repetitive
                                 * timers */
  struct wdog_s wdog;           /* The watchdog that provides the timing */
  struct work_s work;           /* For deferred timeout operations */
  timerfd_t     counter;        /* timerfd counter */
  spinlock_t    lock;           /* timerfd counter specific lock */
  unsigned int  minor;          /* timerfd minor number */
  uint8_t       crefs;          /* References counts on timerfd (max: 255) */

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
                               timerfd_waiter_sem_t *sem,
                               FAR timerfd_waiter_sem_t **slist);

static unsigned int timerfd_get_unique_minor(void);
static void timerfd_release_minor(unsigned int minor);

static FAR struct timerfd_priv_s *timerfd_allocdev(void);
static void timerfd_destroy(FAR struct timerfd_priv_s *dev);

static void timerfd_timeout_work(FAR void *arg);
static void timerfd_timeout(wdparm_t idev);

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
  work_cancel(TIMER_FD_WORK, &dev->work);
  nxmutex_destroy(&dev->lock);
  kmm_free(dev);
}

static timerfd_t timerfd_get_counter(FAR struct timerfd_priv_s *dev)
{
  timerfd_t counter;
  irqstate_t intflags;

  intflags = spin_lock_irqsave(&dev->lock);
  counter = dev->counter;
  spin_unlock_irqrestore(&dev->lock, intflags);

  return counter;
}

static unsigned int timerfd_get_unique_minor(void)
{
  static unsigned int minor;

  return minor++;
}

static void timerfd_release_minor(unsigned int minor)
{
}

static int timerfd_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct timerfd_priv_s *priv = inode->i_private;
  int ret;

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  finfo("crefs: %d <%s>\n", priv->crefs, inode->i_name);

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
  int ret;
  FAR struct inode *inode = filep->f_inode;
  FAR struct timerfd_priv_s *priv = inode->i_private;

  /* devpath: TIMER_FD_VFS_PATH + /tfd (4) + %u (10) + null char (1) */

  char devpath[sizeof(CONFIG_TIMER_FD_VFS_PATH) + 4 + 10 + 1];

  /* Get exclusive access to the device structures */

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  finfo("crefs: %d <%s>\n", priv->crefs, inode->i_name);

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
  sprintf(devpath, CONFIG_TIMER_FD_VFS_PATH "/tfd%u", priv->minor);

  /* Will be unregistered later after close is done */

  unregister_driver(devpath);

  DEBUGASSERT(nxmutex_is_locked(&priv->lock));
  timerfd_release_minor(priv->minor);
  timerfd_destroy(priv);

  return OK;
}

static int timerfd_blocking_io(FAR struct timerfd_priv_s *dev,
                               timerfd_waiter_sem_t *sem,
                               FAR timerfd_waiter_sem_t **slist)
{
  int ret;
  sem->next = *slist;
  *slist = sem;

  nxmutex_unlock(&dev->lock);

  /* Wait for timerfd to notify */

  ret = nxsem_wait(&sem->sem);

  if (ret < 0)
    {
      /* Interrupted wait, unregister semaphore
       * TODO ensure that lock wait does not fail (ECANCELED)
       */

      nxmutex_lock(&dev->lock);

      timerfd_waiter_sem_t *cur_sem = *slist;

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

      nxmutex_unlock(&dev->lock);
      return ret;
    }

  return nxmutex_lock(&dev->lock);
}

static ssize_t timerfd_read(FAR struct file *filep, FAR char *buffer,
                            size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct timerfd_priv_s *dev = inode->i_private;
  irqstate_t intflags;
  ssize_t ret;

  if (len < sizeof(timerfd_t) || buffer == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for an incoming event */

  if (timerfd_get_counter(dev) == 0)
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          nxmutex_unlock(&dev->lock);
          return -EAGAIN;
        }

      timerfd_waiter_sem_t sem;
      nxsem_init(&sem.sem, 0, 0);
      nxsem_set_protocol(&sem.sem, SEM_PRIO_NONE);

      do
        {
          ret = timerfd_blocking_io(dev, &sem, &dev->rdsems);
          if (ret < 0)
            {
              nxsem_destroy(&sem.sem);
              return ret;
            }
        }
      while (timerfd_get_counter(dev) == 0);

      nxsem_destroy(&sem.sem);
    }

  /* Device ready for read.  Ensure that interrupts are disabled and we
   * do not lose counts if expiration occurs after read, but before setting
   * counter to zero
   */

  intflags = spin_lock_irqsave(&dev->lock);

  *(FAR timerfd_t *)buffer = dev->counter;
  dev->counter = 0;

  spin_unlock_irqrestore(&dev->lock, intflags);

  nxmutex_unlock(&dev->lock);
  return sizeof(timerfd_t);
}

#ifdef CONFIG_TIMER_FD_POLL
static int timerfd_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct timerfd_priv_s *dev = inode->i_private;
  int ret;
  int i;

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  ret = OK;

  if (!setup)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

      /* Remove all memory of the poll setup */

      *slot                = NULL;
      fds->priv            = NULL;
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

  if (timerfd_get_counter(dev) > 0)
    {
#ifdef CONFIG_TIMER_FD_POLL
      poll_notify(dev->fds, CONFIG_TIMER_FD_NPOLLWAITERS, POLLIN);
#endif
    }

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}
#endif

static void timerfd_timeout_work(FAR void *arg)
{
  FAR struct timerfd_priv_s *dev = (FAR struct timerfd_priv_s *)arg;
  int ret;

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      wd_cancel(&dev->wdog);
      return;
    }

#ifdef CONFIG_TIMER_FD_POLL
  /* Notify all poll/select waiters */

  poll_notify(dev->fds, CONFIG_TIMER_FD_NPOLLWAITERS, POLLIN);
#endif

  /* Notify all of the waiting readers */

  timerfd_waiter_sem_t *cur_sem = dev->rdsems;
  while (cur_sem != NULL)
    {
      nxsem_post(&cur_sem->sem);
      cur_sem = cur_sem->next;
    }

  dev->rdsems = NULL;
  nxmutex_unlock(&dev->lock);
}

static void timerfd_timeout(wdparm_t idev)
{
  FAR struct timerfd_priv_s *dev = (FAR struct timerfd_priv_s *)idev;
  irqstate_t intflags;

  /* Disable interrupts to ensure that expiration counter is accessed
   * atomically
   */

  intflags = spin_lock_irqsave(&dev->lock);

  /* Increment timer expiration counter */

  dev->counter++;

  work_queue(TIMER_FD_WORK, &dev->work, timerfd_timeout_work, dev, 0);

  /* If this is a repetitive timer, then restart the watchdog */

  if (dev->delay)
    {
      wd_start(&dev->wdog, dev->delay, timerfd_timeout, idev);
    }

  spin_unlock_irqrestore(&dev->lock, intflags);
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

  if (clockid != CLOCK_REALTIME &&
      clockid != CLOCK_MONOTONIC &&
      clockid != CLOCK_BOOTTIME)
    {
      ret = -EINVAL;
      goto errout;
    }

  /* devpath: TIMER_FD_VFS_PATH + /tfd (4) + %u (10) + null char (1) */

  char devpath[sizeof(CONFIG_TIMER_FD_VFS_PATH) + 4 + 10 + 1];

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
  new_dev->crefs = 1;
  new_dev->delay = 0;
  new_dev->counter = 0;

  /* Request a unique minor device number */

  new_dev->minor = timerfd_get_unique_minor();

  /* Get device path */

  sprintf(devpath, CONFIG_TIMER_FD_VFS_PATH "/tfd%u", new_dev->minor);

  /* Register the driver */

  ret = register_driver(devpath, &g_timerfd_fops, 0444, new_dev);
  if (ret < 0)
    {
      ferr("Failed to register new device %s: %d\n", devpath, ret);
      ret = -ENODEV;
      goto errout_release_minor;
    }

  /* Device is ready for use */

  nxmutex_unlock(&new_dev->lock);

  /* Try open new device */

  new_fd = nx_open(devpath, O_RDONLY |
                   (flags & (TFD_NONBLOCK | TFD_CLOEXEC)));

  if (new_fd < 0)
    {
      ret = new_fd;
      goto errout_unregister_driver;
    }

  return new_fd;

errout_unregister_driver:
  unregister_driver(devpath);
errout_release_minor:
  timerfd_release_minor(new_dev->minor);
  timerfd_destroy(new_dev);
errout:
  set_errno(-ret);
  return ERROR;
}

int timerfd_settime(int fd, int flags,
                    FAR const struct itimerspec *new_value,
                    FAR struct itimerspec *old_value)
{
  FAR struct file *filep;
  FAR struct timerfd_priv_s *dev;
  irqstate_t intflags;
  sclock_t delay;
  int ret;

  /* Some sanity checks */

  if (!new_value)
    {
      ret = -EFAULT;
      goto errout;
    }

  if (flags && (flags & TFD_TIMER_ABSTIME) == 0)
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

  dev = (FAR struct timerfd_priv_s *)filep->f_inode->i_private;

  if (old_value)
    {
      /* Get the number of ticks before the underlying watchdog expires */

      delay = wd_gettime(&dev->wdog);

      /* Convert that to a struct timespec and return it */

      clock_ticks2time(delay, &old_value->it_value);
      clock_ticks2time(dev->delay, &old_value->it_interval);
    }

  /* Disable interrupts here to ensure that expiration counter is accessed
   * atomicaly and timeout work is canceled with the same sequence
   */

  intflags = spin_lock_irqsave(&dev->lock);

  /* Disarm the timer (in case the timer was already armed when
   * timerfd_settime() is called).
   */

  wd_cancel(&dev->wdog);

  /* Cancel notification work */

  work_cancel(TIMER_FD_WORK, &dev->work);

  /* Clear expiration counter */

  dev->counter = 0;

  /* If the it_value member of value is zero, the timer will not be
   * re-armed
   */

  if (new_value->it_value.tv_sec <= 0 && new_value->it_value.tv_nsec <= 0)
    {
      spin_unlock_irqrestore(NULL, intflags);
      return OK;
    }

  /* Setup up any repetitive timer */

  if (new_value->it_interval.tv_sec > 0 ||
      new_value->it_interval.tv_nsec > 0)
    {
      clock_time2ticks(&new_value->it_interval, &delay);

      /* REVISIT: Should delay be sclock_t? */

      dev->delay = (int)delay;
    }
  else
    {
      dev->delay = 0;
    }

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

  if (delay > 0)
    {
      ret = wd_start(&dev->wdog, delay, timerfd_timeout, (wdparm_t)dev);
      if (ret < 0)
        {
          spin_unlock_irqrestore(&dev->lock, intflags);
          goto errout;
        }
    }

  spin_unlock_irqrestore(&dev->lock, intflags);
  return OK;

errout:
  set_errno(-ret);
  return ERROR;
}

int timerfd_gettime(int fd, FAR struct itimerspec *curr_value)
{
  FAR struct file *filep;
  FAR struct timerfd_priv_s *dev;
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

  dev = (FAR struct timerfd_priv_s *)filep->f_inode->i_private;

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
