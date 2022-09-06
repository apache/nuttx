/****************************************************************************
 * fs/vfs/fs_eventfd.c
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
#include <nuttx/mutex.h>
#include <sys/ioctl.h>
#include <sys/eventfd.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct eventfd_waiter_sem_s
{
  sem_t sem;
  struct eventfd_waiter_sem_s *next;
} eventfd_waiter_sem_t;

/* This structure describes the internal state of the driver */

struct eventfd_priv_s
{
  mutex_t                   lock;            /* Enforces device exclusive access */
  FAR eventfd_waiter_sem_t *rdsems;          /* List of blocking readers */
  FAR eventfd_waiter_sem_t *wrsems;          /* List of blocking writers */
  eventfd_t                 counter;         /* eventfd counter */
  unsigned int              minor;           /* eventfd minor number */
  uint8_t                   crefs;           /* References counts on eventfd (max: 255) */
  bool                      mode_semaphore;  /* eventfd mode (semaphore or counter) */

  /* The following is a list if poll structures of threads waiting for
   * driver events.
   */

#ifdef CONFIG_EVENT_FD_POLL
  FAR struct pollfd *fds[CONFIG_EVENT_FD_NPOLLWAITERS];
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int eventfd_do_open(FAR struct file *filep);
static int eventfd_do_close(FAR struct file *filep);

static ssize_t eventfd_do_read(FAR struct file *filep, FAR char *buffer,
                               size_t len);
static ssize_t eventfd_do_write(FAR struct file *filep,
                                FAR const char *buffer, size_t len);
#ifdef CONFIG_EVENT_FD_POLL
static int eventfd_do_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup);
#endif

static int eventfd_blocking_io(FAR struct eventfd_priv_s *dev,
                               FAR eventfd_waiter_sem_t  *sem,
                               FAR eventfd_waiter_sem_t **slist);

static unsigned int eventfd_get_unique_minor(void);
static void eventfd_release_minor(unsigned int minor);

static FAR struct eventfd_priv_s *eventfd_allocdev(void);
static void eventfd_destroy(FAR struct eventfd_priv_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_eventfd_fops =
{
  eventfd_do_open,  /* open */
  eventfd_do_close, /* close */
  eventfd_do_read,  /* read */
  eventfd_do_write, /* write */
  NULL,             /* seek */
  NULL,             /* ioctl */
#ifdef CONFIG_EVENT_FD_POLL
  eventfd_do_poll   /* poll */
#else
  NULL              /* poll */
#endif
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL            /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR struct eventfd_priv_s *eventfd_allocdev(void)
{
  FAR struct eventfd_priv_s *dev;

  dev = (FAR struct eventfd_priv_s *)
    kmm_zalloc(sizeof(struct eventfd_priv_s));
  if (dev)
    {
      /* Initialize the private structure */

      nxmutex_init(&dev->lock);
      nxmutex_lock(&dev->lock);
    }

  return dev;
}

static void eventfd_destroy(FAR struct eventfd_priv_s *dev)
{
  nxmutex_destroy(&dev->lock);
  kmm_free(dev);
}

static unsigned int eventfd_get_unique_minor(void)
{
  static unsigned int minor;

  return minor++;
}

static void eventfd_release_minor(unsigned int minor)
{
}

static int eventfd_do_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct eventfd_priv_s *priv = inode->i_private;
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

static int eventfd_do_close(FAR struct file *filep)
{
  int ret;
  FAR struct inode *inode = filep->f_inode;
  FAR struct eventfd_priv_s *priv = inode->i_private;

  /* devpath: EVENT_FD_VFS_PATH + /efd (4) + %u (10) + null char (1) */

  char devpath[sizeof(CONFIG_EVENT_FD_VFS_PATH) + 4 + 10 + 1];

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

      priv->crefs -= 1;
      nxmutex_unlock(&priv->lock);
      return OK;
    }

  /* Re-create the path to the driver. */

  finfo("destroy\n");
  sprintf(devpath, CONFIG_EVENT_FD_VFS_PATH "/efd%u", priv->minor);

  /* Will be unregistered later after close is done */

  unregister_driver(devpath);

  DEBUGASSERT(nxmutex_is_locked(&priv->lock));
  eventfd_release_minor(priv->minor);
  eventfd_destroy(priv);

  return OK;
}

static int eventfd_blocking_io(FAR struct eventfd_priv_s *dev,
                               FAR eventfd_waiter_sem_t  *sem,
                               FAR eventfd_waiter_sem_t **slist)
{
  int ret;
  sem->next = *slist;
  *slist = sem;

  nxmutex_unlock(&dev->lock);

  /* Wait for eventfd to notify */

  ret = nxsem_wait(&sem->sem);

  if (ret < 0)
    {
      FAR eventfd_waiter_sem_t *cur_sem;

      /* Interrupted wait, unregister semaphore
       * TODO ensure that lock wait does not fail (ECANCELED)
       */

      nxmutex_lock(&dev->lock);

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

      nxmutex_unlock(&dev->lock);
      return ret;
    }

  return nxmutex_lock(&dev->lock);
}

static ssize_t eventfd_do_read(FAR struct file *filep, FAR char *buffer,
                               size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct eventfd_priv_s *dev = inode->i_private;
  FAR eventfd_waiter_sem_t *cur_sem;
  ssize_t ret;

  if (len < sizeof(eventfd_t) || buffer == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for an incoming event */

  if (dev->counter == 0)
    {
      eventfd_waiter_sem_t sem;

      if (filep->f_oflags & O_NONBLOCK)
        {
          nxmutex_unlock(&dev->lock);
          return -EAGAIN;
        }

      nxsem_init(&sem.sem, 0, 0);
      do
        {
          ret = eventfd_blocking_io(dev, &sem, &dev->rdsems);
          if (ret < 0)
            {
              nxsem_destroy(&sem.sem);
              return ret;
            }
        }
      while (dev->counter == 0);

      nxsem_destroy(&sem.sem);
    }

  /* Device ready for read */

  if (dev->mode_semaphore)
    {
      *(FAR eventfd_t *)buffer = 1;
      dev->counter -= 1;
    }
  else
    {
      *(FAR eventfd_t *)buffer = dev->counter;
      dev->counter = 0;
    }

#ifdef CONFIG_EVENT_FD_POLL
  /* Notify all poll/select waiters */

  poll_notify(dev->fds, CONFIG_EVENT_FD_NPOLLWAITERS, POLLOUT);
#endif

  /* Notify all waiting writers that counter have been decremented */

  cur_sem = dev->wrsems;
  while (cur_sem != NULL)
    {
      nxsem_post(&cur_sem->sem);
      cur_sem = cur_sem->next;
    }

  dev->wrsems = NULL;

  nxmutex_unlock(&dev->lock);
  return sizeof(eventfd_t);
}

static ssize_t eventfd_do_write(FAR struct file *filep,
                                FAR const char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct eventfd_priv_s *dev = inode->i_private;
  FAR eventfd_waiter_sem_t *cur_sem;
  ssize_t ret;
  eventfd_t new_counter;

  if (len < sizeof(eventfd_t) || buffer == NULL ||
      (*(FAR eventfd_t *)buffer == (eventfd_t)-1) ||
      (*(FAR eventfd_t *)buffer == (eventfd_t)0))
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  new_counter = dev->counter + *(FAR eventfd_t *)buffer;

  if (new_counter < dev->counter)
    {
      eventfd_waiter_sem_t sem;

      /* Overflow detected */

      if (filep->f_oflags & O_NONBLOCK)
        {
          nxmutex_unlock(&dev->lock);
          return -EAGAIN;
        }

      nxsem_init(&sem.sem, 0, 0);
      do
        {
          ret = eventfd_blocking_io(dev, &sem, &dev->wrsems);
          if (ret < 0)
            {
              nxsem_destroy(&sem.sem);
              return ret;
            }
        }
      while ((new_counter = dev->counter + *(FAR eventfd_t *)buffer)
            < dev->counter);

      nxsem_destroy(&sem.sem);
    }

  /* Ready to write, update counter */

  dev->counter = new_counter;

#ifdef CONFIG_EVENT_FD_POLL
  /* Notify all poll/select waiters */

  poll_notify(dev->fds, CONFIG_EVENT_FD_NPOLLWAITERS, POLLIN);
#endif

  /* Notify all of the waiting readers */

  cur_sem = dev->rdsems;
  while (cur_sem != NULL)
    {
      nxsem_post(&cur_sem->sem);
      cur_sem = cur_sem->next;
    }

  dev->rdsems = NULL;

  nxmutex_unlock(&dev->lock);
  return sizeof(eventfd_t);
}

#ifdef CONFIG_EVENT_FD_POLL
static int eventfd_do_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct eventfd_priv_s *dev = inode->i_private;
  int ret;
  int i;
  pollevent_t eventset;

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

      *slot     = NULL;
      fds->priv = NULL;
      goto out;
    }

  /* This is a request to set up the poll. Find an available
   * slot for the poll structure reference
   */

  for (i = 0; i < CONFIG_EVENT_FD_NPOLLWAITERS; i++)
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

  if (i >= CONFIG_EVENT_FD_NPOLLWAITERS)
    {
      fds->priv = NULL;
      ret       = -EBUSY;
      goto out;
    }

  /* Notify the POLLOUT event if the pipe is not full, but only if
   * there is readers.
   */

  eventset = 0;
  if (dev->counter < (eventfd_t)-1)
    {
      eventset |= POLLOUT;
    }

  /* Notify the POLLIN event if the pipe is not empty */

  if (dev->counter > 0)
    {
      eventset |= POLLIN;
    }

  poll_notify(dev->fds, CONFIG_EVENT_FD_NPOLLWAITERS, eventset);

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int eventfd(unsigned int count, int flags)
{
  int ret;
  int new_fd;
  FAR struct eventfd_priv_s *new_dev;

  /* devpath: EVENT_FD_VFS_PATH + /efd (4) + %u (10) + null char (1) */

  char devpath[sizeof(CONFIG_EVENT_FD_VFS_PATH) + 4 + 10 + 1];

  /* Allocate instance data for this driver */

  new_dev = eventfd_allocdev();
  if (new_dev == NULL)
    {
      /* Failed to allocate new device */

      ret = -ENOMEM;
      goto exit_set_errno;
    }

  new_dev->counter = count;
  new_dev->mode_semaphore = !!(flags & EFD_SEMAPHORE);

  /* Request a unique minor device number */

  new_dev->minor = eventfd_get_unique_minor();

  /* Get device path */

  sprintf(devpath, CONFIG_EVENT_FD_VFS_PATH "/efd%u", new_dev->minor);

  /* Register the driver */

  ret = register_driver(devpath, &g_eventfd_fops, 0666, new_dev);
  if (ret < 0)
    {
      ferr("Failed to register new device %s: %d\n", devpath, ret);
      goto exit_release_minor;
    }

  /* Device is ready for use */

  nxmutex_unlock(&new_dev->lock);

  /* Try open new device */

  new_fd = nx_open(devpath, O_RDWR |
    (flags & (EFD_NONBLOCK | EFD_SEMAPHORE | EFD_CLOEXEC)));

  if (new_fd < 0)
    {
      ret = new_fd;
      goto exit_unregister_driver;
    }

  return new_fd;

exit_unregister_driver:
  unregister_driver(devpath);
exit_release_minor:
  eventfd_release_minor(new_dev->minor);
  eventfd_destroy(new_dev);
exit_set_errno:
  set_errno(-ret);
  return ERROR;
}
