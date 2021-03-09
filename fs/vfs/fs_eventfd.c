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
#include <errno.h>
#include <fcntl.h>

#include <debug.h>

#include <sys/ioctl.h>
#include <sys/eventfd.h>

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_EVENT_FD_VFS_PATH
#define CONFIG_EVENT_FD_VFS_PATH "/dev"
#endif

#ifndef CONFIG_EVENT_FD_NPOLLWAITERS
/* Maximum number of threads than can be waiting for POLL events */
#define CONFIG_EVENT_FD_NPOLLWAITERS 2
#endif

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
  sem_t     exclsem;            /* Enforces device exclusive access */
  eventfd_waiter_sem_t *rdsems; /* List of blocking readers */
  eventfd_waiter_sem_t *wrsems; /* List of blocking writers */
  eventfd_t counter;            /* eventfd counter */
  size_t    minor;              /* eventfd minor number */
  uint8_t   crefs;              /* References counts on eventfd (max: 255) */
  uint8_t   mode_semaphore;     /* eventfd mode (semaphore or counter) */

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
static int eventfd_do_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
#ifdef CONFIG_EVENT_FD_POLL
static int eventfd_do_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup);

static void eventfd_pollnotify(FAR struct eventfd_priv_s *dev,
                               pollevent_t eventset);
#endif

static int eventfd_blocking_io(FAR struct eventfd_priv_s *dev,
                               eventfd_waiter_sem_t *sem,
                               FAR eventfd_waiter_sem_t **slist);

static size_t eventfd_get_unique_minor(void);
static void eventfd_release_minor(size_t minor);

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
  0,                /* seek */
  eventfd_do_ioctl  /* ioctl */
#ifdef CONFIG_EVENT_FD_POLL
  , eventfd_do_poll /* poll */
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

      nxsem_init(&dev->exclsem, 0, 0);
    }

  return dev;
}

static void eventfd_destroy(FAR struct eventfd_priv_s *dev)
{
  nxsem_destroy(&dev->exclsem);
  kmm_free(dev);
}

#ifdef CONFIG_EVENT_FD_POLL
static void eventfd_pollnotify(FAR struct eventfd_priv_s *dev,
                               pollevent_t eventset)
{
  FAR struct pollfd *fds;
  int i;

  for (i = 0; i < CONFIG_EVENT_FD_NPOLLWAITERS; i++)
    {
      fds = dev->fds[i];
      if (fds)
        {
          fds->revents |= eventset & fds->events;

          if (fds->revents != 0)
            {
              nxsem_post(fds->sem);
            }
        }
    }
}
#endif

static size_t eventfd_get_unique_minor(void)
{
  static size_t minor;

  return minor++;
}

static void eventfd_release_minor(size_t minor)
{
}

static int eventfd_do_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct eventfd_priv_s *priv = inode->i_private;
  int ret;

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&priv->exclsem);
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

  nxsem_post(&priv->exclsem);
  return ret;
}

static int eventfd_do_close(FAR struct file *filep)
{
  int ret;
  FAR struct inode *inode = filep->f_inode;
  FAR struct eventfd_priv_s *priv = inode->i_private;

  /* devpath: EVENT_FD_VFS_PATH + /efd (4) + %d (3) + null char (1) */

  char devpath[sizeof(CONFIG_EVENT_FD_VFS_PATH) + 4 + 3 + 1];

  /* Get exclusive access to the device structures */

  ret = nxsem_wait(&priv->exclsem);
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
      nxsem_post(&priv->exclsem);
      return OK;
    }

  /* Re-create the path to the driver. */

  finfo("destroy\n");
  sprintf(devpath, CONFIG_EVENT_FD_VFS_PATH "/efd%d", priv->minor);

  /* Will be unregistered later after close is done */

  unregister_driver(devpath);

  DEBUGASSERT(priv->exclsem.semcount == 0);
  eventfd_release_minor(priv->minor);
  eventfd_destroy(priv);

  return OK;
}

static int eventfd_blocking_io(FAR struct eventfd_priv_s *dev,
                               eventfd_waiter_sem_t *sem,
                               FAR eventfd_waiter_sem_t **slist)
{
  int ret;
  sem->next = *slist;
  *slist = sem;

  nxsem_post(&dev->exclsem);

  /* Wait for eventfd to notify */

  ret = nxsem_wait(&sem->sem);

  if (ret < 0)
    {
      /* Interrupted wait, unregister semaphore
       * TODO ensure that exclsem wait does not fail (ECANCELED)
       */

      nxsem_wait_uninterruptible(&dev->exclsem);

      eventfd_waiter_sem_t *cur_sem = *slist;

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

      nxsem_post(&dev->exclsem);
      return ret;
    }

  return nxsem_wait(&dev->exclsem);
}

static ssize_t eventfd_do_read(FAR struct file *filep, FAR char *buffer,
                               size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct eventfd_priv_s *dev = inode->i_private;
  ssize_t ret;

  if (len < sizeof(eventfd_t) || buffer == NULL)
    {
      return -EINVAL;
    }

  ret = nxsem_wait(&dev->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for an incoming event */

  if (dev->counter == 0)
    {
      if (filep->f_oflags & O_NONBLOCK)
        {
          nxsem_post(&dev->exclsem);
          return -EAGAIN;
        }

      eventfd_waiter_sem_t sem;
      nxsem_init(&sem.sem, 0, 0);
      nxsem_set_protocol(&sem.sem, SEM_PRIO_NONE);

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

  eventfd_pollnotify(dev, POLLOUT);
#endif

  /* Notify all waiting writers that counter have been decremented */

  eventfd_waiter_sem_t *cur_sem = dev->wrsems;
  while (cur_sem != NULL)
    {
      nxsem_post(&cur_sem->sem);
      cur_sem = cur_sem->next;
    }

  dev->wrsems = NULL;

  nxsem_post(&dev->exclsem);
  return sizeof(eventfd_t);
}

static ssize_t eventfd_do_write(FAR struct file *filep,
                                FAR const char *buffer, size_t len)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct eventfd_priv_s *dev = inode->i_private;
  ssize_t ret;
  eventfd_t new_counter;

  if (len < sizeof(eventfd_t) || buffer == NULL ||
      (*(FAR eventfd_t *)buffer == (eventfd_t)-1) ||
      (*(FAR eventfd_t *)buffer == (eventfd_t)0))
    {
      return -EINVAL;
    }

  ret = nxsem_wait(&dev->exclsem);
  if (ret < 0)
    {
      return ret;
    }

  new_counter = dev->counter + *(FAR eventfd_t *)buffer;

  if (new_counter < dev->counter)
    {
      /* Overflow detected */

      if (filep->f_oflags & O_NONBLOCK)
        {
          nxsem_post(&dev->exclsem);
          return -EAGAIN;
        }

      eventfd_waiter_sem_t sem;
      nxsem_init(&sem.sem, 0, 0);
      nxsem_set_protocol(&sem.sem, SEM_PRIO_NONE);

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

  eventfd_pollnotify(dev, POLLIN);
#endif

  /* Notify all of the waiting readers */

  eventfd_waiter_sem_t *cur_sem = dev->rdsems;
  while (cur_sem != NULL)
    {
      nxsem_post(&cur_sem->sem);
      cur_sem = cur_sem->next;
    }

  dev->rdsems = NULL;

  nxsem_post(&dev->exclsem);
  return sizeof(eventfd_t);
}

static int eventfd_do_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct eventfd_priv_s *priv = inode->i_private;

  if (cmd == FIOC_MINOR)
    {
      *(FAR int *)((uintptr_t)arg) = priv->minor;
      return OK;
    }

  return -ENOSYS;
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

  ret = nxsem_wait(&dev->exclsem);
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
      goto errout;
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
      goto errout;
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

  if (eventset)
    {
      eventfd_pollnotify(dev, eventset);
    }

errout:
  nxsem_post(&dev->exclsem);
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

  /* devpath: EVENT_FD_VFS_PATH + /efd (4) + size_t (10) + null char (1) */

  char devpath[sizeof(CONFIG_EVENT_FD_VFS_PATH) + 4 + 10 + 1];

  /* Allocate instance data for this driver */

  new_dev = eventfd_allocdev();
  if (new_dev == NULL)
    {
      /* Failed to allocate new device */

      ret = ENOMEM;
      goto exit_set_errno;
    }

  new_dev->counter = count;
  new_dev->mode_semaphore = !!(flags & EFD_SEMAPHORE);

  /* Request a unique minor device number */

  new_dev->minor = eventfd_get_unique_minor();

  /* Get device path */

  sprintf(devpath, CONFIG_EVENT_FD_VFS_PATH "/efd%d", new_dev->minor);

  /* Register the driver */

  ret = register_driver(devpath, &g_eventfd_fops, 0666, new_dev);
  if (ret < 0)
    {
      ferr("Failed to register new device %s: %d\n", devpath, ret);
      ret = -ret;
      goto exit_release_minor;
    }

  /* Device is ready for use */

  nxsem_post(&new_dev->exclsem);

  /* Try open new device */

  new_fd = nx_open(devpath, O_RDWR |
    (flags & (EFD_NONBLOCK | EFD_SEMAPHORE | EFD_CLOEXEC)));

  if (new_fd < 0)
    {
      ret = -new_fd;
      goto exit_unregister_driver;
    }

  return new_fd;

exit_unregister_driver:
  unregister_driver(devpath);
exit_release_minor:
  eventfd_release_minor(new_dev->minor);
  eventfd_destroy(new_dev);
exit_set_errno:
  set_errno(ret);
  return ERROR;
}
