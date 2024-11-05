/****************************************************************************
 * fs/notify/inotify.c
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
#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <nuttx/mutex.h>
#include <nuttx/fs/fs.h>
#include <nuttx/lib/lib.h>

#include <sys/inotify.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <debug.h>
#include <poll.h>
#include <string.h>
#include <search.h>
#include <libgen.h>

#include "inode/inode.h"
#include "sched/sched.h"
#include "fs_heap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

 #define ROUND_UP(x, y) (((x) + (y) - 1) / (y) * (y))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct inotify_device_s
{
  mutex_t            lock;        /* Enforces device exclusive access */
  sem_t              sem;         /* Used to wait for poll events */
  struct list_node   events;      /* List of queued events */
  struct list_node   watches;     /* List of watches */
  int                count;       /* Reference count */
  uint32_t           event_size;  /* Size of the queue (bytes) */
  uint32_t           event_count; /* Number of pending events */
  FAR struct pollfd *fds[CONFIG_FS_NOTIFY_FD_POLLWAITERS];
};

struct inotify_event_s
{
  struct list_node     node;     /* Entry in inotify_device's list */
  struct inotify_event event;    /* The user-space event */
};

struct inotify_watch_list_s
{
  struct list_node watches;
  FAR char        *path;
};

struct inotify_watch_s
{
  struct list_node                 d_node;  /* Add to device list */
  struct list_node                 l_node;  /* Add to node hash list */
  int                              wd;      /* Watch descriptor */
  uint32_t                         mask;    /* Event mask for this watch */
  FAR struct inotify_device_s     *dev;     /* Associated device */
  FAR struct inotify_watch_list_s *list;    /* Associated watch list */
};

struct inotify_global_s
{
  mutex_t  lock;               /* Enforces global exclusive access */
  int      event_cookie;       /* Event cookie */
  int      watch_cookie;       /* Watch cookie */
  uint32_t read_count;         /* Number of read events */
  uint32_t write_count;        /* Number of write events */
  struct   hsearch_data hash;  /* Hash table for watch lists */
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int     inotify_open(FAR struct file *filep);
static int     inotify_close(FAR struct file *filep);
static ssize_t inotify_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static int     inotify_poll(FAR struct file *filep, FAR struct pollfd *fds,
                            bool setup);
static int     inotify_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_inotify_fops =
{
  inotify_open,     /* open */
  inotify_close,    /* close */
  inotify_read,     /* read */
  NULL,             /* write */
  NULL,             /* seek */
  inotify_ioctl,    /* ioctl */
  NULL,             /* mmap */
  NULL,             /* truncate */
  inotify_poll,     /* poll */
};

static struct inode g_inotify_inode =
{
  NULL,
  NULL,
  NULL,
  1,
  FSNODEFLAG_TYPE_DRIVER,
  {
    &g_inotify_fops
  }
};

static struct inotify_global_s g_inotify =
{
  .lock = NXMUTEX_INITIALIZER,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: notify_add_count
 *
 * Description:
 *   Add the count.
 *
 ****************************************************************************/

static void inotify_add_count(uint32_t mask)
{
  g_inotify.read_count += (mask & IN_ACCESS) ? 1 : 0;
  g_inotify.write_count += (mask & IN_MODIFY) ? 1 : 0;
}

/****************************************************************************
 * Name: inotify_sub_count
 *
 * Description:
 *   Reduce the count.
 *
 ****************************************************************************/

static void inotify_sub_count(uint32_t mask)
{
  g_inotify.read_count -= (mask & IN_ACCESS) ? 1 : 0;
  g_inotify.write_count -= (mask & IN_MODIFY) ? 1 : 0;
}

/****************************************************************************
 * Name: notify_check_mask
 *
 * Description:
 *   Check if the count is valid.
 *
 ****************************************************************************/

static int notify_check_mask(uint32_t mask)
{
  return (((mask & IN_ACCESS) && g_inotify.read_count == 0) ||
          ((mask & IN_MODIFY) && g_inotify.write_count == 0)) ?
          -EBADF : OK;
}

/****************************************************************************
 * Name: inotify_alloc_event
 *
 * Description:
 *   Initialize a kernel event.
 *   Size of name is rounded up to sizeof(struct inotify_event).
 ****************************************************************************/

static FAR struct inotify_event_s *
inotify_alloc_event(int wd, uint32_t mask, uint32_t cookie,
                    FAR const char *name)
{
  FAR struct inotify_event_s *event;
  size_t len = 0;

  if (name != NULL)
    {
      len = ROUND_UP(strlen(name) + 1, sizeof(struct inotify_event));
    }

  event = fs_heap_malloc(sizeof(struct inotify_event_s) + len);
  if (event == NULL)
    {
      return NULL;
    }

  event->event.wd     = wd;
  event->event.mask   = mask;
  event->event.cookie = cookie;
  event->event.len    = len;

  if (name != NULL)
    {
      strcpy(event->event.name, name);
    }

  return event;
}

/****************************************************************************
 * Name: inotify_queue_event
 *
 * Description:
 *   Queue an event to the inotify device.
 *
 ****************************************************************************/

static void inotify_queue_event(FAR struct inotify_device_s *dev, int wd,
                                uint32_t mask, uint32_t cookie,
                                FAR const char *name)
{
  FAR struct inotify_event_s *event;
  FAR struct inotify_event_s *last;
  int semcnt;

  if (!list_is_empty(&dev->events))
    {
      /* Drop this event if it is a dupe of the previous */

      last = list_last_entry(&dev->events,
                             struct inotify_event_s, node);
      if (last->event.mask == mask && last->event.wd == wd &&
          last->event.cookie == cookie &&
          ((name == NULL && last->event.len == 0) ||
           (name && last->event.len && !strcmp(name, last->event.name))))
        {
          return;
        }
    }

  if (dev->event_count > CONFIG_FS_NOTIFY_MAX_EVENTS)
    {
      finfo("Too many events queued\n");
      return;
    }

  if (dev->event_count == CONFIG_FS_NOTIFY_MAX_EVENTS)
    {
      event = inotify_alloc_event(-1, IN_Q_OVERFLOW, cookie, NULL);
    }
  else
    {
      event = inotify_alloc_event(wd, mask, cookie, name);
    }

  if (event == NULL)
    {
      return;
    }

  dev->event_count++;
  dev->event_size += sizeof(struct inotify_event) + event->event.len;
  list_add_tail(&dev->events, &event->node);

  poll_notify(dev->fds, CONFIG_FS_NOTIFY_FD_POLLWAITERS, POLLIN);

  while (nxsem_get_value(&dev->sem, &semcnt) == 0 && semcnt <= 1)
    {
      nxsem_post(&dev->sem);
    }
}

/****************************************************************************
 * Name: inotify_remove_watch_no_event
 *
 * Description:
 *   Remove a watch from the inotify device without sending an event.
 *
 ****************************************************************************/

static void
inotify_remove_watch_no_event(FAR struct inotify_watch_s *watch)
{
  FAR struct inotify_watch_list_s *list = watch->list;

  list_delete(&watch->d_node);
  list_delete(&watch->l_node);
  inotify_sub_count(watch->mask);
  fs_heap_free(watch);

  if (list_is_empty(&list->watches))
    {
      ENTRY item;
      item.key = list->path;
      hsearch_r(item, DELETE, NULL, &g_inotify.hash);
    }
}

/****************************************************************************
 * Name: inotify_remove_watch
 *
 * Description:
 *   Remove a watch from the inotify device.
 *
 ****************************************************************************/

static void inotify_remove_watch(FAR struct inotify_device_s *dev,
                                 FAR struct inotify_watch_s *watch)
{
  inotify_queue_event(dev, watch->wd, IN_IGNORED, 0, NULL);
  inotify_remove_watch_no_event(watch);
}

/****************************************************************************
 * Name: inotify_remove_event
 *
 * Description:
 *   Remove a kernel event from the inotify device.
 *
 ****************************************************************************/

static void inotify_remove_event(FAR struct inotify_device_s *dev,
                                 FAR struct inotify_event_s *event)
{
  list_delete(&event->node);
  dev->event_size -= sizeof(struct inotify_event) + event->event.len;
  dev->event_count--;
  fs_heap_free(event);
}

/****************************************************************************
 * Name: inotify_alloc_device
 *
 * Description:
 *   Allocate a new inotify device.
 *
 ****************************************************************************/

static FAR struct inotify_device_s *inotify_alloc_device(void)
{
  FAR struct inotify_device_s *dev;

  dev = fs_heap_zalloc(sizeof(struct inotify_device_s));
  if (dev == NULL)
    {
      return dev;
    }

  dev->count = 1;
  nxmutex_init(&dev->lock);
  nxsem_init(&dev->sem, 0, 0);
  list_initialize(&dev->events);
  list_initialize(&dev->watches);
  return dev;
}

/****************************************************************************
 * Name: inotify_open
 *
 * Description:
 *   Open the inotify device.
 *
 ****************************************************************************/

static int inotify_open(FAR struct file *filep)
{
  FAR struct inotify_device_s *dev = filep->f_priv;

  nxmutex_lock(&dev->lock);
  dev->count++;
  nxmutex_unlock(&dev->lock);
  return OK;
}

/****************************************************************************
 * Name: inotify_poll
 *
 * Description:
 *   Poll the inotify device.
 *
 ****************************************************************************/

static int inotify_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct inotify_device_s *dev = filep->f_priv;
  int ret = 0;
  int i;

  nxmutex_lock(&dev->lock);
  if (!setup)
    {
      FAR struct pollfd **slot = fds->priv;
      *slot = NULL;
      fds->priv = NULL;
      goto out;
    }

  for (i = 0; i < CONFIG_FS_NOTIFY_FD_POLLWAITERS; i++)
    {
      if (dev->fds[i] == 0)
        {
          dev->fds[i] = fds;
          fds->priv = &dev->fds[i];
          break;
        }
    }

  if (i >= CONFIG_FS_NOTIFY_FD_POLLWAITERS)
    {
      fds->priv = NULL;
      ret = -EBUSY;
      goto out;
    }

  if (!list_is_empty(&dev->events))
    {
      poll_notify(dev->fds, CONFIG_FS_NOTIFY_FD_POLLWAITERS, POLLIN);
    }

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Name: inotify_ioctl
 *
 * Description:
 *   Perform an inotify ioctl.
 *
 ****************************************************************************/

static int inotify_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inotify_device_s *dev = filep->f_priv;
  int ret = -ENOTTY;

  switch (cmd)
    {
      case FIONREAD:
        {
          FAR int *nbytes = (FAR int *)((uintptr_t)arg);
          if (nbytes)
            {
              *nbytes = dev->event_size;
              ret = OK;
            }
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: inotify_read
 *
 * Description:
 *   Read from the event list of inotify device.
 *
 ****************************************************************************/

static ssize_t inotify_read(FAR struct file *filp, FAR char *buffer,
                            size_t len)
{
  FAR struct inotify_device_s *dev = filp->f_priv;
  FAR char *start = buffer;
  int ret = 0;

  if (len < sizeof(struct inotify_event) || buffer == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->lock);
  while (len >= sizeof(struct inotify_event))
    {
      if (list_is_empty(&dev->events))
        {
          if (start != buffer || (filp->f_oflags & O_NONBLOCK) != 0)
            {
              break;
            }

          nxmutex_unlock(&dev->lock);
          ret = nxsem_wait_uninterruptible(&dev->sem);
          nxmutex_lock(&dev->lock);
          if (ret < 0)
            {
              break;
            }
        }
      else
        {
          FAR struct inotify_event_s *event =
            list_first_entry(&dev->events, struct inotify_event_s, node);

          size_t eventlen = sizeof(struct inotify_event) + event->event.len;
          if (len < eventlen)
            {
              break;
            }

          memcpy(buffer, &event->event, eventlen);
          buffer += eventlen;
          len -= eventlen;
          inotify_remove_event(dev, event);
        }
    }

  nxmutex_unlock(&dev->lock);
  if (start != buffer)
    {
      ret = buffer - start;
    }

  return ret == 0 ? -EAGAIN : ret;
}

/****************************************************************************
 * Name: inotify_close
 *
 * Description:
 *   Close the inotify device.
 *
 ****************************************************************************/

static int inotify_close(FAR struct file *filep)
{
  FAR struct inotify_device_s *dev = filep->f_priv;

  nxmutex_lock(&g_inotify.lock);
  nxmutex_lock(&dev->lock);
  if (--dev->count > 0)
    {
      nxmutex_unlock(&dev->lock);
      nxmutex_unlock(&g_inotify.lock);
      return OK;
    }

  /* Destroy all of the watches on this device  */

  while (!list_is_empty(&dev->watches))
    {
      FAR struct inotify_watch_s *watch;
      watch = list_first_entry(&dev->watches, struct inotify_watch_s,
                               d_node);
      inotify_remove_watch_no_event(watch);
    }

  /* Destroy all of the events on this device  */

  while (!list_is_empty(&dev->events))
    {
      FAR struct inotify_event_s *event;
      event = list_first_entry(&dev->events, struct inotify_event_s, node);
      inotify_remove_event(dev, event);
    }

  nxmutex_unlock(&dev->lock);
  nxmutex_unlock(&g_inotify.lock);
  nxmutex_destroy(&dev->lock);
  nxsem_destroy(&dev->sem);
  fs_heap_free(dev);
  return OK;
}

/****************************************************************************
 * Name: inotify_get_device_from_fd
 *
 * Description:
 *   Get the inotify device from the file descriptor.
 *
 ****************************************************************************/

static FAR struct inotify_device_s *
inotify_get_device_from_fd(int fd, FAR struct file **filep)
{
  if (fs_getfilep(fd, filep) < 0)
    {
      return NULL;
    }

  if ((*filep)->f_inode != &g_inotify_inode)
    {
      fs_putfilep(*filep);
      return NULL;
    }

  return (*filep)->f_priv;
}

/****************************************************************************
 * Name: inotify_get_watch_from_list
 *
 * Description:
 *   Get the inotify watch from the file node.
 *
 ****************************************************************************/

static FAR struct inotify_watch_s *
inotify_get_watch_from_list(FAR struct inotify_device_s *dev,
                            FAR struct inotify_watch_list_s *list)
{
  FAR struct inotify_watch_s *watch;

  list_for_every_entry(&list->watches, watch, struct inotify_watch_s, l_node)
    {
      if (watch->dev == dev)
        {
          return watch;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: inotify_alloc_watch
 *
 * Description:
 *   Allocate a new inotify watch.
 *
 ****************************************************************************/

static FAR struct inotify_watch_s *
inotify_alloc_watch(FAR struct inotify_device_s *dev,
                    FAR struct inotify_watch_list_s *list,
                    uint32_t mask)
{
  FAR struct inotify_watch_s *watch;

  watch = fs_heap_zalloc(sizeof(struct inotify_watch_s));
  if (watch == NULL)
    {
      return NULL;
    }

  watch->dev = dev;
  watch->mask = mask;
  watch->wd = ++g_inotify.watch_cookie;
  watch->list = list;
  list_add_tail(&dev->watches, &watch->d_node);
  list_add_tail(&list->watches, &watch->l_node);
  return watch;
}

/****************************************************************************
 * Name: inotify_find_watch
 *
 * Description:
 *   Find the inotify watch from the watch descriptor.
 *
 ****************************************************************************/

static FAR struct inotify_watch_s *
inotify_find_watch(FAR struct inotify_device_s *dev, int wd)
{
  FAR struct inotify_watch_s *watch;

  list_for_every_entry(&dev->watches, watch, struct inotify_watch_s, d_node)
    {
      if (watch->wd == wd)
        {
          return watch;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: inotify_free_watch_list
 *
 * Description:
 *   Release a reference to the inotify node.
 *
 ****************************************************************************/

static void inotify_free_watch_list(FAR struct inotify_watch_list_s *list)
{
  FAR struct inotify_watch_s *watch;
  FAR struct inotify_device_s *dev;
  bool last_iteration = false;

  do
    {
      if (list_is_singular(&list->watches))
        {
          last_iteration = true;
        }

      watch = list_first_entry(&list->watches, struct inotify_watch_s,
                               l_node);
      dev = watch->dev;
      nxmutex_lock(&dev->lock);
      inotify_remove_watch(dev, watch);
      nxmutex_unlock(&dev->lock);
    }
  while (!last_iteration);
}

/****************************************************************************
 * Name: inotify_alloc_watch_list
 *
 * Description:
 *   Add the inotify node from the path.
 *
 ****************************************************************************/

static FAR struct inotify_watch_list_s *
inotify_alloc_watch_list(FAR const char *path)
{
  FAR struct inotify_watch_list_s *list;
  FAR ENTRY *result;
  ENTRY item;

  list = fs_heap_zalloc(sizeof(struct inotify_watch_list_s));
  if (list == NULL)
    {
      return NULL;
    }

  list_initialize(&list->watches);
  list->path = fs_heap_strdup(path);
  if (list->path == NULL)
    {
      fs_heap_free(list);
      return NULL;
    }

  item.key = list->path;
  item.data = list;
  if (hsearch_r(item, ENTER, &result, &g_inotify.hash) == 0)
    {
      fs_heap_free(list->path);
      fs_heap_free(list);
      return NULL;
    }

  return list;
}

/****************************************************************************
 * Name: inotify_get_watch_list
 *
 * Description:
 *   Get the watch list from the path.
 *
 ****************************************************************************/

static FAR struct inotify_watch_list_s *
inotify_get_watch_list(FAR const char *path)
{
  FAR ENTRY *result;
  ENTRY item;

  item.key = (FAR char *)path;
  if (hsearch_r(item, FIND, &result, &g_inotify.hash) == 0)
    {
      return NULL;
    }

  return (FAR struct inotify_watch_list_s *)result->data;
}

/****************************************************************************
 * Name: inotify_queue_watch_list_event
 *
 * Description:
 *   Queue an event to the watch list.
 *
 ****************************************************************************/

static void
inotify_queue_watch_list_event(FAR struct inotify_watch_list_s *list,
                               uint32_t mask, uint32_t cookie,
                               FAR const char *name)
{
  FAR struct inotify_watch_s *watch;
  FAR struct inotify_watch_s *w_tmp;

  list_for_every_entry_safe(&list->watches, watch, w_tmp,
                            struct inotify_watch_s, l_node)
    {
      uint32_t watch_mask = watch->mask;

      if (watch_mask & mask)
        {
          FAR struct inotify_device_s *dev = watch->dev;
          bool last_iteration = list_is_singular(&list->watches);

          nxmutex_lock(&dev->lock);
          inotify_queue_event(dev, watch->wd, mask, cookie, name);
          if (watch_mask & IN_ONESHOT)
            {
              inotify_remove_watch(dev, watch);
              if (last_iteration)
                {
                  nxmutex_unlock(&dev->lock);
                  return;
                }
            }

          nxmutex_unlock(&dev->lock);
        }
    }

  if (mask & IN_DELETE_SELF)
    {
      inotify_free_watch_list(list);
    }
}

/****************************************************************************
 * Name: inotify_queue_parent_event
 *
 * Description:
 *   Queue an event to the inotify inode.
 *
 ****************************************************************************/

static void inotify_queue_parent_event(FAR char *path, uint32_t mask,
                                       uint32_t cookie)
{
  FAR struct inotify_watch_list_s *list;
  FAR char *name;

  name = basename(path);
  if (name == NULL || name == path)
    {
      return;
    }

  *(name - 1) = '\0';
  list = inotify_get_watch_list(path);
  if (list != NULL)
    {
      inotify_queue_watch_list_event(list, mask | IN_ISDIR, cookie, name);
    }
}

/****************************************************************************
 * Name: notify_queue_path_event
 *
 * Description:
 *   Send the notification by the path.
 *
 ****************************************************************************/

static void notify_queue_path_event(FAR const char *path, uint32_t mask)
{
  FAR struct inotify_watch_list_s *list;
  FAR char *abspath;
  FAR char *pathbuffer;
  uint32_t cookie = 0;

  pathbuffer = lib_get_pathbuffer();
  if (pathbuffer == NULL)
    {
      return;
    }

  abspath = lib_realpath(path, pathbuffer, true);
  if (abspath == NULL)
    {
      lib_put_pathbuffer(pathbuffer);
      return;
    }

  if (mask & IN_MOVE)
    {
      if (mask & IN_MOVED_FROM)
        {
          ++g_inotify.event_cookie;
        }

      cookie = g_inotify.event_cookie;
    }

  list = inotify_get_watch_list(abspath);
  inotify_queue_parent_event(abspath, mask, cookie);
  lib_put_pathbuffer(pathbuffer);
  if (list == NULL)
    {
      return;
    }

  if (mask & IN_MOVED_FROM)
    {
      mask ^= IN_MOVED_FROM;
      mask |= IN_MOVE_SELF;
    }

  if (mask & IN_MOVED_TO)
    {
      mask ^= IN_MOVED_TO;
    }

  if (mask & IN_DELETE)
    {
      mask ^= IN_DELETE;
      mask |= IN_DELETE_SELF;
    }

  if (mask != 0)
    {
      inotify_queue_watch_list_event(list, mask, cookie, NULL);
    }
}

/****************************************************************************
 * Name: notify_check_inode
 *
 * Description:
 *   Check if the inode is a mount point.
 *
 ****************************************************************************/

static int notify_check_inode(FAR struct file *filep)
{
  FAR struct tcb_s *tcb = this_task();

  /* We only apply notify on mount points (f_inode won't be NULL). */

  if ((tcb->flags & TCB_FLAG_SIGNAL_ACTION) ||
      (!INODE_IS_MOUNTPT(filep->f_inode) &&
      !INODE_IS_PSEUDODIR(filep->f_inode) &&
      !INODE_IS_DRIVER(filep->f_inode)))
    {
      return -EBADF;
    }

  return OK;
}

/****************************************************************************
 * Name: notify_queue_filep_event
 *
 * Description:
 *   Send the notification by the file pointer.
 *
 ****************************************************************************/

static inline void notify_queue_filep_event(FAR struct file *filep,
                                            uint32_t mask)
{
  FAR char *pathbuffer;
  int ret;

  ret = notify_check_inode(filep);
  if (ret < 0)
    {
      return;
    }

  nxmutex_lock(&g_inotify.lock);
  ret = notify_check_mask(mask);
  nxmutex_unlock(&g_inotify.lock);
  if (ret < 0)
    {
      return;
    }

  pathbuffer = lib_get_pathbuffer();
  if (pathbuffer == NULL)
    {
      return;
    }

  ret = file_fcntl(filep, F_GETPATH, pathbuffer);
  if (ret < 0)
    {
      lib_put_pathbuffer(pathbuffer);
      return;
    }

  if (filep->f_oflags & O_DIRECTORY)
    {
      mask |= IN_ISDIR;
    }

  nxmutex_lock(&g_inotify.lock);
  notify_queue_path_event(pathbuffer, mask);
  lib_put_pathbuffer(pathbuffer);
  nxmutex_unlock(&g_inotify.lock);
}

/****************************************************************************
 * Name: notify_free_entry
 *
 * Description:
 *   Deallocate the hash entry.
 *
 ****************************************************************************/

static void notify_free_entry(FAR ENTRY *entry)
{
  /* Key is alloced by lib_malloc, value is alloced by fs_heap_malloc */

  fs_heap_free(entry->key);
  fs_heap_free(entry->data);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inotify_add_watch
 *
 * Description:
 *  Adds a new watch, or modifies an existing watch, for the file whose
 *  location is specified in pathname; The caller must have read permission
 *  for this file.  The fd argument is a file descriptor referring to the
 *  inotify instance whose watch list is to be modified.  The events to be
 *  monitored for pathname are specified in the mask bit-mask argument.
 *
 * Input Parameters:
 *  fd - The file descriptor associated with an instance of inotify.
 *  pathname - The path to the file to be monitored.
 *  mask - The bit mask of events to be monitored.
 *
 * Returned Value:
 *  On success, inotify_add_watch() returns a nonnegative watch descriptor.
 *  On error, -1 is returned and errno is set appropriately.
 *
 ****************************************************************************/

int inotify_add_watch(int fd, FAR const char *pathname, uint32_t mask)
{
  FAR struct inotify_watch_list_s *list;
  FAR struct inotify_watch_s *watch;
  FAR struct inotify_watch_s *old;
  FAR struct inotify_device_s *dev;
  FAR struct file *filep;
  FAR char *abspath;
  struct stat buf;
  int ret;

  if ((mask & IN_ALL_EVENTS) == 0)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  dev = inotify_get_device_from_fd(fd, &filep);
  if (dev == NULL)
    {
      set_errno(EBADF);
      return ERROR;
    }

  abspath = lib_realpath(pathname, NULL, mask & IN_DONT_FOLLOW);
  if (abspath == NULL)
    {
      fs_putfilep(filep);
      return ERROR;
    }

  ret = nx_stat(abspath, &buf, 1);
  if (ret < 0)
    {
      goto out_free;
    }

  if (!S_ISDIR(buf.st_mode) && (mask & IN_ONLYDIR))
    {
      ret = -ENOTDIR;
      goto out_free;
    }

  nxmutex_lock(&g_inotify.lock);
  nxmutex_lock(&dev->lock);
  list = inotify_get_watch_list(abspath);
  if (list == NULL)
    {
      list = inotify_alloc_watch_list(abspath);
      if (list == NULL)
        {
          ret = -ENOMEM;
          goto out;
        }
    }

  old = inotify_get_watch_from_list(dev, list);
  if (old != NULL)
    {
      uint32_t tmpmask = old->mask;
      if (mask & IN_MASK_CREATE)
        {
          ret = -EEXIST;
          goto out;
        }
      else if (mask & IN_MASK_ADD)
        {
          old->mask |= mask;
        }
      else
        {
          old->mask = mask;
        }

      ret = old->wd;
      inotify_add_count(tmpmask ^ old->mask);
    }
  else
    {
      watch = inotify_alloc_watch(dev, list, mask);
      if (watch == NULL && list_is_empty(&list->watches))
        {
          ENTRY item;
          item.key = list->path;
          hsearch_r(item, DELETE, NULL, &g_inotify.hash);
          ret = -ENOMEM;
          goto out;
        }

      ret = watch->wd;
      inotify_add_count(mask);
    }

out:
  nxmutex_unlock(&dev->lock);
  nxmutex_unlock(&g_inotify.lock);

out_free:
  fs_putfilep(filep);
  fs_heap_free(abspath);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: inotify_rm_watch
 *
 * Description:
 *  Removes the watch associated with the watch descriptor wd from the
 *  inotify instance associated with the file descriptor fd.
 *
 * Input Parameters:
 *  fd - The file descriptor associated with an instance of inotify.
 *  wd - The watch descriptor to be removed.
 *
 * Returned Value:
 *  On success, inotify_rm_watch() returns zero.  On error, -1 is returned
 *  and errno is set appropriately.
 *
 ****************************************************************************/

int inotify_rm_watch(int fd, int wd)
{
  FAR struct inotify_device_s *dev;
  FAR struct inotify_watch_s *watch;
  FAR struct file *filep;

  dev = inotify_get_device_from_fd(fd, &filep);
  if (dev == NULL)
    {
      set_errno(EBADF);
      return ERROR;
    }

  nxmutex_lock(&g_inotify.lock);
  nxmutex_lock(&dev->lock);
  watch = inotify_find_watch(dev, wd);
  if (watch == NULL)
    {
      nxmutex_unlock(&dev->lock);
      nxmutex_unlock(&g_inotify.lock);
      fs_putfilep(filep);
      set_errno(EINVAL);
      return ERROR;
    }

  inotify_remove_watch(dev, watch);
  nxmutex_unlock(&dev->lock);
  nxmutex_unlock(&g_inotify.lock);
  fs_putfilep(filep);
  return OK;
}

/****************************************************************************
 * Name: inotify_init1
 *
 * Description:
 *  Initializes a new inotify instance and returns a file descriptor
 *  associated with a new inotify event queue.
 *
 * Input Parameters:
 *  flags - The following values are recognized in flags:
 *    IN_NONBLOCK - Set the O_NONBLOCK file status flag on the new open file
 *      description. Using this flag saves extra calls to fcntl(2) to achieve
 *      the same result.
 *    IN_CLOEXEC - Set the close-on-exec (FD_CLOEXEC) flag on the new file
 *      descriptor. See the description of the O_CLOEXEC flag in open(2) for
 *      reasons why this may be useful.
 *
 * Returned Value:
 *  On success, these system calls return a new file descriptor.
 *  On error, -1 is returned and errno is set appropriately.
 *
 ****************************************************************************/

int inotify_init1(int flags)
{
  FAR struct inotify_device_s *dev;
  int ret;

  if ((flags & ~(IN_NONBLOCK | IN_CLOEXEC)) != 0)
    {
      ret = -EINVAL;
      goto exit_set_errno;
    }

  dev = inotify_alloc_device();
  if (dev == NULL)
    {
      ferr("Failed to allocate inotify device\n");
      ret = -ENOMEM;
      goto exit_set_errno;
    }

  ret = file_allocate(&g_inotify_inode, O_RDOK | flags,
                      0, dev, 0, true);
  if (ret < 0)
    {
      ferr("Failed to allocate inotify fd\n");
      goto exit_with_dev;
    }

  return ret;

exit_with_dev:
  nxmutex_destroy(&dev->lock);
  nxsem_destroy(&dev->sem);
  fs_heap_free(dev);
exit_set_errno:
  set_errno(-ret);
  return ERROR;
}

/****************************************************************************
 * Name: inotify_init
 *
 * Description:
 *  Initializes a new inotify instance and returns a file descriptor
 *  associated with a new inotify event queue.
 *
 * Returned Value:
 *  On success, these system calls return a new file descriptor.
 *  On error, -1 is returned and errno is set appropriately.
 *
 ****************************************************************************/

int inotify_init(void)
{
  return inotify_init1(0);
}

/****************************************************************************
 * Name: notify_initialize
 *
 * Description:
 *  Initializes a new inotify root node.
 *
 ****************************************************************************/

void notify_initialize(void)
{
  int ret;

  g_inotify.hash.free_entry = notify_free_entry;
  ret = hcreate_r(CONFIG_FS_NOTIFY_BUCKET_SIZE, &g_inotify.hash);
  if (ret != 1)
    {
      ferr("Failed to create hash table\n");
    }
}

/****************************************************************************
 * Name: notify_open
 *
 * Description:
 *   The hook is called when the file is opened.
 *
 ****************************************************************************/

void notify_open(FAR const char *path, int oflags)
{
  uint32_t mask = IN_OPEN;

  if (oflags & O_DIRECTORY)
    {
      mask |= IN_ISDIR;
    }

  if (oflags & O_CREAT)
    {
      mask |= IN_CREATE;
    }

  nxmutex_lock(&g_inotify.lock);
  notify_queue_path_event(path, mask);
  nxmutex_unlock(&g_inotify.lock);
}

/****************************************************************************
 * Name: notify_close
 *
 * Description:
 *   The hook is called when the file is closed.
 *
 ****************************************************************************/

void notify_close(FAR const char *path, int oflags)
{
  if (oflags & O_WROK)
    {
      notify_queue_path_event(path, IN_CLOSE_WRITE);
    }
  else
    {
      notify_queue_path_event(path, IN_CLOSE_NOWRITE);
    }
}

/****************************************************************************
 * Name: notify_close2
 *
 * Description:
 *   The hook is called when the file is closed.
 *
 ****************************************************************************/

void notify_close2(FAR struct inode *inode)
{
  FAR char *pathbuffer;

  pathbuffer = lib_get_pathbuffer();
  if (pathbuffer == NULL)
    {
      return;
    }

  nxmutex_lock(&g_inotify.lock);
  if (inode_getpath(inode, pathbuffer, PATH_MAX) >= 0)
    {
      notify_queue_path_event(pathbuffer, IN_CLOSE_WRITE);
    }

  lib_put_pathbuffer(pathbuffer);
  nxmutex_unlock(&g_inotify.lock);
}

/****************************************************************************
 * Name: notify_read
 *
 * Description:
 *   The hook is called when the file is read.
 *
 ****************************************************************************/

void notify_read(FAR struct file *filep)
{
  notify_queue_filep_event(filep, IN_ACCESS);
}

/****************************************************************************
 * Name: notify_write
 *
 * Description:
 *   The hook is called when the file is written.
 *
 ****************************************************************************/

void notify_write(FAR struct file *filep)
{
  notify_queue_filep_event(filep, IN_MODIFY);
}

/****************************************************************************
 * Name: notify_chstat
 *
 * Description:
 *   The hook is called when the file attribute is changed.
 *
 ****************************************************************************/

void notify_chstat(FAR struct file *filep)
{
  notify_queue_filep_event(filep, IN_ATTRIB);
}

/****************************************************************************
 * Name: notify_unlink
 *
 * Description:
 *   The hook is called when the file is unlinked.
 *
 ****************************************************************************/

void notify_unlink(FAR const char *path)
{
  nxmutex_lock(&g_inotify.lock);
  notify_queue_path_event(path, IN_DELETE);
  nxmutex_unlock(&g_inotify.lock);
}

/****************************************************************************
 * Name: notify_unmount
 *
 * Description:
 *   The hook is called when the file system is unmounted.
 *
 ****************************************************************************/

void notify_unmount(FAR const char *path)
{
  nxmutex_lock(&g_inotify.lock);
  notify_queue_path_event(path, IN_DELETE | IN_UNMOUNT);
  nxmutex_unlock(&g_inotify.lock);
}

/****************************************************************************
 * Name: notify_mkdir
 *
 * Description:
 *   The hook is called when the directory is created.
 *
 ****************************************************************************/

void notify_mkdir(FAR const char *path)
{
  nxmutex_lock(&g_inotify.lock);
  notify_queue_path_event(path, IN_CREATE | IN_ISDIR);
  nxmutex_unlock(&g_inotify.lock);
}

/****************************************************************************
 * Name: notify_create
 *
 * Description:
 *   The hook is called symlink is created.
 *
 ****************************************************************************/

void notify_create(FAR const char *path)
{
  nxmutex_lock(&g_inotify.lock);
  notify_queue_path_event(path, IN_CREATE);
  nxmutex_unlock(&g_inotify.lock);
}

/****************************************************************************
 * Name: notify_rename
 *
 * Description:
 *   The hook is called when the file is moved.
 *
 ****************************************************************************/

void notify_rename(FAR const char *oldpath, bool oldisdir,
                   FAR const char *newpath, bool newisdir)
{
  uint32_t newmask = IN_MOVED_TO;
  uint32_t oldmask = IN_MOVED_FROM;

  if (newisdir)
    {
      newmask |= IN_ISDIR;
    }

  if (oldisdir)
    {
      oldmask |= IN_ISDIR;
    }

  nxmutex_lock(&g_inotify.lock);
  notify_queue_path_event(oldpath, oldmask);
  notify_queue_path_event(newpath, newmask);
  nxmutex_unlock(&g_inotify.lock);
}
