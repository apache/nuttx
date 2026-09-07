/****************************************************************************
 * fs/vfs/fs_inotify.c
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
#include <nuttx/nuttx.h>
#include <nuttx/fs/fs.h>
#include <nuttx/lib/lib.h>

#include <sys/inotify.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <nuttx/debug.h>
#include <poll.h>
#include <string.h>
#include <search.h>
#include <libgen.h>

#include "inode/inode.h"
#include "sched/sched.h"
#include "fs_heap.h"

/* Event classes a watch can register for, IN_UNMOUNT being the highest */

#define INOTIFY_CLASSES (IN_ALL_EVENTS | IN_UNMOUNT)
#define INOTIFY_NCLASS  14

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A list with a file system watch handle receives its events from the
 * file system through notify_event(); locally generated events are
 * suppressed on it to avoid duplicates.
 */

#ifndef CONFIG_DISABLE_MOUNTPOINT
#  define NOTIFY_IS_REMOTE(list) ((list)->mwd >= 0)
#else
#  define NOTIFY_IS_REMOTE(list) false
#endif

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
  FAR struct pollfd_s *fds[CONFIG_FS_NOTIFY_FD_POLLWAITERS];

  /* An in-kernel instance delivers events through this callback instead
   * of the event queue (see file_notify_open).
   */

  inotify_callback_t callback;
  FAR void          *arg;
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
#ifndef CONFIG_DISABLE_MOUNTPOINT
  int              mwd;      /* File system watch handle, -1 if none */
#endif
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

  /* Number of watches registered for each IN_* bit */

  uint32_t count[INOTIFY_NCLASS];
  struct   hsearch_data hash;  /* Hash table for watch lists */
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int     inotify_open(FAR struct file *filep);
static int     inotify_close(FAR struct file *filep);
static ssize_t inotify_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
#ifdef CONFIG_FS_NOTIFY_INJECT
static ssize_t inotify_write(FAR struct file *filep,
                             FAR const char *buffer, size_t buflen);
#endif
static int     inotify_poll(FAR struct file *filep, FAR struct pollfd_s *fds,
                            bool setup);
static int     inotify_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);
#ifndef CONFIG_DISABLE_MOUNTPOINT
static FAR struct inotify_watch_list_s *
               inotify_get_watch_list(FAR const char *path);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_inotify_fops =
{
  inotify_open,     /* open */
  inotify_close,    /* close */
  inotify_read,     /* read */
#ifdef CONFIG_FS_NOTIFY_INJECT
  inotify_write,    /* write */
#else
  NULL,             /* write */
#endif
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
  int i;

  mask &= INOTIFY_CLASSES;
  for (i = 0; mask != 0; i++, mask >>= 1)
    {
      g_inotify.count[i] += mask & 1;
    }
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
  int i;

  mask &= INOTIFY_CLASSES;
  for (i = 0; mask != 0; i++, mask >>= 1)
    {
      g_inotify.count[i] -= mask & 1;
    }
}

/****************************************************************************
 * Name: notify_check_mask
 *
 * Description:
 *   Return OK if some watch may receive an event of this mask, -EBADF if
 *   none can.
 *
 ****************************************************************************/

static int notify_check_mask(uint32_t mask)
{
  int i;

  /* IN_MOVED_FROM also reaches IN_MOVE_SELF watches.  IN_DELETE frees
   * the watch list of the deleted path whatever its watches asked for,
   * so it matters as long as any watch exists.
   */

  if (mask & IN_MOVED_FROM)
    {
      mask |= IN_MOVE_SELF;
    }

  if (mask & IN_DELETE)
    {
      mask |= INOTIFY_CLASSES;
    }

  mask &= INOTIFY_CLASSES;
  for (i = 0; mask != 0; i++, mask >>= 1)
    {
      if ((mask & 1) && g_inotify.count[i] != 0)
        {
          return OK;
        }
    }

  return -EBADF;
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
      len = ALIGN_UP(strlen(name) + 1, sizeof(struct inotify_event));
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

  /* In-kernel instance: hand the event to the callback and return */

  if (dev->callback != NULL)
    {
      dev->callback(dev->arg, wd, mask, name);
      return;
    }

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

  if (nxsem_get_value(&dev->sem, &semcnt) >= 0)
    {
      while (semcnt++ <= 1)
        {
          nxsem_post(&dev->sem);
        }
    }
}

/****************************************************************************
 * Name: inotify_notify_fs
 *
 * Description:
 *   Look up the file system backing path and invoke its notify method.
 *   Returns the method result, or -ENODEV when path is not on a
 *   mountpoint with a notify method.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_MOUNTPOINT
static int inotify_notify_fs(FAR const char *path, int action, int wd,
                             uint32_t mask)
{
  struct inode_search_s desc;
  FAR struct inode *inode;
  int ret;

  ret = inode_search_setup(&desc, path, false);
  if (ret < 0)
    {
      return ret;
    }

  ret = -ENODEV;
  if (inode_find(&desc, &inode) >= 0)
    {
      if (INODE_IS_MOUNTPT(inode) && inode->u.i_mops->notify != NULL)
        {
          ret = inode->u.i_mops->notify(inode, wd, desc.relpath != NULL ?
                                        desc.relpath : "", action, mask);
        }

      inode_release(inode);
    }

  inode_search_release(&desc);
  return ret;
}

/****************************************************************************
 * Name: inotify_update_watch
 *
 * Description:
 *   Add or remove the file system watch that backs abspath.  Must be
 *   called WITHOUT the inotify locks: the file system method may wait
 *   on a local server whose own file operations take them.
 *
 *   For NOTIFY_WATCH_ADD the file system is (re)told the union of all
 *   watch masks on the path so a later broader watch widens the file
 *   system watch; the file system returns the same handle for a path it
 *   already watches (see the notify method contract).  The list is
 *   revalidated before the handle is stored; if the watch disappeared
 *   meanwhile the file system watch is removed again.
 *
 ****************************************************************************/

static void inotify_update_watch(FAR struct inotify_watch_list_s *list,
                                 FAR const char *abspath, int action)
{
  FAR struct inotify_watch_s *watch;
  uint32_t mask = 0;
  bool present;
  int wd;

  if (action == NOTIFY_WATCH_RM)
    {
      if (list->mwd >= 0)
        {
          inotify_notify_fs(abspath, NOTIFY_WATCH_RM, list->mwd, 0);
        }

      return;
    }

  /* Compute the union of the watch masks (the list may have gone away) */

  nxmutex_lock(&g_inotify.lock);
  present = inotify_get_watch_list(abspath) == list;
  if (present)
    {
      list_for_every_entry(&list->watches, watch, struct inotify_watch_s,
                           l_node)
        {
          mask |= watch->mask;
        }
    }

  nxmutex_unlock(&g_inotify.lock);
  if (!present || mask == 0)
    {
      return;
    }

  wd = inotify_notify_fs(abspath, NOTIFY_WATCH_ADD, -1, mask);
  if (wd < 0)
    {
      return;
    }

  /* Store the handle if the watch list still exists; undo otherwise */

  nxmutex_lock(&g_inotify.lock);
  present = inotify_get_watch_list(abspath) == list;
  if (present)
    {
      list->mwd = wd;
    }

  nxmutex_unlock(&g_inotify.lock);
  if (!present)
    {
      inotify_notify_fs(abspath, NOTIFY_WATCH_RM, wd, 0);
    }
}
#else
#  define inotify_update_watch(list, abspath, action)
#endif

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

      inotify_update_watch(list, list->path, NOTIFY_WATCH_RM);
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
 * Name: inotify_write
 *
 * Description:
 *   Inject one event per write of a struct notify_inject_s payload
 *   (event mask + NUL terminated absolute path).
 *
 ****************************************************************************/

#ifdef CONFIG_FS_NOTIFY_INJECT
static ssize_t inotify_write(FAR struct file *filep,
                             FAR const char *buffer, size_t buflen)
{
  FAR const struct notify_inject_s *event;
  size_t hdrlen = offsetof(struct notify_inject_s, path);
  size_t pathlen;

  event = (FAR const struct notify_inject_s *)buffer;
  if (event == NULL || buflen <= hdrlen || buflen > hdrlen + PATH_MAX)
    {
      return -EINVAL;
    }

  pathlen = strnlen(event->path, buflen - hdrlen);
  if (pathlen == buflen - hdrlen || event->path[0] != '/')
    {
      /* Not NUL terminated within the write, or not an absolute path */

      return -EINVAL;
    }

  notify_event(NULL, event->path, event->mask);
  return buflen;
}
#endif

/****************************************************************************
 * Name: inotify_poll
 *
 * Description:
 *   Poll the inotify device.
 *
 ****************************************************************************/

static int inotify_poll(FAR struct file *filep, FAR struct pollfd_s *fds,
                        bool setup)
{
  FAR struct inotify_device_s *dev = filep->f_priv;
  int ret = 0;
  int i;

  nxmutex_lock(&dev->lock);
  if (!setup)
    {
      FAR struct pollfd_s **slot = fds->priv;

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
#ifndef CONFIG_DISABLE_MOUNTPOINT
  list->mwd = -1;
#endif
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
                                       uint32_t cookie, bool remote)
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
  if (list != NULL && (remote || !NOTIFY_IS_REMOTE(list)))
    {
      inotify_queue_watch_list_event(list, mask | IN_ISDIR, cookie, name);
    }
}

/****************************************************************************
 * Name: notify_queue_path_event2
 *
 * Description:
 *   Send the notification by the path.  remote marks an event injected
 *   by the file system through notify_event().
 *
 ****************************************************************************/

static void notify_queue_path_event2(FAR const char *path, uint32_t mask,
                                     bool remote)
{
  FAR struct inotify_watch_list_s *list;
  FAR char *abspath;
  FAR char *pathbuffer;
  uint32_t cookie = 0;

  /* Advance the cookie even if nobody watches IN_MOVED_FROM, so that the
   * IN_MOVED_TO of each rename still carries its own cookie.
   */

  if (mask & IN_MOVE)
    {
      if (mask & IN_MOVED_FROM)
        {
          ++g_inotify.event_cookie;
        }

      cookie = g_inotify.event_cookie;
    }

  if (notify_check_mask(mask) < 0)
    {
      return;
    }

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

  list = inotify_get_watch_list(abspath);
  inotify_queue_parent_event(abspath, mask, cookie, remote);
  lib_put_pathbuffer(pathbuffer);
  if (list == NULL || (!remote && NOTIFY_IS_REMOTE(list)))
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
 * Name: notify_queue_path_event
 *
 * Description:
 *   Send the notification for a locally generated event.
 *
 ****************************************************************************/

static void notify_queue_path_event(FAR const char *path, uint32_t mask)
{
  notify_queue_path_event2(path, mask, false);
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

  /* Checked without g_inotify.lock so that readers and writers do not
   * queue behind a path event holding it.  The counters are aligned
   * uint32_t loads; racing a concurrent watch add/remove is the same
   * race as the one between this check and the lock taken below.
   */

  if (notify_check_mask(mask) < 0)
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
  /* Key is allocated by lib_malloc, value is allocated by fs_heap_malloc */

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
  FAR struct file *filep;
  int ret;

  ret = file_get(fd, &filep);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  if (filep->f_inode != &g_inotify_inode)
    {
      file_put(filep);
      set_errno(EBADF);
      return ERROR;
    }

  ret = file_notify_add_watch(filep, pathname, mask);
  file_put(filep);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
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
  FAR struct file *filep;
  int ret;

  ret = file_get(fd, &filep);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  if (filep->f_inode != &g_inotify_inode)
    {
      file_put(filep);
      set_errno(EBADF);
      return ERROR;
    }

  ret = file_notify_rm_watch(filep, wd);
  file_put(filep);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

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

#ifdef CONFIG_FS_NOTIFY_INJECT
  ret = file_allocate_from_inode(&g_inotify_inode, O_RDWR | flags,
                                 0, dev, 0);
#else
  ret = file_allocate_from_inode(&g_inotify_inode, O_RDONLY | flags,
                                 0, dev, 0);
#endif
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
 * Name: file_notify_open
 *
 * Description:
 *   Open an in-kernel inotify instance.  Instead of queueing events for
 *   a reader, the instance delivers each event to callback in the
 *   context that generated it (so the callback must not block).  The
 *   instance is closed with file_notify_close().
 *
 * Input Parameters:
 *   filep    - Caller provided file structure to initialize
 *   callback - Invoked for every event on a watched path
 *   arg      - Opaque argument passed to callback
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int file_notify_open(FAR struct file *filep, inotify_callback_t callback,
                     FAR void *arg)
{
  FAR struct inotify_device_s *dev;

  DEBUGASSERT(filep != NULL && callback != NULL);

  dev = inotify_alloc_device();
  if (dev == NULL)
    {
      return -ENOMEM;
    }

  dev->callback = callback;
  dev->arg      = arg;

  memset(filep, 0, sizeof(*filep));
  inode_addref(&g_inotify_inode);
  filep->f_inode  = &g_inotify_inode;
  filep->f_oflags = O_RDONLY;
  filep->f_priv   = dev;
  return OK;
}

/****************************************************************************
 * Name: file_notify_add_watch
 *
 * Description:
 *   Add a watch to an in-kernel inotify instance (see file_notify_open).
 *
 * Returned Value:
 *   A nonnegative watch descriptor on success; a negated errno value on
 *   failure.
 *
 ****************************************************************************/

int file_notify_add_watch(FAR struct file *filep, FAR const char *pathname,
                          uint32_t mask)
{
  FAR struct inotify_device_s *dev;
  FAR struct inotify_watch_list_s *list;
  FAR struct inotify_watch_s *watch;
  FAR struct inotify_watch_s *old;
  FAR char *tmpbuffer;
  FAR char *abspath;
  struct stat buf;
  int ret;

  DEBUGASSERT(filep != NULL && filep->f_priv != NULL);
  dev = filep->f_priv;

  if ((mask & IN_ALL_EVENTS) == 0)
    {
      return -EINVAL;
    }

  /* Allocate temporary buffer for realpath */

  tmpbuffer = lib_get_tempbuffer(PATH_MAX);
  if (tmpbuffer == NULL)
    {
      return -ENOMEM;
    }

  abspath = lib_realpath(pathname, tmpbuffer, mask & IN_DONT_FOLLOW);
  if (abspath == NULL)
    {
      ret = -get_errno();
      goto out_free;
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
      inotify_add_count(old->mask);
      inotify_sub_count(tmpmask);
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

  /* Establish the file system watch (idempotent) without the locks
   * held (see inotify_update_watch).
   */

  if (ret >= 0)
    {
      inotify_update_watch(list, abspath, NOTIFY_WATCH_ADD);
    }

out_free:
  lib_put_tempbuffer(tmpbuffer);
  return ret;
}

/****************************************************************************
 * Name: file_notify_rm_watch
 *
 * Description:
 *   Remove a watch from an in-kernel inotify instance.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int file_notify_rm_watch(FAR struct file *filep, int wd)
{
  FAR struct inotify_device_s *dev;
  FAR struct inotify_watch_s *watch;

  DEBUGASSERT(filep != NULL && filep->f_priv != NULL);
  dev = filep->f_priv;

  nxmutex_lock(&g_inotify.lock);
  nxmutex_lock(&dev->lock);
  watch = inotify_find_watch(dev, wd);
  if (watch == NULL)
    {
      nxmutex_unlock(&dev->lock);
      nxmutex_unlock(&g_inotify.lock);
      return -EINVAL;
    }

  inotify_remove_watch(dev, watch);
  nxmutex_unlock(&dev->lock);
  nxmutex_unlock(&g_inotify.lock);
  return OK;
}

/****************************************************************************
 * Name: file_notify_close
 *
 * Description:
 *   Close an in-kernel inotify instance and remove all its watches.
 *
 ****************************************************************************/

void file_notify_close(FAR struct file *filep)
{
  DEBUGASSERT(filep != NULL);
  inotify_close(filep);
  inode_release(filep->f_inode);
  filep->f_inode = NULL;
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
  if ((oflags & O_ACCMODE) != O_RDONLY)
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

/****************************************************************************
 * Name: notify_event
 *
 * Description:
 *   Inject a file change notification that originated outside the local
 *   VFS.  Must be called from thread context.
 *
 ****************************************************************************/

void notify_event(FAR struct inode *mountpt, FAR const char *relpath,
                  uint32_t mask)
{
  FAR char *pathbuffer;

  if (relpath == NULL)
    {
      return;
    }

  pathbuffer = lib_get_pathbuffer();
  if (pathbuffer == NULL)
    {
      return;
    }

  if (mountpt != NULL)
    {
      if (inode_getpath(mountpt, pathbuffer, PATH_MAX) < 0)
        {
          lib_put_pathbuffer(pathbuffer);
          return;
        }

      /* inode_getpath() terminates a mountpoint path with '/' */

      while (*relpath == '/')
        {
          relpath++;
        }

      strlcat(pathbuffer, relpath, PATH_MAX);
    }
  else
    {
      strlcpy(pathbuffer, relpath, PATH_MAX);
    }

  nxmutex_lock(&g_inotify.lock);
  notify_queue_path_event2(pathbuffer, mask, true);
  nxmutex_unlock(&g_inotify.lock);
  lib_put_pathbuffer(pathbuffer);
}
