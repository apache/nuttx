/****************************************************************************
 * fs/procfs/fs_procfspressure.c
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

#include <debug.h>
#include <errno.h>
#include <poll.h>
#include <stdint.h>
#include <string.h>
#include <sys/stat.h>

#include <nuttx/fs/procfs.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/queue.h>
#include <nuttx/spinlock.h>

#include "fs_heap.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct pressure_file_s
{
  struct procfs_file_s base;        /* Base open file structure */
  dq_entry_t entry;                 /* Supports a linked list */
  FAR struct pollfd *fds;           /* Polling structure of waiting thread */
  size_t threshold;                 /* Memory notification threshold */
  clock_t lasttick;                 /* Last time notified */
  clock_t interval;                 /* Notification interval in us */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static dq_queue_t g_pressure_memory_queue;
static spinlock_t g_pressure_lock;
static size_t g_remaining;
static size_t g_largest;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     pressure_open(FAR struct file *filep, FAR const char *relpath,
                             int oflags, mode_t mode);
static int     pressure_close(FAR struct file *filep);
static ssize_t pressure_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t pressure_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen);
static int     pressure_poll(FAR struct file *filep, FAR struct pollfd *fds,
                             bool setup);
static int     pressure_dup(FAR const struct file *oldp,
                            FAR struct file *newp);
static int     pressure_opendir(FAR const char *relpath,
                                FAR struct fs_dirent_s **dir);
static int     pressure_closedir(FAR struct fs_dirent_s *dir);
static int     pressure_readdir(FAR struct fs_dirent_s *dir,
                                FAR struct dirent *entry);
static int     pressure_rewinddir(FAR struct fs_dirent_s *dir);
static int     pressure_stat(FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct procfs_operations g_pressure_operations =
{
  pressure_open,       /* open */
  pressure_close,      /* close */
  pressure_read,       /* read */
  pressure_write,      /* write */
  pressure_poll,       /* poll */
  pressure_dup,        /* dup */
  pressure_opendir,    /* opendir */
  pressure_closedir,   /* closedir */
  pressure_readdir,    /* readdir */
  pressure_rewinddir,  /* rewinddir */
  pressure_stat        /* stat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pressure_open
 ****************************************************************************/

static int pressure_open(FAR struct file *filep, FAR const char *relpath,
                         int oflags, mode_t mode)
{
  FAR struct pressure_file_s *priv;
  uint32_t flags;

  if (strcmp(relpath, "pressure/memory") != 0)
    {
      ferr("ERROR: relpath is invalid: %s\n", relpath);
      return -ENOENT;
    }

  priv = fs_heap_zalloc(sizeof(struct pressure_file_s));
  if (!priv)
    {
      return -ENOMEM;
    }

  flags = spin_lock_irqsave(&g_pressure_lock);
  priv->interval = CLOCK_MAX;
  filep->f_priv = priv;
  dq_addfirst(&priv->entry, &g_pressure_memory_queue);
  spin_unlock_irqrestore(&g_pressure_lock, flags);
  return OK;
}

/****************************************************************************
 * Name: pressure_close
 ****************************************************************************/

static int pressure_close(FAR struct file *filep)
{
  FAR struct pressure_file_s *priv = filep->f_priv;
  uint32_t flags;

  flags = spin_lock_irqsave(&g_pressure_lock);
  dq_rem(&priv->entry, &g_pressure_memory_queue);
  spin_unlock_irqrestore(&g_pressure_lock, flags);
  fs_heap_free(priv);
  return OK;
}

/****************************************************************************
 * Name: pressure_read
 ****************************************************************************/

static ssize_t pressure_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  char buf[128];
  uint32_t flags;
  size_t remain;
  size_t largest;
  off_t offset;
  ssize_t ret;

  flags   = spin_lock_irqsave(&g_pressure_lock);
  remain  = g_remaining;
  largest = g_largest;
  spin_unlock_irqrestore(&g_pressure_lock, flags);

  ret = procfs_snprintf(buf, sizeof(buf), "remaining %zu, largest:%zu\n",
                        remain, largest);

  if (ret > buflen)
    {
      return -ENOMEM;
    }

  offset = filep->f_pos;
  ret    = procfs_memcpy(buf, ret, buffer, buflen, &offset);

  filep->f_pos += ret;
  return ret;
}

/****************************************************************************
 * Name: pressure_write
 ****************************************************************************/

static ssize_t pressure_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
  FAR struct pressure_file_s *priv = filep->f_priv;
  FAR char *endptr;
  size_t threshold;
  clock_t interval;
  uint32_t flags;

  if (buffer == NULL)
    {
      return -EINVAL;
    }

  threshold = strtoul(buffer, &endptr, 0);
  if (threshold == 0)
    {
      return -EINVAL;
    }

  interval = strtol(endptr + 1, NULL, 0);

  /* Check if the interval is valid, -1 means only notify once */

  if (interval == -1)
    {
      interval = CLOCK_MAX;
    }
  else
    {
      interval = USEC2TICK(interval);
    }

  flags = spin_lock_irqsave(&g_pressure_lock);

  /* We should trigger the first event immediately */

  priv->lasttick  = CLOCK_MAX;
  priv->threshold = threshold;
  priv->interval  = interval;
  spin_unlock_irqrestore(&g_pressure_lock, flags);
  return buflen;
}

/****************************************************************************
 * Name: pressure_poll
 ****************************************************************************/

static int pressure_poll(FAR struct file *filep, FAR struct pollfd *fds,
                         bool setup)
{
  FAR struct pressure_file_s *priv = filep->f_priv;
  clock_t current = clock_systime_ticks();
  uint32_t flags;

  flags = spin_lock_irqsave(&g_pressure_lock);
  if (setup)
    {
      if (priv->fds == NULL)
        {
          priv->fds = fds;
          fds->priv = &priv->fds;

          /* If the remaining memory is less than the threshold and
           * lasttick is CLOCK_MAX, it means the event is triggered for
           * the first time and we should always send a notification.
           */

          if (g_remaining <= priv->threshold && (priv->lasttick ==
              CLOCK_MAX || current - priv->lasttick >= priv->interval))
            {
              priv->lasttick = current;
              spin_unlock_irqrestore(&g_pressure_lock, flags);
              poll_notify(&priv->fds, 1, POLLPRI);
              return OK;
            }
        }
      else
        {
          spin_unlock_irqrestore(&g_pressure_lock, flags);
          return -EBUSY;
        }
    }
  else if (fds->priv)
    {
      priv->fds = NULL;
      fds->priv = NULL;
    }

  spin_unlock_irqrestore(&g_pressure_lock, flags);
  return OK;
}

/****************************************************************************
 * Name: pressure_dup
 ****************************************************************************/

static int pressure_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct pressure_file_s *oldpriv = oldp->f_priv;
  FAR struct pressure_file_s *newpriv;
  uint32_t flags;

  newpriv = fs_heap_zalloc(sizeof(struct pressure_file_s));
  if (newpriv == NULL)
    {
      return -ENOMEM;
    }

  flags = spin_lock_irqsave(&g_pressure_lock);
  memcpy(newpriv, oldpriv, sizeof(struct pressure_file_s));
  dq_addfirst(&newpriv->entry, &g_pressure_memory_queue);
  newpriv->fds = NULL;
  newp->f_priv = newpriv;
  spin_unlock_irqrestore(&g_pressure_lock, flags);
  return OK;
}

/****************************************************************************
 * Name: pressure_opendir
 ****************************************************************************/

static int pressure_opendir(FAR const char *relpath,
                            FAR struct fs_dirent_s **dir)
{
  FAR struct procfs_dir_priv_s *level;

  finfo("relpath: \"%s\"\n", relpath ? relpath : "NULL");
  DEBUGASSERT(relpath);

  level = fs_heap_zalloc(sizeof(struct procfs_dir_priv_s));
  if (level == NULL)
    {
      return -ENOMEM;
    }

  level->level    = 1;
  level->nentries = 1;

  *dir = (FAR struct fs_dirent_s *)level;
  return OK;
}

/****************************************************************************
 * Name: pressure_closedir
 ****************************************************************************/

static int pressure_closedir(FAR struct fs_dirent_s *dir)
{
  DEBUGASSERT(dir);
  fs_heap_free(dir);
  return OK;
}

/****************************************************************************
 * Name: pressure_readdir
 ****************************************************************************/

static int pressure_readdir(FAR struct fs_dirent_s *dir,
                            FAR struct dirent *entry)
{
  FAR struct procfs_dir_priv_s *level;

  DEBUGASSERT(dir);
  level = (FAR struct procfs_dir_priv_s *)dir;
  if (level->index >= level->nentries)
    {
      finfo("No more entries\n");
      return -ENOENT;
    }

  entry->d_type = DTYPE_FILE;
  strncpy(entry->d_name, "memory", sizeof(entry->d_name));
  level->index++;
  return OK;
}

/****************************************************************************
 * Name: pressure_rewinddir
 ****************************************************************************/

static int pressure_rewinddir(FAR struct fs_dirent_s *dir)
{
  FAR struct procfs_dir_priv_s *level;

  DEBUGASSERT(dir);
  level        = (FAR struct procfs_dir_priv_s *)dir;
  level->index = 0;
  return OK;
}

/****************************************************************************
 * Name: pressure_stat
 ****************************************************************************/

static int pressure_stat(const char *relpath, struct stat *buf)
{
  memset(buf, 0, sizeof(struct stat));

  if (strcmp(relpath, "pressure") == 0 || strcmp(relpath, "pressure/") == 0)
    {
      buf->st_mode = S_IFDIR | S_IROTH | S_IRGRP | S_IRUSR;
    }
  else if (strcmp(relpath, "pressure/memory") == 0)
    {
      buf->st_mode = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR | S_IWOTH |
                     S_IWGRP | S_IWUSR;
    }
  else
    {
      ferr("ERROR: No such file or directory: %s\n", relpath);
      return -ENOENT;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * mm_notify_pressure
 ****************************************************************************/

void mm_notify_pressure(size_t remaining, size_t largest)
{
  clock_t current = clock_systime_ticks();
  FAR dq_entry_t *entry;
  FAR dq_entry_t *tmp;
  uint32_t flags;

  flags       = spin_lock_irqsave(&g_pressure_lock);
  g_remaining = remaining;
  g_largest   = largest;

  dq_for_every_safe(&g_pressure_memory_queue, entry, tmp)
    {
      FAR struct pressure_file_s *pressure =
          container_of(entry, struct pressure_file_s, entry);

      /* If the largest available block is less than the threshold,
       * send a notification
       */

      if (largest > pressure->threshold)
        {
          continue;
        }

      /* If lasttick is CLOCK_MAX, it means that the event is triggered
       * for the first time and we should always send notifications.
       */

      if (pressure->lasttick != CLOCK_MAX && current - pressure->lasttick <
          pressure->interval)
        {
          continue;
        }

      /* If fds is NULL, it means no one is listening for the event and
       * we should delay sending the notification.
       */

      if (pressure->fds == NULL)
        {
          continue;
        }

      pressure->lasttick = current;
      spin_unlock_irqrestore(&g_pressure_lock, flags);
      poll_notify(&pressure->fds, 1, POLLPRI);
      flags = spin_lock_irqsave(&g_pressure_lock);
    }

  spin_unlock_irqrestore(&g_pressure_lock, flags);
}

