/****************************************************************************
 * drivers/thermal/thermal_procfs.c
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

#include <nuttx/list.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/kmalloc.h>

#include <assert.h>
#include <sys/stat.h>

#include "thermal_core.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct thermal_procfs_s
{
  struct procfs_file_s base;
  FAR struct thermal_zone_device_s *zdev;
  struct list_node node;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Procfs operations */

static int     thermal_procfs_open     (FAR struct file *filep,
                                        FAR const char *relpath,
                                        int oflags, mode_t mode);
static int     thermal_procfs_close    (FAR struct file *filep);
static ssize_t thermal_procfs_read     (FAR struct file *filep,
                                        FAR char *buffer,
                                        size_t buflen);
static ssize_t thermal_procfs_write    (FAR struct file *filep,
                                        FAR const char *buffer,
                                        size_t buflen);
static int     thermal_procfs_dup      (FAR const struct file *oldp,
                                        FAR struct file *newp);
static int     thermal_procfs_opendir  (FAR const char *relpath,
                                        FAR struct fs_dirent_s **dir);
static int     thermal_procfs_closedir (FAR struct fs_dirent_s *dir);
static int     thermal_procfs_readdir  (FAR struct fs_dirent_s *dir,
                                        FAR struct dirent *entry);
static int     thermal_procfs_rewinddir(FAR struct fs_dirent_s *dir);
static int     thermal_procfs_stat     (FAR const char *relpath,
                                        FAR struct stat *buf);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct list_node
g_thermal_procfs_list = LIST_INITIAL_VALUE(g_thermal_procfs_list);
static mutex_t g_thermal_procfs_lock = NXMUTEX_INITIALIZER;

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct procfs_operations g_thermal_operations =
{
  .open      = thermal_procfs_open,
  .close     = thermal_procfs_close,
  .read      = thermal_procfs_read,
  .write     = thermal_procfs_write,
  .poll      = NULL,
  .dup       = thermal_procfs_dup,
  .opendir   = thermal_procfs_opendir,
  .closedir  = thermal_procfs_closedir,
  .readdir   = thermal_procfs_readdir,
  .rewinddir = thermal_procfs_rewinddir,
  .stat      = thermal_procfs_stat,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int thermal_procfs_open(FAR struct file *filep,
                               FAR const char *relpath,
                               int oflags, mode_t mode)
{
  FAR struct thermal_procfs_s *child;

  relpath += strlen("thermal/");
  nxmutex_lock(&g_thermal_procfs_lock);

  list_for_every_entry(&g_thermal_procfs_list, child,
                       struct thermal_procfs_s, node)
    {
      if (!strcmp(child->zdev->name, relpath))
        {
          filep->f_priv = child;
          nxmutex_unlock(&g_thermal_procfs_lock);
          return OK;
        }
    }

  nxmutex_unlock(&g_thermal_procfs_lock);
  return -ENOENT;
}

static int thermal_procfs_close(FAR struct file *filep)
{
  filep->f_priv = NULL;
  return OK;
}

static ssize_t thermal_procfs_read(FAR struct file *filep,
                                   FAR char *buffer,
                                   size_t buflen)
{
  FAR struct thermal_procfs_s *p = filep->f_priv;
  FAR struct thermal_instance_s *ins;
  off_t offset = filep->f_pos;
  unsigned int current;

  list_for_every_entry(&p->zdev->instance_list, ins,
                       struct thermal_instance_s, zdev_node)
    {
      ins->cdev->ops->get_state(ins->cdev, &current);
      procfs_sprintf(buffer, buflen, &offset,
                     "z:%s t:%d t:%d h:%u l:%u c:%s ",
                     ins->zdev->name,
                     ins->zdev->temperature,
                     ins->trip,
                     ins->upper,
                     ins->lower,
                     ins->cdev->name);

      if (ins->target == THERMAL_NO_TARGET)
        {
          procfs_sprintf(buffer, buflen, &offset, "s:%u|%s",
                         current, "(invalid)");
        }
      else
        {
          procfs_sprintf(buffer, buflen, &offset, "s:%u|%u",
                         current, ins->target);
        }

      procfs_sprintf(buffer, buflen, &offset, "\n");
    }

  if (offset < 0)
    {
      offset = -offset;
    }
  else
    {
      offset = 0;
    }

  filep->f_pos += offset;
  return offset;
}

static ssize_t thermal_procfs_write(FAR struct file *filep,
                                    FAR const char *buffer,
                                    size_t buflen)
{
  FAR struct thermal_procfs_s *p = filep->f_priv;

  if (!strncmp(buffer, "1", 1))
    {
      thermal_zone_enable(p->zdev, true);
    }
  else if (!strncmp(buffer, "0", 1))
    {
      thermal_zone_enable(p->zdev, false);
    }

  return buflen;
}

static int thermal_procfs_dup(FAR const struct file *oldp,
                              FAR struct file *newp)
{
  newp->f_priv = oldp->f_priv;
  return OK;
}

static int thermal_procfs_opendir(FAR const char *relpath,
                                  FAR struct fs_dirent_s **dir)
{
  FAR struct procfs_dir_priv_s *level1;

  level1 = kmm_zalloc(sizeof(struct procfs_dir_priv_s));
  if (level1 == NULL)
    {
      *dir = NULL;
      return -ENOMEM;
    }

  level1->level = 1;

  nxmutex_lock(&g_thermal_procfs_lock);
  level1->nentries = list_length(&g_thermal_procfs_list);
  nxmutex_unlock(&g_thermal_procfs_lock);

  *dir = (FAR struct fs_dirent_s *)level1;
  return OK;
}

static int thermal_procfs_closedir(FAR struct fs_dirent_s *dir)
{
  kmm_free(dir);
  return OK;
}

static int thermal_procfs_readdir(FAR struct fs_dirent_s *dir,
                                  FAR struct dirent *entry)
{
  FAR struct procfs_dir_priv_s *level1;
  FAR struct thermal_procfs_s *child;
  int index = 0;

  DEBUGASSERT(dir);
  level1 = (FAR struct procfs_dir_priv_s *)dir;

  if (level1->index >= level1->nentries)
    {
      /* We signal the end of the directory by returning the special
       * error -ENOENT
       */

      return -ENOENT;
    }

  nxmutex_lock(&g_thermal_procfs_lock);

  list_for_every_entry(&g_thermal_procfs_list, child,
                       struct thermal_procfs_s, node)
    {
      if (index == level1->index)
        {
          entry->d_type = DTYPE_FILE;

          strlcpy(entry->d_name, child->zdev->name, NAME_MAX);
          level1->index++;
          nxmutex_unlock(&g_thermal_procfs_lock);
          return OK;
        }

      index++;
    }

  nxmutex_unlock(&g_thermal_procfs_lock);
  return -ENOENT;
}

static int thermal_procfs_rewinddir(FAR struct fs_dirent_s *dir)
{
  FAR struct procfs_dir_priv_s *level1;

  DEBUGASSERT(dir);
  level1 = (FAR struct procfs_dir_priv_s *)dir;
  level1->index = 0;
  return OK;
}

static int thermal_procfs_stat(FAR const char *relpath,
                               FAR struct stat *buf)
{
  FAR struct thermal_procfs_s *child;

  memset(buf, 0, sizeof(struct stat));

  if (strcmp(relpath, "thermal") == 0 || strcmp(relpath, "thermal/") == 0)
    {
      buf->st_mode = S_IFDIR | S_IROTH | S_IRGRP | S_IRUSR;
      return OK;
    }
  else
    {
      relpath += strlen("thermal/");

      nxmutex_lock(&g_thermal_procfs_lock);

      list_for_every_entry(&g_thermal_procfs_list, child,
                           struct thermal_procfs_s, node)
        {
          if (!strcmp(child->zdev->name, relpath))
            {
              buf->st_mode = S_IFREG | S_IROTH | S_IRGRP | S_IRUSR;
              nxmutex_unlock(&g_thermal_procfs_lock);
              return OK;
            }
        }

      nxmutex_unlock(&g_thermal_procfs_lock);
    }

  return -ENOENT;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int thermal_zone_procfs_register(FAR struct thermal_zone_device_s *zdev)
{
  FAR struct thermal_procfs_s *p;

  p = kmm_zalloc(sizeof(struct thermal_procfs_s));
  if (p == NULL)
    {
      return -ENOMEM;
    }

  p->zdev  = zdev;

  nxmutex_lock(&g_thermal_procfs_lock);
  list_add_tail(&g_thermal_procfs_list, &p->node);
  nxmutex_unlock(&g_thermal_procfs_lock);
  return OK;
}

void thermal_zone_procfs_unregister(FAR struct thermal_zone_device_s *zdev)
{
  FAR struct thermal_procfs_s *p;

  nxmutex_lock(&g_thermal_procfs_lock);

  list_for_every_entry(&g_thermal_procfs_list, p,
                       FAR struct thermal_procfs_s, node)
    {
      if (p->zdev == zdev)
        {
          list_delete(&p->node);
          kmm_free(p);
          break;
        }
    }

  nxmutex_unlock(&g_thermal_procfs_lock);
}
