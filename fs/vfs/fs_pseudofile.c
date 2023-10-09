/****************************************************************************
 * fs/vfs/fs_pseudofile.c
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
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <fcntl.h>
#include <sys/param.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/lib/math32.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct fs_pseudofile_s
{
  mutex_t lock;
  uint8_t crefs;
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  bool unlinked;
#endif
  FAR char *content;
};

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

static int     pseudofile_open(FAR struct file *filep);
static int     pseudofile_close(FAR struct file *filep);
static ssize_t pseudofile_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen);
static ssize_t pseudofile_read(FAR struct file *filep, FAR char *buffer,
                               size_t buflen);
static off_t   pseudofile_seek(FAR struct file *filep, off_t offset,
                               int whence);
static int     pseudofile_mmap(FAR struct file *filep,
                               FAR struct mm_map_entry_s *map);
static int     pseudofile_munmap(FAR struct task_group_s *group,
                                 FAR struct mm_map_entry_s *map,
                                 FAR void *start,
                                 size_t length);
static int     pseudofile_truncate(FAR struct file *filep, off_t length);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     pseudofile_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_pseudofile_ops =
{
  pseudofile_open,     /* open */
  pseudofile_close,    /* close */
  pseudofile_read,     /* read */
  pseudofile_write,    /* write */
  pseudofile_seek,     /* seek */
  NULL,                /* ioctl */
  pseudofile_mmap,     /* mmap */
  pseudofile_truncate, /* truncate */
  NULL,                /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  pseudofile_unlink,   /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int pseudofile_open(FAR struct file *filep)
{
  FAR struct inode *node = filep->f_inode;
  FAR struct fs_pseudofile_s *pf = node->i_private;
  int ret;

  ret = nxmutex_lock(&pf->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (pf->crefs >= 255)
    {
      ret = -EMFILE;
    }
  else
    {
      pf->crefs += 1;
      ret = OK;
    }

#ifdef CONFIG_PSEUDOFS_ATTRIBUTES
  node->i_atime.tv_sec = time(NULL);
#endif
  nxmutex_unlock(&pf->lock);
  return ret;
}

static void pseudofile_remove(FAR struct fs_pseudofile_s *pf)
{
  nxmutex_unlock(&pf->lock);
  nxmutex_destroy(&pf->lock);
  kmm_free(pf->content);
  kmm_free(pf);
}

static int pseudofile_close(FAR struct file *filep)
{
  FAR struct inode *node = filep->f_inode;
  FAR struct fs_pseudofile_s *pf = node->i_private;
  int ret;

  ret = nxmutex_lock(&pf->lock);
  if (ret < 0)
    {
      return ret;
    }

  pf->crefs--;
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  if (pf->crefs <= 0 && pf->unlinked)
#else
  if (pf->crefs <= 0)
#endif
    {
      pseudofile_remove(pf);
      return OK;
    }

  nxmutex_unlock(&pf->lock);
  return OK;
}

static int pseudofile_expand(FAR struct inode *node,
                             size_t size)
{
  FAR struct fs_pseudofile_s *pf = node->i_private;
  FAR void *tmp;

  if (pf->content && kmm_malloc_size(pf->content) >= size)
    {
      node->i_size = size;
      return 0;
    }

  tmp = kmm_realloc(pf->content, 1 << LOG2_CEIL(size));
  if (tmp == NULL)
    {
      return -ENOMEM;
    }

  pf->content = tmp;
  node->i_size = size;
  return 0;
}

static ssize_t pseudofile_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen)
{
  FAR struct inode *node = filep->f_inode;
  FAR struct fs_pseudofile_s *pf = node->i_private;
  int ret;

  ret = nxmutex_lock(&pf->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (filep->f_oflags & O_APPEND)
    {
      ret = pseudofile_expand(node, node->i_size + buflen);
      if (ret < 0)
        {
          nxmutex_unlock(&pf->lock);
          return ret;
        }

      filep->f_pos = node->i_size - buflen;
    }
  else
    {
      ret = pseudofile_expand(node, filep->f_pos + buflen);
      if (ret < 0)
        {
          nxmutex_unlock(&pf->lock);
          return ret;
        }
    }

  memcpy(pf->content + filep->f_pos, buffer, buflen);
  filep->f_pos += buflen;
#ifdef CONFIG_PSEUDOFS_ATTRIBUTES
  node->i_mtime.tv_sec = time(NULL);
#endif

  nxmutex_unlock(&pf->lock);
  return buflen;
}

static ssize_t pseudofile_read(FAR struct file *filep, FAR char *buffer,
                               size_t buflen)
{
  FAR struct inode *node = filep->f_inode;
  FAR struct fs_pseudofile_s *pf = node->i_private;
  int ret;

  if (buffer == NULL || node->i_size < filep->f_pos)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&pf->lock);
  if (ret < 0)
    {
      return ret;
    }

  buflen = MIN(node->i_size - filep->f_pos, buflen);
  memcpy(buffer, pf->content + filep->f_pos, buflen);
  filep->f_pos += buflen;

#ifdef CONFIG_PSEUDOFS_ATTRIBUTES
  node->i_atime.tv_sec = time(NULL);
#endif
  nxmutex_unlock(&pf->lock);

  return buflen;
}

static off_t pseudofile_seek(FAR struct file *filep, off_t offset,
                             int whence)
{
  FAR struct inode *node = filep->f_inode;
  FAR struct fs_pseudofile_s *pf = node->i_private;
  int ret;

  ret = nxmutex_lock(&pf->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Map the offset according to the whence option */

  switch (whence)
    {
      case SEEK_SET:
          break;

      case SEEK_CUR:
          offset += filep->f_pos;
          break;

      case SEEK_END:
          offset += node->i_size;
          break;

      default:
          nxmutex_unlock(&pf->lock);
          return -EINVAL;
    }

  if (offset < 0)
    {
      nxmutex_unlock(&pf->lock);
      return -EINVAL;
    }

  filep->f_pos = offset;
#ifdef CONFIG_PSEUDOFS_ATTRIBUTES
  node->i_atime.tv_sec = time(NULL);
#endif
  nxmutex_unlock(&pf->lock);
  return offset;
}

static int pseudofile_mmap(FAR struct file *filep,
                           FAR struct mm_map_entry_s *map)
{
  FAR struct inode *node = filep->f_inode;
  FAR struct fs_pseudofile_s *pf = node->i_private;

  /* Keep the inode when mmapped, increase refcount */

  int ret = inode_addref(node);
  if (ret >= 0)
    {
      if (map->offset >= 0 && map->offset < node->i_size &&
          map->length != 0 && map->offset + map->length <= node->i_size)
        {
          map->vaddr = pf->content + map->offset;
          map->munmap = pseudofile_munmap;
          map->priv.p = (FAR void *)node;
          ret = mm_map_add(get_current_mm(), map);
        }
      else
        {
          ret = -EINVAL;
        }

      if (ret < 0)
        {
          inode_release(node);
        }
    }

  return ret;
}

static int pseudofile_munmap(FAR struct task_group_s *group,
                             FAR struct mm_map_entry_s *map,
                             FAR void *start,
                             size_t length)
{
  FAR struct inode *inode = (FAR struct inode *)map->priv.p;

  /* If the file has been unlinked previously, delete the contents.
   * The inode is released after this call, hence checking if i_crefs <= 1.
   */

  int ret = inode_lock();
  if (ret >= 0)
    {
      if (inode->i_parent == NULL &&
          inode->i_crefs <= 1)
        {
          /* Delete the inode metadata */

          if (inode->i_private)
            {
              kmm_free(inode->i_private);
            }

          inode->i_private = NULL;
          ret = OK;
        }

      inode_unlock();
    }

  /* Unkeep the inode when unmapped, decrease refcount */

  if (ret == OK)
    {
      inode_release(inode);

      /* Remove the mapping. */

      ret = mm_map_remove(get_group_mm(group), map);
    }

  return ret;
}

static int pseudofile_truncate(FAR struct file *filep, off_t length)
{
  FAR struct inode *node = filep->f_inode;
  FAR struct fs_pseudofile_s *pf = node->i_private;
  int ret;

  ret = nxmutex_lock(&pf->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (length < node->i_size)
    {
      FAR void *tmp;

      tmp = kmm_realloc(pf->content, length);
      if (tmp == NULL)
        {
          ret = -ENOMEM;
          goto out;
        }

      pf->content = tmp;
      node->i_size = length;
    }
  else
    {
      ret = pseudofile_expand(node, length);
      if (ret < 0)
        {
          goto out;
        }

      memset(pf->content + node->i_size, 0, length - node->i_size);
    }

#ifdef CONFIG_PSEUDOFS_ATTRIBUTES
  node->i_mtime.tv_sec = time(NULL);
#endif

out:
  nxmutex_unlock(&pf->lock);
  return ret;
}

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int pseudofile_unlink(FAR struct inode *node)
{
  FAR struct fs_pseudofile_s *pf = node->i_private;
  int ret;

  ret = nxmutex_lock(&pf->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (pf->crefs <= 0)
    {
      pseudofile_remove(pf);
      return OK;
    }

  pf->unlinked = true;
  nxmutex_unlock(&pf->lock);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pseudofile_create
 *
 * Description:
 *   Create the pseudo-file with specified path and mode, and alloc inode
 *   of this pseudo-file.
 *
 ****************************************************************************/

int pseudofile_create(FAR struct inode **node, FAR const char *path,
                      mode_t mode)
{
  FAR struct fs_pseudofile_s *pf;
  int ret;

  if (node == NULL || path == NULL)
    {
      return -EINVAL;
    }

  pf = kmm_zalloc(sizeof(struct fs_pseudofile_s));
  if (pf == NULL)
    {
      return -ENOMEM;
    }

  nxmutex_init(&pf->lock);

  ret = inode_lock();
  if (ret < 0)
    {
      goto lock_err;
    }

  ret = inode_reserve(path, mode, node);
  if (ret < 0)
    {
      goto reserve_err;
    }

  (*node)->i_crefs = 0;
  (*node)->i_flags = 1;
  (*node)->u.i_ops = &g_pseudofile_ops;
  (*node)->i_private = pf;

  inode_unlock();
  return 0;

reserve_err:
  inode_unlock();
lock_err:
  nxmutex_destroy(&pf->lock);
  kmm_free(pf);
  return ret;
}

/****************************************************************************
 * Name: inode_is_pseudofile
 *
 * Description:
 *    Check inode whether is a pseudo file.
 *
 ****************************************************************************/

bool inode_is_pseudofile(FAR struct inode *inode)
{
  return inode != NULL && inode->u.i_ops == &g_pseudofile_ops;
}
