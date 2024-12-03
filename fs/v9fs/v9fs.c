/****************************************************************************
 * fs/v9fs/v9fs.c
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
#include <fcntl.h>
#include <inttypes.h>
#include <libgen.h>
#include <string.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>

#include "inode/inode.h"
#include "client.h"
#include "fs_heap.h"

/****************************************************************************
 * Private Type
 ****************************************************************************/

struct v9fs_vfs_file_s
{
  uint32_t fid;
  mutex_t  lock;
};

struct v9fs_vfs_dirent_s
{
  struct fs_dirent_s base;
  uint32_t           fid;
  mutex_t            lock;
  off_t              offset;
  off_t              head;
  size_t             size;
  uint8_t            buffer[1];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int v9fs_vfs_open(FAR struct file *filep, FAR const char *relpath,
                         int oflags, mode_t mode);
static int v9fs_vfs_close(FAR struct file *filep);
static ssize_t v9fs_vfs_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen);
static ssize_t v9fs_vfs_write(FAR struct file *filep,
                              FAR const char *buffer, size_t buflen);
static off_t v9fs_vfs_seek(FAR struct file *filep, off_t offset,
                           int whence);
static int v9fs_vfs_ioctl(FAR struct file *filep, int cmd,
                          unsigned long arg);
static int v9fs_vfs_sync(FAR struct file *filep);
static int v9fs_vfs_dup(FAR const struct file *oldp, FAR struct file *newp);
static int v9fs_vfs_fstat(FAR const struct file *filep,
                          FAR struct stat *buf);
static int v9fs_vfs_fchstat(FAR const struct file *filep,
                            FAR const struct stat *buf, int flags);
static int v9fs_vfs_truncate(FAR struct file *filep, off_t length);
static int v9fs_vfs_opendir(FAR struct inode *mountpt,
                            FAR const char *relpath,
                            FAR struct fs_dirent_s **dir);
static int v9fs_vfs_closedir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir);
static int v9fs_vfs_readdir(FAR struct inode *mountpt,
                            FAR struct fs_dirent_s *dir,
                            FAR struct dirent *entry);
static int v9fs_vfs_rewinddir(FAR struct inode *mountpt,
                              FAR struct fs_dirent_s *dir);
static int v9fs_vfs_statfs(FAR struct inode *mountpt,
                           FAR struct statfs *buf);
static int v9fs_vfs_unlink(FAR struct inode *mountpt,
                           FAR const char *relpath);
static int v9fs_vfs_mkdir(FAR struct inode *mountpt,
                          FAR const char *relpath, mode_t mode);
static int v9fs_vfs_rmdir(FAR struct inode *mountpt,
                          FAR const char *relpath);
static int v9fs_vfs_rename(FAR struct inode *mountpt,
                           FAR const char *oldrelpath,
                           FAR const char *newrelpath);
static int v9fs_vfs_stat(FAR struct inode *mountpt,
                         FAR const char *relpath, FAR struct stat *buf);
static int v9fs_vfs_chstat(FAR struct inode *mountpt,
                           FAR const char *relpath,
                           FAR const struct stat *buf, int flags);
static int v9fs_vfs_bind(FAR struct inode *driver, FAR const void *data,
                         FAR void **handle);
static int v9fs_vfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                           unsigned int flags);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct mountpt_operations g_v9fs_operations =
{
  v9fs_vfs_open,                /* open */
  v9fs_vfs_close,               /* close */
  v9fs_vfs_read,                /* read */
  v9fs_vfs_write,               /* write */
  v9fs_vfs_seek,                /* seek */
  v9fs_vfs_ioctl,               /* ioctl */
  NULL,                         /* mmap */
  v9fs_vfs_truncate,            /* truncate */
  NULL,                         /* poll */
  NULL,                         /* readv */
  NULL,                         /* writev */

  v9fs_vfs_sync,                /* sync */
  v9fs_vfs_dup,                 /* dup */
  v9fs_vfs_fstat,               /* fstat */
  v9fs_vfs_fchstat,             /* fchstat */

  v9fs_vfs_opendir,             /* opendir */
  v9fs_vfs_closedir,            /* closedir */
  v9fs_vfs_readdir,             /* readdir */
  v9fs_vfs_rewinddir,           /* rewinddir */

  v9fs_vfs_bind,                /* bind */
  v9fs_vfs_unbind,              /* unbind */
  v9fs_vfs_statfs,              /* statfs */

  v9fs_vfs_unlink,              /* unlink */
  v9fs_vfs_mkdir,               /* mkdir */
  v9fs_vfs_rmdir,               /* rmdir */
  v9fs_vfs_rename,              /* rename */
  v9fs_vfs_stat,                /* stat */
  v9fs_vfs_chstat               /* chstat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: v9fs_vfs_open
 ****************************************************************************/

static int v9fs_vfs_open(FAR struct file *filep, FAR const char *relpath,
                         int oflags, mode_t mode)
{
  FAR struct v9fs_vfs_file_s *file;
  FAR struct v9fs_client_s *client;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);

  /* Recover out private data from the struct file instance */

  client = filep->f_inode->i_private;

  file = fs_heap_zalloc(sizeof(struct v9fs_vfs_file_s));
  if (file == NULL)
    {
      return -ENOMEM;
    }

  ret = v9fs_client_walk(client, relpath, NULL);
  if (ret >= 0)
    {
      file->fid = ret;
      ret = v9fs_client_open(client, file->fid, oflags);
      if (ret < 0)
        {
          ferr("ERROR: Failed to open the fid: %d\n", ret);
          goto err_put;
        }
    }
  else if ((oflags & O_CREAT) != 0)
    {
      /* We expect to create this file */

      ret = v9fs_client_walk(client, relpath, &relpath);
      if (ret < 0)
        {
          ferr("ERROR: Can't find the parent fid of relpath: %d\n", ret);
          goto err_free;
        }

      file->fid = ret;
      ret = v9fs_client_create(client, file->fid, relpath, oflags, mode);
      if (ret < 0)
        {
          ferr("ERROR: Failed to create the file: %d\n", ret);
          goto err_put;
        }
    }
  else
    {
      /* We expect to open this file */

      ferr("ERROR: Can't find the fid of relpath: %d\n", ret);
      ret = -ENOENT;
      goto err_free;
    }

  if ((oflags & O_APPEND) != 0)
    {
      filep->f_pos = v9fs_client_getsize(client, file->fid);
      if (filep->f_pos < 0)
        {
          ret = filep->f_pos;
          ferr("ERROR: Failed to get the file status: %d\n", ret);
          goto err_put;
        }
    }

  nxmutex_init(&file->lock);
  filep->f_priv = file;
  return 0;

err_put:
  v9fs_fid_put(client, file->fid);
err_free:
  fs_heap_free(file);
  return ret;
}

/****************************************************************************
 * Name: v9fs_vfs_close
 ****************************************************************************/

static int v9fs_vfs_close(FAR struct file *filep)
{
  FAR struct v9fs_client_s *client;
  FAR struct v9fs_vfs_file_s *file;

  /* Sanity checks */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);

  /* Recover out private data from the struct file instance */

  client = filep->f_inode->i_private;
  file = filep->f_priv;

  v9fs_fid_put(client, file->fid);
  nxmutex_destroy(&file->lock);
  fs_heap_free(file);
  return 0;
}

/****************************************************************************
 * Name: v9fs_vfs_read
 ****************************************************************************/

static ssize_t v9fs_vfs_read(FAR struct file *filep, FAR char *buffer,
                             size_t buflen)
{
  FAR struct v9fs_vfs_file_s *file;
  FAR struct v9fs_client_s *client;
  ssize_t ret;

  /* Sanity checks */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);

  /* Recover out private data from the struct file instance */

  client = filep->f_inode->i_private;
  file = filep->f_priv;

  nxmutex_lock(&file->lock);
  ret = v9fs_client_read(client, file->fid, buffer, filep->f_pos, buflen);
  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  nxmutex_unlock(&file->lock);
  return ret;
}

/****************************************************************************
 * Name: v9fs_vfs_write
 ****************************************************************************/

static ssize_t v9fs_vfs_write(FAR struct file *filep, FAR const char *buffer,
                              size_t buflen)
{
  FAR struct v9fs_vfs_file_s *file;
  FAR struct v9fs_client_s *client;
  ssize_t ret;

  /* Sanity checks */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);

  /* Recover out private data from the struct file instance */

  client = filep->f_inode->i_private;
  file = filep->f_priv;

  nxmutex_lock(&file->lock);
  ret = v9fs_client_write(client, file->fid, buffer, filep->f_pos, buflen);
  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  nxmutex_unlock(&file->lock);
  return ret;
}

/****************************************************************************
 * Name: v9fs_vfs_seek
 ****************************************************************************/

static off_t v9fs_vfs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct v9fs_vfs_file_s *file;
  FAR struct v9fs_client_s *client;
  off_t ret;

  /* Sanity checks */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);

  /* Recover out private data from the struct file instance */

  client = filep->f_inode->i_private;
  file = filep->f_priv;

  nxmutex_lock(&file->lock);
  switch (whence)
    {
      case SEEK_SET:
        ret = offset;
        break;
      case SEEK_CUR:
        ret = filep->f_pos + offset;
        break;
      case SEEK_END:
        ret = v9fs_client_getsize(client, file->fid);
        if (ret >= 0)
          {
            ret += offset;
          }

        break;
      default:
        ferr("ERROR: Invalid whence: %d\n", whence);
        ret = -EINVAL;
        break;
    }

  if (ret >= 0)
    {
      filep->f_pos = ret;
    }

  nxmutex_unlock(&file->lock);
  return ret;
}

/****************************************************************************
 * Name: v9fs_vfs_ioctl
 ****************************************************************************/

static int v9fs_vfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct v9fs_vfs_file_s *file;
  FAR struct v9fs_client_s *client;
  int ret = -ENOTTY;

  client = filep->f_inode->i_private;
  file = filep->f_priv;

  if (cmd == FIOC_FILEPATH)
    {
      FAR char *ptr = (FAR char *)((uintptr_t)arg);
      inode_getpath(filep->f_inode, ptr, PATH_MAX);
      ret = v9fs_client_getname(client, file->fid, ptr);
    }

  return ret;
}

/****************************************************************************
 * Name: v9fs_vfs_sync
 ****************************************************************************/

static int v9fs_vfs_sync(FAR struct file *filep)
{
  FAR struct v9fs_vfs_file_s *file;
  FAR struct v9fs_client_s *client;

  client = filep->f_inode->i_private;
  file = filep->f_priv;

  return v9fs_client_fsync(client, file->fid);
}

/****************************************************************************
 * Name: v9fs_vfs_dup
 ****************************************************************************/

static int v9fs_vfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct v9fs_vfs_file_s *newfile;
  FAR struct v9fs_vfs_file_s *file;
  FAR struct v9fs_client_s *client;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(oldp != NULL && oldp->f_inode != NULL &&
              newp != NULL && newp->f_inode != NULL);

  /* Recover out private data from the struct file instance */

  client = oldp->f_inode->i_private;
  file = oldp->f_priv;

  newfile = fs_heap_zalloc(sizeof(struct v9fs_vfs_file_s));
  if (newfile == NULL)
    {
      return -ENOMEM;
    }

  ret = v9fs_fid_get(client, file->fid);
  if (ret < 0)
    {
      fs_heap_free(newfile);
      return ret;
    }

  nxmutex_init(&newfile->lock);
  newfile->fid = file->fid;
  newp->f_priv = newfile;
  return ret;
}

/****************************************************************************
 * Name: v9fs_vfs_fstat
 ****************************************************************************/

static int v9fs_vfs_fstat(FAR const struct file *filep,
                          FAR struct stat *buf)
{
  FAR struct v9fs_vfs_file_s *file;
  FAR struct v9fs_client_s *client;

  /* Sanity checks */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);

  /* Recover out private data from the struct file instance */

  client = filep->f_inode->i_private;
  file = filep->f_priv;
  memset(buf, 0, sizeof(struct stat));

  return v9fs_client_stat(client, file->fid, buf);
}

/****************************************************************************
 * Name: v9fs_vfs_fchstat
 ****************************************************************************/

static int v9fs_vfs_fchstat(FAR const struct file *filep,
                            FAR const struct stat *buf, int flags)
{
  FAR struct v9fs_vfs_file_s *file;
  FAR struct v9fs_client_s *client;

  /* Sanity checks */

  DEBUGASSERT(filep != NULL && filep->f_inode != NULL);

  /* Recover out private data from the struct file instance */

  client = filep->f_inode->i_private;
  file = filep->f_priv;

  return v9fs_client_chstat(client, file->fid, buf, flags);
}

/****************************************************************************
 * Name: v9fs_vfs_truncate
 ****************************************************************************/

static int v9fs_vfs_truncate(FAR struct file *filep, off_t length)
{
  struct stat buf;

  buf.st_size = length;
  return v9fs_vfs_fchstat(filep, &buf, CH_STAT_SIZE);
}

/****************************************************************************
 * Name: v9fs_vfs_opendir
 ****************************************************************************/

static int v9fs_vfs_opendir(FAR struct inode *mountpt,
                            FAR const char *relpath,
                            FAR struct fs_dirent_s **dir)
{
  FAR struct v9fs_vfs_dirent_s *fsdir;
  FAR struct v9fs_client_s *client;
  uint32_t fid;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover out private data from the struct file instance */

  client = mountpt->i_private;

  fsdir = fs_heap_zalloc(sizeof(struct v9fs_vfs_dirent_s) + client->msize);
  if (fsdir == NULL)
    {
      return -ENOMEM;
    }

  ret = v9fs_client_walk(client, relpath, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Can't find the fid of the relpath: %d\n", ret);
      goto err;
    }

  fid = ret;
  ret = v9fs_client_open(client, fid, 0);
  if (ret < 0)
    {
      ferr("ERROR: Failed to open the fid: %d\n", ret);
      v9fs_fid_put(client, fid);
      goto err;
    }

  nxmutex_init(&fsdir->lock);
  fsdir->fid = fid;
  *dir = &fsdir->base;
  return 0;

err:
  fs_heap_free(fsdir);
  return ret;
}

/****************************************************************************
 * Name: v9fs_vfs_closedir
 ****************************************************************************/

static int v9fs_vfs_closedir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir)
{
  FAR struct v9fs_vfs_dirent_s *fsdir;
  FAR struct v9fs_client_s *client;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover out private data from the struct file instance */

  fsdir = (FAR struct v9fs_vfs_dirent_s *)dir;
  client = mountpt->i_private;

  v9fs_fid_put(client, fsdir->fid);
  nxmutex_destroy(&fsdir->lock);
  fs_heap_free(fsdir);
  return 0;
}

/****************************************************************************
 * Name: v9fs_readdir
 ****************************************************************************/

static int v9fs_vfs_readdir(FAR struct inode *mountpt,
                            FAR struct fs_dirent_s *dir,
                            FAR struct dirent *entry)
{
  FAR struct v9fs_vfs_dirent_s *fsdir;
  FAR struct v9fs_client_s *client;
  ssize_t ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover out private data from the struct file instance */

  fsdir = (FAR struct v9fs_vfs_dirent_s *)dir;
  client = mountpt->i_private;

  nxmutex_lock(&fsdir->lock);
  for (; ; )
    {
      if (fsdir->head == fsdir->size)
        {
          ret = v9fs_client_readdir(client, fsdir->fid, fsdir->buffer,
                                    fsdir->offset, client->msize);
          if (ret < 0)
            {
              break;
            }

          fsdir->head = 0;
          fsdir->size = ret;
        }

      ret = v9fs_client_convertdir(fsdir->buffer, fsdir->size, fsdir->head,
                                   &fsdir->offset, entry);
      if (ret < 0)
        {
          break;
        }

      fsdir->head += ret;
      if (strcmp(entry->d_name, ".") != 0 &&
          strcmp(entry->d_name, "..") != 0)
        {
          break;
        }
    }

  nxmutex_unlock(&fsdir->lock);
  return ret < 0 ? ret : 0;
}

/****************************************************************************
 * Name: v9fs_vfs_rewinddir
 ****************************************************************************/

static int v9fs_vfs_rewinddir(FAR struct inode *mountpt,
                              FAR struct fs_dirent_s *dir)
{
  FAR struct v9fs_vfs_dirent_s *fsdir;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover out private data from the struct file instance */

  fsdir = (FAR struct v9fs_vfs_dirent_s *)dir;

  nxmutex_lock(&fsdir->lock);
  fsdir->head = 0;
  fsdir->offset = 0;
  fsdir->size = 0;
  nxmutex_unlock(&fsdir->lock);
  return 0;
}

/****************************************************************************
 * Name: v9fs_vfs_statfs
 ****************************************************************************/

static int v9fs_vfs_statfs(FAR struct inode *mountpt,
                           FAR struct statfs *buf)
{
  FAR struct v9fs_client_s *client;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover out private data from the struct file instance */

  client = mountpt->i_private;

  memset(buf, 0, sizeof(struct statfs));
  return v9fs_client_statfs(client, buf);
}

/****************************************************************************
 * Name: v9fs_vfs_unlink
 ****************************************************************************/

static int v9fs_vfs_unlink(FAR struct inode *mountpt,
                           FAR const char *relpath)
{
  FAR struct v9fs_client_s *client;
  FAR const char *filename;
  uint32_t fid;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover out private data from the struct file instance */

  client = mountpt->i_private;

  ret = v9fs_client_walk(client, relpath, &filename);
  if (ret < 0)
    {
      ferr("ERROR: Can't find the parent fid of relpath: %d\n", ret);
      return ret;
    }

  fid = ret;
  ret = v9fs_client_unlink(client, fid, filename, false);
  v9fs_fid_put(client, fid);
  if (ret == -EOPNOTSUPP)
    {
      /* Maybe the server does not support this method of deletion.
       * Let's change the protocol to send
       */

      ret = v9fs_client_walk(client, relpath, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Can't find the fid of relpath: %d\n", ret);
          return ret;
        }

      fid = ret;
      ret = v9fs_client_remove(client, fid);
      v9fs_fid_put(client, fid);
    }

  return ret;
}

/****************************************************************************
 * Name: v9fs_vfs_mkdir
 ****************************************************************************/

static int v9fs_vfs_mkdir(FAR struct inode *mountpt,
                          FAR const char *relpath, mode_t mode)
{
  FAR struct v9fs_client_s *client;
  uint32_t fid;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover out private data from the struct file instance */

  client = mountpt->i_private;

  ret = v9fs_client_walk(client, relpath, &relpath);
  if (ret < 0)
    {
      ferr("ERROR: Can't find the parent fid of relpath: %d\n", ret);
      return ret;
    }

  fid = ret;
  ret = v9fs_client_mkdir(client, fid, relpath, mode);
  v9fs_fid_put(client, fid);
  return ret;
}

/****************************************************************************
 * Name: v9fs_vfs_rmdir
 ****************************************************************************/

static int v9fs_vfs_rmdir(FAR struct inode *mountpt,
                          FAR const char *relpath)
{
  FAR struct v9fs_client_s *client;
  FAR const char *dirname;
  uint32_t fid;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover out private data from the struct file instance */

  client = mountpt->i_private;

  ret = v9fs_client_walk(client, relpath, &dirname);
  if (ret < 0)
    {
      ferr("ERROR: Can't find the parent fid of relpath: %d\n", ret);
      return ret;
    }

  fid = ret;
  ret = v9fs_client_unlink(client, fid, dirname, true);
  v9fs_fid_put(client, fid);
  if (ret == -EOPNOTSUPP)
    {
      /* Maybe the server does not support this method of deletion.
       * Let's change the protocol to send
       */

      ret = v9fs_client_walk(client, relpath, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Can't find the fid of relpath: %d\n", ret);
          return ret;
        }

      fid = ret;
      ret = v9fs_client_remove(client, fid);
      v9fs_fid_put(client, fid);
    }

  return ret;
}

/****************************************************************************
 * Name: v9fs_vfs_rename
 ****************************************************************************/

static int v9fs_vfs_rename(FAR struct inode *mountpt,
                           FAR const char *oldrelpath,
                           FAR const char *newrelpath)
{
  FAR struct v9fs_client_s *client;
  uint32_t oldfid;
  uint32_t newpfid;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover out private data from the struct file instance */

  client = mountpt->i_private;

  ret = v9fs_client_walk(client, oldrelpath, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Can't find the fid of the oldrelpath: %d\n", ret);
      return ret;
    }

  oldfid = ret;
  ret = v9fs_client_walk(client, newrelpath, &newrelpath);
  if (ret < 0)
    {
      ferr("ERROR: Can't find the new parent fid of the newrelpath: %d\n",
           ret);
      v9fs_fid_put(client, oldfid);
      return ret;
    }

  newpfid = ret;
  ret = v9fs_client_rename(client, oldfid, newpfid, newrelpath);
  v9fs_fid_put(client, oldfid);
  v9fs_fid_put(client, newpfid);
  return ret;
}

/****************************************************************************
 * Name: v9fs_vfs_stat
 ****************************************************************************/

static int v9fs_vfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                         FAR struct stat *buf)
{
  FAR struct v9fs_client_s *client;
  uint32_t fid;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover out private data from the struct file instance */

  client = mountpt->i_private;
  memset(buf, 0, sizeof(struct stat));

  ret = v9fs_client_walk(client, relpath, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Can't find the fid of the relpath: %d\n", ret);
      return ret;
    }

  fid = ret;
  ret = v9fs_client_stat(client, fid, buf);
  v9fs_fid_put(client, fid);
  return ret;
}

/****************************************************************************
 * Name: v9fs_vfs_chstat
 ****************************************************************************/

static int v9fs_vfs_chstat(FAR struct inode *mountpt,
                           FAR const char *relpath,
                           FAR const struct stat *buf, int flags)
{
  FAR struct v9fs_client_s *client;
  uint32_t fid;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover out private data from the struct file instance */

  client = mountpt->i_private;

  ret = v9fs_client_walk(client, relpath, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Can't find the fid of the relpath %d\n", ret);
      return ret;
    }

  fid = ret;
  ret = v9fs_client_chstat(client, fid, buf, flags);
  v9fs_fid_put(client, fid);
  return ret;
}

/****************************************************************************
 * Name: v9fs_vfs_unbind
 ****************************************************************************/

static int v9fs_vfs_unbind(FAR void *handle, FAR struct inode **blkdriver,
                           unsigned int flags)
{
  FAR struct v9fs_client_s *client;
  int ret;

  client = handle;

  ret = v9fs_client_uninit(client);
  if (ret < 0)
    {
      ferr("ERROR: Failed to clunk root fid\n");
      return ret;
    }

  fs_heap_free(client);
  return ret;
}

/****************************************************************************
 * Name: v9fs_vfs_bind
 ****************************************************************************/

static int v9fs_vfs_bind(FAR struct inode *driver, FAR const void *data,
                         FAR void **handle)
{
  FAR struct v9fs_client_s *client;
  int ret;

  client = fs_heap_zalloc(sizeof(struct v9fs_client_s));
  if (client == NULL)
    {
      return -ENOMEM;
    }

  ret = v9fs_client_init(client, data);
  if (ret < 0)
    {
      ferr("ERROR: Failed to initialize for client: %d\n", ret);
      fs_heap_free(client);
      return ret;
    }

  *handle = client;
  return ret;
}
