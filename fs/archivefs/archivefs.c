/****************************************************************************
 * fs/archivefs/archivefs.c
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

#include <debug.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/statfs.h>

#include <archive_entry.h>
#include <archive.h>

#include "fs_heap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct archivefs_priv_s
{
  FAR struct archive_entry *entry;
  FAR struct archive *a;

  /* callback data */

  struct file file;
  char buffer[CONFIG_FS_ARCHIVEFS_BUFFER_SIZE];

  /* seek data */

  FAR char *seekbuf;
  mutex_t lock;
};

struct archivefs_dir_s
{
  struct fs_dirent_s dir;
  FAR struct archivefs_priv_s *priv;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int archivefs_open(FAR struct file *filep, FAR const char *relpath,
                          int oflags, mode_t mode);
static int archivefs_close(FAR struct file *filep);
static ssize_t archivefs_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static off_t archivefs_seek(FAR struct file *filep, off_t offset,
                            int whence);
static int archivefs_dup(FAR const struct file *oldp,
                         FAR struct file *newp);
static int archivefs_fstat(FAR const struct file *filep,
                           FAR struct stat *buf);
static int archivefs_opendir(FAR struct inode *mountpt,
                             FAR const char *relpath,
                             FAR struct fs_dirent_s **dir);
static int archivefs_closedir(FAR struct inode *mountpt,
                              FAR struct fs_dirent_s *dir);
static int archivefs_readdir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir,
                             FAR struct dirent *entry);
static int archivefs_rewinddir(FAR struct inode *mountpt,
                               FAR struct fs_dirent_s *dir);
static int archivefs_statfs(FAR struct inode *mountpt,
                            FAR struct statfs *buf);
static int archivefs_stat(FAR struct inode *mountpt,
                          FAR const char *relpath,
                          FAR struct stat *buf);
static int archivefs_bind(FAR struct inode *driver, FAR const void *data,
                          FAR void **handle);
static int archivefs_unbind(FAR void *handle, FAR struct inode **driver,
                            unsigned int flags);

/****************************************************************************
 * Private Data
 ****************************************************************************/

const struct mountpt_operations g_archivefs_operations =
{
  archivefs_open,          /* open */
  archivefs_close,         /* close */
  archivefs_read,          /* read */
  NULL,                    /* write */
  archivefs_seek,          /* seek */
  NULL,                    /* ioctl */
  NULL,                    /* mmap */
  NULL,                    /* truncate */
  NULL,                    /* poll */
  NULL,                    /* sync */
  archivefs_dup,           /* dup */
  archivefs_fstat,         /* fstat */
  NULL,                    /* fchstat */

  archivefs_opendir,       /* opendir */
  archivefs_closedir,      /* closedir */
  archivefs_readdir,       /* readdir */
  archivefs_rewinddir,     /* rewinddir */

  archivefs_bind,          /* bind */
  archivefs_unbind,        /* unbind */
  archivefs_statfs,        /* statfs */

  NULL,                    /* unlink */
  NULL,                    /* mkdir */
  NULL,                    /* rmdir */
  NULL,                    /* rename */
  archivefs_stat,          /* stat */
  NULL                     /* chstat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int archivefs_convert_result(int ret)
{
  switch (ret)
    {
      case ARCHIVE_RETRY:
        return -EAGAIN;
      case ARCHIVE_WARN:
        return -ENOEXEC;
      case ARCHIVE_FAILED:
        return -EINVAL;
      case ARCHIVE_FATAL:
        return -EPERM;

      default:
        return ret;
    }
}

static int archivefs_close_cb(FAR struct archive *a, FAR void *client_data)
{
  FAR struct archivefs_priv_s *priv = client_data;

  return file_close(&priv->file);
}

static la_ssize_t archivefs_read_cb(FAR struct archive *a,
                                    FAR void *client_data,
                                    FAR const void **buff)
{
  FAR struct archivefs_priv_s *priv = client_data;
  ssize_t ret;

  ret = file_read(&priv->file, priv->buffer,
                  CONFIG_FS_ARCHIVEFS_BUFFER_SIZE);
  if (ret <= 0)
    {
      return ret;
    }

  *buff = priv->buffer;
  return ret;
}

static la_int64_t archivefs_seek_cb(FAR struct archive *a,
                                    FAR void *client_data,
                                    la_int64_t offset, int whence)
{
  FAR struct archivefs_priv_s *priv = client_data;

  return file_seek(&priv->file, offset, whence);
}

static off_t archivefs_skip(FAR struct archivefs_priv_s *priv, off_t amount)
{
  off_t next = 0;

  if (priv->seekbuf == NULL)
    {
      priv->seekbuf = fs_heap_malloc(CONFIG_FS_ARCHIVEFS_BUFFER_SIZE);
      if (priv->seekbuf == NULL)
        {
          return -ENOMEM;
        }
    }

  while (next < amount)
    {
      off_t remain = amount - next;

      if (remain > CONFIG_FS_ARCHIVEFS_BUFFER_SIZE)
        {
          remain = CONFIG_FS_ARCHIVEFS_BUFFER_SIZE;
        }

      remain = archive_read_data(priv->a, priv->seekbuf, remain);
      remain = archivefs_convert_result(remain);
      if (remain <= 0)
        {
          return next ? next : remain;
        }

      next += remain;
    }

  return next;
}

int archivefs_new(FAR const char *abspath, FAR const char *relpath,
                  FAR struct archivefs_priv_s **priv)
{
  FAR struct archivefs_priv_s *newp;
  int ret = -ENOMEM;

  newp = fs_heap_zalloc(sizeof(struct archivefs_priv_s));
  if (newp == NULL)
    {
      return ret;
    }

  newp->a = archive_read_new();
  if (newp->a == NULL)
    {
      ferr("ERROR: archive_read_new() failed\n");
      goto err_with_priv;
    }

  nxmutex_init(&newp->lock);
#ifdef CONFIG_FS_ARCHIVEFS_FORMAT_ALL
  ret = archivefs_convert_result(archive_read_support_format_all(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_all() failed\n");
      goto err_with_lock;
    }
#else
#  ifdef CONFIG_FS_ARCHIVEFS_FORMAT_ZIP
  ret = archivefs_convert_result(archive_read_support_format_zip(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_zip() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ARCHIVEFS_FORMAT_7ZIP
  ret = archivefs_convert_result(archive_read_support_format_7zip(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_7zip() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ARCHIVEFS_FORMAT_AR
  ret = archivefs_convert_result(archive_read_support_format_ar(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_ar() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ARCHIVEFS_FORMAT_CAB
  ret = archivefs_convert_result(archive_read_support_format_cab(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_cab() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ARCHIVEFS_FORMAT_CPIO
  ret = archivefs_convert_result(archive_read_support_format_cpio(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_cpio() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ARCHIVEFS_FORMAT_EMPTY
  ret = archivefs_convert_result(archive_read_support_format_empty(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_empty() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ARCHIVEFS_FORMAT_ISO9660
  ret = archive_read_support_format_iso9660(newp->a);
  ret = archivefs_convert_result(ret);
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_iso9660() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ARCHIVEFS_FORMAT_LHA
  ret = archivefs_convert_result(archive_read_support_format_lha(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_lha() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ARCHIVEFS_FORMAT_MTREE
  ret = archivefs_convert_result(archive_read_support_format_mtree(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_mtree() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ARCHIVEFS_FORMAT_RAR
  ret = archivefs_convert_result(archive_read_support_format_rar(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_rar() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ARCHIVEFS_FORMAT_RAR_V5
  ret = archivefs_convert_result(archive_read_support_format_rar5(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_rar5() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ARCHIVEFS_FORMAT_RAW
  ret = archivefs_convert_result(archive_read_support_format_raw(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_raw() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ARCHIVEFS_FORMAT_TAR
  ret = archivefs_convert_result(archive_read_support_format_tar(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_tar() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ARCHIVEFS_FORMAT_WARC
  ret = archivefs_convert_result(archive_read_support_format_warc(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_warc() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ARCHIVEFS_FORMAT_XAR
  ret = archivefs_convert_result(archive_read_support_format_xar(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_xar() failed\n");
      goto err_with_lock;
    }
#  endif
#endif

  ret = file_open(&newp->file, abspath, O_RDONLY);
  if (ret < 0)
    {
      ferr("ERROR: file_open() failed\n");
      goto err_with_lock;
    }

  archive_read_set_seek_callback(newp->a, archivefs_seek_cb);
  ret = archive_read_open(newp->a, newp, NULL,
                          archivefs_read_cb, archivefs_close_cb);
  ret = archivefs_convert_result(ret);
  if (ret < 0)
    {
      ferr("ERROR: archive_read_open() failed\n");
      goto err_with_open;
    }

  newp->entry = archive_entry_new2(newp->a);
  if (newp->entry == NULL)
    {
      ferr("ERROR: archive_entry_new2() failed\n");
      ret = -ENOMEM;
      goto err_with_close;
    }

  if (relpath == NULL)
    {
      /* Open root dir  */

      *priv = newp;
      return OK;
    }

  while (true)
    {
      ret = archive_read_next_header2(newp->a, newp->entry);
      ret = archivefs_convert_result(ret);
      if (ret < 0)
        {
          break;
        }

      if (strcmp(archive_entry_pathname(newp->entry), relpath) == 0)
        {
          *priv = newp;
          return OK;
        }
    }

  ret = -ENOENT;
  archive_entry_free(newp->entry);
err_with_close:
  archive_read_close(newp->a);
err_with_open:
  file_close(&newp->file);
err_with_lock:
  nxmutex_destroy(&newp->lock);
  archive_read_free(newp->a);
err_with_priv:
  fs_heap_free(newp);
  return ret;
}

static void archivefs_free(FAR struct archivefs_priv_s *priv)
{
  if (priv->seekbuf != NULL)
    {
      fs_heap_free(priv->seekbuf);
    }

  archive_entry_free(priv->entry);
  archive_read_close(priv->a);
  nxmutex_destroy(&priv->lock);
  archive_read_free(priv->a);
  fs_heap_free(priv);
}

static int archivefs_stats_common(FAR struct archivefs_priv_s *priv,
                                  FAR struct stat *buf)
{
  FAR const struct stat *stat = archive_entry_stat(priv->entry);

  if (stat == NULL)
    {
      return -EINVAL;
    }

  memcpy(buf, stat, sizeof(struct stat));
  return OK;
}

/****************************************************************************
 * Archivefs methods
 ****************************************************************************/

static int archivefs_open(FAR struct file *filep, FAR const char *relpath,
                          int oflags, mode_t mode)
{
  FAR const char *abspath = filep->f_inode->i_private;
  FAR struct archivefs_priv_s *priv;
  int ret;

  ret = archivefs_new(abspath, relpath, &priv);
  if (ret < 0)
    {
      return ret;
    }

  filep->f_priv = priv;
  return OK;
}

static int archivefs_close(FAR struct file *filep)
{
  FAR struct archivefs_priv_s *priv = filep->f_priv;

  archivefs_free(priv);
  return OK;
}

static ssize_t archivefs_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen)
{
  FAR struct archivefs_priv_s *priv = filep->f_priv;
  ssize_t ret;

  nxmutex_lock(&priv->lock);
  ret = archivefs_convert_result(archive_read_data(priv->a, buffer, buflen));
  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

static off_t archivefs_seek(FAR struct file *filep, off_t offset,
                            int whence)
{
  FAR const char *abspath = filep->f_inode->i_private;
  FAR struct archivefs_priv_s *priv = filep->f_priv;
  FAR struct archivefs_priv_s *newp;
  off_t ret = 0;

  nxmutex_lock(&priv->lock);
  switch (whence)
    {
      case SEEK_SET:
        break;
      case SEEK_CUR:
        offset += filep->f_pos;
        break;
      case SEEK_END:
        offset += archive_entry_size(priv->entry);
        break;
      default:
        ret = -EINVAL;
        goto err_with_lock;
    }

  if (filep->f_pos == offset)
    {
      goto err_with_lock;
    }
  else if (filep->f_pos > offset)
    {
      ret = archivefs_new(abspath, archive_entry_pathname(priv->entry),
                          &newp);
      if (ret < 0)
        {
          goto err_with_lock;
        }

      nxmutex_unlock(&priv->lock);
      archivefs_free(priv);
      priv = newp;
      nxmutex_lock(&priv->lock);
      filep->f_priv = newp;
      filep->f_pos = 0;
    }

  ret = archivefs_skip(priv, offset - filep->f_pos);
  if (ret < 0)
    {
      goto err_with_lock;
    }

  filep->f_pos += ret;

err_with_lock:
  nxmutex_unlock(&priv->lock);
  return ret < 0 ? ret : filep->f_pos;
}

static int archivefs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct archivefs_priv_s *priv = oldp->f_priv;
  FAR const char *relpath = archive_entry_pathname(priv->entry);

  return archivefs_open(newp, relpath, oldp->f_oflags, 0);
}

static int archivefs_fstat(FAR const struct file *filep,
                           FAR struct stat *buf)
{
  FAR struct archivefs_priv_s *priv = filep->f_priv;

  return archivefs_stats_common(priv, buf);
}

static int archivefs_opendir(FAR struct inode *mountpt,
                             FAR const char *relpath,
                             FAR struct fs_dirent_s **dir)
{
  FAR const char *abspath = mountpt->i_private;
  FAR struct archivefs_dir_s *adir;
  int ret;

  adir = fs_heap_zalloc(sizeof(struct archivefs_dir_s));
  if (adir == NULL)
    {
      return -ENOMEM;
    }

  ret = archivefs_new(abspath, NULL, &adir->priv);
  if (ret < 0)
    {
      fs_heap_free(adir);
      return ret;
    }

  *dir = &adir->dir;
  return OK;
}

static int archivefs_closedir(FAR struct inode *mountpt,
                              FAR struct fs_dirent_s *dir)
{
  FAR struct archivefs_dir_s *adir = (FAR struct archivefs_dir_s *)dir;

  archivefs_free(adir->priv);
  fs_heap_free(adir);
  return OK;
}

static int archivefs_readdir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir,
                             FAR struct dirent *entry)
{
  FAR struct archivefs_dir_s *adir = (FAR struct archivefs_dir_s *)dir;
  FAR struct archivefs_priv_s *priv = adir->priv;
  int ret;

  nxmutex_lock(&priv->lock);
  ret = archive_read_next_header2(priv->a, priv->entry);
  ret = archivefs_convert_result(ret);
  if (ret < 0)
    {
      goto err;
    }

  if (ret == ARCHIVE_EOF)
    {
      ret = EOF;
      goto err;
    }

  strlcpy(entry->d_name, archive_entry_pathname(priv->entry),
          sizeof(entry->d_name));
err:
  nxmutex_unlock(&priv->lock);
  return ret;
}

static int archivefs_rewinddir(FAR struct inode *mountpt,
                               FAR struct fs_dirent_s *dir)
{
  FAR struct archivefs_dir_s *adir = (FAR struct archivefs_dir_s *)dir;
  FAR struct archivefs_priv_s *priv = adir->priv;

  nxmutex_lock(&priv->lock);
  archive_entry_free(priv->entry);
  priv->entry = archive_entry_new2(priv->a);
  if (priv->entry == NULL)
    {
      nxmutex_unlock(&priv->lock);
      return -ENOMEM;
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

static int archivefs_statfs(FAR struct inode *mountpt,
                            FAR struct statfs *buf)
{
  buf->f_type = ARCHIVEFS_MAGIC;
  buf->f_namelen = NAME_MAX;
  return OK;
}

static int archivefs_stat(FAR struct inode *mountpt,
                          FAR const char *relpath,
                          FAR struct stat *buf)
{
  FAR const char *abspath = mountpt->i_private;
  FAR struct archivefs_priv_s *priv;
  int ret;

  if (relpath[0] == 0)
    {
      buf->st_mode = S_IFDIR | 0555;
      return OK;
    }

  ret = archivefs_new(abspath, relpath, &priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = archivefs_stats_common(priv, buf);
  archivefs_free(priv);
  return ret;
}

static int archivefs_bind(FAR struct inode *driver, FAR const void *data,
                          FAR void **handle)
{
  FAR struct archivefs_priv_s *priv;
  FAR char *abspath;
  int ret;

  if (data == NULL)
    {
      return -ENODEV;
    }

  /* Try access to archive file */

  ret = archivefs_new(data, NULL, &priv);
  if (ret < 0)
    {
      return ret;
    }

  archivefs_free(priv);
  abspath = fs_heap_zalloc(PATH_MAX);
  if (abspath == NULL)
    {
      return -ENOMEM;
    }

  *handle = abspath;
  strlcpy(abspath, data, PATH_MAX);
  return OK;
}

static int archivefs_unbind(FAR void *handle, FAR struct inode **driver,
                            unsigned int flags)
{
  fs_heap_free(handle);
  return OK;
}
