/****************************************************************************
 * fs/zipfs/wzip_vfs.c
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

/* wzipfs -- write-only streaming ZIP filesystem.
 *
 * Mount type "wzipfs".  Backing target may be a regular file, FIFO, or
 * socket.  Non-empty regular-file backings are rejected with -EEXIST;
 * remove or truncate the file before mounting to start a fresh archive.
 *
 * Compression is selected at mount time via "method=store" (default) or
 * "method=deflate" (requires CONFIG_FS_ZIPFS_WRITE_DEFLATE=y).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <time.h>
#include <unistd.h>

#include <archive.h>
#include <archive_entry.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>

#include "fs_heap.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct wzipfs_priv_s
{
  struct file backing;
  FAR struct archive *a;
  FAR struct archive_entry *entry;
  sem_t open_sem;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int wzipfs_open(FAR struct file *filep, FAR const char *relpath,
                       int oflags, mode_t mode);
static int wzipfs_close(FAR struct file *filep);
static ssize_t wzipfs_write(FAR struct file *filep,
                            FAR const char *buffer, size_t buflen);
static int wzipfs_sync(FAR struct file *filep);
static int wzipfs_bind(FAR struct inode *driver, FAR const void *data,
                       FAR void **handle);
static int wzipfs_unbind(FAR void *handle, FAR struct inode **driver,
                         unsigned int flags);
static int wzipfs_statfs(FAR struct inode *mountpt,
                         FAR struct statfs *buf);
static int wzipfs_mkdir(FAR struct inode *mountpt,
                        FAR const char *relpath, mode_t mode);

/****************************************************************************
 * Private Data
 ****************************************************************************/

const struct mountpt_operations g_wzipfs_operations =
{
  wzipfs_open,           /* open */
  wzipfs_close,          /* close */
  NULL,                  /* read   -- write-only FS */
  wzipfs_write,          /* write */
  NULL,                  /* seek   -- write-only FS, no seeking mid-stream */
  NULL,                  /* ioctl  -- method= mount option replaces ioctl */
  NULL,                  /* mmap */
  NULL,                  /* truncate */
  NULL,                  /* poll */
  NULL,                  /* readv */
  NULL,                  /* writev */
  wzipfs_sync,           /* sync */
  NULL,                  /* dup */
  NULL,                  /* fstat  -- write-only FS */
  NULL,                  /* fchstat */

  NULL,                  /* opendir  -- write-only FS */
  NULL,                  /* closedir */
  NULL,                  /* readdir  -- write-only FS */
  NULL,                  /* rewinddir */

  wzipfs_bind,           /* bind */
  wzipfs_unbind,         /* unbind */
  wzipfs_statfs,         /* statfs */

  NULL,                  /* unlink */
  wzipfs_mkdir,          /* mkdir */
  NULL,                  /* rmdir */
  NULL,                  /* rename */
  NULL,                  /* stat   -- write-only FS */
  NULL                   /* chstat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int wzipfs_convert_result(int ret)
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

/****************************************************************************
 * Name: wzipfs_parse_options
 *
 * Description:
 *   Parse a NuttX mount "-o" options string.
 *   Accepted keys: src=<path>, method=store|deflate.
 *   Legacy positional path (no '=' and no ',') is accepted as src.
 *
 * Returns 0 on success, negative errno on failure.
 ****************************************************************************/

static int wzipfs_parse_options(FAR const void *data,
                                FAR char *src, size_t len,
                                FAR bool *deflate)
{
  FAR const char *p = data;
  FAR const char *end;

  src[0]   = '\0';
  *deflate = false;

  if (p == NULL || *p == '\0')
    {
      return -ENODEV;
    }

  /* Legacy positional path: no '=' and no ',' anywhere. */

  if (strchr(p, '=') == NULL && strchr(p, ',') == NULL)
    {
      size_t n = strlen(p);

      if (n >= len)
        {
          return -ENAMETOOLONG;
        }

      memcpy(src, p, n + 1);
      return 0;
    }

  while (*p != '\0')
    {
      end = strchrnul(p, ',');

      if (end - p > 4 && memcmp(p, "src=", 4) == 0)
        {
          size_t pl = end - p - 4;

          if (pl >= len)
            {
              return -EINVAL;
            }

          memcpy(src, p + 4, pl);
          src[pl] = '\0';
        }
      else if (end - p > 7 && memcmp(p, "method=", 7) == 0)
        {
          FAR const char *val    = p + 7;
          size_t          vallen = end - p - 7;

          if (vallen == 7 && memcmp(val, "deflate", 7) == 0)
            {
#ifdef CONFIG_FS_ZIPFS_WRITE_DEFLATE
              *deflate = true;
#else
              ferr("wzipfs: method=deflate requires "
                   "CONFIG_FS_ZIPFS_WRITE_DEFLATE\n");
              return -ENOTSUP;
#endif
            }
          else if (vallen != 5 || memcmp(val, "store", 5) != 0)
            {
              ferr("wzipfs: unknown method value\n");
              return -EINVAL;
            }
        }
      else
        {
          ferr("wzipfs: unknown mount option\n");
          return -EINVAL;
        }

      p = *end ? end + 1 : end;
    }

  return 0;
}

static la_ssize_t wzipfs_write_cb(FAR struct archive *a,
                                  FAR void *client_data,
                                  FAR const void *buff,
                                  size_t len)
{
  FAR struct wzipfs_priv_s *priv = client_data;

  return file_write(&priv->backing, buff, len);
}

/****************************************************************************
 * Name: wzipfs_init_entry
 *
 * Prepare the reusable archive_entry in priv->entry for a header write.
 * Size is intentionally left unset: for regular files this triggers the
 * streaming (data descriptor) write path; for directories the ZIP writer
 * forces size to 0 internally.
 ****************************************************************************/

static void wzipfs_init_entry(FAR struct wzipfs_priv_s *priv,
                              FAR const char *path, mode_t mode)
{
  archive_entry_clear(priv->entry);
  archive_entry_set_pathname(priv->entry, path);
  archive_entry_set_mode(priv->entry, mode);
  archive_entry_set_mtime(priv->entry, time(NULL), 0);
  archive_entry_set_filetype(priv->entry,
                             S_ISDIR(mode) ? AE_IFDIR : AE_IFREG);
}

/****************************************************************************
 * Name: wzipfs_create_backing
 *
 * Open the backing target.  Accepted cases:
 *   - backing does not exist (ENOENT)    -- create new regular file
 *   - backing is a FIFO or socket        -- stream unconditionally
 *   - backing is a regular file of zero size -- create fresh archive
 *
 * Rejected cases:
 *   - regular file with st_size > 0      -- returns -EEXIST
 *
 * Callers that want to replace an existing archive should unlink/truncate
 * the backing file before mounting.
 ****************************************************************************/

static int wzipfs_create_backing(FAR struct wzipfs_priv_s *priv,
                                 FAR const char *src)
{
  struct stat st;
  int ret;

  ret = file_open(&priv->backing, src, O_WRONLY | O_CREAT, 0644);
  if (ret < 0)
    {
      ferr("wzipfs: open backing '%s' failed: %d\n", src, ret);
      return ret;
    }

  ret = file_fstat(&priv->backing, &st);
  if (ret < 0)
    {
      file_close(&priv->backing);
      return ret;
    }

  if (S_ISREG(st.st_mode) && st.st_size > 0)
    {
      ferr("wzipfs: backing '%s' is non-empty; "
           "remove it before mounting\n", src);
      file_close(&priv->backing);
      return -EEXIST;
    }

  return 0;
}

/****************************************************************************
 * Name: wzipfs_create_archive
 *
 * Allocate the libarchive writer + reusable entry, set mount-time
 * compression method, and open the write stream onto priv->backing.
 ****************************************************************************/

static int wzipfs_create_archive(FAR struct wzipfs_priv_s *priv,
                                 bool deflate)
{
  int ret;

  priv->a = archive_write_new();
  if (priv->a == NULL)
    {
      return -ENOMEM;
    }

  priv->entry = archive_entry_new();
  if (priv->entry == NULL)
    {
      ret = -ENOMEM;
      goto err;
    }

  ret = wzipfs_convert_result(archive_write_set_format_zip(priv->a));
  if (ret < 0)
    {
      ferr("wzipfs: set_format_zip: %s\n", archive_error_string(priv->a));
      goto err;
    }

  ret = wzipfs_convert_result(archive_write_set_format_option(priv->a,
                                                              "zip",
                                                              "hdrcharset",
                                                              "UTF-8"));
  if (ret < 0)
    {
      ferr("wzipfs: hdrcharset=UTF-8: %s\n", archive_error_string(priv->a));
      goto err;
    }

  /* Configure mount-time compression method once for the entire session.
   * libarchive's COMPRESSION_UNSPECIFIED fallback is DEFLATE when zlib is
   * compiled in, so STORE must be selected explicitly.
   */

  if (deflate)
    {
#ifdef CONFIG_FS_ZIPFS_WRITE_DEFLATE
      ret = wzipfs_convert_result(
          archive_write_zip_set_compression_deflate(priv->a));
      if (ret < 0)
        {
          ferr("wzipfs: set_compression_deflate: %s\n",
               archive_error_string(priv->a));
          goto err;
        }
#else
      ret = -ENOTSUP;
      goto err;
#endif
    }
  else
    {
      ret = wzipfs_convert_result(
          archive_write_zip_set_compression_store(priv->a));
      if (ret < 0)
        {
          ferr("wzipfs: set_compression_store: %s\n",
               archive_error_string(priv->a));
          goto err;
        }
    }

  ret = wzipfs_convert_result(archive_write_open2(priv->a, priv, NULL,
                                                  wzipfs_write_cb,
                                                  NULL, NULL));
  if (ret < 0)
    {
      ferr("wzipfs: archive_write_open2: %s\n",
           archive_error_string(priv->a));
      goto err;
    }

  return 0;

err:
  if (priv->entry != NULL)
    {
      archive_entry_free(priv->entry);
      priv->entry = NULL;
    }

  archive_write_free(priv->a);
  priv->a = NULL;
  return ret;
}

/****************************************************************************
 * Name: wzipfs_open
 ****************************************************************************/

static int wzipfs_open(FAR struct file *filep, FAR const char *relpath,
                       int oflags, mode_t mode)
{
  FAR struct wzipfs_priv_s *priv = filep->f_inode->i_private;
  int ret;

  if ((oflags & O_ACCMODE) == O_RDONLY)
    {
      return -EACCES;
    }

  if ((oflags & O_CREAT) == 0)
    {
      return -ENOSYS;
    }

  if (*relpath == '\0')
    {
      return -EINVAL;
    }

  ret = nxsem_wait_uninterruptible(&priv->open_sem);
  if (ret < 0)
    {
      return ret;
    }

  wzipfs_init_entry(priv, relpath, S_IFREG | mode);

  ret = wzipfs_convert_result(archive_write_header(priv->a, priv->entry));
  if (ret < 0)
    {
      ferr("wzipfs: open write_header: %s\n",
           archive_error_string(priv->a));
      nxsem_post(&priv->open_sem);
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: wzipfs_close
 ****************************************************************************/

static int wzipfs_close(FAR struct file *filep)
{
  FAR struct wzipfs_priv_s *priv = filep->f_inode->i_private;
  int ret;

  ret = wzipfs_convert_result(archive_write_finish_entry(priv->a));
  if (ret < 0)
    {
      ferr("wzipfs: finish_entry: %s\n", archive_error_string(priv->a));
    }

  nxsem_post(&priv->open_sem);
  return ret;
}

/****************************************************************************
 * Name: wzipfs_write
 ****************************************************************************/

static ssize_t wzipfs_write(FAR struct file *filep,
                            FAR const char *buffer, size_t buflen)
{
  FAR struct wzipfs_priv_s *priv = filep->f_inode->i_private;
  la_ssize_t ret;

  ret = archive_write_data(priv->a, buffer, buflen);
  if (ret < 0)
    {
      ferr("wzipfs: write_data: %s\n", archive_error_string(priv->a));
      return wzipfs_convert_result(ret);
    }

  return ret;
}

/****************************************************************************
 * Name: wzipfs_sync
 *
 * Flush the backing file.  Only the bytes already emitted by libarchive
 * (through the write callback) are guaranteed to be persisted; libarchive
 * retains its own block buffer internally with no public flush hook, so
 * data descriptors for an in-progress entry may still be pending in RAM.
 ****************************************************************************/

static int wzipfs_sync(FAR struct file *filep)
{
  FAR struct wzipfs_priv_s *priv = filep->f_inode->i_private;

  return file_fsync(&priv->backing);
}

/****************************************************************************
 * Name: wzipfs_bind
 ****************************************************************************/

static int wzipfs_bind(FAR struct inode *driver, FAR const void *data,
                       FAR void **handle)
{
  FAR struct wzipfs_priv_s *priv;
  char src[PATH_MAX];
  bool deflate;
  int ret;

  ret = wzipfs_parse_options(data, src, sizeof(src), &deflate);
  if (ret < 0)
    {
      return ret;
    }

  priv = fs_heap_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  nxsem_init(&priv->open_sem, 0, 1);

  ret = wzipfs_create_backing(priv, src);
  if (ret < 0)
    {
      goto err;
    }

  ret = wzipfs_create_archive(priv, deflate);
  if (ret < 0)
    {
      goto err;
    }

  *handle = priv;
  return 0;

err:
  wzipfs_unbind(priv, NULL, 0);
  return ret;
}

/****************************************************************************
 * Name: wzipfs_unbind
 ****************************************************************************/

static int wzipfs_unbind(FAR void *handle, FAR struct inode **driver,
                         unsigned int flags)
{
  FAR struct wzipfs_priv_s *priv = handle;
  int ret = 0;

  if (priv->a != NULL)
    {
      ret = wzipfs_convert_result(archive_write_close(priv->a));
      if (ret < 0)
        {
          ferr("wzipfs: unbind archive_write_close: %s\n",
               archive_error_string(priv->a));
        }

      archive_write_free(priv->a);
      priv->a = NULL;
    }

  if (priv->entry != NULL)
    {
      archive_entry_free(priv->entry);
      priv->entry = NULL;
    }

  file_close(&priv->backing);
  nxsem_destroy(&priv->open_sem);
  fs_heap_free(priv);

  return ret;
}

/****************************************************************************
 * Name: wzipfs_statfs
 ****************************************************************************/

static int wzipfs_statfs(FAR struct inode *mountpt,
                         FAR struct statfs *buf)
{
  memset(buf, 0, sizeof(*buf));
  buf->f_type    = ZIPFS_MAGIC;
  buf->f_namelen = NAME_MAX;
  return 0;
}

/****************************************************************************
 * Name: wzipfs_mkdir
 ****************************************************************************/

static int wzipfs_mkdir(FAR struct inode *mountpt,
                        FAR const char *relpath, mode_t mode)
{
  FAR struct wzipfs_priv_s *priv = mountpt->i_private;
  char path[PATH_MAX];
  size_t len;
  int ret;

  len = strlen(relpath);
  if (len == 0 || len + 1 >= sizeof(path))
    {
      return -EINVAL;
    }

  /* Build "dir/" -- trailing slash marks directory entries in ZIP. */

  memcpy(path, relpath, len);
  path[len]     = '/';
  path[len + 1] = '\0';

  wzipfs_init_entry(priv, path, S_IFDIR | mode);

  ret = wzipfs_convert_result(archive_write_header(priv->a, priv->entry));
  if (ret < 0)
    {
      ferr("wzipfs: mkdir header: %s\n", archive_error_string(priv->a));
      return ret;
    }

  ret = wzipfs_convert_result(archive_write_finish_entry(priv->a));
  if (ret < 0)
    {
      ferr("wzipfs: mkdir finish: %s\n", archive_error_string(priv->a));
    }

  return ret;
}

