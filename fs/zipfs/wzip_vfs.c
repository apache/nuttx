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
static int wzipfs_stat(FAR struct inode *mountpt,
                       FAR const char *relpath, FAR struct stat *buf);

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
  wzipfs_stat,           /* stat */
  NULL                   /* chstat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wzipfs_convert_result
 *
 * Description:
 *   Convert a libarchive return code to a negated errno value. ARCHIVE_RETRY
 *   becomes -EAGAIN, ARCHIVE_WARN becomes -ENOEXEC, ARCHIVE_FAILED becomes
 *   -EINVAL and ARCHIVE_FATAL becomes -EPERM. Any other value, such as
 *   ARCHIVE_OK, ARCHIVE_EOF or a byte count, is returned unchanged.
 *
 * Input Parameters:
 *   ret - The value that a libarchive function returned.
 *
 * Returned Value:
 *   The negated errno value for a libarchive error, otherwise ret.
 *
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
 *   Parse the mount option string. The accepted options are src=<path>,
 *   fd=<N> and method=store|deflate, separated by commas. A string without =
 *   and , is accepted as the path of the backing target, as src=. One of
 *   src= and fd= is required, and they exclude each other.
 *
 * Input Parameters:
 *   data    - The mount option string.
 *   src     - Buffer to return the path of the backing target. It is empty
 *             when fd= is used.
 *   len     - Size of the buffer src.
 *   fd      - Location to return the file descriptor number of fd=, or -1.
 *   deflate - Location to return true for method=deflate, otherwise false.
 *
 * Returned Value:
 *   Zero on success. -ENODEV if data is NULL or empty, or if neither src=
 *   nor fd= is given. -ENAMETOOLONG if a path without src= does not fit in
 *   src. -EINVAL if src= does not fit in src, the value of fd= is not valid,
 *   the method or the option is unknown, or both src= and fd= are given.
 *   -ENOTSUP for method=deflate when CONFIG_FS_ZIPFS_WRITE_DEFLATE is not
 *   set.
 *
 ****************************************************************************/

static int wzipfs_parse_options(FAR const void *data,
                                FAR char *src, size_t len,
                                FAR int *fd, FAR bool *deflate)
{
  FAR const char *p = data;
  FAR const char *end;

  src[0]   = '\0';
  *fd      = -1;
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
      else if (end - p > 3 && memcmp(p, "fd=", 3) == 0)
        {
          FAR char *tail;
          long      v;

          v = strtol(p + 3, &tail, 0);
          if (tail != end || v < 0 || v > INT_MAX)
            {
              ferr("wzipfs: invalid fd value\n");
              return -EINVAL;
            }

          *fd = (int)v;
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

  if (*fd >= 0 && src[0] != '\0')
    {
      ferr("wzipfs: src= and fd= are mutually exclusive\n");
      return -EINVAL;
    }

  if (*fd < 0 && src[0] == '\0')
    {
      return -ENODEV;
    }

  return 0;
}

/****************************************************************************
 * Name: wzipfs_write_cb
 *
 * Description:
 *   libarchive write callback. Write the data to the backing file.
 *
 * Input Parameters:
 *   a           - The libarchive handle. Not used.
 *   client_data - The struct wzipfs_priv_s of the mount.
 *   buff        - The data to write.
 *   len         - Number of bytes to write.
 *
 * Returned Value:
 *   The number of bytes written, or the negated errno value of file_write().
 *
 ****************************************************************************/

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
 * Description:
 *   Prepare the reusable archive_entry in priv->entry for a header write:
 *   set the name, the mode, the modification time (now) and the file type,
 *   which is a directory if mode is a directory and a regular file
 *   otherwise. The size is left unset on purpose: for a regular file this
 *   selects the streaming (data descriptor) write of libarchive, and for a
 *   directory the ZIP writer sets the size to 0.
 *
 * Input Parameters:
 *   priv - The state of the mount.
 *   path - Name of the entry. The name of a directory ends with /.
 *   mode - File type and permission bits, S_IFREG or S_IFDIR with the
 *          permissions.
 *
 * Returned Value:
 *   None.
 *
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
 * Description:
 *   Open the backing target for writing, and create it if it does not exist.
 *   A regular file must be empty: one with a size greater than zero is
 *   rejected. Any other target, such as a FIFO or a socket, is used without
 *   a check. On failure the backing file is closed.
 *
 * Input Parameters:
 *   priv - The state of the mount. priv->backing is opened.
 *   src  - Path of the backing target.
 *
 * Returned Value:
 *   Zero on success, -EEXIST for a regular file that is not empty, or the
 *   negated errno value of file_open() or file_fstat().
 *
 * Assumptions/Limitations:
 *   To replace an existing archive, remove or truncate the backing file
 *   before mounting.
 *
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
 * Description:
 *   Create the libarchive writer and the reusable archive_entry, select the
 *   zip format with UTF-8 names, select the compression of the whole mount,
 *   and open the write stream on priv->backing. libarchive uses DEFLATE when
 *   it is not told otherwise and zlib is built in, so STORE is selected
 *   explicitly.
 *
 * Input Parameters:
 *   priv    - The state of the mount. priv->a and priv->entry are set.
 *   deflate - True to compress with DEFLATE, false to store. DEFLATE needs
 *             CONFIG_FS_ZIPFS_WRITE_DEFLATE.
 *
 * Returned Value:
 *   Zero on success, -ENOMEM if memory runs out, -ENOTSUP for DEFLATE
 *   without CONFIG_FS_ZIPFS_WRITE_DEFLATE, or the negated errno value
 *   converted from a libarchive error. On failure the writer and the entry
 *   are freed and priv->a and priv->entry are NULL.
 *
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
 *
 * Description:
 *   Start a new entry of the archive. Only one file can be open for writing
 *   at a time: this function waits until the previous file is closed. The
 *   header of the entry is written with archive_write_header(). The file
 *   must be opened for writing with O_CREAT.
 *
 * Input Parameters:
 *   filep   - The file structure. The state of the mount is in the i_private
 *             field of its inode.
 *   relpath - Name of the entry, relative to the mount point.
 *   oflags  - Open flags.
 *   mode    - Permission bits of the entry.
 *
 * Returned Value:
 *   Zero on success. -EACCES if the file is opened read only, -ENOSYS if
 *   O_CREAT is not set, -EINVAL if relpath is empty, or the negated errno
 *   value of the wait or of writing the header. If writing the header fails,
 *   the lock is released before the function returns.
 *
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
 *
 * Description:
 *   Finish the current entry with archive_write_finish_entry() and release
 *   the lock that wzipfs_open() took, so that the next file can be opened.
 *
 * Input Parameters:
 *   filep - The file structure.
 *
 * Returned Value:
 *   Zero on success, otherwise the negated errno value converted from a
 *   libarchive error. The lock is released in both cases.
 *
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
 *
 * Description:
 *   Append data to the current entry with archive_write_data(). libarchive
 *   collects what the zip writer produces in blocks (10240 bytes by default)
 *   and calls wzipfs_write_cb() for each full block, so the data reaches the
 *   backing target block by block. The rest is written when the archive is
 *   closed at unmount.
 *
 * Input Parameters:
 *   filep  - The file structure.
 *   buffer - The data to write.
 *   buflen - Number of bytes to write.
 *
 * Returned Value:
 *   The number of bytes written, or a negated errno value.
 *
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
 * Description:
 *   Flush the backing file with file_fsync(). Only the bytes that libarchive
 *   already passed to the write callback are persisted. libarchive keeps its
 *   own block buffer and has no public way to flush it, so the data
 *   descriptors of an entry that is being written can still be in memory.
 *
 * Input Parameters:
 *   filep - The file structure.
 *
 * Returned Value:
 *   Zero on success, otherwise the negated errno value of file_fsync().
 *
 ****************************************************************************/

static int wzipfs_sync(FAR struct file *filep)
{
  FAR struct wzipfs_priv_s *priv = filep->f_inode->i_private;

  return file_fsync(&priv->backing);
}

/****************************************************************************
 * Name: wzipfs_bind
 *
 * Description:
 *   Mount: parse the options, allocate the state of the mount, and open the
 *   backing target. With fd=, the file descriptor is duplicated into the
 *   state of the mount, and with src= the path is opened with
 *   wzipfs_create_backing(). Then the libarchive writer is created with
 *   wzipfs_create_archive(). No block driver is used.
 *
 * Input Parameters:
 *   driver - Not used.
 *   data   - The mount option string, see wzipfs_parse_options().
 *   handle - Location to return the state of the mount.
 *
 * Returned Value:
 *   Zero on success, otherwise a negated errno value: an error of
 *   wzipfs_parse_options(), -ENOMEM, an error of file_get() or file_dup2()
 *   for fd=, or an error of wzipfs_create_backing() or
 *   wzipfs_create_archive(). On failure the state of the mount is freed.
 *
 * Assumptions/Limitations:
 *   With fd=, the caller keeps its own file descriptor and has to keep it
 *   open until the file system is unmounted.
 *
 ****************************************************************************/

static int wzipfs_bind(FAR struct inode *driver, FAR const void *data,
                       FAR void **handle)
{
  FAR struct wzipfs_priv_s *priv;
  FAR struct file *src_filep;
  char src[PATH_MAX];
  bool deflate;
  int ret;
  int fd;

  ret = wzipfs_parse_options(data, src, sizeof(src), &fd, &deflate);
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

  if (fd >= 0)
    {
      ret = file_get(fd, &src_filep);
      if (ret >= 0)
        {
          ret = file_dup2(src_filep, &priv->backing);
          file_put(src_filep);
        }
    }
  else
    {
      ret = wzipfs_create_backing(priv, src);
    }

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
 *
 * Description:
 *   Unmount: close the archive with archive_write_close(), which writes the
 *   central directory, free the writer and the entry, close the backing
 *   file, and free the state of the mount.
 *
 * Input Parameters:
 *   handle - The state of the mount that wzipfs_bind() returned.
 *   driver - Not used.
 *   flags  - Not used.
 *
 * Returned Value:
 *   Zero on success, or the negated errno value converted from the error of
 *   archive_write_close(). The resources are released in both cases.
 *
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
 *
 * Description:
 *   Return information about the file system: the structure is cleared,
 *   f_type is set to ZIPFS_MAGIC and f_namelen to NAME_MAX.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode. Not used.
 *   buf     - Location to return the information.
 *
 * Returned Value:
 *   Zero always.
 *
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
 *
 * Description:
 *   Add a directory entry to the archive. The name is relpath followed by /,
 *   and its header is written and finished at once.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode. The state of the mount is in its
 *             i_private field.
 *   relpath - Name of the directory, relative to the mount point.
 *   mode    - Permission bits of the directory.
 *
 * Returned Value:
 *   Zero on success, -EINVAL if relpath is empty or too long, otherwise the
 *   negated errno value converted from a libarchive error.
 *
 * Assumptions/Limitations:
 *   It does not take the lock that wzipfs_open() takes. Call it only when no
 *   file of the mount is open for writing.
 *
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

/****************************************************************************
 * Name: wzipfs_stat
 *
 * Description:
 *   Minimal stat of wzipfs: whatever the path is, report a directory with
 *   mode 0755, so that rpmsgfs can verify the root of the mount without
 *   blocking. wzipfs does not keep information about the entries that were
 *   written.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode. Not used.
 *   relpath - Not used.
 *   buf     - Location to return the stat. It is cleared first.
 *
 * Returned Value:
 *   Zero always.
 *
 ****************************************************************************/

static int wzipfs_stat(FAR struct inode *mountpt,
                       FAR const char *relpath, FAR struct stat *buf)
{
  memset(buf, 0, sizeof(*buf));
  buf->st_mode = S_IFDIR | 0755;
  return 0;
}
