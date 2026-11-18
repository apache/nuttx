/****************************************************************************
 * fs/zipfs/zip_vfs.c
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

#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>

#include "fs_heap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct zipfs_priv_s
{
  FAR struct archive_entry *entry;
  FAR struct archive *a;

  /* callback data */

  struct file file;
  char buffer[CONFIG_FS_ZIPFS_BUFFER_SIZE];

  /* seek data */

  FAR char *seekbuf;
  mutex_t lock;
};

struct zipfs_dir_s
{
  struct fs_dirent_s dir;
  FAR struct zipfs_priv_s *priv;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int zipfs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode);
static int zipfs_close(FAR struct file *filep);
static ssize_t zipfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static off_t zipfs_seek(FAR struct file *filep, off_t offset,
                        int whence);
static int zipfs_dup(FAR const struct file *oldp,
                     FAR struct file *newp);
static int zipfs_fstat(FAR const struct file *filep,
                       FAR struct stat *buf);
static int zipfs_opendir(FAR struct inode *mountpt,
                         FAR const char *relpath,
                         FAR struct fs_dirent_s **dir);
static int zipfs_closedir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir);
static int zipfs_readdir(FAR struct inode *mountpt,
                         FAR struct fs_dirent_s *dir,
                         FAR struct dirent *entry);
static int zipfs_rewinddir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir);
static int zipfs_statfs(FAR struct inode *mountpt,
                        FAR struct statfs *buf);
static int zipfs_stat(FAR struct inode *mountpt,
                      FAR const char *relpath,
                      FAR struct stat *buf);
static int zipfs_bind(FAR struct inode *driver, FAR const void *data,
                      FAR void **handle);
static int zipfs_unbind(FAR void *handle, FAR struct inode **driver,
                        unsigned int flags);

/****************************************************************************
 * Private Data
 ****************************************************************************/

const struct mountpt_operations g_zipfs_operations =
{
  zipfs_open,          /* open */
  zipfs_close,         /* close */
  zipfs_read,          /* read */
  NULL,                /* write */
  zipfs_seek,          /* seek */
  NULL,                /* ioctl */
  NULL,                /* mmap */
  NULL,                /* truncate */
  NULL,                /* poll */
  NULL,                /* readv */
  NULL,                /* writev */
  NULL,                /* sync */
  zipfs_dup,           /* dup */
  zipfs_fstat,         /* fstat */
  NULL,                /* fchstat */

  zipfs_opendir,       /* opendir */
  zipfs_closedir,      /* closedir */
  zipfs_readdir,       /* readdir */
  zipfs_rewinddir,     /* rewinddir */

  zipfs_bind,          /* bind */
  zipfs_unbind,        /* unbind */
  zipfs_statfs,        /* statfs */

  NULL,                /* unlink */
  NULL,                /* mkdir */
  NULL,                /* rmdir */
  NULL,                /* rename */
  zipfs_stat,          /* stat */
  NULL                 /* chstat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: zipfs_convert_result
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

static int zipfs_convert_result(int ret)
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
 * Name: zipfs_close_cb
 *
 * Description:
 *   libarchive close callback. Close the zip file that the reader reads.
 *
 * Input Parameters:
 *   a           - The libarchive handle. Not used.
 *   client_data - The struct zipfs_priv_s of the reader.
 *
 * Returned Value:
 *   Zero (OK) on success, otherwise the negated errno value of file_close().
 *
 ****************************************************************************/

static int zipfs_close_cb(FAR struct archive *a, FAR void *client_data)
{
  FAR struct zipfs_priv_s *priv = client_data;

  return file_close(&priv->file);
}

/****************************************************************************
 * Name: zipfs_read_cb
 *
 * Description:
 *   libarchive read callback. Read the next block of the zip file, at most
 *   CONFIG_FS_ZIPFS_BUFFER_SIZE bytes, into the buffer of the reader.
 *
 * Input Parameters:
 *   a           - The libarchive handle. Not used.
 *   client_data - The struct zipfs_priv_s of the reader.
 *   buff        - Location to return a pointer to the data that was read. It
 *                 is set only when data was read.
 *
 * Returned Value:
 *   The number of bytes read, zero at the end of the zip file, or the
 *   negated errno value of file_read().
 *
 ****************************************************************************/

static la_ssize_t zipfs_read_cb(FAR struct archive *a,
                                FAR void *client_data,
                                FAR const void **buff)
{
  FAR struct zipfs_priv_s *priv = client_data;
  ssize_t ret;

  ret = file_read(&priv->file, priv->buffer,
                  CONFIG_FS_ZIPFS_BUFFER_SIZE);
  if (ret <= 0)
    {
      return ret;
    }

  *buff = priv->buffer;
  return ret;
}

/****************************************************************************
 * Name: zipfs_seek_cb
 *
 * Description:
 *   libarchive seek callback. Set the position in the zip file.
 *
 * Input Parameters:
 *   a           - The libarchive handle. Not used.
 *   client_data - The struct zipfs_priv_s of the reader.
 *   offset      - The new position, relative to whence.
 *   whence      - SEEK_SET, SEEK_CUR or SEEK_END.
 *
 * Returned Value:
 *   The new position in the zip file, or the negated errno value of
 *   file_seek().
 *
 ****************************************************************************/

static la_int64_t zipfs_seek_cb(FAR struct archive *a,
                                FAR void *client_data,
                                la_int64_t offset, int whence)
{
  FAR struct zipfs_priv_s *priv = client_data;

  return file_seek(&priv->file, offset, whence);
}

/****************************************************************************
 * Name: zipfs_skip
 *
 * Description:
 *   Skip data of the current entry by reading and discarding it. The data is
 *   read in blocks of CONFIG_FS_ZIPFS_BUFFER_SIZE bytes into a buffer that
 *   is allocated on the first call and freed by zipfs_free().
 *
 * Input Parameters:
 *   priv   - The reader, positioned in the data of an entry.
 *   amount - The number of bytes to skip.
 *
 * Returned Value:
 *   The number of bytes skipped. It is less than amount when the data ends
 *   or a read fails after some bytes were skipped. If no byte was skipped,
 *   the value is zero at the end of the data, -ENOMEM if the buffer cannot
 *   be allocated, or the negated errno value converted from a libarchive
 *   error.
 *
 ****************************************************************************/

static off_t zipfs_skip(FAR struct zipfs_priv_s *priv, off_t amount)
{
  off_t next = 0;

  if (priv->seekbuf == NULL)
    {
      priv->seekbuf = fs_heap_malloc(CONFIG_FS_ZIPFS_BUFFER_SIZE);
      if (priv->seekbuf == NULL)
        {
          return -ENOMEM;
        }
    }

  while (next < amount)
    {
      off_t remain = amount - next;

      if (remain > CONFIG_FS_ZIPFS_BUFFER_SIZE)
        {
          remain = CONFIG_FS_ZIPFS_BUFFER_SIZE;
        }

      remain = archive_read_data(priv->a, priv->seekbuf, remain);
      remain = zipfs_convert_result(remain);
      if (remain <= 0)
        {
          return next ? next : remain;
        }

      next += remain;
    }

  return next;
}

/****************************************************************************
 * Name: zipfs_new
 *
 * Description:
 *   Open the zip file with libarchive and create a reader for it. The
 *   archive formats that are selected with CONFIG_FS_ZIPFS_FORMAT_* are
 *   enabled. If relpath is NULL, the reader is returned before the first
 *   entry, which is how a directory is read. Otherwise the entries are read
 *   until one has the name relpath, and the reader is returned positioned in
 *   the data of that entry.
 *
 * Input Parameters:
 *   abspath - Path of the zip file.
 *   relpath - Name of the entry to find, as it is stored in the archive, or
 *             NULL.
 *   priv    - Location to return the reader.
 *
 * Returned Value:
 *   Zero (OK) on success. -ENOMEM if memory runs out, -ENOENT if no entry
 *   has the name relpath (also when reading the archive fails during the
 *   search), or the negated errno value of file_open() or of a libarchive
 *   function.
 *
 * Assumptions/Limitations:
 *   The caller releases the reader with zipfs_free().
 *
 ****************************************************************************/

int zipfs_new(FAR const char *abspath, FAR const char *relpath,
              FAR struct zipfs_priv_s **priv)
{
  FAR struct zipfs_priv_s *newp;
  int ret = -ENOMEM;

  newp = fs_heap_zalloc(sizeof(struct zipfs_priv_s));
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
#ifdef CONFIG_FS_ZIPFS_FORMAT_ALL
  ret = zipfs_convert_result(archive_read_support_format_all(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_all() failed\n");
      goto err_with_lock;
    }
#else
#  ifdef CONFIG_FS_ZIPFS_FORMAT_ZIP
  ret = zipfs_convert_result(archive_read_support_format_zip(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_zip() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ZIPFS_FORMAT_7ZIP
  ret = zipfs_convert_result(archive_read_support_format_7zip(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_7zip() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ZIPFS_FORMAT_AR
  ret = zipfs_convert_result(archive_read_support_format_ar(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_ar() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ZIPFS_FORMAT_CAB
  ret = zipfs_convert_result(archive_read_support_format_cab(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_cab() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ZIPFS_FORMAT_CPIO
  ret = zipfs_convert_result(archive_read_support_format_cpio(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_cpio() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ZIPFS_FORMAT_EMPTY
  ret = zipfs_convert_result(archive_read_support_format_empty(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_empty() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ZIPFS_FORMAT_ISO9660
  ret = archive_read_support_format_iso9660(newp->a);
  ret = zipfs_convert_result(ret);
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_iso9660() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ZIPFS_FORMAT_LHA
  ret = zipfs_convert_result(archive_read_support_format_lha(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_lha() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ZIPFS_FORMAT_MTREE
  ret = zipfs_convert_result(archive_read_support_format_mtree(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_mtree() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ZIPFS_FORMAT_RAR
  ret = zipfs_convert_result(archive_read_support_format_rar(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_rar() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ZIPFS_FORMAT_RAR_V5
  ret = zipfs_convert_result(archive_read_support_format_rar5(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_rar5() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ZIPFS_FORMAT_RAW
  ret = zipfs_convert_result(archive_read_support_format_raw(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_raw() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ZIPFS_FORMAT_TAR
  ret = zipfs_convert_result(archive_read_support_format_tar(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_tar() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ZIPFS_FORMAT_WARC
  ret = zipfs_convert_result(archive_read_support_format_warc(newp->a));
  if (ret < 0)
    {
      ferr("ERROR: archive_read_support_format_warc() failed\n");
      goto err_with_lock;
    }
#  endif

#  ifdef CONFIG_FS_ZIPFS_FORMAT_XAR
  ret = zipfs_convert_result(archive_read_support_format_xar(newp->a));
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

  archive_read_set_seek_callback(newp->a, zipfs_seek_cb);
  ret = archive_read_open(newp->a, newp, NULL,
                          zipfs_read_cb, zipfs_close_cb);
  ret = zipfs_convert_result(ret);
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
      FAR const char *name;

      ret = archive_read_next_header2(newp->a, newp->entry);
      ret = zipfs_convert_result(ret);

      /* Stop at end of archive (entry cleared, name NULL) or on error */

      if (ret == ARCHIVE_EOF || ret < 0)
        {
          break;
        }

      name = archive_entry_pathname(newp->entry);
      if (name != NULL && strcmp(name, relpath) == 0)
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

/****************************************************************************
 * Name: zipfs_free
 *
 * Description:
 *   Release a reader: the skip buffer, the current entry, the libarchive
 *   handle (which closes the zip file), the mutex and the structure itself.
 *
 * Input Parameters:
 *   priv - The reader that zipfs_new() returned.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void zipfs_free(FAR struct zipfs_priv_s *priv)
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

/****************************************************************************
 * Name: zipfs_stats_common
 *
 * Description:
 *   Copy the stat of the current entry of the reader.
 *
 * Input Parameters:
 *   priv - The reader, positioned on an entry.
 *   buf  - Location to return the stat.
 *
 * Returned Value:
 *   Zero (OK) on success, -EINVAL if libarchive has no stat for the entry.
 *
 ****************************************************************************/

static int zipfs_stats_common(FAR struct zipfs_priv_s *priv,
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
 * Zipfs methods
 ****************************************************************************/

/****************************************************************************
 * Name: zipfs_open
 *
 * Description:
 *   Open a file of the archive. A new reader is created with zipfs_new() and
 *   is kept in the f_priv field of the file structure.
 *
 * Input Parameters:
 *   filep   - The file structure. The path of the zip file is in the
 *             i_private field of its inode.
 *   relpath - Name of the entry, relative to the mount point.
 *   oflags  - Not used.
 *   mode    - Not used.
 *
 * Returned Value:
 *   Zero (OK) on success, otherwise the negated errno value of zipfs_new(),
 *   for example -ENOENT if the archive has no entry with that name.
 *
 ****************************************************************************/

static int zipfs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR const char *abspath = filep->f_inode->i_private;
  FAR struct zipfs_priv_s *priv;
  int ret;

  ret = zipfs_new(abspath, relpath, &priv);
  if (ret < 0)
    {
      return ret;
    }

  filep->f_priv = priv;
  return OK;
}

/****************************************************************************
 * Name: zipfs_close
 *
 * Description:
 *   Close a file: release its reader with zipfs_free().
 *
 * Input Parameters:
 *   filep - The file structure.
 *
 * Returned Value:
 *   Zero (OK) always.
 *
 ****************************************************************************/

static int zipfs_close(FAR struct file *filep)
{
  FAR struct zipfs_priv_s *priv = filep->f_priv;

  zipfs_free(priv);
  return OK;
}

/****************************************************************************
 * Name: zipfs_read
 *
 * Description:
 *   Read data of an open file. Up to buflen bytes of the current entry are
 *   read with archive_read_data(), and the file position is advanced by the
 *   number of bytes read. The reader is locked.
 *
 * Input Parameters:
 *   filep  - The file structure.
 *   buffer - Location to return the data.
 *   buflen - Maximum number of bytes to read.
 *
 * Returned Value:
 *   The number of bytes read, zero at the end of the entry, or a negated
 *   errno value.
 *
 ****************************************************************************/

static ssize_t zipfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct zipfs_priv_s *priv = filep->f_priv;
  ssize_t ret;

  nxmutex_lock(&priv->lock);
  ret = zipfs_convert_result(archive_read_data(priv->a, buffer, buflen));
  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: zipfs_seek
 *
 * Description:
 *   Set the position of an open file. A libarchive reader only moves
 *   forward, so a forward seek reads and discards data with zipfs_skip(),
 *   and a backward seek creates a new reader for the same entry and skips
 *   from its beginning. SEEK_END is relative to the size of the entry.
 *
 * Input Parameters:
 *   filep  - The file structure.
 *   offset - The new position, relative to whence.
 *   whence - SEEK_SET, SEEK_CUR or SEEK_END.
 *
 * Returned Value:
 *   The new file position, or a negated errno value: -EINVAL for another
 *   value of whence, or an error of zipfs_new() or zipfs_skip(). If a new
 *   reader cannot be created, the old one is kept.
 *
 ****************************************************************************/

static off_t zipfs_seek(FAR struct file *filep, off_t offset,
                        int whence)
{
  FAR const char *abspath = filep->f_inode->i_private;
  FAR struct zipfs_priv_s *priv = filep->f_priv;
  FAR struct zipfs_priv_s *newp;
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
      ret = zipfs_new(abspath, archive_entry_pathname(priv->entry),
                      &newp);
      if (ret < 0)
        {
          goto err_with_lock;
        }

      nxmutex_unlock(&priv->lock);
      zipfs_free(priv);
      priv = newp;
      nxmutex_lock(&priv->lock);
      filep->f_priv = newp;
      filep->f_pos = 0;
    }

  ret = zipfs_skip(priv, offset - filep->f_pos);
  if (ret < 0)
    {
      goto err_with_lock;
    }

  filep->f_pos += ret;

err_with_lock:
  nxmutex_unlock(&priv->lock);
  return ret < 0 ? ret : filep->f_pos;
}

/****************************************************************************
 * Name: zipfs_dup
 *
 * Description:
 *   Duplicate an open file. The entry of oldp is opened again with
 *   zipfs_open(), so newp has its own reader that starts at the beginning of
 *   the entry.
 *
 * Input Parameters:
 *   oldp - The file structure to duplicate.
 *   newp - The new file structure.
 *
 * Returned Value:
 *   Zero (OK) on success, otherwise the negated errno value of zipfs_open().
 *
 ****************************************************************************/

static int zipfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct zipfs_priv_s *priv = oldp->f_priv;
  FAR const char *relpath = archive_entry_pathname(priv->entry);

  return zipfs_open(newp, relpath, oldp->f_oflags, 0);
}

/****************************************************************************
 * Name: zipfs_fstat
 *
 * Description:
 *   Return information about an open file: the stat of its entry.
 *
 * Input Parameters:
 *   filep - The file structure.
 *   buf   - Location to return the stat.
 *
 * Returned Value:
 *   Zero (OK) on success, -EINVAL if libarchive has no stat for the entry.
 *
 ****************************************************************************/

static int zipfs_fstat(FAR const struct file *filep,
                       FAR struct stat *buf)
{
  FAR struct zipfs_priv_s *priv = filep->f_priv;

  return zipfs_stats_common(priv, buf);
}

/****************************************************************************
 * Name: zipfs_opendir
 *
 * Description:
 *   Open a directory. A reader positioned before the first entry of the
 *   archive is created with zipfs_new() and is kept in a new directory
 *   structure.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode. The path of the zip file is in its
 *             i_private field.
 *   relpath - Not used.
 *   dir     - Location to return the directory structure.
 *
 * Returned Value:
 *   Zero (OK) on success, -ENOMEM if memory runs out, or the negated errno
 *   value of zipfs_new().
 *
 * Assumptions/Limitations:
 *   relpath is not used: every directory lists all the entries of the
 *   archive, with their full names, whichever directory is opened.
 *
 ****************************************************************************/

static int zipfs_opendir(FAR struct inode *mountpt,
                         FAR const char *relpath,
                         FAR struct fs_dirent_s **dir)
{
  FAR const char *abspath = mountpt->i_private;
  FAR struct zipfs_dir_s *adir;
  int ret;

  adir = fs_heap_zalloc(sizeof(struct zipfs_dir_s));
  if (adir == NULL)
    {
      return -ENOMEM;
    }

  ret = zipfs_new(abspath, NULL, &adir->priv);
  if (ret < 0)
    {
      fs_heap_free(adir);
      return ret;
    }

  *dir = &adir->dir;
  return OK;
}

/****************************************************************************
 * Name: zipfs_closedir
 *
 * Description:
 *   Close a directory: release its reader and the directory structure.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode.
 *   dir     - The directory structure that zipfs_opendir() returned.
 *
 * Returned Value:
 *   Zero (OK) always.
 *
 ****************************************************************************/

static int zipfs_closedir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir)
{
  FAR struct zipfs_dir_s *adir = (FAR struct zipfs_dir_s *)dir;

  zipfs_free(adir->priv);
  fs_heap_free(adir);
  return OK;
}

/****************************************************************************
 * Name: zipfs_readdir
 *
 * Description:
 *   Read the next entry of the archive. The next header is read with
 *   archive_read_next_header2(), and the full name of the entry is copied to
 *   the d_name field of entry, truncated if it does not fit. The reader is
 *   locked.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode.
 *   dir     - The directory structure that zipfs_opendir() returned.
 *   entry   - Location to return the directory entry.
 *
 * Returned Value:
 *   Zero (OK) when an entry was returned, EOF (-1) at the end of the
 *   archive, or a negated errno value when reading the archive fails.
 *
 ****************************************************************************/

static int zipfs_readdir(FAR struct inode *mountpt,
                         FAR struct fs_dirent_s *dir,
                         FAR struct dirent *entry)
{
  FAR struct zipfs_dir_s *adir = (FAR struct zipfs_dir_s *)dir;
  FAR struct zipfs_priv_s *priv = adir->priv;
  int ret;

  nxmutex_lock(&priv->lock);
  ret = archive_read_next_header2(priv->a, priv->entry);
  ret = zipfs_convert_result(ret);
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

/****************************************************************************
 * Name: zipfs_rewinddir
 *
 * Description:
 *   Replace the current entry of the reader of a directory with a new, empty
 *   one.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode.
 *   dir     - The directory structure that zipfs_opendir() returned.
 *
 * Returned Value:
 *   Zero (OK) on success, -ENOMEM if the new entry cannot be allocated.
 *
 * Assumptions/Limitations:
 *   The libarchive reader is not moved, because it only moves forward and no
 *   new reader is created here. The next zipfs_readdir() continues after the
 *   entries that were already returned.
 *
 ****************************************************************************/

static int zipfs_rewinddir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir)
{
  FAR struct zipfs_dir_s *adir = (FAR struct zipfs_dir_s *)dir;
  FAR struct zipfs_priv_s *priv = adir->priv;

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

/****************************************************************************
 * Name: zipfs_statfs
 *
 * Description:
 *   Return information about the file system: f_type is set to ZIPFS_MAGIC
 *   and f_namelen to NAME_MAX. The other fields are not changed.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode. Not used.
 *   buf     - Location to return the information.
 *
 * Returned Value:
 *   Zero (OK) always.
 *
 ****************************************************************************/

static int zipfs_statfs(FAR struct inode *mountpt,
                        FAR struct statfs *buf)
{
  buf->f_type = ZIPFS_MAGIC;
  buf->f_namelen = NAME_MAX;
  return OK;
}

/****************************************************************************
 * Name: zipfs_stat
 *
 * Description:
 *   Return information about a path. For the root of the mount point, an
 *   empty relpath, the result is a directory with mode 0555, and only
 *   st_mode is set. For any other path the entry is looked up with
 *   zipfs_new() and its stat is returned.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode. The path of the zip file is in its
 *             i_private field.
 *   relpath - Name of the entry, relative to the mount point.
 *   buf     - Location to return the stat.
 *
 * Returned Value:
 *   Zero (OK) on success, -EINVAL if libarchive has no stat for the entry,
 *   otherwise the negated errno value of zipfs_new(), for example -ENOENT.
 *
 * Assumptions/Limitations:
 *   relpath must be equal to the name stored in the archive. A directory is
 *   stored with a trailing /, and it is not found without it.
 *
 ****************************************************************************/

static int zipfs_stat(FAR struct inode *mountpt,
                      FAR const char *relpath,
                      FAR struct stat *buf)
{
  FAR const char *abspath = mountpt->i_private;
  FAR struct zipfs_priv_s *priv;
  int ret;

  if (relpath[0] == 0)
    {
      buf->st_mode = S_IFDIR | 0555;
      return OK;
    }

  ret = zipfs_new(abspath, relpath, &priv);
  if (ret < 0)
    {
      return ret;
    }

  ret = zipfs_stats_common(priv, buf);
  zipfs_free(priv);
  return ret;
}

/****************************************************************************
 * Name: zipfs_bind
 *
 * Description:
 *   Mount a zip file. The zip file is opened once with zipfs_new() to check
 *   that it can be read, and its path is copied to a new buffer that becomes
 *   the mount handle. No block driver is used.
 *
 * Input Parameters:
 *   driver - Not used.
 *   data   - Path of the zip file.
 *   handle - Location to return the mount handle, which is the copy of the
 *            path.
 *
 * Returned Value:
 *   Zero (OK) on success, -ENODEV if data is NULL, -ENOMEM if memory runs
 *   out, or the negated errno value of zipfs_new().
 *
 ****************************************************************************/

static int zipfs_bind(FAR struct inode *driver, FAR const void *data,
                      FAR void **handle)
{
  FAR struct zipfs_priv_s *priv;
  FAR char *abspath;
  int ret;

  if (data == NULL)
    {
      return -ENODEV;
    }

  /* Try access to zip file */

  ret = zipfs_new(data, NULL, &priv);
  if (ret < 0)
    {
      return ret;
    }

  zipfs_free(priv);
  abspath = fs_heap_zalloc(PATH_MAX);
  if (abspath == NULL)
    {
      return -ENOMEM;
    }

  *handle = abspath;
  strlcpy(abspath, data, PATH_MAX);
  return OK;
}

/****************************************************************************
 * Name: zipfs_unbind
 *
 * Description:
 *   Unmount: free the mount handle.
 *
 * Input Parameters:
 *   handle - The mount handle that zipfs_bind() returned.
 *   driver - Not used.
 *   flags  - Not used.
 *
 * Returned Value:
 *   Zero (OK) always.
 *
 ****************************************************************************/

static int zipfs_unbind(FAR void *handle, FAR struct inode **driver,
                        unsigned int flags)
{
  fs_heap_free(handle);
  return OK;
}
