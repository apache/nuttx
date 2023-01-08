/****************************************************************************
 * fs/zipfs/zip_vfs.c
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

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/statfs.h>
#include <nuttx/mutex.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

#include <unzip.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct zipfs_dir_s
{
  struct fs_dirent_s base;
  mutex_t lock;
  unzFile uf;
  bool last;
};

struct zipfs_mountpt_s
{
  char abspath[1];
};

struct zipfs_file_s
{
  unzFile uf;
  mutex_t lock;
  char *seekbuf;
  char relpath[1];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static voidpf zipfs_real_open(voidpf opaque, const void *filename, int mode);
static uLong zipfs_real_read(voidpf opaque, voidpf stream, void *buf,
                             uLong size);
static long zipfs_real_seek(voidpf opaque, voidpf stream, ZPOS64_T offset,
                            int origin);
static ZPOS64_T zipfs_real_tell(voidpf opaque, voidpf stream);
static int zipfs_real_close(voidpf opaque, voidpf stream);
static int zipfs_real_error(voidpf opaque, voidpf stream);

static int     zipfs_open(FAR struct file *filep, FAR const char *relpath,
                          int oflags, mode_t mode);
static int     zipfs_close(FAR struct file *filep);
static ssize_t zipfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static off_t   zipfs_seek(FAR struct file *filep, off_t offset,
                          int whence);
static int     zipfs_dup(FAR const struct file *oldp,
                         FAR struct file *newp);
static int     zipfs_fstat(FAR const struct file *filep,
                           FAR struct stat *buf);
static int     zipfs_opendir(FAR struct inode *mountpt,
                             FAR const char *relpath,
                             FAR struct fs_dirent_s **dir);
static int     zipfs_closedir(FAR struct inode *mountpt,
                              FAR struct fs_dirent_s *dir);
static int     zipfs_readdir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir,
                             FAR struct dirent *entry);
static int     zipfs_rewinddir(FAR struct inode *mountpt,
                               FAR struct fs_dirent_s *dir);
static int     zipfs_bind(FAR struct inode *driver,
                          FAR const void *data, FAR void **handle);
static int     zipfs_unbind(FAR void *handle, FAR struct inode **driver,
                            unsigned int flags);
static int     zipfs_statfs(FAR struct inode *mountpt,
                            FAR struct statfs *buf);
static int     zipfs_stat(FAR struct inode *mountpt,
                          FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static zlib_filefunc64_def zipfs_real_ops =
{
  zipfs_real_open,
  zipfs_real_read,
  NULL,
  zipfs_real_tell,
  zipfs_real_seek,
  zipfs_real_close,
  zipfs_real_error,
  NULL
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct mountpt_operations zipfs_operations =
{
  zipfs_open,          /* open */
  zipfs_close,         /* close */
  zipfs_read,          /* read */
  NULL,                /* write */
  zipfs_seek,          /* seek */
  NULL,                /* ioctl */
  NULL,                /* mmap */
  NULL,                /* truncate */

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

static voidpf zipfs_real_open(voidpf opaque, const void *filename, int mode)
{
  FAR struct file *filep;
  int ret;

  filep = kmm_malloc(sizeof(struct file));
  if (filep == NULL)
    {
      return NULL;
    }

  ret = file_open(filep, filename, O_RDONLY);
  if (ret < 0)
    {
      kmm_free(filep);
      return NULL;
    }

  return filep;
}

static uLong zipfs_real_read(voidpf opaque, voidpf stream, void *buf,
                             uLong size)
{
  return file_read(stream, buf, size);
}

static ZPOS64_T zipfs_real_tell(voidpf opaque, voidpf stream)
{
  return file_seek(stream, 0, SEEK_CUR);
}

static long zipfs_real_seek(voidpf opaque, voidpf stream, ZPOS64_T offset,
                            int origin)
{
  int ret;

  ret = file_seek(stream, offset, origin);
  return ret >= 0 ? 0 : ret;
}

static int zipfs_real_close(voidpf opaque, voidpf stream)
{
  int ret;

  ret = file_close(stream);
  kmm_free(stream);
  return ret;
}

static int zipfs_real_error(voidpf opaque, voidpf stream)
{
  return OK;
}

static int zipfs_convert_result(int ziperr)
{
  switch (ziperr)
    {
      case UNZ_END_OF_LIST_OF_FILE:
        return -ENOENT;
      case UNZ_CRCERROR:
        return -ESTALE;
      case UNZ_INTERNALERROR:
        return -EPERM;
      case UNZ_BADZIPFILE:
        return -EBADF;
      case UNZ_PARAMERROR:
        return -EINVAL;
      default:
        return ziperr;
    }
}

static int zipfs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR struct zipfs_mountpt_s *fs = filep->f_inode->i_private;
  FAR struct zipfs_file_s *fp;
  int ret;

  DEBUGASSERT(fs != NULL);

  fp = kmm_malloc(sizeof(*fp) + strlen(relpath));
  if (fp == NULL)
    {
      return -ENOMEM;
    }

  ret = nxmutex_init(&fp->lock);
  if (ret < 0)
    {
      goto err_with_fp;
    }

  fp->uf = unzOpen2_64(fs->abspath, &zipfs_real_ops);
  if (fp->uf == NULL)
    {
      goto err_with_mutex;
    }

  ret = zipfs_convert_result(unzLocateFile(fp->uf, relpath, 0));
  if (ret < 0)
    {
      goto err_with_zip;
    }

  ret = zipfs_convert_result(unzOpenCurrentFile(fp->uf));
  if (ret < 0)
    {
      goto err_with_zip;
    }

  if (ret == OK)
    {
      fp->seekbuf = NULL;
      strcpy(fp->relpath, relpath);
      filep->f_priv = fp;
    }
  else
    {
err_with_zip:
      unzClose(fp->uf);
err_with_mutex:
      nxmutex_destroy(&fp->lock);
err_with_fp:
      kmm_free(fp);
    }

  return ret;
}

static int zipfs_close(FAR struct file *filep)
{
  FAR struct zipfs_file_s *fp = filep->f_priv;
  int ret;

  ret = zipfs_convert_result(unzClose(fp->uf));
  nxmutex_destroy(&fp->lock);
  kmm_free(fp->seekbuf);
  kmm_free(fp);
  return ret;
}

static ssize_t zipfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct zipfs_file_s *fp = filep->f_priv;
  ssize_t ret;

  nxmutex_lock(&fp->lock);
  ret = zipfs_convert_result(unzReadCurrentFile(fp->uf, buffer, buflen));
  if (ret > 0)
    {
      filep->f_pos += ret;
    }

  nxmutex_unlock(&fp->lock);
  return ret;
}

static off_t zipfs_skip(struct zipfs_file_s *fp, off_t amount)
{
  off_t next = 0;

  if (fp->seekbuf == NULL)
    {
      fp->seekbuf = kmm_malloc(CONFIG_ZIPFS_SEEK_BUFSIZE);
      if (fp->seekbuf == NULL)
        {
          return -ENOMEM;
        }
    }

  while (next < amount)
    {
      off_t remain = amount - next;

      if (remain > CONFIG_ZIPFS_SEEK_BUFSIZE)
        {
          remain = CONFIG_ZIPFS_SEEK_BUFSIZE;
        }

      remain = unzReadCurrentFile(fp->uf, fp->seekbuf, remain);
      remain = zipfs_convert_result(remain);
      if (remain <= 0)
        {
          return next ? next : remain;
        }

      next += remain;
    }

  return next;
}

static off_t zipfs_seek(FAR struct file *filep, off_t offset,
                        int whence)
{
  FAR struct zipfs_mountpt_s *fs = filep->f_inode->i_private;
  FAR struct zipfs_file_s *fp = filep->f_priv;
  unz_file_info64 file_info;
  off_t ret = 0;

  nxmutex_lock(&fp->lock);
  switch (whence)
    {
      case SEEK_SET:
        break;
      case SEEK_CUR:
        offset += filep->f_pos;
        break;
      case SEEK_END:
        ret = unzGetCurrentFileInfo64(fp->uf, &file_info,
                                      NULL, 0, NULL, 0, NULL, 0);
        ret = zipfs_convert_result(ret);
        if (ret < 0)
          {
            goto err_with_lock;
          }

          offset += file_info.uncompressed_size;
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
      ret = zipfs_convert_result(unzClose(fp->uf));
      if (ret < 0)
        {
          goto err_with_lock;
        }

      fp->uf = unzOpen2_64(fs->abspath, &zipfs_real_ops);
      if (fp->uf == NULL)
        {
          ret = -EINVAL;
          goto err_with_lock;
        }

      ret = zipfs_convert_result(unzLocateFile(fp->uf, fp->relpath, 0));
      if (ret < 0)
        {
          goto err_with_lock;
        }

      filep->f_pos = 0;
    }

  ret = zipfs_skip(fp->uf, offset - filep->f_pos);
  if (ret < 0)
    {
      goto err_with_lock;
    }

  if (ret >= 0)
    {
      filep->f_pos += ret;
    }

err_with_lock:
  nxmutex_unlock(&fp->lock);
  return ret < 0 ? ret : filep->f_pos;
}

static int zipfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct zipfs_file_s *fp;

  fp = oldp->f_priv;
  return zipfs_open(newp, fp->relpath, oldp->f_oflags, 0);
}

static int zipfs_stat_common(unzFile uf, FAR struct stat *buf)
{
  unz_file_info64 file_info;
  int ret;

  memset(buf, 0, sizeof(struct stat));

  ret = unzGetCurrentFileInfo64(uf, &file_info, NULL, 0,
                                NULL, 0, NULL, 0);
  ret = zipfs_convert_result(ret);
  if (ret >= 0)
    {
      buf->st_size = file_info.uncompressed_size;
      buf->st_mode = S_IFREG | 0444;
    }

  return ret;
}

static int zipfs_fstat(FAR const struct file *filep,
                       FAR struct stat *buf)
{
  FAR struct zipfs_file_s *fp = filep->f_priv;

  return zipfs_stat_common(fp->uf, buf);
}

static int zipfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                         FAR struct fs_dirent_s **dir)
{
  FAR struct zipfs_mountpt_s *fs = mountpt->i_private;
  FAR struct zipfs_dir_s *zdir;
  int ret;

  DEBUGASSERT(fs != NULL);

  zdir = kmm_malloc(sizeof(*zdir));
  if (zdir == NULL)
    {
      return -ENOMEM;
    }

  ret = nxmutex_init(&zdir->lock);
  if (ret < 0)
    {
      kmm_free(zdir);
      return ret;
    }

  zdir->uf = unzOpen2_64(fs->abspath, &zipfs_real_ops);
  if (zdir->uf == NULL)
    {
      nxmutex_destroy(&zdir->lock);
      kmm_free(zdir);
      return -EINVAL;
    }

  zdir->last = false;
  *dir = &zdir->base;
  return ret;
}

static int zipfs_closedir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir)
{
  FAR struct zipfs_dir_s *zdir = (FAR struct zipfs_dir_s *)dir;
  int ret;

  zdir = (FAR struct zipfs_dir_s *)dir;
  ret = zipfs_convert_result(unzClose(zdir->uf));
  nxmutex_destroy(&zdir->lock);
  kmm_free(zdir);
  return ret;
}

static int zipfs_readdir(FAR struct inode *mountpt,
                         FAR struct fs_dirent_s *dir,
                         FAR struct dirent *entry)
{
  FAR struct zipfs_dir_s *zdir = (FAR struct zipfs_dir_s *)dir;
  unz_file_info64 file_info;
  int ret;

  nxmutex_lock(&zdir->lock);
  ret = unzGetCurrentFileInfo64(zdir->uf,
                                &file_info,
                                entry->d_name,
                                NAME_MAX, NULL, 0, NULL, 0);

  ret = zipfs_convert_result(ret);
  if (ret < 0)
    {
      goto err_with_lock;
    }

  ret = zipfs_convert_result(unzGoToNextFile(zdir->uf));
  if (ret == -ENOENT)
    {
      if (zdir->last == false)
        {
          ret = OK;
          zdir->last = true;
        }
    }

err_with_lock:
  nxmutex_unlock(&zdir->lock);
  return ret;
}

static int zipfs_rewinddir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir)
{
  FAR struct zipfs_dir_s *zdir = (FAR struct zipfs_dir_s *)dir;
  int ret;

  nxmutex_lock(&zdir->lock);
  zdir->last = false;
  ret = zipfs_convert_result(unzGoToFirstFile(zdir->uf));
  nxmutex_unlock(&zdir->lock);
  return ret;
}

static int zipfs_bind(FAR struct inode *driver, FAR const void *data,
                      FAR void **handle)
{
  FAR struct zipfs_mountpt_s *fs;
  unzFile uf;

  if (data == NULL)
    {
      return -ENODEV;
    }

  fs = kmm_zalloc(sizeof(struct zipfs_mountpt_s) + strlen(data));
  if (fs == NULL)
    {
      return -ENOMEM;
    }

  uf = unzOpen2_64(data, &zipfs_real_ops);
  if (uf == NULL)
    {
      kmm_free(fs);
      return -EINVAL;
    }

  unzClose(uf);
  strcpy(fs->abspath, data);
  *handle = fs;

  return OK;
}

static int zipfs_unbind(FAR void *handle, FAR struct inode **driver,
                        unsigned int flags)
{
  kmm_free(handle);
  return OK;
}

static int zipfs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  memset(buf, 0, sizeof(struct statfs));
  buf->f_type = ZIPFS_MAGIC;
  buf->f_namelen = NAME_MAX;
  return OK;
}

static int zipfs_stat(FAR struct inode *mountpt,
                      FAR const char *relpath, FAR struct stat *buf)
{
  FAR struct zipfs_mountpt_s *fs;
  unzFile uf;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  if (relpath[0] == 0)
    {
      buf->st_mode = S_IFDIR;
      return OK;
    }

  fs = mountpt->i_private;
  uf = unzOpen2_64(fs->abspath, &zipfs_real_ops);
  if (uf == NULL)
    {
      return -EINVAL;
    }

  ret = zipfs_convert_result(unzLocateFile(uf, relpath, 0));
  if (ret < 0)
    {
      unzClose(uf);
      return ret;
    }

  ret = zipfs_stat_common(uf, buf);

  unzClose(uf);
  return ret;
}
