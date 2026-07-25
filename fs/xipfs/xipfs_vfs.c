/****************************************************************************
 * fs/xipfs/xipfs_vfs.c
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
 *
 * The VFS surface is deliberately narrow.  Files here are written once at
 * creation and are immutable thereafter, so the write path supports exactly
 * that shape and refuses anything else rather than half implementing POSIX
 * random write semantics that would break the contiguity guarantee the
 * whole design rests on.
 *
 * The namespace is flat: modules are files, there are no directories.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/stat.h>
#include <sys/statfs.h>

#include <assert.h>
#include <debug.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>

#include "xipfs.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     xipfs_open(FAR struct file *filep, FAR const char *relpath,
                          int oflags, mode_t mode);
static int     xipfs_close(FAR struct file *filep);
static ssize_t xipfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t xipfs_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen);
static off_t   xipfs_seek(FAR struct file *filep, off_t offset, int whence);
static int     xipfs_volume_cmd(FAR struct xipfs_mount_s *fs, int cmd,
                                unsigned long arg);
static int     xipfs_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);
static int     xipfs_ioctldir(FAR struct inode *mountpt,
                              FAR struct fs_dirent_s *dir,
                              int cmd, unsigned long arg);
static int     xipfs_truncate(FAR struct file *filep, off_t length);
static int     xipfs_sync(FAR struct file *filep);
static int     xipfs_dup(FAR const struct file *oldp,
                         FAR struct file *newp);
static int     xipfs_fstat(FAR const struct file *filep,
                           FAR struct stat *buf);
static int     xipfs_opendir(FAR struct inode *mountpt,
                             FAR const char *relpath,
                             FAR struct fs_dirent_s **dir);
static int     xipfs_closedir(FAR struct inode *mountpt,
                              FAR struct fs_dirent_s *dir);
static int     xipfs_readdir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir,
                             FAR struct dirent *entry);
static int     xipfs_rewinddir(FAR struct inode *mountpt,
                               FAR struct fs_dirent_s *dir);
static int     xipfs_bind(FAR struct inode *driver, FAR const void *data,
                          FAR void **handle);
static int     xipfs_unbind(FAR void *handle, FAR struct inode **driver,
                            unsigned int flags);
static int     xipfs_statfs(FAR struct inode *mountpt,
                            FAR struct statfs *buf);
static int     xipfs_unlink(FAR struct inode *mountpt,
                            FAR const char *relpath);
static int     xipfs_mkdir(FAR struct inode *mountpt,
                           FAR const char *relpath, mode_t mode);
static int     xipfs_rmdir(FAR struct inode *mountpt,
                           FAR const char *relpath);
static int     xipfs_stat(FAR struct inode *mountpt,
                          FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct mountpt_operations g_xipfs_operations =
{
  xipfs_open,      /* open */
  xipfs_close,     /* close */
  xipfs_read,      /* read */
  xipfs_write,     /* write */
  xipfs_seek,      /* seek */
  xipfs_ioctl,     /* ioctl */
  xipfs_mmap,      /* mmap */
  xipfs_truncate,  /* truncate */
  NULL,            /* poll */
  NULL,            /* readv */
  NULL,            /* writev */
  xipfs_sync,      /* sync */
  xipfs_dup,       /* dup */
  xipfs_fstat,     /* fstat */
  NULL,            /* fchstat */
  xipfs_opendir,   /* opendir */
  xipfs_closedir,  /* closedir */
  xipfs_readdir,   /* readdir */
  xipfs_rewinddir, /* rewinddir */
  xipfs_bind,      /* bind */
  xipfs_unbind,    /* unbind */
  xipfs_statfs,    /* statfs */
  xipfs_unlink,    /* unlink */
  xipfs_mkdir,     /* mkdir */
  xipfs_rmdir,     /* rmdir */
  NULL,            /* rename */
  xipfs_stat,      /* stat */
  NULL,            /* chstat */
  NULL,            /* syncfs */
  xipfs_ioctldir   /* ioctldir */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xipfs_blocksneeded
 ****************************************************************************/

static uint32_t xipfs_blocksneeded(FAR struct xipfs_mount_s *fs,
                                   uint32_t nbytes)
{
  if (nbytes == 0)
    {
      return 1;
    }

  return (nbytes + fs->geo.erasesize - 1) / fs->geo.erasesize;
}

/****************************************************************************
 * Name: xipfs_component_ok
 *
 * Description:
 *   Validate one path component.  "." and ".." are refused rather than
 *   interpreted: a stored entry by either name could never be reached again,
 *   and the VFS has already resolved the ones that were meant to navigate.
 *
 ****************************************************************************/

static int xipfs_component_ok(FAR const char *name, size_t len)
{
  if (len == 0)
    {
      return -EINVAL;
    }

  if (len > XIPFS_NAME_MAX)
    {
      return -ENAMETOOLONG;
    }

  if (name[0] == '.' && (len == 1 || (len == 2 && name[1] == '.')))
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: xipfs_walk
 *
 * Description:
 *   Resolve every component of 'relpath' but the last, and report the
 *   directory that would contain it along with where the last component
 *   starts and how long it is.
 *
 *   Trailing separators are ignored, so "bin/" resolves the same as "bin".
 *   An empty path is the root, reported as a zero length leaf.
 *
 *   The caller must hold the lock.
 *
 ****************************************************************************/

static int xipfs_walk(FAR struct xipfs_mount_s *fs, FAR const char *relpath,
                      FAR uint16_t *parent, FAR const char **leaf,
                      FAR size_t *leaflen)
{
  FAR struct xipfs_extent_s *ext;
  FAR const char *p = relpath;
  size_t len;
  int ret;

  *parent  = XIPFS_ROOT_ID;
  *leaf    = p;
  *leaflen = 0;

  if (p == NULL)
    {
      return OK;
    }

  for (; ; )
    {
      while (*p == '/')
        {
          p++;
        }

      len = 0;
      while (p[len] != '\0' && p[len] != '/')
        {
          len++;
        }

      if (len == 0)
        {
          /* Nothing further, so what was found last time is the leaf */

          return OK;
        }

      /* Is this the last component?  Anything after it is separators only */

      if (strspn(p + len, "/") == strlen(p + len))
        {
          ret = xipfs_component_ok(p, len);
          if (ret < 0)
            {
              return ret;
            }

          *leaf    = p;
          *leaflen = len;
          return OK;
        }

      /* An interior component, which has to be an existing directory */

      ext = xipfs_meta_find(fs, *parent, p, len);
      if (ext == NULL)
        {
          return -ENOENT;
        }

      if (!ext->isdir)
        {
          return -ENOTDIR;
        }

      *parent = ext->id;
      p      += len;
    }
}

/****************************************************************************
 * Name: xipfs_lookup
 *
 * Description:
 *   Resolve a whole path.  On success *ext is the entry, or NULL for the
 *   root, which has no record of its own.
 *
 *   The caller must hold the lock.
 *
 ****************************************************************************/

static int xipfs_lookup(FAR struct xipfs_mount_s *fs,
                        FAR const char *relpath,
                        FAR struct xipfs_extent_s **ext)
{
  FAR const char *leaf;
  size_t leaflen;
  uint16_t parent;
  int ret;

  ret = xipfs_walk(fs, relpath, &parent, &leaf, &leaflen);
  if (ret < 0)
    {
      return ret;
    }

  if (leaflen == 0)
    {
      *ext = NULL;      /* The root */
      return OK;
    }

  *ext = xipfs_meta_find(fs, parent, leaf, leaflen);
  return *ext != NULL ? OK : -ENOENT;
}

/****************************************************************************
 * Name: xipfs_readdata
 *
 * Description:
 *   Read from an extent's data at a byte offset.  When the media is memory
 *   mapped this is a straight memcpy from flash; otherwise it falls back to
 *   block reads.
 *
 ****************************************************************************/

static int xipfs_readdata(FAR struct xipfs_mount_s *fs,
                          FAR struct xipfs_extent_s *ext, off_t offset,
                          FAR uint8_t *buffer, size_t nbytes)
{
  FAR uint8_t *addr;
  uint32_t block;
  uint32_t blkoff;
  size_t chunk;
  int ret;

  addr = xipfs_flash_addr(fs, ext->start_block);
  if (addr != NULL)
    {
      memcpy(buffer, addr + offset, nbytes);
      return OK;
    }

  while (nbytes > 0)
    {
      block  = ext->start_block + offset / fs->geo.erasesize;
      blkoff = offset % fs->geo.erasesize;
      chunk  = fs->geo.erasesize - blkoff;

      if (chunk > nbytes)
        {
          chunk = nbytes;
        }

      ret = xipfs_flash_read(fs, block, blkoff, buffer, chunk);
      if (ret < 0)
        {
          return ret;
        }

      buffer += chunk;
      offset += chunk;
      nbytes -= chunk;
    }

  return OK;
}

/****************************************************************************
 * Name: xipfs_flushpage
 *
 * Description:
 *   Write the accumulated partial page out to flash.  Called when the
 *   buffer fills and once more at close to flush the tail.
 *
 ****************************************************************************/

static int xipfs_flushpage(FAR struct xipfs_mount_s *fs,
                           FAR struct xipfs_file_s *xf)
{
  FAR struct xipfs_extent_s *ext = xf->ext;
  uint32_t block;
  uint32_t blkoff;
  uint32_t relblk;
  int ret;

  if (xf->wrpos == 0)
    {
      return OK;
    }

  /* NOR is programmed in whole pages, so pad the tail with the erased
   * value rather than leaving it undefined.
   */

  if (xf->wrpos < fs->geo.blocksize)
    {
      memset(xf->wrbuf + xf->wrpos, 0xff, fs->geo.blocksize - xf->wrpos);
    }

  relblk = xf->written / fs->geo.erasesize;
  block  = ext->start_block + relblk;
  blkoff = xf->written % fs->geo.erasesize;

  /* Under a greedy reservation the run was not erased when it was taken, so
   * each block has to be erased just before its first page is programmed.
   * Writes are sequential, so this advances one block at a time.
   */

  if (relblk >= xf->erased)
    {
      ret = xipfs_flash_erase(fs, ext->start_block + xf->erased,
                              relblk - xf->erased + 1);
      if (ret < 0)
        {
          return ret;
        }

      xf->erased = relblk + 1;
    }

  ret = xipfs_flash_write(fs, block, blkoff, xf->wrbuf, fs->geo.blocksize);
  if (ret < 0)
    {
      return ret;
    }

  xf->written += fs->geo.blocksize;
  xf->wrpos    = 0;
  return OK;
}

/****************************************************************************
 * Name: xipfs_reserve
 *
 * Description:
 *   Reserve the extent for a file being created.
 *
 *   When the size is known up front -- which is the normal case for a
 *   downloaded module, and what ftruncate communicates -- exactly that many
 *   blocks are reserved and the file can never fragment.  When it is not,
 *   the largest free run is reserved greedily and trimmed back at close.
 *   Either way the file occupies one contiguous run for its whole life.
 *
 ****************************************************************************/

static int xipfs_reserve(FAR struct xipfs_mount_s *fs,
                         FAR struct xipfs_file_s *xf, uint32_t nblocks,
                         bool greedy)
{
  FAR struct xipfs_extent_s *ext = xf->ext;
  uint32_t start;
  int ret;

  ret = xipfs_alloc(fs, nblocks, &start);
  if (ret < 0)
    {
      return ret;
    }

  /* A greedy reservation covers the largest free run, which on an empty
   * filesystem is very nearly the whole partition.  Erasing all of it here
   * would cost time proportional to the free space rather than to the file
   * -- seconds, with interrupts disabled and the other core parked -- for a
   * file that may turn out to be 2 KB.  Erase lazily instead, one block
   * ahead of the writer, and let the trim at close hand back the tail that
   * was never touched.
   *
   * When the size is known the reservation is already exact, so there is
   * nothing to gain by deferring: erase it up front and be done.
   */

  if (greedy)
    {
      xf->erased = 0;
    }
  else
    {
      ret = xipfs_flash_erase(fs, start, nblocks);
      if (ret < 0)
        {
          xipfs_free(fs, start, nblocks);
          return ret;
        }

      xf->erased = nblocks;
    }

  ext->start_block = start;
  ext->nblocks     = nblocks;
  xf->greedy       = greedy;

  return OK;
}

/****************************************************************************
 * Name: xipfs_finalize
 *
 * Description:
 *   Complete a create: flush the tail page, trim any greedy reservation
 *   down to what was actually written, and commit.
 *
 ****************************************************************************/

static int xipfs_finalize(FAR struct xipfs_mount_s *fs,
                          FAR struct xipfs_file_s *xf)
{
  FAR struct xipfs_extent_s *ext = xf->ext;
  uint32_t needed;
  int ret;

  ret = xipfs_flushpage(fs, xf);
  if (ret < 0)
    {
      return ret;
    }

  needed = xipfs_blocksneeded(fs, ext->size);

  if (needed < ext->nblocks)
    {
      /* Give back the tail of a greedy reservation.  The extent keeps its
       * start, so it stays exactly as contiguous as it was.
       */

      xipfs_free(fs, ext->start_block + needed, ext->nblocks - needed);
      ext->nblocks = needed;
    }

  ext->writing = false;
  xf->writing  = false;

  ret = xipfs_meta_commit(fs);
  if (ret < 0)
    {
      return ret;
    }

  xipfs_alloc_rebuild(fs);
  return OK;
}

/****************************************************************************
 * Name: xipfs_release
 *
 * Description:
 *   Drop one open reference, freeing an unlinked extent once nothing at all
 *   references it any more.
 *
 ****************************************************************************/

static void xipfs_release(FAR struct xipfs_mount_s *fs,
                          FAR struct xipfs_extent_s *ext)
{
  DEBUGASSERT(ext->openrefs > 0);
  ext->openrefs--;

  if (ext->unlinked && ext->openrefs == 0 && ext->pincount == 0)
    {
      xipfs_free(fs, ext->start_block, ext->nblocks);
      kmm_free(ext);
    }
}

/****************************************************************************
 * Name: xipfs_open
 ****************************************************************************/

static int xipfs_open(FAR struct file *filep, FAR const char *relpath,
                      int oflags, mode_t mode)
{
  FAR struct xipfs_mount_s *fs;
  FAR struct xipfs_extent_s *ext;
  FAR struct xipfs_file_s *xf;
  FAR const char *leaf;
  size_t leaflen;
  uint16_t parent;
  int ret;

  fs = filep->f_inode->i_private;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = xipfs_walk(fs, relpath, &parent, &leaf, &leaflen);
  if (ret < 0)
    {
      goto errout_with_lock;
    }

  if (leaflen == 0)
    {
      ret = -EISDIR;    /* The mountpoint itself */
      goto errout_with_lock;
    }

  ext = xipfs_meta_find(fs, parent, leaf, leaflen);

  if (ext != NULL && ext->isdir)
    {
      ret = -EISDIR;
      goto errout_with_lock;
    }

  if ((oflags & O_WRONLY) != 0 || (oflags & O_RDWR) != 0)
    {
      /* Files are write once.  Reopening an existing file for writing is
       * not an append or an overwrite here -- it has no meaning under the
       * immutability assumption, so it is refused rather than silently
       * doing something surprising.
       */

      if (ext != NULL)
        {
          if ((oflags & O_TRUNC) == 0 || (oflags & O_CREAT) == 0)
            {
              ret = -EACCES;
              goto errout_with_lock;
            }

          /* O_CREAT|O_TRUNC on an existing file means "replace it".  Detach
           * the old extent; it is freed once nothing references it.
           */

          if (ext->pincount > 0)
            {
              ret = -EBUSY;
              goto errout_with_lock;
            }

          xipfs_meta_detach(fs, ext);
          if (ext->openrefs == 0)
            {
              xipfs_free(fs, ext->start_block, ext->nblocks);
              kmm_free(ext);
            }

          ext = NULL;
        }

      if ((oflags & O_CREAT) == 0)
        {
          ret = -ENOENT;
          goto errout_with_lock;
        }

      ext = xipfs_meta_add(fs, parent, leaf, leaflen, false, 0, 0, 0);
      if (ext == NULL)
        {
          ret = -ENOMEM;
          goto errout_with_lock;
        }

      ext->writing = true;
    }
  else
    {
      if (ext == NULL)
        {
          ret = -ENOENT;
          goto errout_with_lock;
        }

      if (ext->writing)
        {
          ret = -EBUSY;
          goto errout_with_lock;
        }
    }

  xf = kmm_zalloc(sizeof(struct xipfs_file_s));
  if (xf == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_extent;
    }

  xf->ext     = ext;
  xf->writing = ext->writing;

  if (xf->writing)
    {
      xf->wrbuf = kmm_malloc(fs->geo.blocksize);
      if (xf->wrbuf == NULL)
        {
          kmm_free(xf);
          ret = -ENOMEM;
          goto errout_with_extent;
        }
    }

  ext->openrefs++;
  filep->f_priv = xf;
  filep->f_pos  = 0;

  xipfs_unlock(fs);
  return OK;

errout_with_extent:
  if (ext != NULL && ext->writing)
    {
      xipfs_meta_detach(fs, ext);
      kmm_free(ext);
    }

errout_with_lock:
  xipfs_unlock(fs);
  return ret;
}

/****************************************************************************
 * Name: xipfs_close
 ****************************************************************************/

static int xipfs_close(FAR struct file *filep)
{
  FAR struct xipfs_mount_s *fs;
  FAR struct xipfs_file_s *xf;
  int ret;

  fs = filep->f_inode->i_private;
  xf = filep->f_priv;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  if (xf->writing)
    {
      ret = xipfs_finalize(fs, xf);
      if (ret < 0)
        {
          /* The create did not become durable.  Discard it entirely rather
           * than leaving a half written file visible.  Detaching marks the
           * extent unlinked; xipfs_release below is what actually reclaims
           * its blocks once this last reference drops, so freeing here as
           * well would hand the same run back twice.
           */

          ferr("ERROR: Failed to finalize '%s': %d\n", xf->ext->name, ret);
          xipfs_meta_detach(fs, xf->ext);
        }
    }

  if (xf->wrbuf != NULL)
    {
      kmm_free(xf->wrbuf);
    }

  xipfs_release(fs, xf->ext);
  kmm_free(xf);
  filep->f_priv = NULL;

  xipfs_unlock(fs);
  return ret;
}

/****************************************************************************
 * Name: xipfs_read
 ****************************************************************************/

static ssize_t xipfs_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct xipfs_mount_s *fs;
  FAR struct xipfs_file_s *xf;
  FAR struct xipfs_extent_s *ext;
  size_t remaining;
  int ret;

  fs = filep->f_inode->i_private;
  xf = filep->f_priv;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  ext = xf->ext;

  if (ext->writing)
    {
      xipfs_unlock(fs);
      return -EBUSY;
    }

  if (filep->f_pos >= (off_t)ext->size)
    {
      xipfs_unlock(fs);
      return 0;
    }

  remaining = ext->size - filep->f_pos;
  if (buflen > remaining)
    {
      buflen = remaining;
    }

  ret = xipfs_readdata(fs, ext, filep->f_pos, (FAR uint8_t *)buffer,
                       buflen);
  if (ret < 0)
    {
      xipfs_unlock(fs);
      return ret;
    }

  filep->f_pos += buflen;

  xipfs_unlock(fs);
  return buflen;
}

/****************************************************************************
 * Name: xipfs_write
 *
 * Description:
 *   Append to a file being created.  Sequential only: this is the whole
 *   write model, not a subset of a richer one.
 *
 ****************************************************************************/

static ssize_t xipfs_write(FAR struct file *filep, FAR const char *buffer,
                           size_t buflen)
{
  FAR struct xipfs_mount_s *fs;
  FAR struct xipfs_file_s *xf;
  FAR struct xipfs_extent_s *ext;
  size_t written = 0;
  size_t chunk;
  int ret;

  fs = filep->f_inode->i_private;
  xf = filep->f_priv;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  ext = xf->ext;

  if (!xf->writing)
    {
      /* Files are immutable once closed */

      ret = -EACCES;
      goto errout_with_lock;
    }

  if (filep->f_pos != (off_t)ext->size)
    {
      ferr("ERROR: xipfs supports sequential writes only\n");
      ret = -EINVAL;
      goto errout_with_lock;
    }

  /* No extent yet means the size was never declared.  Reserve the largest
   * run available and trim it back at close.
   */

  if (ext->nblocks == 0)
    {
      uint32_t largest = xipfs_alloc_largestrun(fs);

      if (largest == 0)
        {
          ret = -ENOSPC;
          goto errout_with_lock;
        }

      ret = xipfs_reserve(fs, xf, largest, true);
      if (ret < 0)
        {
          goto errout_with_lock;
        }
    }

  while (written < buflen)
    {
      if (ext->size + 1 > ext->nblocks * fs->geo.erasesize)
        {
          ret = -ENOSPC;
          break;
        }

      chunk = fs->geo.blocksize - xf->wrpos;
      if (chunk > buflen - written)
        {
          chunk = buflen - written;
        }

      if (chunk > ext->nblocks * fs->geo.erasesize - ext->size)
        {
          chunk = ext->nblocks * fs->geo.erasesize - ext->size;
        }

      memcpy(xf->wrbuf + xf->wrpos, buffer + written, chunk);
      xf->wrpos += chunk;
      ext->size += chunk;
      written   += chunk;

      if (xf->wrpos == fs->geo.blocksize)
        {
          ret = xipfs_flushpage(fs, xf);
          if (ret < 0)
            {
              break;
            }
        }
    }

  if (written == 0 && ret < 0)
    {
      goto errout_with_lock;
    }

  filep->f_pos = ext->size;

  xipfs_unlock(fs);
  return written;

errout_with_lock:
  xipfs_unlock(fs);
  return ret;
}

/****************************************************************************
 * Name: xipfs_seek
 ****************************************************************************/

static off_t xipfs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct xipfs_mount_s *fs;
  FAR struct xipfs_file_s *xf;
  off_t pos;
  int ret;

  fs = filep->f_inode->i_private;
  xf = filep->f_priv;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  switch (whence)
    {
      case SEEK_SET:
        pos = offset;
        break;

      case SEEK_CUR:
        pos = filep->f_pos + offset;
        break;

      case SEEK_END:
        pos = xf->ext->size + offset;
        break;

      default:
        xipfs_unlock(fs);
        return -EINVAL;
    }

  if (pos < 0)
    {
      xipfs_unlock(fs);
      return -EINVAL;
    }

  /* Seeking while creating would create a hole, and a hole cannot be
   * expressed in a write once, exactly sized extent.
   */

  if (xf->writing && pos != filep->f_pos)
    {
      xipfs_unlock(fs);
      return -EINVAL;
    }

  filep->f_pos = pos;

  xipfs_unlock(fs);
  return pos;
}

/****************************************************************************
 * Name: xipfs_truncate
 *
 * Description:
 *   Declare the final size of a file being created.  This is how a caller
 *   that already knows how big the module is -- which is the normal case --
 *   gets an exactly sized extent instead of a greedy reservation.
 *
 ****************************************************************************/

static int xipfs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct xipfs_mount_s *fs;
  FAR struct xipfs_file_s *xf;
  int ret;

  fs = filep->f_inode->i_private;
  xf = filep->f_priv;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  if (!xf->writing || xf->ext->size != 0 || xf->ext->nblocks != 0)
    {
      /* Only meaningful once, before any data has been written */

      ret = -EINVAL;
      goto errout_with_lock;
    }

  if (length < 0)
    {
      ret = -EINVAL;
      goto errout_with_lock;
    }

  ret = xipfs_reserve(fs, xf, xipfs_blocksneeded(fs, length), false);

errout_with_lock:
  xipfs_unlock(fs);
  return ret;
}

/****************************************************************************
 * Name: xipfs_volume_cmd
 *
 * Description:
 *   The commands that act on the volume rather than on any one file.  They
 *   are reachable both from a descriptor for a file inside the volume and
 *   from one for the mountpoint directory; only the second is free of the
 *   side effect that matters, which is that the file used to carry the
 *   command is itself open, and an open file cannot be relocated.
 *
 ****************************************************************************/

static int xipfs_volume_cmd(FAR struct xipfs_mount_s *fs, int cmd,
                            unsigned long arg)
{
  switch (cmd)
    {
      case XIPFSIOC_DEFRAG:
        {
          FAR struct xipfs_defrag_arg_s *dfg =
            (FAR struct xipfs_defrag_arg_s *)((uintptr_t)arg);

          if (dfg == NULL)
            {
              return -EINVAL;
            }

          return xipfs_defrag(fs, dfg->max_ms, &dfg->result);
        }

      case XIPFSIOC_LISTPINNED:
        {
          FAR struct xipfs_pinned_arg_s *pin =
            (FAR struct xipfs_pinned_arg_s *)((uintptr_t)arg);

          if (pin == NULL)
            {
              return -EINVAL;
            }

          return xipfs_listpinned(fs, pin->entries, pin->nentries,
                                  &pin->count);
        }

#ifdef CONFIG_FS_XIPFS_FAULT_INJECT
      case XIPFSIOC_FAULTINJECT:
        {
          FAR struct xipfs_fault_s *fault =
            (FAR struct xipfs_fault_s *)((uintptr_t)arg);
          int ret;

          if (fault == NULL)
            {
              return -EINVAL;
            }

          ret = xipfs_lock(fs);
          if (ret < 0)
            {
              return ret;
            }

          fs->fault_countdown = fault->count;
          fs->fault_mode      = fault->mode;
          fs->media_dead      = false;

          xipfs_unlock(fs);
          return OK;
        }
#endif

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Name: xipfs_ioctldir
 *
 * Description:
 *   ioctl method for a descriptor on the mountpoint directory, obtained
 *   with open(mountpoint, O_RDONLY | O_DIRECTORY).  This is the route the
 *   volume commands want: no file inside the volume is open, so nothing is
 *   held immovable by the act of asking, and a compaction pass can reach
 *   every extent.  The directory itself is not needed -- the volume comes
 *   from the mountpoint inode -- so dir is unused.
 *
 ****************************************************************************/

static int xipfs_ioctldir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir,
                          int cmd, unsigned long arg)
{
  FAR struct xipfs_mount_s *fs;

  UNUSED(dir);

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);
  fs = mountpt->i_private;

  return xipfs_volume_cmd(fs, cmd, arg);
}

/****************************************************************************
 * Name: xipfs_ioctl
 ****************************************************************************/

static int xipfs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct xipfs_mount_s *fs;
  FAR struct xipfs_file_s *xf;
  int ret;

  fs = filep->f_inode->i_private;
  xf = filep->f_priv;

  switch (cmd)
    {
      case XIPFSIOC_PIN:
        {
          /* Pin the extent and report its flash address.
           *
           * A module loader needs this rather than mmap because the
           * lifetime is wrong for mmap: the mapping would be recorded
           * against whichever task happened to call the loader, but the
           * module lives until the *spawned* task exits, and the release
           * therefore has to happen from a different task entirely.  An
           * explicit pin expresses that ownership directly.
           */

          FAR uintptr_t *addrp = (FAR uintptr_t *)((uintptr_t)arg);
          FAR uint8_t *addr;

          if (addrp == NULL)
            {
              return -EINVAL;
            }

          ret = xipfs_lock(fs);
          if (ret < 0)
            {
              return ret;
            }

          addr = xipfs_flash_addr(fs, xf->ext->start_block);
          if (addr == NULL || xf->ext->writing)
            {
              xipfs_unlock(fs);
              return -ENXIO;
            }

          xf->ext->pincount++;
          *addrp = (uintptr_t)addr;

          xipfs_unlock(fs);
          return OK;
        }

      case XIPFSIOC_UNPIN:
        {
          ret = xipfs_lock(fs);
          if (ret < 0)
            {
              return ret;
            }

          if (xf->ext->pincount == 0)
            {
              xipfs_unlock(fs);
              return -EINVAL;
            }

          xf->ext->pincount--;

          xipfs_unlock(fs);
          return OK;
        }

      case XIPFSIOC_EXTENTINFO:
        {
          FAR struct xipfs_extent_info_s *info =
            (FAR struct xipfs_extent_info_s *)((uintptr_t)arg);
          FAR uint8_t *addr;

          if (info == NULL)
            {
              return -EINVAL;
            }

          ret = xipfs_lock(fs);
          if (ret < 0)
            {
              return ret;
            }

          addr = xipfs_flash_addr(fs, xf->ext->start_block);

          info->start_block = xf->ext->start_block;
          info->nblocks     = xf->ext->nblocks;
          info->erasesize   = fs->geo.erasesize;
          info->size        = xf->ext->size;
          info->pincount    = xf->ext->pincount;
          info->data_start  = fs->data_start;
          info->data_nblocks = fs->data_nblocks;
          info->xipaddr     = (uintptr_t)addr;

          xipfs_unlock(fs);
          return OK;
        }

      case FIOC_FILEPATH:
        return -ENOTTY;

      default:

        /* Not one of the per-file commands.  The volume commands are also
         * accepted here, for a caller that has a file open and nothing
         * else, but the mountpoint directory is the better route: it does
         * not leave a file open across the operation.
         */

        return xipfs_volume_cmd(fs, cmd, arg);
    }
}

/****************************************************************************
 * Name: xipfs_sync
 ****************************************************************************/

static int xipfs_sync(FAR struct file *filep)
{
  /* Every committed operation is already durable when it returns; there is
   * no write back cache to flush.
   */

  UNUSED(filep);
  return OK;
}

/****************************************************************************
 * Name: xipfs_dup
 ****************************************************************************/

static int xipfs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct xipfs_mount_s *fs;
  FAR struct xipfs_file_s *oldxf;
  FAR struct xipfs_file_s *newxf;
  int ret;

  fs    = newp->f_inode->i_private;
  oldxf = oldp->f_priv;

  if (oldxf->writing)
    {
      /* Two descriptors appending to one write once file has no coherent
       * meaning.
       */

      return -EACCES;
    }

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  newxf = kmm_zalloc(sizeof(struct xipfs_file_s));
  if (newxf == NULL)
    {
      xipfs_unlock(fs);
      return -ENOMEM;
    }

  newxf->ext = oldxf->ext;
  newxf->ext->openrefs++;
  newp->f_priv = newxf;

  xipfs_unlock(fs);
  return OK;
}

/****************************************************************************
 * Name: xipfs_fstat
 ****************************************************************************/

static int xipfs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct xipfs_mount_s *fs;
  FAR struct xipfs_file_s *xf;
  int ret;

  fs = filep->f_inode->i_private;
  xf = filep->f_priv;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  memset(buf, 0, sizeof(struct stat));
  buf->st_mode    = S_IFREG | 0444;
  buf->st_size    = xf->ext->size;
  buf->st_blksize = fs->geo.erasesize;
  buf->st_blocks  = xf->ext->nblocks;

  xipfs_unlock(fs);
  return OK;
}

/****************************************************************************
 * Name: xipfs_opendir
 ****************************************************************************/

static int xipfs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                         FAR struct fs_dirent_s **dir)
{
  FAR struct xipfs_mount_s *fs = mountpt->i_private;
  FAR struct xipfs_extent_s *ext;
  FAR struct xipfs_dir_s *xdir;
  int ret;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = xipfs_lookup(fs, relpath, &ext);
  if (ret >= 0 && ext != NULL && !ext->isdir)
    {
      ret = -ENOTDIR;
    }

  xipfs_unlock(fs);

  if (ret < 0)
    {
      return ret;
    }

  xdir = kmm_zalloc(sizeof(struct xipfs_dir_s));
  if (xdir == NULL)
    {
      return -ENOMEM;
    }

  xdir->dirid = ext != NULL ? ext->id : XIPFS_ROOT_ID;

  *dir = &xdir->base;
  return OK;
}

/****************************************************************************
 * Name: xipfs_closedir
 ****************************************************************************/

static int xipfs_closedir(FAR struct inode *mountpt,
                          FAR struct fs_dirent_s *dir)
{
  UNUSED(mountpt);
  kmm_free(dir);
  return OK;
}

/****************************************************************************
 * Name: xipfs_readdir
 ****************************************************************************/

static int xipfs_readdir(FAR struct inode *mountpt,
                         FAR struct fs_dirent_s *dir,
                         FAR struct dirent *entry)
{
  FAR struct xipfs_mount_s *fs;
  FAR struct xipfs_dir_s *xdir;
  FAR struct xipfs_extent_s *ext;
  uint32_t i;
  int ret;

  fs   = mountpt->i_private;
  xdir = (FAR struct xipfs_dir_s *)dir;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  /* Resume where the last call stopped and report the next entry this
   * directory holds.  The list is in no particular order, so neither is a
   * directory listing.
   */

  ext = fs->extents;
  for (i = 0; i < xdir->index && ext != NULL; i++)
    {
      ext = ext->flink;
    }

  for (; ext != NULL; ext = ext->flink, xdir->index++)
    {
      if (ext->parent != xdir->dirid)
        {
          continue;
        }

      entry->d_type = ext->isdir ? DTYPE_DIRECTORY : DTYPE_FILE;
      strlcpy(entry->d_name, ext->name, sizeof(entry->d_name));
      xdir->index++;

      xipfs_unlock(fs);
      return OK;
    }

  xipfs_unlock(fs);
  return -ENOENT;
}

/****************************************************************************
 * Name: xipfs_rewinddir
 ****************************************************************************/

static int xipfs_rewinddir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir)
{
  UNUSED(mountpt);
  ((FAR struct xipfs_dir_s *)dir)->index = 0;
  return OK;
}

/****************************************************************************
 * Name: xipfs_bind
 ****************************************************************************/

static int xipfs_bind(FAR struct inode *driver, FAR const void *data,
                      FAR void **handle)
{
  FAR struct xipfs_mount_s *fs;
  FAR const char *options = data;
  bool autoformat;
  int ret;

  if (driver == NULL || handle == NULL)
    {
      return -EINVAL;
    }

  if (!INODE_IS_MTD(driver) || driver->u.i_mtd == NULL)
    {
      ferr("ERROR: xipfs requires an MTD driver\n");
      return -ENODEV;
    }

  autoformat = options != NULL &&
               strstr(options, "autoformat") != NULL;

  fs = kmm_zalloc(sizeof(struct xipfs_mount_s));
  if (fs == NULL)
    {
      return -ENOMEM;
    }

  nxrmutex_init(&fs->lock);
  fs->driver = driver;
  fs->mtd    = driver->u.i_mtd;

#ifdef CONFIG_FS_XIPFS_FAULT_INJECT
  fs->fault_countdown = -1;
  fs->fault_mode      = XIPFS_FAULT_CLEAN;
#endif

  ret = xipfs_flash_probe(fs);
  if (ret < 0)
    {
      goto errout_with_fs;
    }

  fs->entries_per_blk = fs->geo.blocksize / XIPFS_DIRENT_SIZE;
  fs->max_entries     = (fs->geo.erasesize / fs->geo.blocksize - 1) *
                        fs->entries_per_blk;

  if (fs->entries_per_blk == 0)
    {
      ferr("ERROR: blocksize %" PRIu32 " is too small for a dirent\n",
           fs->geo.blocksize);
      ret = -EINVAL;
      goto errout_with_fs;
    }

  /* Both scratch buffers are reserved once, here.  In particular the defrag
   * staging buffer must never be allocated at the moment defrag runs: that
   * would turn a transient memory shortage into a relocation that cannot
   * finish.
   */

  fs->metabuf = kmm_malloc(fs->geo.blocksize);
  fs->stage   = kmm_malloc(fs->geo.blocksize);

  if (fs->metabuf == NULL || fs->stage == NULL)
    {
      ret = -ENOMEM;
      goto errout_with_buffers;
    }

  ret = xipfs_meta_mount(fs);
  if (ret == -EFTYPE && autoformat)
    {
      finfo("xipfs: no valid filesystem found, formatting\n");
      ret = xipfs_meta_format(fs);
    }

  if (ret < 0)
    {
      goto errout_with_buffers;
    }

  ret = xipfs_alloc_init(fs);
  if (ret < 0)
    {
      goto errout_with_buffers;
    }

  xipfs_alloc_rebuild(fs);

  *handle = fs;
  return OK;

errout_with_buffers:
  xipfs_meta_freeextents(fs);
  kmm_free(fs->metabuf);
  kmm_free(fs->stage);

errout_with_fs:
  nxrmutex_destroy(&fs->lock);
  kmm_free(fs);
  return ret;
}

/****************************************************************************
 * Name: xipfs_unbind
 ****************************************************************************/

static int xipfs_unbind(FAR void *handle, FAR struct inode **driver,
                        unsigned int flags)
{
  FAR struct xipfs_mount_s *fs = handle;
  FAR struct xipfs_extent_s *ext;
  int ret;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  /* Refuse while anything is still mapped.  An XIP mapping is a raw pointer
   * into flash held by code that has no idea the filesystem is going away,
   * and the extent object behind it must outlive the mapping.
   */

  for (ext = fs->extents; ext != NULL; ext = ext->flink)
    {
      if (ext->pincount > 0 || ext->openrefs > 0)
        {
          xipfs_unlock(fs);
          return -EBUSY;
        }
    }

  if (driver != NULL)
    {
      *driver = fs->driver;
    }

  xipfs_meta_freeextents(fs);
  kmm_free(fs->bitmap);
  kmm_free(fs->metabuf);
  kmm_free(fs->stage);

  xipfs_unlock(fs);
  nxrmutex_destroy(&fs->lock);
  kmm_free(fs);

  return OK;
}

/****************************************************************************
 * Name: xipfs_statfs
 ****************************************************************************/

static int xipfs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct xipfs_mount_s *fs = mountpt->i_private;
  int ret;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  memset(buf, 0, sizeof(struct statfs));
  buf->f_type    = XIPFS_SBLK_MAGIC;
  buf->f_bsize   = fs->geo.erasesize;
  buf->f_blocks  = fs->data_nblocks;
  buf->f_bfree   = xipfs_alloc_freeblocks(fs);
  buf->f_bavail  = buf->f_bfree;
  buf->f_files   = fs->nextents;
  buf->f_ffree   = fs->max_entries - fs->nextents;
  buf->f_namelen = XIPFS_NAME_MAX;

  xipfs_unlock(fs);
  return OK;
}

/****************************************************************************
 * Name: xipfs_unlink
 ****************************************************************************/

static int xipfs_unlink(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct xipfs_mount_s *fs = mountpt->i_private;
  FAR struct xipfs_extent_s *ext;
  int ret;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = xipfs_lookup(fs, relpath, &ext);
  if (ret < 0)
    {
      goto errout_with_lock;
    }

  if (ext == NULL || ext->isdir)
    {
      ret = -EISDIR;
      goto errout_with_lock;
    }

  if (ext->writing)
    {
      ret = -EBUSY;
      goto errout_with_lock;
    }

  /* Detach first, then commit.  If the commit fails the extent is put back,
   * because the on-media directory is the authority and it still names it.
   */

  xipfs_meta_detach(fs, ext);

  ret = xipfs_meta_commit(fs);
  if (ret < 0)
    {
      ext->unlinked = false;
      ext->flink    = fs->extents;
      fs->extents   = ext;
      fs->nextents++;
      goto errout_with_lock;
    }

  /* The blocks are only reusable once nothing references the extent.  A
   * pinned extent stays exactly where it is until the last mapping goes,
   * because a running module is still executing out of it.
   */

  if (ext->openrefs == 0 && ext->pincount == 0)
    {
      xipfs_free(fs, ext->start_block, ext->nblocks);
      kmm_free(ext);
    }

errout_with_lock:
  xipfs_unlock(fs);
  return ret;
}

/****************************************************************************
 * Name: xipfs_mkdir
 *
 * Description:
 *   Create a directory, which is one record in the next metadata generation
 *   and nothing else.  It costs no data block and no erase in the data
 *   region, only an entry out of the volume's fixed supply.
 *
 ****************************************************************************/

static int xipfs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
                       mode_t mode)
{
  FAR struct xipfs_mount_s *fs = mountpt->i_private;
  FAR struct xipfs_extent_s *ext;
  FAR const char *leaf;
  size_t leaflen;
  uint16_t parent;
  int ret;

  UNUSED(mode);

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = xipfs_walk(fs, relpath, &parent, &leaf, &leaflen);
  if (ret < 0)
    {
      goto errout_with_lock;
    }

  if (leaflen == 0)
    {
      ret = -EEXIST;    /* The root is always there */
      goto errout_with_lock;
    }

  if (xipfs_meta_find(fs, parent, leaf, leaflen) != NULL)
    {
      ret = -EEXIST;
      goto errout_with_lock;
    }

  ext = xipfs_meta_add(fs, parent, leaf, leaflen, true, 0, 0, 0);
  if (ext == NULL)
    {
      ret = -ENOSPC;
      goto errout_with_lock;
    }

  ret = xipfs_meta_commit(fs);
  if (ret < 0)
    {
      /* The medium is the authority and it does not name the directory, so
       * take it back out again.
       */

      xipfs_meta_detach(fs, ext);
      kmm_free(ext);
    }

errout_with_lock:
  xipfs_unlock(fs);
  return ret;
}

/****************************************************************************
 * Name: xipfs_rmdir
 ****************************************************************************/

static int xipfs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct xipfs_mount_s *fs = mountpt->i_private;
  FAR struct xipfs_extent_s *ext;
  int ret;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  ret = xipfs_lookup(fs, relpath, &ext);
  if (ret < 0)
    {
      goto errout_with_lock;
    }

  if (ext == NULL)
    {
      ret = -EBUSY;     /* The mountpoint itself */
      goto errout_with_lock;
    }

  if (!ext->isdir)
    {
      ret = -ENOTDIR;
      goto errout_with_lock;
    }

  if (xipfs_meta_haschildren(fs, ext->id))
    {
      ret = -ENOTEMPTY;
      goto errout_with_lock;
    }

  xipfs_meta_detach(fs, ext);

  ret = xipfs_meta_commit(fs);
  if (ret < 0)
    {
      /* Put it back: the committed generation still names it */

      ext->unlinked = false;
      ext->flink    = fs->extents;
      fs->extents   = ext;
      fs->nextents++;
      goto errout_with_lock;
    }

  kmm_free(ext);

errout_with_lock:
  xipfs_unlock(fs);
  return ret;
}

/****************************************************************************
 * Name: xipfs_stat
 ****************************************************************************/

static int xipfs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                      FAR struct stat *buf)
{
  FAR struct xipfs_mount_s *fs = mountpt->i_private;
  FAR struct xipfs_extent_s *ext;
  int ret;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  memset(buf, 0, sizeof(struct stat));

  ret = xipfs_lookup(fs, relpath, &ext);
  if (ret < 0)
    {
      xipfs_unlock(fs);
      return ret;
    }

  if (ext == NULL || ext->isdir)
    {
      buf->st_mode    = S_IFDIR | 0555;
      buf->st_blksize = fs->geo.erasesize;
      xipfs_unlock(fs);
      return OK;
    }

  buf->st_mode    = S_IFREG | 0444;
  buf->st_size    = ext->size;
  buf->st_blksize = fs->geo.erasesize;
  buf->st_blocks  = ext->nblocks;

  xipfs_unlock(fs);
  return OK;
}
