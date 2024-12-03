/****************************************************************************
 * fs/mnemofs/mnemofs.c
 *
 * SPDX-License-Identifier: Apache-2.0 or BSD-3-Clause
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
 * Alternatively, the contents of this file may be used under the terms of
 * the BSD-3-Clause license:
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2024 Saurav Pal
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/* mnemofs has these components:
 * - Block Allocator
 * - Journal
 * - LRU
 * - Master Node
 *
 * All the files and directories are stored as a reversed CTZ skip list, a
 * data structure defined by littlefs. CTZs list have multiple nodes
 * connected by pointers stored in the nodes. So, each node has a pointer
 * area, which is at the end of the CTZ node (CTZ block), and a data area,
 * which forms the rest of the area, which stores the actual file data. The
 * size of the data area can be deterministically found given the index of
 * the node in the CTZ list. In mnemofs, each CTZ node/CTZ block takes up one
 * page in the NAND flash.
 *
 * Due to the presence of a data area, the mnemofs VFS methods can only view
 * an abstracted view of the CTZ list they are interacting with. VFS methods
 * work with CTZ methods (mnemofs_ctz.c), and can not see the existence of
 * the pointer area, nor do they have to care about it. They work with
 * offsets in the data area from the beginning of the list, and the CTZ
 * methods take care of converting that offset into actual NAND flash
 * coordinates (block number, or page number).
 *
 * More info in mnemofs_ctz.c, mnemofs_lru.c, mnemofs_journal.c and
 * mnemofs_master.c.
 */

/* TODO:
 * - LRU and journal store multiple deltas/commits together respectively.
 *   These are supposed to be applied together atomically.
 * - Journal moves.
 * - Return values of all functions.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <fcntl.h>
#include <math.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <stdio.h>
#include <sys/param.h>
#include <sys/stat.h>
#include <sys/statfs.h>

#include "mnemofs.h"
#include "fs_heap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     mnemofs_open(FAR struct file *filep, FAR const char *relpath,
                            int oflags, mode_t mode);
static int     mnemofs_close(FAR struct file *filep);
static ssize_t mnemofs_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t mnemofs_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static off_t   mnemofs_seek(FAR struct file *filep, off_t offset,
                            int whence);
static int     mnemofs_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg);
static int     mnemofs_truncate(FAR struct file *filep, off_t length);

static int     mnemofs_sync(FAR struct file *filep);
static int     mnemofs_dup(FAR const struct file *oldp,
                           FAR struct file *newp);
static int     mnemofs_fstat(FAR const struct file *filep,
                             FAR struct stat *buf);

static int     mnemofs_opendir(FAR struct inode *mountpt,
                               FAR const char *relpath,
                               FAR struct fs_dirent_s **dir);
static int     mnemofs_closedir(FAR struct inode *mountpt,
                                FAR struct fs_dirent_s *dir);
static int     mnemofs_readdir(FAR struct inode *mountpt,
                               FAR struct fs_dirent_s *dir,
                               FAR struct dirent *entry);
static int     mnemofs_rewinddir(FAR struct inode *mountpt,
                                 FAR struct fs_dirent_s *dir);

static int     mnemofs_bind(FAR struct inode *driver, FAR const void *data,
                            FAR void** handle);

static int     mnemofs_unbind(FAR void *handle, FAR struct inode **driver,
                              unsigned int flags);
static int     mnemofs_statfs(FAR struct inode *mountpt,
                              FAR struct statfs *buf);

static int     mnemofs_unlink(FAR struct inode *mountpt,
                              FAR const char *relpath);
static int     mnemofs_mkdir(FAR struct inode *mountpt,
                             FAR const char *relpath, mode_t mode);
static int     mnemofs_rmdir(FAR struct inode *mountpt,
                             FAR const char *relpath);
static int     mnemofs_rename(FAR struct inode *mountpt,
                              FAR const char *oldrelpath,
                              FAR const char *newrelpath);
static int     mnemofs_stat(FAR struct inode *mountpt,
                            FAR const char *relpath, FAR struct stat *buf);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Name: g_mnemofs_operations
 *
 * Description:
 *  The global list of VFS methods implemented by mnemofs
 *
 ****************************************************************************/

const struct mountpt_operations g_mnemofs_operations =
{
  mnemofs_open,      /* open */
  mnemofs_close,     /* close */
  mnemofs_read,      /* read */
  mnemofs_write,     /* write */
  mnemofs_seek,      /* seek */
  mnemofs_ioctl,     /* ioctl */
  NULL,              /* mmap */
  mnemofs_truncate,  /* truncate */
  NULL,              /* poll */
  NULL,              /* readv */
  NULL,              /* writev */

  mnemofs_sync,      /* sync */
  mnemofs_dup,       /* dup */
  mnemofs_fstat,     /* fstat */
  NULL,              /* fchstat */

  mnemofs_opendir,   /* opendir */
  mnemofs_closedir,  /* closedir */
  mnemofs_readdir,   /* readdir */
  mnemofs_rewinddir, /* rewinddir */

  mnemofs_bind,      /* bind */
  mnemofs_unbind,    /* unbind */
  mnemofs_statfs,    /* statfs */

  mnemofs_unlink,    /* unlink */
  mnemofs_mkdir,     /* mkdir */
  mnemofs_rmdir,     /* rmdir */
  mnemofs_rename,    /* rename */
  mnemofs_stat,      /* stat */
  NULL               /* chstat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mnemofs_open
 *
 * Description:
 *   Open a file given its path and populate its file pointer. The file
 *   pointer's private field is filled with information about the open file.
 *   See `open(2)` for details on the work and parameters of this function.
 *
 *   File pointers' private member have two parts...`mfs_ofd_d` is the actual
 *   entry for the file descriptor (fd), while the inner part is a reference
 *   counted shared portion formed from `mfs_ocom_s`. This way, multiple fds
 *   can point to the same file without problems. The shared portion is
 *   shared among the fds of the same file, while the upper part is different
 *   for each fd, regardless of whether they belong to the same file or not.
 *   A list of upper parts of files is stored in `sb` as a kernel linked
 *   list.
 *
 * Input Parameters:
 *   filep   - File pointer.
 *   relpath - Relative path to the file from the mount root.
 *   oflags  - Flags for opening the file.
 *   mode    - Access Mode (User|Group|All) if creating the file.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 ****************************************************************************/

static int mnemofs_open(FAR struct file *filep, FAR const char *relpath,
                        int oflags, mode_t mode)
{
  int                      ret    = OK;
  int                      flags;
  FAR const char          *child  = NULL;
  FAR struct inode        *inode;
  struct mfs_pitr_s        pitr;
  FAR struct mfs_sb_s     *sb;
  FAR struct mfs_ofd_s    *f;
  FAR struct mfs_ocom_s   *fcom;
  FAR struct mfs_dirent_s *dirent = NULL;

  MFS_LOG("OPEN", "Entry.");
  MFS_LOG("OPEN", "Requested path is %s.", relpath);
  MFS_LOG("OPEN", "Flags set as 0x%x.", oflags);
  MFS_LOG("OPEN", "Mode set as 0x%x.", mode);

  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);
  sb    = inode->i_private;
  DEBUGASSERT(sb != NULL);

  ret   = nxmutex_lock(&MFS_LOCK(sb));
  if (predict_false(ret < 0))
    {
      MFS_LOG("OPEN", "Failed to acquire mutex.");
      goto errout;
    }
  else
    {
      MFS_EXTRA_LOG("OPEN", "Mutex acquired.");
    }

  f     = fs_heap_zalloc(sizeof(*f));
  if (predict_false(f == NULL))
    {
      MFS_LOG("OPEN", "Failed to allocate file structure.");
      ret = -ENOMEM;
      goto errout_with_lock;
    }
  else
    {
      MFS_EXTRA_LOG("OPEN", "Allocated file structure at %p.", f);
    }

  fcom  = fs_heap_zalloc(sizeof(*fcom));
  if (predict_false(fcom == NULL))
    {
      MFS_LOG("OPEN", "Failed to allocate common file structure.");
      ret = -ENOMEM;
      goto errout_with_f;
    }
  else
    {
      MFS_EXTRA_LOG("OPEN", "Allocated common file structure at %p.", fcom);
    }

  f->com = fcom;
  f->com->refcount++;

  /* Check creation flags. */

  flags = mfs_get_patharr(sb, relpath, &f->com->path, &f->com->depth);
  MFS_EXTRA_LOG("OPEN", "Retrieved flags are 0x%x.", flags);

  if ((flags & MFS_EXIST) == 0)
    {
      MFS_EXTRA_LOG("OPEN", "Path doesn't exists.");

      if ((flags & MFS_P_ISDIR) != 0)
        {
          MFS_EXTRA_LOG("OPEN", "Parent is a directory.");

          if ((oflags & O_CREAT) != 0)
            {
              MFS_EXTRA_LOG("OPEN", "Creation flag is set.");

              /* Add direntry to parent's directory file. */

              f->com->new_ent = true;

              mfs_pitr_init(sb, f->com->path, f->com->depth, &pitr, true);
              child = mfs_path2childname(relpath);

              MFS_EXTRA_LOG("OPEN", "The final child is \"%s\".", child);

              mfs_pitr_appendnew(sb, f->com->path, f->com->depth, &pitr,
                                child, mode);
              mfs_pitr_free(&pitr);

              MFS_EXTRA_LOG("OPEN", "Added child direntry.");

              /* OK */

              MFS_EXTRA_LOG("OPEN", "All OK");
            }
        }
      else
        {
          MFS_LOG("OPEN", "Ancestor is not a directory.");

          ret = -ENOTDIR;
          goto errout_with_fcom;
        }
    }
  else
    {
      MFS_EXTRA_LOG("OPEN", "Path exists.");

      if ((flags & MFS_ISFILE) != 0)
        {
          MFS_EXTRA_LOG("OPEN", "Path points to a file.");

          /* NOTE: O_DIRECTORY is not supported. Use opendir. */

          if ((oflags & (O_CREAT | O_EXCL)) == (O_CREAT | O_EXCL))
            {
              MFS_LOG("OPEN", "O_CREAT and O_EXCL flag are set.");
              MFS_LOG("OPEN", "Operation failed as file exists with O_EXCL"
                      "flag set.");

              ret = -EEXIST;
              goto errout_with_fcom;
            }
          else
            {
              /* OK */

              MFS_EXTRA_LOG("OPEN", "All OK");
            }
        }
      else
        {
          MFS_EXTRA_LOG("OPEN", "Path points to a directory.");

          ret = -EISDIR;
          goto errout_with_fcom;
        }
    }

  /* Check r/w permission flags. */

  /* TODO: Update mtime and atime. */

  mfs_pitr_init(sb, f->com->path, f->com->depth, &pitr, true);
  mfs_pitr_readdirent(sb, f->com->path, &pitr, &dirent);

  if (dirent != NULL)
    {
      MFS_EXTRA_LOG_DIRENT(dirent);

      DEBUGASSERT((flags & MFS_EXIST) != 0);

      MFS_EXTRA_LOG("OPEN", "Direntry exists at %p.", dirent);

      if ((oflags & O_WRONLY) != 0 && (dirent->mode & O_WRONLY) == 0)
        {
          MFS_LOG("OPEN", "Write is not allowed.");

          ret = -EACCES;
          goto errout_with_dirent;
        }

      /* man page says: The argument flags must include one of the following
       * access modes: O_RDONLY, O_WRONLY, or O_RDWR.
       */

      mfs_free_dirent(dirent);
      dirent  = NULL;
    }
  else
    {
      DEBUGASSERT((flags & MFS_EXIST) == 0);
    }

  f->com->sz     = mfs_get_fsz(sb, f->com->path, f->com->depth);
  f->com->off    = 0;
  f->com->oflags = oflags;

  mfs_pitr_free(&pitr);

  MFS_EXTRA_LOG("OPEN", "Direntry processing is done.");

  /* Check Offset flags. */

  if ((oflags & (O_TRUNC | O_WRONLY)) == (O_TRUNC | O_WRONLY) ||
      (oflags & (O_TRUNC | O_RDWR)) == (O_TRUNC | O_RDWR))
    {
      /* Truncate to size 0. If write and truncate are mentioned only
       * then it's truncated. Else, the truncate flag is ignored.
       */

      MFS_EXTRA_LOG("OPEN", "O_TRUNC is set.");

      ret = mfs_lru_del(sb, 0, f->com->sz, f->com->path, f->com->depth);
      if (predict_false(ret < 0))
        {
          MFS_LOG("OPEN", "Could not truncate file.");
          goto errout_with_dirent;
        }
      else
        {
          MFS_EXTRA_LOG("OPEN", "File truncated.");
        }
    }

  if ((oflags & O_APPEND) != 0)
    {
      MFS_EXTRA_LOG("OPEN", "Append flag is set.");
      f->com->off = f->com->sz;
    }

  MFS_EXTRA_LOG_F(f);

  list_add_tail(&MFS_OFILES(sb), &f->list);
  filep->f_priv = f;
  MFS_EXTRA_LOG("OPEN", "File structure is set at %p.", filep);

  nxmutex_unlock(&MFS_LOCK(sb));
  MFS_EXTRA_LOG("OPEN", "Mutex released.");

  finfo("Mnemofs open exited with %d.", ret);
  return ret;

errout_with_dirent:
  if (dirent != NULL)
    {
      mfs_free_dirent(dirent);
    }

  mfs_free_patharr(fcom->path);

errout_with_fcom:
  fs_heap_free(fcom);

errout_with_f:
  fs_heap_free(f);

errout_with_lock:
  nxmutex_unlock(&MFS_LOCK(sb));
  MFS_EXTRA_LOG("OPEN", "Mutex released.");

errout:
  MFS_LOG("OPEN", "Exit | Return: %d.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_close
 *
 * Description:
 *   Closes a file, and frees up the allocated space for the open file's
 *   representation in the file pointer's private field. mnemofs also syncs
 *   up the on-flash data after closing of a file (unlike a typical fs as
 *   mentioned in `man`). This is to be prepared for random power loss at all
 *   possible times. See `close(2)` for details on the work and parameters
 *   of this function.
 *
 * Input Parameters:
 *   filep   - File pointer.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 ****************************************************************************/

static int mnemofs_close(FAR struct file *filep)
{
  int                   ret   = OK;
  FAR struct inode     *inode;
  FAR struct mfs_sb_s  *sb;
  FAR struct mfs_ofd_s *f;

  MFS_LOG("CLOSE", "Entry.");
  MFS_LOG("CLOSE", "File structure is at %p.", filep);

  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);
  sb    = inode->i_private;
  DEBUGASSERT(sb != NULL);

  ret   = nxmutex_lock(&MFS_LOCK(sb));
  if (predict_false(ret < 0))
    {
      MFS_LOG("CLOSE", "Failed to acquire mutex.");
      goto errout;
    }
  else
    {
      MFS_EXTRA_LOG("CLOSE", "Mutex acquired.");
    }

  f     = filep->f_priv;
  DEBUGASSERT(f != NULL);

  MFS_EXTRA_LOG_F(f);

  /* Flushing in-memory data to on-flash journal. */

  f->com->refcount--;

  MFS_EXTRA_LOG("CLOSE", "Reference Counter updated.");
  MFS_EXTRA_LOG_F(f);

  if (f->com->refcount == 0)
    {
      MFS_EXTRA_LOG("CLOSE", "Reference Counter is 0.");

      ret = mnemofs_flush(sb);
      if (predict_false(ret < 0))
        {
          MFS_LOG("CLOSE", "Could not flush file system.");

          /* This could be problematic.
           * TODO: For now, this is same as no-op.
           */

          f->com->refcount++; /* Revert to old refcount. */
          goto errout_with_lock;
        }
      else
        {
          MFS_EXTRA_LOG("CLOSE", "File system flushed.");
        }

      fs_heap_free(f->com->path);
      MFS_EXTRA_LOG("CLOSE", "Freed file structure path.");

      fs_heap_free(f->com);
      MFS_EXTRA_LOG("CLOSE", "Freed common file structure.");
    }

  list_delete(&f->list);
  MFS_EXTRA_LOG("CLOSE", "Removed file structure from open files list.");

  fs_heap_free(f);
  MFS_EXTRA_LOG("CLOSE", "Freed file structure.");

  filep->f_priv = NULL;

errout_with_lock:
  nxmutex_unlock(&MFS_LOCK(sb));
  MFS_EXTRA_LOG("CLOSE", "Mutex released.");

errout:
  MFS_LOG("CLOSE", "Exit | Return: %d.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_read
 *
 * Description:
 *   This reads an open file at the current offset into the provided `buffer`
 *   and puts in at most `buflen` bytes of data from the file. See `read(2)`
 *   for details on the work and parameters of this function.
 *
 *   Files (and directories) are just inverted CTZ skip lists. The CTZ
 *   methods provide a seamless interface to abstract away the data structure
 *   to provide an interface which allows the caller to only focus on the
 *   offset of the data from the start of the file, and not worry about the
 *   underlying details, pointers, blocks/pages, etc.
 *
 * Input Parameters:
 *   filep   - File pointer.
 *   buffer  - Buffer where to put read data.
 *   buflen  - Length of the buffer.
 *
 * Returned Value:
 *   0   - OK
 *   <= 0 - Bytes read.
 *
 ****************************************************************************/

static ssize_t mnemofs_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  ssize_t               ret   = 0;
  FAR struct inode     *inode;
  FAR struct mfs_sb_s  *sb;
  FAR struct mfs_ofd_s *f;

  MFS_LOG("READ", "Entry.");
  MFS_LOG("READ", "File structure is at %p.", filep);
  MFS_LOG("READ", "Buffer is at %p.", buffer);
  MFS_LOG("READ", "Length of the buffer is %zu.", buflen);

  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);
  sb    = inode->i_private;
  DEBUGASSERT(sb != NULL);

  ret   = nxmutex_lock(&MFS_LOCK(sb));
  if (predict_false(ret < 0))
    {
      MFS_LOG("READ", "Failed to acquire mutex.");
      goto errout;
    }
  else
    {
      MFS_EXTRA_LOG("READ", "Mutex acquired.");
    }

  f     = filep->f_priv;
  DEBUGASSERT(f != NULL);

  MFS_EXTRA_LOG_F(f);

  /* Check if allowed to read. */

  if ((f->com->oflags & O_RDONLY) == 0)
    {
      MFS_EXTRA_LOG("READ", "Not allowed to read file.");

      ret = -EINVAL;
      goto errout_with_lock;
    }

  /* Read data in CTZ from the current offset. */

  buflen = MIN(buflen, f->com->sz - f->com->off); /* TODO: Need to consider
                                                   * if this needs to be
                                                   * lower down the chain.
                                                   */

  MFS_EXTRA_LOG("READ", "Final buffer length is %zu.", buflen);

  ret = mfs_lru_rdfromoff(sb, f->com->off, f->com->path, f->com->depth,
                          buffer, buflen);
  if (ret < 0)
    {
      MFS_EXTRA_LOG("READ", "Could not read.");
      goto errout_with_lock;
    }

  ret = buflen;

  /* Update offset. */

  f->com->off += buflen;
  MFS_EXTRA_LOG("READ", "File structure offset updated.");
  MFS_EXTRA_LOG_F(f);

errout_with_lock:
  nxmutex_unlock(&MFS_LOCK(sb));
  MFS_EXTRA_LOG("READ", "Mutex released.");

errout:
  MFS_LOG("READ", "Exit | Return: %zd.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_write
 *
 * Description:
 *   This writes to an open file at the current offset from the provided
 *   `buffer` and puts in at most `buflen` bytes of data from the buffer. See
 *   `write(2)` for details on the work and parameters of this function.
 *
 * Input Parameters:
 *   filep   - File pointer.
 *   buffer  - Buffer from where to write data.
 *   buflen  - Length of the buffer.
 *
 * Returned Value:
 *   0   - OK
 *   <= 0 - Bytes written.
 *
 ****************************************************************************/

static ssize_t mnemofs_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  ssize_t               ret   = OK;
  FAR struct inode     *inode;
  FAR struct mfs_sb_s  *sb;
  FAR struct mfs_ofd_s *f;

  MFS_LOG("WRITE", "Entry.");
  MFS_LOG("WRITE", "File structure is at %p.", filep);
  MFS_LOG("WRITE", "Buffer is at %p.", buffer);
  MFS_LOG("WRITE", "Length of the buffer is %zu.", buflen);

  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);
  sb    = inode->i_private;
  DEBUGASSERT(sb != NULL);

  ret   = nxmutex_lock(&MFS_LOCK(sb));
  if (predict_false(ret < 0))
    {
      MFS_LOG("WRITE", "Failed to acquire mutex.");
      goto errout;
    }
  else
    {
      MFS_EXTRA_LOG("WRITE", "Mutex acquired.");
    }

  f     = filep->f_priv;
  DEBUGASSERT(f != NULL);

  MFS_EXTRA_LOG_F(f);

  /* Check if allowed to write. */

  if ((f->com->oflags & O_WRONLY) == 0)
    {
      MFS_EXTRA_LOG("WRITE", "Write not allowed.");
      ret = -EINVAL;
      goto errout_with_lock;
    }

  /* Write data to CTZ at the current offset. */

  ret = mfs_lru_wr(sb, f->com->off, buflen, f->com->path, f->com->depth,
                  buffer);
  if (predict_false(ret < 0))
    {
      MFS_LOG("WRITE", "Could not write to LRU.");
      goto errout_with_lock;
    }
  else
    {
      MFS_EXTRA_LOG("WRITE", "Completed writing to LRU.");
    }

  /* Update offset and size. */

  f->com->off += buflen;
  f->com->sz   = MAX(f->com->sz, f->com->off);
  ret          = buflen;

  MFS_EXTRA_LOG("WRITE", "Updated file offset and size.");
  MFS_EXTRA_LOG_F(f);

errout_with_lock:
  nxmutex_unlock(&MFS_LOCK(sb));
  MFS_EXTRA_LOG("WRITE", "Mutex  released.");

errout:
  MFS_LOG("WRITE", "Exit | Return: %zd.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_seek
 *
 * Description:
 *   Repositions the current offset of the file pointed by the file
 *   descriptor. See `lseek(2)` for details on the work and parameters of
 *   this function.
 *
 *   The new offset may be beyond the current file size. If that's the case,
 *   then on any subsequent writes, it will be such that there are bytes with
 *   '\0' in the gap/hole. In mnemofs, the whole hole situation is that the
 *   LRU doesn't know about the existence of holes, but comitting a file from
 *   LRU to the journal does write all the holes to the flash.
 *
 * Input Parameters:
 *   filep   - File pointer.
 *   offset  - Offset.
 *   whence  - From where the offset should be applied.
 *
 * Returned Value:
 *   0   - OK
 *   <= 0 - Final position.
 *
 ****************************************************************************/

static off_t mnemofs_seek(FAR struct file *filep, off_t offset, int whence)
{
  int                   ret   = OK;
  mfs_t                 pos;
  FAR struct inode     *inode;
  FAR struct mfs_sb_s  *sb;
  FAR struct mfs_ofd_s *f;

  MFS_LOG("SEEK", "Entry.");
  MFS_LOG("SEEK", "Offset: %u, Whence: %d", offset, whence);

  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);
  sb    = inode->i_private;
  DEBUGASSERT(sb != NULL);

  ret   = nxmutex_lock(&MFS_LOCK(sb));
  if (ret < 0)
    {
      MFS_LOG("SEEK", "Failed to acquire mutex.");
      goto errout;
    }
  else
    {
      MFS_EXTRA_LOG("SEEK", "Mutex acquired.");
    }

  f     = filep->f_priv;
  DEBUGASSERT(f != NULL);

  MFS_EXTRA_LOG("SEEK", "File offset: %u, Command offset: %u, Whence %d.",
                f->com->off, offset, whence);

  pos   = f->com->off;
  switch (whence)
    {
      case SEEK_SET:
        pos = offset;
        break;

      case SEEK_CUR:
        pos += offset;
        break;

      case SEEK_END:
        pos = f->com->sz + offset;
        break;

      default:
        ret = -EINVAL;
        goto errout_with_lock;
    }

  MFS_EXTRA_LOG("SEEK", "Proposed final position: %" PRIu32, pos);

  /* Check bounds of the position data type. */

  if (pos > f->com->off && offset < 0)
  {
    MFS_LOG("SEEK", "Proposed final position (%" PRIu32 ") out of bounds.",
            pos);
    ret = -EINVAL;
    goto errout_with_lock;
  }

  f->com->off = pos;
  ret         = pos;

  MFS_EXTRA_LOG("SEEK", "Final position: %" PRIu32, pos);

errout_with_lock:
  nxmutex_unlock(&MFS_LOCK(sb));
  MFS_EXTRA_LOG("SEEK", "Mutex released.");

errout:
  MFS_LOG("SEEK", "Exit | Return: %d.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_ioctl
 *
 * Description:
 *   Allows caller to directly control the underlying storage device. See
 *   `ioctl(2)` for details on the work and parameters of this function.
 *
 * Input Parameters:
 *   filep   - File pointer.
 *   cmd     - Command.
 *   arg     - Arguments to the command.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 ****************************************************************************/

static int mnemofs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  int                  ret   = OK;
  FAR struct inode    *inode;
  FAR struct inode    *drv;
  FAR struct mfs_sb_s *sb;

  finfo("Mnemofs ioctl with cmd %" PRIu32 "and arg %zu.", cmd, arg);

  inode = filep->f_inode;
  sb    = inode->i_private;
  drv   = sb->drv;

  ret   = nxmutex_lock(&MFS_LOCK(sb));
  if (ret < 0)
    {
      goto errout;
    }

  finfo("Lock acquired.");

  if (INODE_IS_MTD(drv))
    {
      ret = MTD_IOCTL(drv->u.i_mtd, cmd, arg);
    }
  else
    {
      finfo("Not an MTD driver.");
      ret = -ENOTTY;
    }

  nxmutex_unlock(&MFS_LOCK(sb));
  finfo("Lock released.");

errout:
  finfo("Mnemofs ioctl exited with %d.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_truncate
 *
 * Description:
 *   Truncates file specified to specified length. See `truncate(2)` for
 *   details on the work and parameters of this function.
 *
 * Input Parameters:
 *   filep   - File pointer.
 *   length  - Final size of the truncated file.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 * Assumptions/Limitations:
 *   If the length specified is bigger than the file, it won't increase the
 *   file size in itself. Only when there is a subsequent write to the offset
 *   will there be an increase in the file size. This can be changed, but is
 *   kept for efficiency.
 *
 ****************************************************************************/

static int mnemofs_truncate(FAR struct file *filep, off_t length)
{
  int                   ret   = OK;
  FAR struct inode     *inode;
  FAR struct mfs_sb_s  *sb;
  FAR struct mfs_ofd_s *f;

  finfo("Mnemofs truncate to length %d.", length);

  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);
  sb    = inode->i_private;
  DEBUGASSERT(sb != NULL);

  ret   = nxmutex_lock(&MFS_LOCK(sb));
  if (ret < 0)
    {
      goto errout;
    }

  finfo("Lock acquired.");

  f     = filep->f_priv;
  DEBUGASSERT(f != NULL);

  if (length < f->com->sz)
    {
      ret = mfs_lru_del(sb, length, f->com->sz - length, f->com->path,
                        f->com->depth);
      if (predict_false(ret < 0))
        {
          finfo("Error during truncate. Ret: %d.", ret);
          goto errout_with_lock;
        }
    }

errout_with_lock:
  nxmutex_unlock(&MFS_LOCK(sb));
  finfo("Lock released.");

errout:
  finfo("Mnemofs truncate exited with %d.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_sync
 *
 * Description:
 *   Causes all pending modifications that are in memory for a file
 *   descriptor to be written to the device. See `sync(2)` for  details on
 *   the work and parameters of this function.
 *
 *   For mnemofs, this means that the LRU entry for the file corresponding to
 *   the fd is committed to the journal.
 *
 * Input Parameters:
 *   filep   - File pointer.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 ****************************************************************************/

static int mnemofs_sync(FAR struct file *filep)
{
  int                   ret   = OK;
  FAR struct inode     *inode;
  FAR struct mfs_sb_s  *sb;
  FAR struct mfs_ofd_s *f;

  finfo("Mnemofs sync.");

  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);
  sb    = inode->i_private;
  DEBUGASSERT(sb != NULL);

  ret   = nxmutex_lock(&MFS_LOCK(sb));
  if (ret < 0)
    {
      goto errout;
    }

  finfo("Lock acquired.");

  f     = filep->f_priv;
  DEBUGASSERT(f != NULL);

  ret   = mnemofs_flush(sb);

  nxmutex_unlock(&MFS_LOCK(sb));
  finfo("Lock released.");

errout:
  finfo("Mnemofs sync exited with %d.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_dup
 *
 * Description:
 *   Creates a duplicate file descriptor. The new and old fds must be
 *   interchangeable and they share file offsets and file status flags.
 *   See `dup(2)` for  details on the work and parameters of this function.
 *
 *   The file descriptors have file pointers behind them. In mnemofs,
 *   the file pointers have different upper halves. But duplicate file
 *   pointers share a reference counted common part among themselves.
 *
 * Input Parameters:
 *   oldp   - Old file pointer.
 *   newp   - New file pointer.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 ****************************************************************************/

static int mnemofs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  int                    ret   = OK;
  FAR struct inode      *inode;
  FAR struct mfs_sb_s   *sb;
  FAR struct mfs_ofd_s  *of;
  FAR struct mfs_ofd_s  *nf;

  finfo("Mnemofs dup.");

  inode = oldp->f_inode;
  DEBUGASSERT(inode != NULL);
  sb    = inode->i_private;
  DEBUGASSERT(sb != NULL);

  ret   = nxmutex_lock(&MFS_LOCK(sb));
  if (ret < 0)
    {
      goto errout;
    }

  finfo("Lock acquired.");

  of    = oldp->f_priv;
  DEBUGASSERT(of != NULL);

  nf    = fs_heap_zalloc(sizeof(*nf));
  if (predict_false(nf == NULL))
    {
      finfo("No memory left.");
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  nf->com = of->com;

  /* Refcount limit. */

  if (nf->com->refcount == UINT8_MAX)
    {
      finfo("Refcount limit reached.");
      ret = -EMFILE;
      goto errout_with_nf;
    }

  nf->com->refcount++;

  /* Add the new upper half to the list of open fds. */

  list_add_tail(&MFS_OFILES(sb), &nf->list);
  newp->f_priv = nf;
  finfo("New file descriptor added to the end of the list of open files.");

  nxmutex_unlock(&MFS_LOCK(sb));
  finfo("Lock released.");
  return ret;

errout_with_nf:
  fs_heap_free(nf);

errout_with_lock:
  nxmutex_unlock(&MFS_LOCK(sb));
  finfo("Lock released.");

errout:
  finfo("Mnemofs dup exited with %d.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_fstat
 *
 * Description:
 *   Get file's status. See `fstat(2)` for  details on the work and
 *   parameters of this function.
 *
 *   In mnemofs, most of the metadata of a file is stored in the directory
 *   entry of the file inside the parent directories. The directory entries
 *   are stored in the form of a CTZ, and this is what a directory refers to.
 *
 * Input Parameters:
 *   filep  - File pointer.
 *   buf    - Stat result.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 ****************************************************************************/

static int mnemofs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  int                       ret    = OK;
  struct mfs_pitr_s         pitr;
  FAR struct inode         *inode;
  FAR struct mfs_sb_s      *sb;
  FAR struct mfs_ofd_s     *f;
  FAR struct mfs_dirent_s  *dirent = NULL;

  finfo("Mnemofs fstat.");

  inode = filep->f_inode;
  DEBUGASSERT(inode != NULL);
  sb    = inode->i_private;
  DEBUGASSERT(sb != NULL);

  ret   = nxmutex_lock(&MFS_LOCK(sb));
  if (ret < 0)
    {
      goto errout;
    }

  finfo("Lock acquired.");

  f     = filep->f_priv;
  DEBUGASSERT(f != NULL);

  mfs_pitr_init(sb, f->com->path, f->com->depth, &pitr, true);
  mfs_pitr_adv_tochild(&pitr, f->com->path);
  mfs_pitr_readdirent(sb, f->com->path, &pitr, &dirent);
  mfs_pitr_free(&pitr);

  DEBUGASSERT(dirent != NULL);

  buf->st_mode    = dirent->mode;
  buf->st_nlink   = 1; /* mnemofs doesn't yet support links. */
  buf->st_size    = dirent->sz;
  buf->st_atim    = dirent->st_atim;
  buf->st_ctim    = dirent->st_ctim;
  buf->st_blksize = sb->pg_sz;
  buf->st_blocks  = ((dirent->sz  + (sb->pg_sz - 1)) / sb->pg_sz); /* Ceil */

  mfs_free_dirent(dirent);

  nxmutex_unlock(&MFS_LOCK(sb));
  finfo("Lock released.");

errout:
  finfo("Mnemofs fstat exited with %d.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_opendir
 *
 * Description:
 *   Opens a directory for traversal. See `opendir(3)` for  details on the
 *   work and parameters of this function.
 *
 *   In mnemofs, the directories are in the form of a CTZ list like files,
 *   but the content in the list is the directory entries of its children.
 *   There is mfs_fsdirent->count which exists due to the fact that the first
 *   entry in readdir is `.`, second is `..` and then the actual contents. To
 *   save space, this count never goes above 2 because it's not needed as
 *   above 2, the offset in the CTZ list is sufficient for traversal.
 *
 * Input Parameters:
 *   mountpt - Mount point of the file system.
 *   relpath - Relative path of the directory.
 *   dir     - To populate this with open directory data structure.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 * Assumptions/Limitations:
 *   There is no attempt to `unlink` or `rmdir` when a directory is open.
 *   This will mess up the iterator, and mnemofs does not yet store a list
 *   of open directories to prevent unlink or rmdir when open or something
 *   like that. This is yet to be tested if mnemofs can handle such a case.
 *   This bug is marked in code in `readdir`.
 *
 ****************************************************************************/

static int mnemofs_opendir(FAR struct inode *mountpt,
                           FAR const char *relpath,
                           FAR struct fs_dirent_s **dir)
{
  int                        ret      = OK;
  int                        flags;
  mfs_t                      depth;
  FAR struct mfs_sb_s       *sb;
  FAR struct mfs_path_s     *path;
  FAR struct mfs_pitr_s     *pitr;
  FAR struct mfs_fsdirent_s *fsdirent;

  MFS_LOG("OPENDIR", "Entry.");
  MFS_LOG("OPENDIR", "Requested path is \"%s\".", relpath);

  DEBUGASSERT(mountpt != NULL);
  sb       = mountpt->i_private;
  DEBUGASSERT(mountpt->i_private);

  ret      = nxmutex_lock(&MFS_LOCK(sb));
  if (ret < 0)
    {
      MFS_LOG("OPENDIR", "Failed to acquire mutex.");
      goto errout;
    }
  else
    {
      MFS_EXTRA_LOG("OPENDIR", "Mutex acquired.");
    }

  flags    = mfs_get_patharr(sb, relpath, &path, &depth);
  if ((flags & MFS_ISDIR) == 0)
    {
      MFS_LOG("OPENDIR", "Not a directory.");
      ret = -ENOTDIR;
      goto errout_with_lock;
    }
  else
    {
      MFS_EXTRA_LOG("OPENDIR", "Path is at %p.", path);
      MFS_EXTRA_LOG("OPENDIR", "Path is at %" PRIu32, depth);
      MFS_EXTRA_LOG("OPENDIR", "Retrieved flags is %d.", flags);
    }

  ret = mfs_lru_getupdatedinfo(sb, path, depth);
  if (predict_false(ret < 0))
    {
      MFS_LOG("OPENDIR", "Failed to get updated information from LRU.");
      goto errout_with_path;
    }
  else
    {
      MFS_EXTRA_LOG("OPENDIR", "Got updated information from LRU.");
    }

  pitr     = fs_heap_zalloc(sizeof(*pitr));
  if (predict_false(pitr == NULL))
    {
      MFS_LOG("OPENDIR", "Could not allocate space for pitr.");
      ret  = -ENOMEM;
      goto errout_with_path;
    }
  else
    {
      MFS_EXTRA_LOG("OPENDIR", "Space allocated for pitr at %p.", pitr);
    }

  fsdirent = fs_heap_zalloc(sizeof(*fsdirent));
  if (predict_false(fsdirent == NULL))
    {
      MFS_LOG("OPENDIR", "Could not allocate space for FS Direntry.");
      ret  = -ENOMEM;
      goto errout_with_pitr;
    }
  else
    {
      MFS_EXTRA_LOG("OPENDIR", "Space allocated for FS Direntry at %p.",
                    fsdirent);
    }

  ret = mfs_pitr_init(sb, path, depth, pitr, false);
  if (predict_false(ret < 0))
    {
      MFS_LOG("OPENDIR", "Failed to initialize pitr.");
      goto errout_with_fsdirent;
    }
  else
    {
      MFS_EXTRA_LOG("OPENDIR", "Pitr initialized successfully.");
    }

  fsdirent->idx   = 0;
  fsdirent->path  = path;
  fsdirent->depth = depth;
  fsdirent->pitr  = pitr;

  MFS_EXTRA_LOG_FSDIRENT(fsdirent);

  *dir = (FAR struct fs_dirent_s *) fsdirent;
  MFS_EXTRA_LOG("OPENDIR", "Directory structure is %p.", dir);

  nxmutex_unlock(&MFS_LOCK(sb));
  MFS_EXTRA_LOG("OPENDIR", "Mutex released.");

  MFS_LOG("OPENDIR", "Exit | Return: %d.", ret);
  return ret;

errout_with_fsdirent:
  fs_heap_free(fsdirent);

errout_with_pitr:
  fs_heap_free(pitr);

errout_with_path:
  mfs_free_patharr(path);

errout_with_lock:
  nxmutex_unlock(&MFS_LOCK(sb));
  MFS_EXTRA_LOG("OPENDIR", "Mutex released.");

errout:
  MFS_LOG("OPENDIR", "Exit | Return: %d.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_closedir
 *
 * Description:
 *   Closes an opened directory. See `closedir(3)` for  details on the
 *   work and parameters of this function.
 *
 * Input Parameters:
 *   mountpt - Mount point of the file system.
 *   dir     - Open directory data structure.
 *
 * Returned Value:
 *   0   - OK
 *
 ****************************************************************************/

static int mnemofs_closedir(FAR struct inode *mountpt,
                            FAR struct fs_dirent_s *dir)
{
  struct mfs_fsdirent_s *fsdirent = (struct mfs_fsdirent_s *) dir;

  MFS_LOG("CLOSEDIR", "Entry.");
  MFS_LOG("CLOSEDIR", "FS Direntry at %p.", fsdirent);

  MFS_EXTRA_LOG_FSDIRENT(fsdirent);

  mfs_free_patharr(fsdirent->path);
  mfs_pitr_free(fsdirent->pitr);
  fs_heap_free(fsdirent->pitr);
  fs_heap_free(fsdirent);

  MFS_LOG("CLOSEDIR", "Exit | Return: %d.", OK);
  return OK;
}

/****************************************************************************
 * Name: mnemofs_readdir
 *
 * Description:
 *   Gives the directory entry of where the directory iterator currently
 *   points to and advances the iterator to the next directory entry. See
 *   `readdir(3)` for  details on the work and parameters of this function.
 *
 *  The first two entries are `.` and `..` respectively. The way mnemofs
 *  keeps a track of the current directory entry is through offset in the
 *  directory entry file. This offset is an unsigned integer, and to know
 *  when to return the "dots" and when to return actual directory entries,
 *  a counter is used that gives `.` when 0, `..` when 1, and the actual
 *  directory entries when 2. The offset increases only when counter is 2.
 *  The counter never increases above 2.
 *
 * Input Parameters:
 *   mountpt - Mount point of the file system.
 *   dir     - Open directory data structure.
 *   entry   - To populate structure with directory entry.
 *
 * Returned Value:
 *   0   - OK
 *
 ****************************************************************************/

static int mnemofs_readdir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir,
                           FAR struct dirent *entry)
{
  int                        ret      = OK;
  FAR struct mfs_sb_s       *sb;
  FAR struct mfs_dirent_s   *dirent;
  FAR struct mfs_fsdirent_s *fsdirent = (FAR struct mfs_fsdirent_s *) dir;

  MFS_LOG("READDIR", "Entry.");
  MFS_LOG("READDIR", "FS Direntry at %p.", fsdirent);
  MFS_EXTRA_LOG_FSDIRENT(fsdirent);

  DEBUGASSERT(mountpt != NULL);
  sb  = mountpt->i_private;
  DEBUGASSERT(sb != NULL);

  ret = nxmutex_lock(&MFS_LOCK(sb));
  if (ret < 0)
    {
      MFS_LOG("READDIR", "Failed to acquire mutex.");
      goto errout;
    }
  else
    {
      MFS_EXTRA_LOG("READDIR", "Mutex acquired.");
    }

  MFS_EXTRA_LOG("READDIR", "Curretn direntry index is %" PRIu8,
                fsdirent->idx);

  if (fsdirent->idx == 0)
    {
      /* . */

      MFS_EXTRA_LOG("READDIR", "Direntry for \".\"");

      snprintf(entry->d_name, NAME_MAX + 1, ".");
      entry->d_type = DTYPE_DIRECTORY;
      fsdirent->idx++;

      MFS_EXTRA_LOG("READDIR", "Direntry index updated to %" PRIu8,
                    fsdirent->idx);
      goto errout_with_lock;
    }
  else if (fsdirent->idx == 1)
    {
      /* .. */

      MFS_EXTRA_LOG("READDIR", "Direntry for \"..\"");

      snprintf(entry->d_name, NAME_MAX + 1, "..");
      entry->d_type = DTYPE_DIRECTORY;
      fsdirent->idx++;

      MFS_EXTRA_LOG("READDIR", "Direntry index updated to %" PRIu8,
                    fsdirent->idx);
      goto errout_with_lock;
    }
  else
    {
      MFS_EXTRA_LOG("READDIR", "Direntry for regular directory items.");
    }

  /* TODO: Need to think why *exactly* below line is needed. The LRU node
   * seems to contain wrong size during opendir, but updating it here
   * updates it to correct size, even though this updatedinfo is also
   * called in opendir?
   */

  ret = mfs_lru_getupdatedinfo(sb, fsdirent->path, fsdirent->depth);
  if (predict_false(ret < 0))
    {
      MFS_LOG("READDIR", "Failed to get updated information from LRU.");
      goto errout_with_lock;
    }
  else
    {
      MFS_EXTRA_LOG("READDIR", "Got updated information from LRU.");
    }

  ret = mfs_pitr_readdirent(sb, fsdirent->path, fsdirent->pitr, &dirent);
  if (predict_false(ret < 0))
    {
      MFS_LOG("READDIR", "Could not read direntry.");
      goto errout_with_lock;
    }
  else if (dirent == NULL)
    {
      MFS_LOG("READDIR", "No more direntries left.");
      MFS_LOG("READDIR", "End of directory.");

      ret = -ENOENT;
      goto errout_with_lock;
    }
  else
    {
      MFS_LOG("READDIR", "Direntry retrieved.");
      MFS_EXTRA_LOG_DIRENT(dirent);
    }

  memset(entry->d_name, 0, NAME_MAX + 1);
  MFS_EXTRA_LOG("READDIR", "Resetting entry name.");

  memcpy(entry->d_name, dirent->name, dirent->namelen);
  MFS_EXTRA_LOG("READDIR", "Setting entry name to be \"%.*s\".",
                dirent->namelen, dirent->name);

  entry->d_type = (S_ISDIR(dirent->mode) ? DTYPE_DIRECTORY: DTYPE_FILE);
  MFS_EXTRA_LOG("READDIR", "Setting entry d_type to %" PRIu8, entry->d_type);

  mfs_pitr_adv_bydirent(fsdirent->pitr, dirent);
  mfs_free_dirent(dirent);

errout_with_lock:
  nxmutex_unlock(&MFS_LOCK(sb));
  MFS_EXTRA_LOG("READDIR", "Mutex released.");

errout:
  MFS_LOG("READDIR", "Exit | Return: %d.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_rewinddir
 *
 * Description:
 *   Rewinds the directory iterator to the start of the directory. See
 *   `rewinddir(3)` for  details on the work and parameters of this function.
 *
 *  In mnemofs, this means resetting offset of CTZ list of directory to 0 and
 *  counter to 0;
 *
 * Input Parameters:
 *   mountpt - Mount point of the file system.
 *   dir     - Open directory data structure.
 *
 * Returned Value:
 *   0   - OK
 *
 ****************************************************************************/

static int mnemofs_rewinddir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir)
{
  int                    ret      = OK;
  FAR struct mfs_sb_s   *sb;
  struct mfs_fsdirent_s *fsdirent = (struct mfs_fsdirent_s *) dir;

  MFS_LOG("REWINDDIR", "Entry.");
  MFS_LOG("REWINDDIR", "FS Direntry at %p.", fsdirent);
  MFS_EXTRA_LOG_FSDIRENT(fsdirent);

  DEBUGASSERT(mountpt != NULL);
  sb  = mountpt->i_private;
  DEBUGASSERT(sb != NULL);

  ret = nxmutex_lock(&MFS_LOCK(sb));
  if (ret < 0)
    {
      MFS_LOG("REWINDDIR", "Failed to acquire mutex.");
      goto errout;
    }
  else
    {
      MFS_EXTRA_LOG("REWINDDIR", "Mutex acquired.");
    }

  mfs_pitr_reset(fsdirent->pitr);

  fsdirent->idx = 0;
  MFS_EXTRA_LOG("REWINDDIR", "Direntry index reset to 0.");

  MFS_EXTRA_LOG_FSDIRENT(fsdirent);

  nxmutex_unlock(&MFS_LOCK(sb));
  MFS_EXTRA_LOG("REWINDDIR", "Mutex released.");

errout:
  MFS_LOG("REWINDDIR", "Exit | Return: %d.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_bind
 *
 * Description:
 *   Mounts the file system. This is responsible for initializing all the
 *   in-memory data structures, which may including scanning the storage
 *   for serialized information, or formatting the storage, depending on the
 *   options provided during mount.
 *
 *   See `mount(2)` and `mount(8)` for more information.
 *
 *   In mnemofs, the superblock is not stored on disk yet. It does not have
 *   any information about the current state of the device, but rather just
 *   the information about the storage device, which is obtained from the
 *   driver anyway. To know if the device is formatted, the entire device
 *   is scanned, block by block, for the existence of the first block of the
 *   journal. This is quicker than searching page by page, as there are much
 *   fewer blocks than pages. The first 8 bytes of the first journal block
 *   are a special sequence. The journal contains the location of the master
 *   node (and vice versa, but it's easier to find/use journal and then
 *   master node).
 *
 * Input Parameters:
 *   driver - MTD driver
 *   data   - Mount options
 *   handle - To be updated with file system information.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 ****************************************************************************/

static int mnemofs_bind(FAR struct inode *driver, FAR const void *data,
                        FAR void** handle)
{
  int                    ret      = OK;
  bool                   format   = false;
  FAR char               buf[8];
  mfs_t                  i        = 0;
  mfs_t                  j        = 0;
  mfs_t                  mnblk1;
  mfs_t                  mnblk2;
  mfs_t                  jrnl_blk;
  FAR struct mfs_sb_s   *sb       = NULL;
  struct mtd_geometry_s  geo;

  MFS_LOG("BIND", "Entry.");

  MFS_EXTRA_LOG("BIND", "Resetting temporary buffer.");
  memset(buf, 0, 8);

  MFS_EXTRA_LOG("BIND", "Allocating superblock in memory.");
  sb = fs_heap_zalloc(sizeof(*sb));
  if (!sb)
    {
      MFS_LOG("BIND", "SB in-memory allocation error.");
      ret = -ENOMEM;
      goto errout;
    }
  else
    {
      MFS_EXTRA_LOG("BIND", "Superblock allocated at %p", sb);
    }

  /* Currently only supports NAND flashes (MTD devices). */

  if (driver && INODE_IS_MTD(driver))
    {
      if (!driver || !driver->u.i_mtd || !driver->u.i_mtd->ioctl)
        {
          MFS_LOG("BIND", "Unsupported device.");
          ret = -ENODEV;
          goto errout_with_sb;
        }
      else
        {
          MFS_EXTRA_LOG("BIND", "Device is of MTD type.");
        }

      ret = MTD_IOCTL(driver->u.i_mtd, MTDIOC_GEOMETRY,
                      (unsigned long) &geo);

      MFS_LOG("BIND", "MTD Driver Geometry read.");
      MFS_EXTRA_LOG("BIND", "MTD Driver Geometry details."
                    " Page size: %d, Block size: %d,"
                    " Pages/Block: %d, Blocks: %d\n",
                    geo.blocksize, geo.erasesize,
                    geo.erasesize / geo.blocksize, geo.neraseblocks);
    }
  else
    {
      MFS_LOG("BIND", "Device is not an MTD device.");
      ret = -ENODEV;
      goto errout_with_sb;
    }

  ret = nxmutex_init(&MFS_LOCK(sb));
  if (predict_false(ret < 0))
    {
      MFS_LOG("BIND", "FS-wide Mutex failed to initialize.");
      goto errout_with_sb;
    }
  else
    {
      MFS_EXTRA_LOG("BIND", "FS-wide Mutex Initialized.");
    }

  ret = nxmutex_lock(&MFS_LOCK(sb));
  if (ret < 0)
    {
      MFS_LOG("BIND", "Mutex failed to lock. Return %d.", ret);
      goto errout_with_lockinit;
    }
  else
    {
      MFS_EXTRA_LOG("BIND", "Mutex acquired.");
    }

  sb->drv             = driver;
  sb->pg_sz           = geo.blocksize;
  sb->blk_sz          = geo.erasesize;
  sb->n_blks          = geo.neraseblocks;
  sb->pg_in_blk       = MFS_BLKSZ(sb) / sb->pg_sz;
#ifdef CONFIG_MNEMOFS_JOURNAL_NBLKS
  MFS_JRNL(sb).n_blks = CONFIG_MNEMOFS_JOURNAL_NBLKS;
#else
  MFS_JRNL(sb).n_blks = MIN(5, MFS_NBLKS(sb) / 2);
#endif
  sb->log_blk_sz      = log2(MFS_BLKSZ(sb));
  sb->log_pg_sz       = log2(sb->pg_sz);
  sb->log_pg_in_blk   = log2(sb->pg_in_blk);
  sb->log_n_blks      = log2(MFS_NBLKS(sb));
  MFS_FLUSH(sb)       = false;

  list_initialize(&MFS_OFILES(sb));

  MFS_EXTRA_LOG("BIND", "SB initialized in-memory.");
  MFS_EXTRA_LOG("BIND", "SB Details.");
  MFS_EXTRA_LOG("BIND", "\tDriver: %p", driver);
  MFS_EXTRA_LOG("BIND", "\tPage Size: %" PRIu32, sb->pg_sz);
  MFS_EXTRA_LOG("BIND", "\tLog Page Size: %" PRIu8, sb->log_pg_sz);
  MFS_EXTRA_LOG("BIND", "\tBlock Size: %" PRIu32, sb->blk_sz);
  MFS_EXTRA_LOG("BIND", "\tLog Block Size: %" PRIu8,
                sb->log_blk_sz);
  MFS_EXTRA_LOG("BIND", "\tPages Per Block: %" PRIu16,
                sb->pg_in_blk);
  MFS_EXTRA_LOG("BIND", "\tBlocks: %" PRIu32, sb->n_blks);
  MFS_EXTRA_LOG("BIND", "\tLog Blocks: %" PRIu8, sb->log_n_blks);
  MFS_EXTRA_LOG("BIND", "\tJournal Blocks: %" PRIu16,
                MFS_JRNL(sb).n_blks);
  MFS_EXTRA_LOG("BIND", "\tFlush State: %" PRIu8, MFS_FLUSH(sb));

  sb->rw_buf        = fs_heap_zalloc(MFS_PGSZ(sb));
  if (predict_false(sb->rw_buf == NULL))
    {
      MFS_LOG("BIND", "RW Buffer in-memory allocation error.");
      goto errout_with_lock;
    }
  else
    {
      MFS_EXTRA_LOG("BIND", "RW Buffer allocated.");
    }

  /* TODO: Format the superblock in Block 0. */

  srand(time(NULL));

  if (!MFS_STRLITCMP(data, "autoformat"))
    {
      MFS_LOG("BIND", "Autoformat is ON.");

      /* Look for journal and maybe hopefully, the master node
       * if it comes first.
       */

      MFS_LOG("BIND", "Checking for valid mnemofs formatting.");

      for (i = 0; i < MFS_NBLKS(sb); i++)
        {
          MFS_EXTRA_LOG("BIND", "Checking start of Block %" PRIu32,
                        i + 1);
          mfs_read_page(sb, buf, 8, MFS_BLK2PG(sb, i), 0);

          for (j = 0; j < 8; j++)
            {
              MFS_EXTRA_LOG("BIND", "\tBlock %" PRIu32
                            ", Offset %" PRIu32 ": %x", i, j, buf[j]);
            }

          if (!MFS_STRLITCMP(buf, MFS_JRNL_MAGIC))
            {
              MFS_LOG("BIND", "Found Journal at Block %" PRIu32,
                      i + 1);

              ret = mfs_jrnl_init(sb, i);
              if (predict_false(ret < 0))
                {
                  MFS_LOG("BIND", "Error initializing journal.");
                  goto errout_with_rwbuf;
                }
              else
                {
                  MFS_LOG("BIND", "Journal initialized.");
                }

              ret = mfs_mn_init(sb, i);
              if (predict_false(ret < 0))
                {
                  MFS_LOG("BIND", "Error initializing masternode.");
                  goto errout_with_rwbuf;
                }
              else
                {
                  MFS_LOG("BIND", "Master node initialized.");
                }

              break;
            }

          MFS_EXTRA_LOG("BIND", "Resetting temporary buffer.");
          memset(buf, 0, 8);
        }

      if (predict_false(sb->mn.pg == 0))
        {
          MFS_LOG("BIND", "Journal not found on device.");
          MFS_LOG("BIND", "Device needs formatting.");

          format = true;
          memset(&MFS_JRNL(sb), 0, sizeof(struct mfs_jrnl_state_s));
          memset(&MFS_MN(sb), 0, sizeof(struct mfs_mn_s));
        }
      else
        {
          MFS_LOG("BIND", "Device already formatted.");

          mfs_lru_init(sb);
          mfs_ba_init(sb);
        }
    }

  if (format || !strncmp(data, "forceformat", 12))
    {
      /* Format. */

      if (format)
        {
          MFS_LOG("BIND", "Device format necessary.");
        }
      else
        {
          MFS_EXTRA_LOG("BIND", "Device formatting configured.");
        }

      mfs_ba_fmt(sb);
      mfs_lru_init(sb);

      mnblk1 = 0;
      mnblk2 = 0;

      ret = mfs_jrnl_fmt(sb, &mnblk1, &mnblk2, &jrnl_blk);
      if (predict_false(ret < 0))
        {
          MFS_LOG("BIND", "Error formatting Journal");
          goto errout_with_rwbuf;
        }
      else
        {
          MFS_LOG("BIND", "Journal format completed.");
        }

      ret = mfs_mn_fmt(sb, mnblk1, mnblk2, jrnl_blk);
      if (predict_false(ret < 0))
        {
          MFS_LOG("BIND", "Error formatting Master Node");
          goto errout_with_rwbuf;
        }
      else
        {
          MFS_LOG("BIND", "Master node format completed.");
        }

      MFS_LOG("BIND", "Device formatted.");
    }

  *handle = (FAR void *)sb;
  MFS_LOG("BIND", "Mount Successful. Super Block %p.", sb);

  nxmutex_unlock(&MFS_LOCK(sb));
  MFS_LOG("BIND", "Mutex released.");

  MFS_LOG("BIND", "Exit | Return: %d.", ret);
  return ret;

errout_with_rwbuf:
  fs_heap_free(sb->rw_buf);
  MFS_LOG("BIND", "RW Buffer freed.");

errout_with_lock:
  nxmutex_unlock(&MFS_LOCK(sb));
  MFS_EXTRA_LOG("BIND", "Mutex released.");

errout_with_sb:
  fs_heap_free(sb);
  MFS_LOG("BIND", "Superblock freed.");

errout_with_lockinit:
  nxmutex_destroy(&MFS_LOCK(sb));
  MFS_EXTRA_LOG("BIND", "Mutex destroyed.");

errout:
  MFS_LOG("BIND", "Exit | Return: %d.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_unbind
 *
 * Description:
 *   Unmounts the file system.
 *
 *   See `umount(2)` and `umount(8)` for more information.
 *
 * Input Parameters:
 *   handle - File system information.
 *   driver - To be populated with the MTD driver
 *   flags  - Flags for unmounting.
 *
 * Returned Value:
 *   0   - OK
 *
 ****************************************************************************/

static int mnemofs_unbind(FAR void *handle, FAR struct inode **driver,
                          unsigned int flags)
{
  FAR struct mfs_sb_s *sb;

  MFS_LOG("UNBIND", "Entry.");

  DEBUGASSERT(handle);
  sb      = handle;
  MFS_LOG("UNBIND", "Superblock %p.", sb);

  *driver = sb->drv;
  MFS_LOG("UNBIND", "Driver %p.", driver);

  mfs_jrnl_free(sb);
  mfs_ba_free(sb);

  nxmutex_destroy(&MFS_LOCK(sb));
  MFS_EXTRA_LOG("UNBIND", "Mutex destroyed.");

  fs_heap_free(sb->rw_buf);
  MFS_LOG("UNBIND", "RW Buffer freed.");

  fs_heap_free(sb);
  MFS_LOG("UNBIND", "Superblock freed.");

  MFS_LOG("UNBIND", "Exit.");
  return OK;
}

/****************************************************************************
 * Name: mnemofs_statfs
 *
 * Description:
 *   Get file system statistics. See `statfs(2)` for more information.
 *
 * Input Parameters:
 *   mountpt - Mount point of the file system.
 *   buf     - To populate with file system statistics.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 ****************************************************************************/

static int mnemofs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  int                  ret = OK;
  FAR struct mfs_sb_s *sb;

  finfo("Mnemofs statfs.");

  DEBUGASSERT(mountpt != NULL);
  sb  = mountpt->i_private;
  DEBUGASSERT(sb != NULL);

  ret = nxmutex_lock(&MFS_LOCK(sb));
  if (ret < 0)
    {
      goto errout;
    }

  finfo("Lock acquired.");

  buf->f_type    = MNEMOFS_SUPER_MAGIC;
  buf->f_bsize   = sb->pg_sz;
  buf->f_blocks  = MFS_NBLKS(sb) * sb->pg_in_blk;
  buf->f_bavail  = mfs_ba_getavailpgs(sb);
  buf->f_namelen = UINT8_MAX;

  nxmutex_unlock(&MFS_LOCK(sb));
  finfo("Lock released.");

errout:
  finfo("Mnemofs statfs exited with %d.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_unlink
 *
 * Description:
 *   Remove a file's entry from the file system. This might delete the file
 *   as well. See `unlink(2)` for more information.
 *
 *   In mnemofs, the directory entry of the file is removed from its parent's
 *   directory file (CTZ list). The pages used by the file are marked for
 *   delete to the block allocator. When the master node is moved, and if the
 *   blocks of which the pages are part of is ready for erase, then the
 *   block allocator erases those blocks.
 *
 * Input Parameters:
 *   mountpt - Mount point of the file system.
 *   buf     - To populate with file system statistics.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 ****************************************************************************/

static int mnemofs_unlink(FAR struct inode *mountpt, FAR const char *relpath)
{
  int                    ret       = OK;
  int                    flags;
  mfs_t                  depth;
  FAR struct mfs_sb_s   *sb;
  FAR struct mfs_path_s *path;

  finfo("Mnemofs unlink at path \"%s\".", relpath);

  DEBUGASSERT(mountpt != NULL);
  sb  = mountpt->i_private;
  DEBUGASSERT(sb != NULL);

  ret = nxmutex_lock(&MFS_LOCK(sb));
  if (ret < 0)
    {
      goto errout;
    }

  finfo("Lock acquired.");

  flags = mfs_get_patharr(sb, relpath, &path, &depth);
  if ((flags & MFS_ISFILE) == 0)
    {
      ret = -EISDIR;
      goto errout_with_lock;
    }

  mfs_pitr_rm(sb, path, depth, true);

  mfs_free_patharr(path);

errout_with_lock:
  nxmutex_unlock(&MFS_LOCK(sb));
  finfo("Lock released.");

errout:
  finfo("Mnemofs unlink exited with %d.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_mkdir
 *
 * Description:
 *   Create a directory at given relpath. See `mkdir(2)` for more
 *   information.
 *
 *   In mnemofs, the directory entry of the file is appended to its parent's
 *   directory file (CTZ list).
 *
 * Input Parameters:
 *   mountpt - Mount point of the file system.
 *   relpath - Relative path of the new directory.
 *   mode    - Mode of the new directory (ACL).
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 ****************************************************************************/

static int mnemofs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
                         mode_t mode)
{
  int                    ret   = OK;
  int                    flags;
  mfs_t                  depth;
  struct mfs_pitr_s      pitr;
  FAR struct mfs_sb_s   *sb;
  FAR struct mfs_path_s *path;

  MFS_LOG("MKDIR", "Entry.");
  MFS_LOG("MKDIR", "New directory at \"%s\".", relpath);

  mode |= S_IFDIR;

  MFS_LOG("MKDIR", "Mode is 0x%x.", mode);

  DEBUGASSERT(mountpt != NULL);
  sb  = mountpt->i_private;
  MFS_EXTRA_LOG("MKDIR", "Superblock is %p.", sb);
  DEBUGASSERT(sb != NULL);

  ret = nxmutex_lock(&MFS_LOCK(sb));
  if (ret < 0)
    {
      goto errout;
    }
  else
    {
      MFS_EXTRA_LOG("MKDIR", "Mutex lock acquired.");
    }

  flags = mfs_get_patharr(sb, relpath, &path, &depth);
  MFS_EXTRA_LOG("MKDIR", "Retrieved flags are 0x%x.", flags);
  MFS_EXTRA_LOG("MKDIR", "Path received is at %p.", path);
  MFS_EXTRA_LOG("MKDIR", "Depth of path is %" PRIu32 ".", depth);

  if ((flags & MFS_EXIST) != 0)
    {
      MFS_LOG("MKDIR", "The requested directory already exists.");
      ret = -EEXIST;
      goto errout_with_path;
    }
  else
    {
      MFS_LOG("MKDIR", "The requested directory does not exist.");
      if ((flags & MFS_P_EXIST) != 0)
        {
          MFS_EXTRA_LOG("MKDIR", "Parent exists.");
          if ((flags  & MFS_P_ISDIR) != 0)
            {
              /* OK */

              MFS_EXTRA_LOG("MKDIR", "Parent is all right.");
            }
          else
            {
              MFS_EXTRA_LOG("MKDIR", "Ancestor is not a directory.");
              ret = -ENOTDIR;
              goto errout_with_path;
            }
        }
      else
        {
          MFS_EXTRA_LOG("MKDIR", "Parent not found.");
          ret = -ENOENT;
          goto errout_with_path;
        }
    }

  memset(&path[depth - 1], 0, sizeof(struct mfs_path_s));
  MFS_EXTRA_LOG("MKDIR", "Resetting, at index %u, path array %p.",
                depth - 1, path);

  mfs_pitr_init(sb, path, depth, &pitr, true);
  MFS_EXTRA_LOG("MKDIR", "The path contains the child.");
  MFS_EXTRA_LOG("MKDIR", "Parent iterator initialized.");
  MFS_EXTRA_LOG("MKDIR", "\tDepth of parent %" PRIu32 ".",
                pitr.depth);
  MFS_EXTRA_LOG("MKDIR", "\tCurrent iteration offset %" PRIu32 ".",
                pitr.c_off);
  MFS_EXTRA_LOG("MKDIR", "\tParent's offset %" PRIu32 ".",
                pitr.p.off);
  MFS_EXTRA_LOG("MKDIR", "\tParent's size %" PRIu32 ".", pitr.p.sz);
  MFS_EXTRA_LOG("MKDIR", "\tParent's CTZ (%" PRIu32 ", %" PRIu32 ")"
                , pitr.p.ctz.idx_e, pitr.p.ctz.pg_e);

  /* The last incomplete direntry will be added by mfs_pitr_appendnew. */

  ret = mfs_pitr_appendnew(sb, path, depth, &pitr, relpath, mode);
  if (predict_false(ret < 0))
    {
      MFS_LOG("MKDIR", "Could not append direntry. Return %d", ret);
      goto errout_with_path;
    }
  else
    {
      MFS_EXTRA_LOG("MKDIR", "Direntry append successful.");
      MFS_EXTRA_LOG("MKDIR", "\tDepth of parent %" PRIu32 ".", pitr.depth);
      MFS_EXTRA_LOG("MKDIR", "\tCurrent iteration offset %" PRIu32 ".",
                    pitr.c_off);
      MFS_EXTRA_LOG("MKDIR", "\tParent's offset %" PRIu32 ".", pitr.p.off);
      MFS_EXTRA_LOG("MKDIR", "\tParent's size %" PRIu32 ".", pitr.p.sz);
      MFS_EXTRA_LOG("MKDIR", "\tParent's CTZ (%" PRIu32 ", %" PRIu32 ")",
                    pitr.p.ctz.idx_e, pitr.p.ctz.pg_e);
    }

  mfs_pitr_free(&pitr);

  MFS_LOG("MKDIR", "Directory created at \"%s\".", relpath);

  mfs_free_patharr(path);

  nxmutex_unlock(&MFS_LOCK(sb));
  MFS_EXTRA_LOG("MKDIR", "Mutex released.");

  MFS_LOG("MKDIR", "Exit | Return: %d.", ret);
  return ret;

errout_with_path:
  mfs_free_patharr(path);

  mfs_pitr_free(&pitr);

  nxmutex_unlock(&MFS_LOCK(sb));
  MFS_EXTRA_LOG("MKDIR", "Mutex released.");

  /* TODO: The flush operation does not work properly, and causes memory
   * leaks by most likely not flushing out anything and keeping it in
   * memory.
   */

errout:
  MFS_LOG("MKDIR", "Exit | Return: %d.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_rmdir
 *
 * Description:
 *   Removes a directory from given relpath. See `rmdir(2)` for more
 *   information.
 *
 * Input Parameters:
 *   mountpt - Mount point of the file system.
 *   relpath - Relative path of the new directory.
 *   mode    - Mode of the new directory (ACL).
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 ****************************************************************************/

static int mnemofs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  int                    ret   = OK;
  int                    flags;
  mfs_t                  depth;
  struct mfs_pitr_s      pitr;
  FAR struct mfs_sb_s   *sb;
  FAR struct mfs_path_s *path;

  MFS_LOG("RMDIR", "Entry");
  MFS_LOG("RMDIR", "Directory \"%s\" to be removed.", relpath);

  DEBUGASSERT(mountpt != NULL);
  sb  = mountpt->i_private;
  DEBUGASSERT(sb != NULL);

  ret = nxmutex_lock(&MFS_LOCK(sb));
  if (ret < 0)
    {
      MFS_LOG("RMDIR", "Mutex could not be acquired.");
      goto errout;
    }
  else
    {
      MFS_EXTRA_LOG("RMDIR", "Mutex acquired.");
    }

  flags = mfs_get_patharr(sb, relpath, &path, &depth);
  if ((flags & MFS_ISDIR) == 0)
    {
      MFS_LOG("RMDIR", "FS Object is not a directory.");
      ret = -ENOTDIR;
      goto errout_with_lock;
    }
  else
    {
      MFS_EXTRA_LOG("RMDIR", "Path is at %p with depth %" PRIu32, path,
                    depth);
      MFS_EXTRA_LOG("RMDIR", "FS Object is a directory.");
      MFS_EXTRA_LOG("RMDIR", "Retrieved flags are 0x%x.", flags);
    }

  mfs_pitr_init(sb, path, depth, &pitr, true);
  mfs_pitr_adv_tochild(&pitr, path);

  if (!mfs_obj_isempty(sb, path, &pitr))
    {
      MFS_EXTRA_LOG("RMDIR", "Directory is not empty.");
      ret = -ENOTEMPTY;
      goto errout_with_pitr;
    }

  mfs_pitr_free(&pitr);

  mfs_pitr_rm(sb, path, depth, true);

errout_with_pitr:
  mfs_free_patharr(path);

errout_with_lock:
  nxmutex_unlock(&MFS_LOCK(sb));
  MFS_EXTRA_LOG("RMDIR", "Mutex released.");

errout:
  MFS_LOG("RMDIR", "Exit | Return: %d.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_rename
 *
 * Description:
 *   Moves a file or directory between paths. See `rename(2)` for more
 *   information.
 *
 *   In mnemofs, this involves erasing a direntry from old path, and adding
 *   it to the new path. There are some further complications regarding
 *   directories and files, as mentioned in `rename(2)`.
 *
 * Input Parameters:
 *   mountpt - Mount point of the file system.
 *   relpath - Relative path of the new directory.
 *   mode    - Mode of the new directory (ACL).
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 ****************************************************************************/

static int mnemofs_rename(FAR struct inode *mountpt,
                          FAR const char *oldrelpath,
                          FAR const char *newrelpath)
{
  int                      ret     = OK;
  int                      oflags;
  int                      nflags;
  bool                     nexists;
  bool                     odir    = false;
  bool                     ndir    = false;
  mfs_t                    odepth;
  mfs_t                    ndepth;
  struct mfs_pitr_s        opitr;
  struct mfs_pitr_s        npitr;
  FAR struct mfs_sb_s     *sb;
  FAR struct mfs_path_s   *opath;
  FAR struct mfs_path_s   *npath;
  FAR struct mfs_dirent_s *odirent = NULL;

  finfo("Mnemofs rename \"%s\" to \"%s\".", oldrelpath, newrelpath);

  DEBUGASSERT(mountpt != NULL);
  sb  = mountpt->i_private;
  DEBUGASSERT(sb != NULL);

  ret = nxmutex_lock(&MFS_LOCK(sb));
  if (ret < 0)
    {
      goto errout;
    }

  finfo("Lock acquired.");

  oflags = mfs_get_patharr(sb, oldrelpath, &opath, &odepth);
  if ((oflags & MFS_EXIST) == 0)
    {
      ret = -ENOENT;
      goto errout_with_opath;
    }

  nflags = mfs_get_patharr(sb, newrelpath, &npath, &ndepth);
  if ((nflags & MFS_P_EXIST) == 0)
    {
      ret = -ENONET;
      goto errout_with_npath;
    }

  odir    = ((oflags & MFS_ISDIR) != 0);
  ndir    = ((nflags & MFS_ISDIR) != 0);
  nexists = ((nflags & MFS_EXIST) != 0);

  if (nexists && odir && !ndir)
    {
      ret = -ENOTDIR;
      goto errout_with_npath;
    }

  if (nexists && !odir && ndir)
    {
      ret = -EISDIR;
      goto errout_with_npath;
    }

  mfs_pitr_init(sb, npath, ndepth, &npitr, nexists);
  mfs_pitr_init(sb, opath, odepth, &opitr, true);

  /* If new path already exists, remove the direntry. If it's a non-empty
   * directory, then raise error.
   */

  if (nexists)
    {
      mfs_pitr_adv_tochild(&opitr, opath);
      mfs_pitr_adv_tochild(&npitr, npath);

      mfs_pitr_readdirent(sb, opath, &opitr, &odirent);
      if (ndir && !mfs_obj_isempty(sb, npath, &npitr))
        {
          ret = -ENOTEMPTY;
          goto errout_with_pitr;
        }

      mfs_pitr_reset(&npitr);

      mfs_pitr_rm(sb, npath, ndepth, false);
    }

  mfs_pitr_adv_tochild(&npitr, npath);

  mfs_pitr_appenddirent(sb, npath, ndepth, &npitr, odirent);
  mfs_pitr_rmdirent(sb, opath, odepth, &opitr, odirent);

errout_with_pitr:
  mfs_free_dirent(odirent);
  mfs_pitr_free(&opitr);
  mfs_pitr_free(&npitr);

errout_with_npath:
  mfs_free_patharr(npath);

errout_with_opath:
  mfs_free_patharr(opath);
  nxmutex_unlock(&MFS_LOCK(sb));
  finfo("Lock released.");

errout:
  finfo("Mnemofs rename exited with ret %d.", ret);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_stat
 *
 * Description:
 *   Get stats of a file. See `stat(2)` for more information.
 *
 *   In mnemofs, most of the relevant information is stored in the dirent
 *   of the file in its parent's directory file. The stats of the root are
 *   available in the master node.
 *
 * Input Parameters:
 *   mountpt - Mount point of the file system.
 *   relpath - Relative path of the new directory.
 *   buf     - File stats to populate.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 ****************************************************************************/

static int mnemofs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                        FAR struct stat *buf)
{
  int                     ret       = OK;
  int                     flags;
  mfs_t                   depth;
  struct mfs_pitr_s       pitr;
  FAR struct mfs_sb_s     *sb;
  FAR struct mfs_path_s   *path;
  FAR struct mfs_dirent_s *dirent   = NULL;

  MFS_LOG("STAT", "Entry.");
  MFS_LOG("STAT", "Requested path is \"%s\".", relpath);

  DEBUGASSERT(mountpt != NULL);
  sb  = mountpt->i_private;
  DEBUGASSERT(sb != NULL);

  ret = nxmutex_lock(&MFS_LOCK(sb));
  if (ret < 0)
    {
      MFS_LOG("STAT", "Could not acquire mutex.");
      goto errout;
    }
  else
    {
      MFS_EXTRA_LOG("STAT", "Mutex acquired.");
    }

  MFS_EXTRA_LOG_MN(&MFS_MN(sb));

  flags = mfs_get_patharr(sb, relpath, &path, &depth);
  if ((flags & MFS_EXIST) == 0)
    {
      MFS_LOG("STAT", "File does not exist.");
      ret = -ENOENT;
      goto errout_with_path;
    }
  else
    {
      MFS_EXTRA_LOG("STAT", "Path is at %p.", path);
      MFS_EXTRA_LOG("STAT", "Depth is %" PRIu32, depth);
      MFS_EXTRA_LOG("STAT", "Retrieved flags are 0x%x.", flags);
    }

  ret = mfs_lru_getupdatedinfo(sb, path, depth);
  if (predict_false(ret < 0))
    {
      MFS_LOG("STAT", "Could not get updated information.");
      goto errout_with_path;
    }
  else
    {
      MFS_EXTRA_LOG("STAT", "Updated information received from LRU.");
    }

  mfs_pitr_init(sb, path, depth, &pitr, true);
  mfs_pitr_adv_tochild(&pitr, path);

  ret = mfs_pitr_readdirent(sb, path, &pitr, &dirent);
  if (predict_false(ret < 0))
    {
      MFS_LOG("STAT", "Could not read from direntry.");
      goto errout_with_path;
    }
  else if (dirent == NULL)
    {
      MFS_LOG("STAT", "No entry found.");
      ret = -ENOENT;
      goto errout_with_path;
    }
  else
    {
      MFS_EXTRA_LOG("STAT", "Direntry read successfully.");
    }

  MFS_EXTRA_LOG_DIRENT(dirent);

  buf->st_nlink   = 1;
  buf->st_blksize = sb->pg_sz;
  buf->st_size    = dirent->sz;
  buf->st_mode    = dirent->mode;
  buf->st_atim    = dirent->st_atim;
  buf->st_ctim    = dirent->st_ctim;
  buf->st_mtim    = dirent->st_mtim;
  buf->st_blocks  = dirent->ctz.idx_e + 1;

  mfs_free_dirent(dirent);
  mfs_pitr_free(&pitr);

errout_with_path:
  mfs_free_patharr(path);

  nxmutex_unlock(&MFS_LOCK(sb));
  MFS_EXTRA_LOG("STAT", "Mutex released.");

errout:
  MFS_LOG("STAT", "Exit | Return: %d.", ret);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mnemofs_flush(FAR struct mfs_sb_s *sb)
{
  int  ret    = OK;
  bool change;

  /* Emtpy the LRU, and maybe the journal as well. */

  finfo("Flush operation started.");

  for (; ; )
    {
      change = false;
      if (!mfs_lru_isempty(sb))
        {
          finfo("LRU needs to be flushed.");

          change = true;
          ret    = mfs_lru_flush(sb);
          if (predict_false(ret < 0))
            {
              goto errout;
            }
        }

      if (!mfs_jrnl_isempty(sb) &&
          MFS_JRNL(sb).log_cblkidx >= MFS_JRNL_LIM(sb))
        {
          finfo("Journal needs to be flushed.");

          change = true;

          ret = mfs_jrnl_flush(sb);
          if (predict_false(ret < 0))
            {
              goto errout;
            }
        }

      if (!change)
        {
          break;
        }

      finfo("Finished Iteration.");
    }

errout:
  return ret;
}
