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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/crc8.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>

#include <errno.h>
#include <string.h>
#include <sys/statfs.h>

#include "mnemofs.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int mnemofs_bind(FAR struct inode *driver, FAR const void *data,
                        FAR void **handle);
static int mnemofs_unbind(FAR void *handle, FAR struct inode **driver,
                          unsigned int flags);
static int mnemofs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf);
static int mnemofs_open(FAR struct file *filep, FAR const char *relpath,
                        int oflags, mode_t mode);
static int mnemofs_close(FAR struct file *filep);
static ssize_t mnemofs_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t mnemofs_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static off_t mnemofs_seek(FAR struct file *filep, off_t offset, int whence);
static int mnemofs_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int mnemofs_truncate(FAR struct file *filep, off_t length);
static int mnemofs_sync(FAR struct file *filep);
static int mnemofs_dup(FAR const struct file *oldp, FAR struct file *newp);
static int mnemofs_fstat(FAR const struct file *filep, FAR struct stat *buf);
static int mnemofs_opendir(FAR struct inode *mountpt,
                           FAR const char *relpath,
                           FAR struct fs_dirent_s **dir);
static int mnemofs_closedir(FAR struct inode *mountpt,
                            FAR struct fs_dirent_s *dir);
static int mnemofs_readdir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir,
                           FAR struct dirent *entry);
static int mnemofs_rewinddir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir);
static int mnemofs_unlink(FAR struct inode *mountpt,
                          FAR const char *relpath);
static int mnemofs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
                         mode_t mode);
static int mnemofs_rmdir(FAR struct inode *mountpt, FAR const char *relpath);
static int mnemofs_rename(FAR struct inode *mountpt,
                          FAR const char *oldrelpath,
                          FAR const char *newrelpath);
static int mnemofs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                        FAR struct stat *buf);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MFS_MOUNTOPT_NONE        0
#define MFS_MOUNTOPT_AUTOFORMAT  1
#define MFS_MOUNTOPT_FORCEFORMAT 2

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mnemofs_mountpt_to_sb
 *
 * Description:
 * Return the mounted file system state associated with one mountpoint
 * inode.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode to inspect.
 *
 * Returned Value:
 * A pointer to the mounted file system state is returned on success.
 * NULL is returned if mountpt is NULL.
 *
 ****************************************************************************/

static FAR struct mfs_sb_s *mnemofs_mountpt_to_sb(FAR struct inode *mountpt)
{
  if (mountpt == NULL)
    {
      return NULL;
    }

  return mountpt->i_private;
}

/****************************************************************************
 * Name: mnemofs_file_to_sb
 *
 * Description:
 * Return the mounted file system state associated with one VFS file
 * structure.
 *
 * Input Parameters:
 *   filep - The VFS file structure to inspect.
 *
 * Returned Value:
 * A pointer to the mounted file system state is returned on success.
 * NULL is returned if no file-system state can be resolved.
 *
 ****************************************************************************/

static FAR struct mfs_sb_s *mnemofs_file_to_sb(FAR const struct file *filep)
{
  FAR struct mfs_ofd_s *ofd;

  if (filep == NULL)
    {
      return NULL;
    }

  ofd = filep->f_priv;
  if (ofd != NULL)
    {
      return ofd->sb;
    }

  if (filep->f_inode == NULL)
    {
      return NULL;
    }

  return filep->f_inode->i_private;
}

/****************************************************************************
 * Name: mnemofs_lock
 *
 * Description:
 * Acquire the per-mount mnemofs mutex.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * invalid input or lock failure.
 *
 ****************************************************************************/

static int mnemofs_lock(FAR struct mfs_sb_s *sb)
{
  if (sb == NULL)
    {
      return -EINVAL;
    }

  return nxmutex_lock(&sb->lock);
}

/****************************************************************************
 * Name: mnemofs_unlock
 *
 * Description:
 * Flush and invalidate the shared read/write buffer, then release the
 * per-mount mutex.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mnemofs_unlock(FAR struct mfs_sb_s *sb)
{
  int ret;

  DEBUGASSERT(sb != NULL);
  ret = mfs_rwbuf_sync(sb);
  if (ret < 0)
    {
      ferr("Could not sync read/write buffer: %d\n", ret);
    }

  mfs_rwbuf_invalidate(sb);
  nxmutex_unlock(&sb->lock);
}

/****************************************************************************
 * Name: mnemofs_dirmeta_checksum
 *
 * Description:
 * Compute the checksum stored in an on-flash directory metadata record.
 *
 * Input Parameters:
 *   dirmeta - The directory metadata record to checksum.
 *
 * Returned Value:
 * The computed checksum value is returned.
 *
 ****************************************************************************/

static uint8_t mnemofs_dirmeta_checksum(
               FAR const struct mfs_dirmeta_s *dirmeta)
{
  return crc8((FAR const uint8_t *)dirmeta, sizeof(*dirmeta) - 1);
}

/****************************************************************************
 * Name: mnemofs_dirmeta_is_valid
 *
 * Description:
 * Verify the magic and checksum stored in a directory metadata record.
 *
 * Input Parameters:
 *   dirmeta - The directory metadata record to validate.
 *
 * Returned Value:
 * true is returned if dirmeta contains a valid directory metadata record.
 * false is returned otherwise.
 *
 ****************************************************************************/

static bool mnemofs_dirmeta_is_valid(
            FAR const struct mfs_dirmeta_s *dirmeta)
{
  if (dirmeta == NULL)
    {
      return false;
    }

  return dirmeta->magic == MFS_DIR_MAGIC &&
         dirmeta->checksum == mnemofs_dirmeta_checksum(dirmeta);
}

/****************************************************************************
 * Name: mnemofs_superblock_init
 *
 * Description:
 * Initialize one in-memory superblock image for writing to flash.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   rootdir - The root directory block to store.
 *   superblock - The location to receive the initialized image.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mnemofs_superblock_init(FAR const struct mfs_sb_s *sb,
                                    mfs_t rootdir,
                                    FAR struct mfs_superblock_s *superblock)
{
  DEBUGASSERT(sb != NULL && superblock != NULL);

  memset(superblock, 0, sizeof(*superblock));
  superblock->magic         = MNEMOFS_SUPER_MAGIC;
  superblock->npages        = MFS_PAGE_COUNT(sb);
  superblock->pagesize      = MFS_PAGE_SIZE(sb);
  superblock->pagesperblock = MFS_PAGES_PER_BLOCK(sb);
  superblock->nblocks       = MFS_BLOCK_COUNT(sb);
  superblock->rootdir       = rootdir;
  superblock->version       = sb->version;
}

/****************************************************************************
 * Name: mnemofs_superblock_is_valid
 *
 * Description:
 * Verify that an on-flash superblock matches the current device geometry
 * and references a plausible root directory block.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   superblock - The on-flash superblock image to validate.
 *
 * Returned Value:
 * true is returned if the superblock is valid. false is returned
 * otherwise.
 *
 ****************************************************************************/

static bool mnemofs_superblock_is_valid(
            FAR const struct mfs_sb_s *sb,
            FAR const struct mfs_superblock_s *superblock)
{
  if (sb == NULL || superblock == NULL)
    {
      return false;
    }

  return superblock->magic == MNEMOFS_SUPER_MAGIC &&
         superblock->npages == MFS_PAGE_COUNT(sb) &&
         superblock->pagesize == MFS_PAGE_SIZE(sb) &&
         superblock->pagesperblock == MFS_PAGES_PER_BLOCK(sb) &&
         superblock->nblocks == MFS_BLOCK_COUNT(sb) &&
         superblock->rootdir < MFS_BLOCK_COUNT(sb);
}

/****************************************************************************
 * Name: mnemofs_find_next_good_block
 *
 * Description:
 * Scan forward from start until a non-bad block is found.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   start - The first block number to inspect.
 *   block - The location to receive the next usable block.
 *
 * Returned Value:
 * Zero (OK) is returned on success. -ENOSPC is returned if no good block
 * exists at or after start. A negated errno value is returned on other
 * failures.
 *
 ****************************************************************************/

static int mnemofs_find_next_good_block(FAR struct mfs_sb_s *sb, mfs_t start,
                                        FAR mfs_t *block)
{
  int ret;

  if (sb == NULL || block == NULL)
    {
      return -EINVAL;
    }

  for (; start < MFS_BLOCK_COUNT(sb); start++)
    {
      ret = mfs_is_bad_block(sb, start);
      if (ret < 0)
        {
          return ret;
        }

      if (ret == 0)
        {
          * block = start;
          return OK;
        }
    }

  return -ENOSPC;
}

/****************************************************************************
 * Name: mnemofs_find_initial_rootdir
 *
 * Description:
 * Find the first usable root-directory block after the chosen superblock.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   superblock - The block number reserved for the superblock.
 *   rootdir - The location to receive the candidate root directory
 *   block.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_find_initial_rootdir(FAR struct mfs_sb_s *sb,
                                        mfs_t superblock,
                                        FAR mfs_t *rootdir)
{
  if (sb == NULL || rootdir == NULL)
    {
      return -EINVAL;
    }

  return mnemofs_find_next_good_block(sb, superblock + 1, rootdir);
}

/****************************************************************************
 * Name: mnemofs_validate_rootdir
 *
 * Description:
 * Verify that rootdir is usable and contains a valid self-parenting
 * directory metadata page.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   superblock - The superblock block number.
 *   rootdir - The root directory block number to validate.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_validate_rootdir(FAR struct mfs_sb_s *sb,
                                    mfs_t superblock, mfs_t rootdir)
{
  struct mfs_dirmeta_s dirmeta;
  ssize_t nread;
  mfs_t page;
  int ret;

  if (sb == NULL)
    {
      return -EINVAL;
    }

  if (rootdir <= superblock || rootdir >= MFS_BLOCK_COUNT(sb))
    {
      return -EINVAL;
    }

  ret = mfs_is_bad_block(sb, rootdir);
  if (ret < 0)
    {
      return ret;
    }

  if (ret != 0)
    {
      return -EINVAL;
    }

  page = MFS_BLOCK_TO_PAGE(sb, rootdir);
  nread = mfs_read_page(sb, page, sb->rwbuf);
  if (nread < 0)
    {
      return nread;
    }

  if (nread != 1)
    {
      return -EIO;
    }

  memcpy(&dirmeta, sb->rwbuf, sizeof(dirmeta));
  if (!mnemofs_dirmeta_is_valid(&dirmeta) || dirmeta.parent != rootdir)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: mnemofs_read_superblock
 *
 * Description:
 * Read and validate the on-flash superblock, then return its root
 * directory block number.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   superblock - The block number that stores the superblock.
 *   rootdir - The location to receive the validated root directory
 *   block.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_read_superblock(FAR struct mfs_sb_s *sb, mfs_t superblock,
                                   FAR mfs_t *rootdir)
{
  struct mfs_superblock_s onflash;
  ssize_t nread;
  mfs_t page;
  int ret;

  if (sb == NULL || rootdir == NULL)
    {
      return -EINVAL;
    }

  page = MFS_BLOCK_TO_PAGE(sb, superblock);
  nread = mfs_read_page(sb, page, sb->rwbuf);
  if (nread < 0)
    {
      return nread;
    }

  if (nread != 1)
    {
      return -EIO;
    }

  memcpy(&onflash, sb->rwbuf, sizeof(onflash));
  if (!mnemofs_superblock_is_valid(sb, &onflash))
    {
      return -EINVAL;
    }

  ret = mnemofs_validate_rootdir(sb, superblock, onflash.rootdir);
  if (ret < 0)
    {
      return ret;
    }

  if (onflash.version > MFS_FORMAT_VERSION_MAJOR)
    {
      ferr("higher version present on flash: %u > %u\n",
           onflash.version, MFS_FORMAT_VERSION_MAJOR);
      return -EPROTONOSUPPORT;
    }

  sb->version = onflash.version;
  * rootdir = onflash.rootdir;
  return OK;
}

/****************************************************************************
 * Name: mnemofs_write_superblock
 *
 * Description:
 * Write a freshly initialized superblock image to flash.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   superblock - The block number that stores the superblock.
 *   rootdir - The root directory block number to record.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_write_superblock(FAR struct mfs_sb_s *sb,
                                    mfs_t superblock, mfs_t rootdir)
{
  struct mfs_superblock_s onflash;
  int ret;
  ssize_t nwritten;
  mfs_t page;

  if (sb == NULL)
    {
      return -EINVAL;
    }

  mnemofs_superblock_init(sb, rootdir, &onflash);
  ret = mfs_rwbuf_prepare_write(sb);
  if (ret < 0)
    {
      return ret;
    }

  memset(sb->rwbuf, 0, MFS_PAGE_SIZE(sb));
  memcpy(sb->rwbuf, &onflash, sizeof(onflash));

  page = MFS_BLOCK_TO_PAGE(sb, superblock);
  nwritten = mfs_write_page(sb, page, sb->rwbuf);
  if (nwritten < 0)
    {
      return nwritten;
    }

  return nwritten == 1 ? OK : -EIO;
}

/****************************************************************************
 * Name: mnemofs_format_rootdir
 *
 * Description:
 * Write the metadata page for a freshly formatted root directory block.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   rootdir - The root directory block number to initialize.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_format_rootdir(FAR struct mfs_sb_s *sb, mfs_t rootdir)
{
  struct mfs_dirmeta_s dirmeta;
  int ret;
  ssize_t nwritten;
  mfs_t page;

  if (sb == NULL)
    {
      return -EINVAL;
    }

  memset(&dirmeta, 0, sizeof(dirmeta));
  dirmeta.magic    = MFS_DIR_MAGIC;
  dirmeta.parent   = rootdir;
  dirmeta.checksum = mnemofs_dirmeta_checksum(&dirmeta);

  ret = mfs_rwbuf_prepare_write(sb);
  if (ret < 0)
    {
      return ret;
    }

  memset(sb->rwbuf, 0, MFS_PAGE_SIZE(sb));
  memcpy(sb->rwbuf, &dirmeta, sizeof(dirmeta));

  page = MFS_BLOCK_TO_PAGE(sb, rootdir);
  nwritten = mfs_write_page(sb, page, sb->rwbuf);
  if (nwritten < 0)
    {
      return nwritten;
    }

  return nwritten == 1 ? OK : -EIO;
}

/****************************************************************************
 * Name: mnemofs_erase_good_blocks
 *
 * Description:
 * Erase every usable block on the backing device while leaving bad blocks
 * untouched.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_erase_good_blocks(FAR struct mfs_sb_s *sb)
{
  mfs_t block;
  int bad;
  int ret;

  if (sb == NULL)
    {
      return -EINVAL;
    }

  for (block = 0; block < MFS_BLOCK_COUNT(sb); block++)
    {
      bad = mfs_is_bad_block(sb, block);
      if (bad < 0)
        {
          return bad;
        }

      if (bad != 0)
        {
          continue;
        }

      ret = mfs_erase_blocks(sb, block, 1);
      if (ret < 0)
        {
          return ret;
        }

      if (ret != 1)
        {
          return -EIO;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: mnemofs_format
 *
 * Description:
 * Format the device as a fresh mnemofs volume with the specified
 * superblock and root directory blocks.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   superblock - The superblock block number.
 *   rootdir - The root directory block number.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_format(FAR struct mfs_sb_s *sb, mfs_t superblock,
                          mfs_t rootdir)
{
  int ret;

  sb->version = MFS_FORMAT_VERSION_MAJOR;
  ret = mnemofs_erase_good_blocks(sb);
  if (ret < 0)
    {
      return ret;
    }

  ret = mnemofs_format_rootdir(sb, rootdir);
  if (ret < 0)
    {
      return ret;
    }

  return mnemofs_write_superblock(sb, superblock, rootdir);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mfs_update_rootdir
 *
 * Description:
 * Persist a replacement root directory block into both the root directory
 * metadata page and the on-flash superblock.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   rootdir - The new root directory block number.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_update_rootdir(FAR struct mfs_sb_s *sb, mfs_t rootdir)
{
  int ret;

  if (sb == NULL)
    {
      ferr("invalid sb\n");
      return -EINVAL;
    }

  ret = mnemofs_format_rootdir(sb, rootdir);
  if (ret < 0)
    {
      ferr("mnemofs_format_rootdir failed: %d\n", ret);
      return ret;
    }

  ret = mnemofs_write_superblock(sb, sb->superblock, rootdir);
  if (ret < 0)
    {
      ferr("mnemofs_write_superblock failed: %d\n", ret);
      return ret;
    }

  sb->rootdir = rootdir;
  finfo("updated rootdir\n");
  return OK;
}

/****************************************************************************
 * Name: mnemofs_bind
 *
 * Description:
 * Bind an MTD device to mnemofs. This captures geometry, allocates the
 * shared read/write buffer, parses mount options, validates or formats
 * the on-flash superblock, and initializes allocator state for the
 * mounted volume.
 *
 * Input Parameters:
 *   driver - The backing MTD inode.
 *   data - Optional mount options string. Supported options (strings) are
 *   "autoformat" and "forceformat".
 *   handle - To be updated with the file system's state (sb).
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_bind(FAR struct inode *driver, FAR const void *data,
                        FAR void **handle)
{
  FAR struct mfs_sb_s *sb;
  FAR const char *mountopt;
  int mountoptid;
  mfs_t superblock;
  mfs_t rootdir;
  int ret;

  if (driver == NULL)
    {
      ferr("The driver is NULL\n");
      return -EINVAL;
    }

  if (handle == NULL)
    {
      ferr("The handle is NULL\n");
      return -EINVAL;
    }

  if (!INODE_IS_MTD(driver) || driver->u.i_mtd == NULL)
    {
      ferr("The required driver not found.\n");
      return -ENODEV;
    }

  sb = kmm_zalloc(sizeof(*sb));
  if (sb == NULL)
    {
      ferr("Could not allocated sb\n");
      return -ENOMEM;
    }

  finfo("sb allocated: %p\n", sb);

  ret = nxmutex_init(&sb->lock);
  if (ret < 0)
    {
      ferr("Could not initialize fs mutex\n");
      goto errout_with_sb;
    }

  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("Could not acquire fs mutex\n");
      goto errout_with_mutex;
    }

  sb->driver = driver;
  sb->mtd    = driver->u.i_mtd;
  ret = MTD_IOCTL(sb->mtd, MTDIOC_GEOMETRY,
                  (unsigned long)(uintptr_t)&sb->geo);
  if (ret < 0)
    {
      ferr("Could not get device geometry\n");
      goto errout_with_lock;
    }

  sb->erasestate = 0xff;
  ret = MTD_IOCTL(sb->mtd, MTDIOC_ERASESTATE,
                  (unsigned long)(uintptr_t)&sb->erasestate);
  if (ret < 0 && ret != -ENOTTY)
    {
      ferr("Could not get erase state\n");
      goto errout_with_lock;
    }

  if (sb->geo.blocksize == 0 || sb->geo.erasesize == 0 ||
      sb->geo.erasesize % sb->geo.blocksize != 0)
    {
      ferr("Invalid geometry\n");

      ret = -EINVAL;
      goto errout_with_lock;
    }

  sb->pagesperblk = sb->geo.erasesize / sb->geo.blocksize;
  if (sb->pagesperblk < 2)
    {
      ferr("Number of pages is too low in MTD device\n");
      ret = -EINVAL;
      goto errout_with_lock;
    }

  sb->rwbuf = kmm_malloc(MFS_PAGE_SIZE(sb));
  if (sb->rwbuf == NULL)
    {
      ferr("Could not allocated read/write buffer for fs\n");
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  memset(sb->rwbuf, 0, MFS_PAGE_SIZE(sb));
  sb->rwpage = MFS_LOCATION_INVALID;
  sb->rwvalid = false;
  sb->rwdirty = false;
  sb->version = MFS_FORMAT_VERSION_MAJOR;

  mountopt = data;
  mountoptid = MFS_MOUNTOPT_NONE;
  if (mountopt != NULL && mountopt[0] != '\0')
    {
      if (strcmp(mountopt, "autoformat") == 0)
        {
          finfo("Mount option set: autoformat\n");
          mountoptid = MFS_MOUNTOPT_AUTOFORMAT;
        }
      else if (strcmp(mountopt, "forceformat") == 0)
        {
          finfo("Mount option set: forceformat\n");
          mountoptid = MFS_MOUNTOPT_FORCEFORMAT;
        }
      else
        {
          ferr("Invalid mount option\n");
          ret = -EINVAL;
          goto errout_with_lock;
        }
    }
  else
    {
      finfo("Mount option set: none (default)\n");
    }

  ret = mnemofs_find_next_good_block(sb, 0, &superblock);
  if (ret < 0)
    {
      ferr("Could not locate superblock\n");
      goto errout_with_lock;
    }

  sb->superblock = superblock;
  if (mountoptid == MFS_MOUNTOPT_FORCEFORMAT)
    {
      /* Force format the disk with mnemofs */

      ret = mnemofs_find_initial_rootdir(sb, superblock, &rootdir);
      if (ret < 0)
        {
          ferr("Could not locate initial root directory block\n");
          goto errout_with_lock;
        }

      sb->rootdir = rootdir;
      ret = mnemofs_format(sb, superblock, rootdir);
      if (ret < 0)
        {
          ferr("Could not force format volume\n");
          goto errout_with_lock;
        }
    }
  else
    {
      ret = mnemofs_read_superblock(sb, superblock, &rootdir);
      if (ret < 0)
        {
          if (ret == -EINVAL && mountoptid == MFS_MOUNTOPT_AUTOFORMAT)
            {
              ret = mnemofs_find_initial_rootdir(sb, superblock, &rootdir);
              if (ret < 0)
                {
                  ferr("Could not locate initial root directory block\n");
                  goto errout_with_lock;
                }

              sb->rootdir = rootdir;
              ret = mnemofs_format(sb, superblock, rootdir);
              if (ret < 0)
                {
                  ferr("Could not auto format volume\n");
                  goto errout_with_lock;
                }
            }
          else
            {
              if (ret != -EPROTONOSUPPORT)
                {
                  ferr("Volume is not formatted as mnemofs\n");
                }

              goto errout_with_lock;
            }
        }
      else
        {
          sb->rootdir = rootdir;
        }
    }

  ret = mfs_alloc_init(sb);
  if (ret < 0)
    {
      ferr("Could not initialize page allocator\n");
      goto errout_with_lock;
    }

  list_initialize(&sb->ofiles);
  * handle = sb;
  mnemofs_unlock(sb);
  return OK;

errout_with_lock:
  mfs_alloc_uninit(sb);
  mnemofs_unlock(sb);
  kmm_free(sb->rwbuf);
errout_with_mutex:
  nxmutex_destroy(&sb->lock);
errout_with_sb:
  kmm_free(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_unbind
 *
 * Description:
 * Tear down a mounted mnemofs instance once no open files remain.
 *
 * Input Parameters:
 *   handle - The bound file system state.
 *   driver - The location to receive the backing MTD inode.
 *   flags - Unbind flags from the VFS.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_unbind(FAR void *handle, FAR struct inode **driver,
                          unsigned int flags)
{
  FAR struct mfs_sb_s *sb = handle;
  int ret;

  switch (flags)
    {
      default:
        break;
    }

  if (sb == NULL || driver == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  if (!list_is_empty(&sb->ofiles))
    {
      ferr("open files still active\n");
      ret = -EBUSY;
      goto errout_with_lock;
    }

  * driver = sb->driver;
  mfs_alloc_uninit(sb);
  mnemofs_unlock(sb);
  kmm_free(sb->rwbuf);
  nxmutex_destroy(&sb->lock);
  kmm_free(sb);
  return OK;

errout_with_lock:
  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_statfs
 *
 * Description:
 * Return file-system-wide statistics for one mounted mnemofs instance.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode.
 *   buf - The location to receive the translated statfs data.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct mfs_sb_s *sb;
  int ret;

  if (buf == NULL)
    {
      ferr("invalid statfs buffer\n");
      return -EINVAL;
    }

  sb = mnemofs_mountpt_to_sb(mountpt);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  memset(buf, 0, sizeof(*buf));
  buf->f_type    = MNEMOFS_SUPER_MAGIC;
  buf->f_bsize   = MFS_PAGE_SIZE(sb);
  buf->f_blocks  = MFS_BLOCK_COUNT(sb);
  buf->f_bfree   = MFS_BLOCK_COUNT(sb) > sb->superblock + 1 ?
                   MFS_BLOCK_COUNT(sb) - sb->superblock - 1 : 0;
  buf->f_bavail  = buf->f_bfree;
  buf->f_namelen = MFS_NAME_MAX;
  mnemofs_unlock(sb);
  return OK;
}

/****************************************************************************
 * Name: mnemofs_open
 *
 * Description:
 * VFS open wrapper for regular files in mnemofs.
 *
 * Input Parameters:
 *   filep - The VFS file structure to initialize.
 *   relpath - The relative path to open.
 *   oflags - Open flags.
 *   mode - Create mode used with O_CREAT.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_open(FAR struct file *filep, FAR const char *relpath,
                        int oflags, mode_t mode)
{
  FAR struct mfs_sb_s *sb;
  int ret;

  sb = mnemofs_file_to_sb(filep);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  ret = mfs_file_open(sb, filep, relpath, oflags, mode);
  if (ret < 0)
    {
      ferr("mfs_file_open failed: %d\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_close
 *
 * Description:
 * VFS close wrapper for regular files in mnemofs.
 *
 * Input Parameters:
 *   filep - The VFS file structure to close.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_close(FAR struct file *filep)
{
  FAR struct mfs_sb_s *sb;
  int ret;

  sb = mnemofs_file_to_sb(filep);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  ret = mfs_file_close(sb, filep);
  if (ret < 0)
    {
      ferr("mfs_file_close failed: %d\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_read
 *
 * Description:
 * VFS read wrapper for regular files in mnemofs.
 *
 * Input Parameters:
 *   filep - The VFS file structure to read from.
 *   buffer - The caller's read buffer.
 *   buflen - The number of bytes requested.
 *
 * Returned Value:
 * A non-negative byte count is returned on success. A negated errno
 * value is returned on failure.
 *
 ****************************************************************************/

static ssize_t mnemofs_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct mfs_sb_s *sb;
  ssize_t ret;

  sb = mnemofs_file_to_sb(filep);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %zd\n", ret);
      return ret;
    }

  ret = mfs_file_read(sb, filep, buffer, buflen);
  if (ret < 0)
    {
      ferr("mfs_file_read failed: %zd\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_write
 *
 * Description:
 * VFS write wrapper for regular files in mnemofs.
 *
 * Input Parameters:
 *   filep - The VFS file structure to write to.
 *   buffer - The source bytes to write.
 *   buflen - The number of bytes to write.
 *
 * Returned Value:
 * A non-negative byte count is returned on success. A negated errno
 * value is returned on failure.
 *
 ****************************************************************************/

static ssize_t mnemofs_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  FAR struct mfs_sb_s *sb;
  ssize_t ret;

  sb = mnemofs_file_to_sb(filep);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %zd\n", ret);
      return ret;
    }

  ret = mfs_file_write(sb, filep, buffer, buflen);
  if (ret < 0)
    {
      ferr("mfs_file_write failed: %zd\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_seek
 *
 * Description:
 * VFS seek wrapper for regular files in mnemofs.
 *
 * Input Parameters:
 *   filep - The VFS file structure to reposition.
 *   offset - The seek offset.
 *   whence - The seek base selector.
 *
 * Returned Value:
 * The new file position is returned on success. A negated errno value is
 * returned on failure.
 *
 ****************************************************************************/

static off_t mnemofs_seek(FAR struct file *filep, off_t offset, int whence)
{
  FAR struct mfs_sb_s *sb;
  off_t ret;

  sb = mnemofs_file_to_sb(filep);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %lld\n", (long long)ret);
      return ret;
    }

  ret = mfs_file_seek(sb, filep, offset, whence);
  if (ret < 0)
    {
      ferr("mfs_file_seek failed: %lld\n", (long long)ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_ioctl
 *
 * Description:
 * VFS ioctl wrapper for regular files in mnemofs.
 *
 * Input Parameters:
 *   filep - The VFS file structure to operate on.
 *   cmd - The ioctl command.
 *   arg - The ioctl argument.
 *
 * Returned Value:
 * Zero (OK) or another command-specific value is returned on success. A
 * negated errno value is returned on failure.
 *
 ****************************************************************************/

static int mnemofs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct mfs_sb_s *sb;
  int ret;

  sb = mnemofs_file_to_sb(filep);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  ret = mfs_file_ioctl(sb, filep, cmd, arg);
  if (ret < 0)
    {
      ferr("mfs_file_ioctl failed: %d\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_truncate
 *
 * Description:
 * VFS truncate wrapper for regular files in mnemofs.
 *
 * Input Parameters:
 *   filep - The VFS file structure to truncate.
 *   length - The new stored file length.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct mfs_sb_s *sb;
  int ret;

  sb = mnemofs_file_to_sb(filep);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  ret = mfs_file_truncate(sb, filep, length);
  if (ret < 0)
    {
      ferr("mfs_file_truncate failed: %d\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_sync
 *
 * Description:
 * VFS sync wrapper for regular files in mnemofs.
 *
 * Input Parameters:
 *   filep - The VFS file structure to sync.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_sync(FAR struct file *filep)
{
  FAR struct mfs_sb_s *sb;
  int ret;

  sb = mnemofs_file_to_sb(filep);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  ret = mfs_file_sync(sb, filep);
  if (ret < 0)
    {
      ferr("mfs_file_sync failed: %d\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_dup
 *
 * Description:
 * VFS dup wrapper for regular files in mnemofs.
 *
 * Input Parameters:
 *   oldp - The existing VFS file structure.
 *   newp - The new VFS file structure to initialize.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_dup(FAR const struct file *oldp, FAR struct file *newp)
{
  FAR struct mfs_sb_s *sb;
  int ret;

  sb = mnemofs_file_to_sb(oldp);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  ret = mfs_file_dup(sb, oldp, newp);
  if (ret < 0)
    {
      ferr("mfs_file_dup failed: %d\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_fstat
 *
 * Description:
 * VFS fstat wrapper for regular files in mnemofs.
 *
 * Input Parameters:
 *   filep - The VFS file structure to inspect.
 *   buf - The location to receive the translated stat data.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct mfs_sb_s *sb;
  int ret;

  sb = mnemofs_file_to_sb(filep);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  ret = mfs_file_fstat(sb, filep, buf);
  if (ret < 0)
    {
      ferr("mfs_file_fstat failed: %d\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_opendir
 *
 * Description:
 * VFS opendir wrapper for directories in mnemofs.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode.
 *   relpath - The relative path of the directory to open.
 *   dir - The location to receive the opened directory stream.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_opendir(FAR struct inode *mountpt,
                           FAR const char *relpath,
                           FAR struct fs_dirent_s **dir)
{
  FAR struct mfs_sb_s *sb;
  int ret;

  sb = mnemofs_mountpt_to_sb(mountpt);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  ret = mfs_dir_opendir(sb, relpath, dir);
  if (ret < 0)
    {
      ferr("mfs_dir_opendir failed: %d\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_closedir
 *
 * Description:
 * VFS closedir wrapper for directories in mnemofs.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode.
 *   dir - The directory stream to close.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_closedir(FAR struct inode *mountpt,
                            FAR struct fs_dirent_s *dir)
{
  FAR struct mfs_sb_s *sb;
  int ret;

  sb = mnemofs_mountpt_to_sb(mountpt);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  ret = mfs_dir_closedir(sb, dir);
  if (ret < 0)
    {
      ferr("mfs_dir_closedir failed: %d\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_readdir
 *
 * Description:
 * VFS readdir wrapper for directories in mnemofs.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode.
 *   dir - The directory stream to read.
 *   entry - The location to receive the next dirent record.
 *
 * Returned Value:
 * Zero (OK) is returned on success. -ENOENT is returned at end of
 * directory. A negated errno value is returned on other failures.
 *
 ****************************************************************************/

static int mnemofs_readdir(FAR struct inode *mountpt,
                           FAR struct fs_dirent_s *dir,
                           FAR struct dirent *entry)
{
  FAR struct mfs_sb_s *sb;
  int ret;

  sb = mnemofs_mountpt_to_sb(mountpt);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  ret = mfs_dir_readdir(sb, dir, entry);
  if (ret < 0 && ret != -ENOENT)
    {
      ferr("mfs_dir_readdir failed: %d\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_rewinddir
 *
 * Description:
 * VFS rewinddir wrapper for directories in mnemofs.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode.
 *   dir - The directory stream to rewind.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_rewinddir(FAR struct inode *mountpt,
                             FAR struct fs_dirent_s *dir)
{
  FAR struct mfs_sb_s *sb;
  int ret;

  sb = mnemofs_mountpt_to_sb(mountpt);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  ret = mfs_dir_rewinddir(sb, dir);
  if (ret < 0)
    {
      ferr("mfs_dir_rewinddir failed: %d\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_unlink
 *
 * Description:
 * VFS unlink wrapper for regular files in mnemofs.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode.
 *   relpath - The relative path of the file to remove.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_unlink(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct mfs_sb_s *sb;
  int ret;

  sb = mnemofs_mountpt_to_sb(mountpt);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  ret = mfs_dir_unlink(sb, relpath);
  if (ret < 0)
    {
      ferr("mfs_dir_unlink failed: %d\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_mkdir
 *
 * Description:
 * VFS mkdir wrapper for directories in mnemofs.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode.
 *   relpath - The relative path of the directory to create.
 *   mode - The requested directory mode.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_mkdir(FAR struct inode *mountpt, FAR const char *relpath,
                         mode_t mode)
{
  FAR struct mfs_sb_s *sb;
  int ret;

  sb = mnemofs_mountpt_to_sb(mountpt);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  ret = mfs_dir_mkdir(sb, relpath, mode);
  if (ret < 0)
    {
      ferr("mfs_dir_mkdir failed: %d\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_rmdir
 *
 * Description:
 * VFS rmdir wrapper for directories in mnemofs.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode.
 *   relpath - The relative path of the directory to remove.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_rmdir(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct mfs_sb_s *sb;
  int ret;

  sb = mnemofs_mountpt_to_sb(mountpt);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  ret = mfs_dir_rmdir(sb, relpath);
  if (ret < 0)
    {
      ferr("mfs_dir_rmdir failed: %d\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_rename
 *
 * Description:
 * VFS rename wrapper for file system objects in mnemofs.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode.
 *   oldrelpath - The current relative path.
 *   newrelpath - The replacement relative path.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_rename(FAR struct inode *mountpt,
                          FAR const char *oldrelpath,
                          FAR const char *newrelpath)
{
  FAR struct mfs_sb_s *sb;
  int ret;

  sb = mnemofs_mountpt_to_sb(mountpt);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  ret = mfs_dir_rename(sb, oldrelpath, newrelpath);
  if (ret < 0)
    {
      ferr("mfs_dir_rename failed: %d\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Name: mnemofs_stat
 *
 * Description:
 * VFS stat wrapper for file system objects in mnemofs.
 *
 * Input Parameters:
 *   mountpt - The mountpoint inode.
 *   relpath - The relative path to inspect.
 *   buf - The location to receive the translated stat data.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mnemofs_stat(FAR struct inode *mountpt, FAR const char *relpath,
                        FAR struct stat *buf)
{
  FAR struct mfs_sb_s *sb;
  int ret;

  sb = mnemofs_mountpt_to_sb(mountpt);
  ret = mnemofs_lock(sb);
  if (ret < 0)
    {
      ferr("mnemofs_lock failed: %d\n", ret);
      return ret;
    }

  ret = mfs_dir_stat(sb, relpath, buf);
  if (ret < 0)
    {
      ferr("mfs_dir_stat failed: %d\n", ret);
    }

  mnemofs_unlock(sb);
  return ret;
}

/****************************************************************************
 * Public Data
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
