/****************************************************************************
 * fs/mnemofs/mnemofs_alloc.c
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

#include <nuttx/kmalloc.h>

#include <errno.h>
#include <stdlib.h>
#include <string.h>

#include "fs_heap.h"
#include "mnemofs.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool mfs_bitmap_get(FAR const uint8_t *bitmap, mfs_t page);
static void mfs_bitmap_set(FAR uint8_t *bitmap, mfs_t page);
static void mfs_bitmap_clear(FAR uint8_t *bitmap, mfs_t page);
static mfs_t mfs_first_data_block(FAR const struct mfs_sb_s *sb);
static mfs_t mfs_first_data_page(FAR const struct mfs_sb_s *sb);
static mfs_t mfs_next_data_block(FAR const struct mfs_sb_s *sb, mfs_t block);
static mfs_t mfs_next_data_page(FAR const struct mfs_sb_s *sb, mfs_t page);
static int mfs_alloc_validate_block(FAR const struct mfs_sb_s *sb,
                                    mfs_t block);
static int mfs_alloc_validate_page(FAR const struct mfs_sb_s *sb,
                                   mfs_t page);
static void mfs_alloc_mark_page_used(FAR struct mfs_sb_s *sb, mfs_t page);
static void mfs_alloc_mark_page_free(FAR struct mfs_sb_s *sb, mfs_t page);
static void mfs_alloc_mark_page_deleted(FAR struct mfs_sb_s *sb, mfs_t page);
static void mfs_alloc_mark_block_used(FAR struct mfs_sb_s *sb, mfs_t block);
static void mfs_alloc_mark_block_free(FAR struct mfs_sb_s *sb, mfs_t block);
static void mfs_alloc_mark_block_deleted(FAR struct mfs_sb_s *sb,
                                         mfs_t block);
static bool mfs_alloc_block_is_free(FAR const struct mfs_sb_s *sb,
                                    mfs_t block);
static bool mfs_alloc_block_is_deleted(FAR const struct mfs_sb_s *sb,
                                       mfs_t block);
static int mfs_alloc_reclaim_block(FAR struct mfs_sb_s *sb, mfs_t block);
static int mfs_alloc_page_erased(FAR struct mfs_sb_s *sb, mfs_t page,
                                 FAR bool *erased);
static FAR char *mfs_alloc_child_path(FAR const char *parent,
                                      FAR const char *name);
static int mfs_alloc_reserve_live_tree(FAR struct mfs_sb_s *sb,
                                       FAR const char *relpath);
static int mfs_alloc_choose_start_page(FAR struct mfs_sb_s *sb);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mfs_bitmap_get
 *
 * Description:
 * Return the state of one bit in a page bitmap.
 *
 * Input Parameters:
 *   bitmap - The bitmap to inspect.
 *   page - The page number whose bit is requested.
 *
 * Returned Value:
 * true is returned if the bit is set. false is returned otherwise.
 *
 ****************************************************************************/

static bool mfs_bitmap_get(FAR const uint8_t *bitmap, mfs_t page)
{
  return (bitmap[page >> 3] & (1u << (page & 7))) != 0;
}

/****************************************************************************
 * Name: mfs_bitmap_set
 *
 * Description:
 * Set one bit in a page bitmap.
 *
 * Input Parameters:
 *   bitmap - The bitmap to modify.
 *   page - The page number whose bit should be set.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mfs_bitmap_set(FAR uint8_t *bitmap, mfs_t page)
{
  bitmap[page >> 3] |= (1u << (page & 7));
}

/****************************************************************************
 * Name: mfs_bitmap_clear
 *
 * Description:
 * Clear one bit in a page bitmap.
 *
 * Input Parameters:
 *   bitmap - The bitmap to modify.
 *   page - The page number whose bit should be cleared.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mfs_bitmap_clear(FAR uint8_t *bitmap, mfs_t page)
{
  bitmap[page >> 3] &= ~(1u << (page & 7));
}

/****************************************************************************
 * Name: mfs_first_data_block
 *
 * Description:
 * Return the first block that may hold live file system data.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *
 * Returned Value:
 * The first usable data block is returned on success.
 * MFS_LOCATION_INVALID is returned if the geometry is not usable.
 *
 ****************************************************************************/

static mfs_t mfs_first_data_block(FAR const struct mfs_sb_s *sb)
{
  if (sb == NULL || sb->superblock + 1 >= MFS_BLOCK_COUNT(sb))
    {
      return MFS_LOCATION_INVALID;
    }

  return sb->superblock + 1;
}

/****************************************************************************
 * Name: mfs_first_data_page
 *
 * Description:
 * Return the first page that may hold live file system data.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *
 * Returned Value:
 * The first usable data page is returned on success.
 * MFS_LOCATION_INVALID is returned if no usable data block exists.
 *
 ****************************************************************************/

static mfs_t mfs_first_data_page(FAR const struct mfs_sb_s *sb)
{
  mfs_t block;

  block = mfs_first_data_block(sb);
  if (block == MFS_LOCATION_INVALID)
    {
      return MFS_LOCATION_INVALID;
    }

  return MFS_BLOCK_TO_PAGE(sb, block);
}

/****************************************************************************
 * Name: mfs_next_data_block
 *
 * Description:
 * Return the next usable data block in circular allocation order.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   block - The current block in the scan.
 *
 * Returned Value:
 * The next data block is returned on success.
 * MFS_LOCATION_INVALID is returned if no usable data range exists.
 *
 ****************************************************************************/

static mfs_t mfs_next_data_block(FAR const struct mfs_sb_s *sb, mfs_t block)
{
  mfs_t firstblock;

  firstblock = mfs_first_data_block(sb);
  if (firstblock == MFS_LOCATION_INVALID)
    {
      return MFS_LOCATION_INVALID;
    }

  block++;
  if (block >= MFS_BLOCK_COUNT(sb))
    {
      return firstblock;
    }

  if (block < firstblock)
    {
      return firstblock;
    }

  return block;
}

/****************************************************************************
 * Name: mfs_next_data_page
 *
 * Description:
 * Return the next usable data page in circular allocation order.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   page - The current page in the scan.
 *
 * Returned Value:
 * The next data page is returned on success.
 * MFS_LOCATION_INVALID is returned if no usable data range exists.
 *
 ****************************************************************************/

static mfs_t mfs_next_data_page(FAR const struct mfs_sb_s *sb, mfs_t page)
{
  mfs_t firstpage;

  firstpage = mfs_first_data_page(sb);
  if (firstpage == MFS_LOCATION_INVALID)
    {
      return MFS_LOCATION_INVALID;
    }

  if (page == MFS_LOCATION_INVALID)
    {
      return firstpage;
    }

  page++;
  if (page >= MFS_PAGE_COUNT(sb))
    {
      return firstpage;
    }

  if (mfs_page_to_block(sb, page) < mfs_first_data_block(sb))
    {
      return firstpage;
    }

  return page;
}

/****************************************************************************
 * Name: mfs_alloc_validate_block
 *
 * Description:
 * Validate that block lies inside the allocator-managed data range.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   block - The block number to validate.
 *
 * Returned Value:
 * Zero (OK) is returned if block is valid. -EINVAL is returned
 * otherwise.
 *
 ****************************************************************************/

static int mfs_alloc_validate_block(FAR const struct mfs_sb_s *sb,
                                    mfs_t block)
{
  mfs_t firstblock;

  if (sb == NULL || sb->freepages == NULL || sb->delpages == NULL)
    {
      ferr("allocator not initialized\n");
      return -EINVAL;
    }

  firstblock = mfs_first_data_block(sb);
  if (firstblock == MFS_LOCATION_INVALID)
    {
      ferr("no usable data block\n");
      return -EINVAL;
    }

  if (block == MFS_LOCATION_INVALID || block >= MFS_BLOCK_COUNT(sb) ||
      block < firstblock)
    {
      ferr("invalid block\n");
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: mfs_alloc_validate_page
 *
 * Description:
 * Validate that page lies inside the allocator-managed data range.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   page - The page number to validate.
 *
 * Returned Value:
 * Zero (OK) is returned if page is valid. -EINVAL is returned
 * otherwise.
 *
 ****************************************************************************/

static int mfs_alloc_validate_page(FAR const struct mfs_sb_s *sb, mfs_t page)
{
  mfs_t block;

  if (sb == NULL || sb->freepages == NULL || sb->delpages == NULL)
    {
      ferr("allocator not initialized\n");
      return -EINVAL;
    }

  if (page == MFS_LOCATION_INVALID || page >= MFS_PAGE_COUNT(sb))
    {
      ferr("invalid page\n");
      return -EINVAL;
    }

  block = mfs_page_to_block(sb, page);
  if (block == MFS_LOCATION_INVALID)
    {
      ferr("page has no valid block\n");
      return -EINVAL;
    }

  return mfs_alloc_validate_block(sb, block);
}

/****************************************************************************
 * Name: mfs_alloc_mark_page_used
 *
 * Description:
 * Mark one page as allocated in the allocator bitmaps.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   page - The page number to mark.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mfs_alloc_mark_page_used(FAR struct mfs_sb_s *sb, mfs_t page)
{
  mfs_bitmap_clear(sb->freepages, page);
  mfs_bitmap_clear(sb->delpages, page);
}

/****************************************************************************
 * Name: mfs_alloc_mark_page_free
 *
 * Description:
 * Mark one page as free in the allocator bitmaps.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   page - The page number to mark.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mfs_alloc_mark_page_free(FAR struct mfs_sb_s *sb, mfs_t page)
{
  mfs_bitmap_set(sb->freepages, page);
  mfs_bitmap_clear(sb->delpages, page);
}

/****************************************************************************
 * Name: mfs_alloc_mark_page_deleted
 *
 * Description:
 * Mark one page as pending deletion in the allocator bitmaps.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   page - The page number to mark.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mfs_alloc_mark_page_deleted(FAR struct mfs_sb_s *sb, mfs_t page)
{
  mfs_bitmap_clear(sb->freepages, page);
  mfs_bitmap_set(sb->delpages, page);
}

/****************************************************************************
 * Name: mfs_alloc_mark_block_used
 *
 * Description:
 * Mark every page in one block as allocated.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   block - The block number to mark.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mfs_alloc_mark_block_used(FAR struct mfs_sb_s *sb, mfs_t block)
{
  mfs_t page;
  mfs_t endpage;

  page    = MFS_BLOCK_TO_PAGE(sb, block);
  endpage = page + MFS_PAGES_PER_BLOCK(sb);
  for (; page < endpage; page++)
    {
      mfs_alloc_mark_page_used(sb, page);
    }
}

/****************************************************************************
 * Name: mfs_alloc_mark_block_free
 *
 * Description:
 * Mark every page in one block as free.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   block - The block number to mark.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mfs_alloc_mark_block_free(FAR struct mfs_sb_s *sb, mfs_t block)
{
  mfs_t page;
  mfs_t endpage;

  page    = MFS_BLOCK_TO_PAGE(sb, block);
  endpage = page + MFS_PAGES_PER_BLOCK(sb);
  for (; page < endpage; page++)
    {
      mfs_alloc_mark_page_free(sb, page);
    }
}

/****************************************************************************
 * Name: mfs_alloc_mark_block_deleted
 *
 * Description:
 * Mark every page in one block as pending deletion.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   block - The block number to mark.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mfs_alloc_mark_block_deleted(FAR struct mfs_sb_s *sb,
                                         mfs_t block)
{
  mfs_t page;
  mfs_t endpage;

  page    = MFS_BLOCK_TO_PAGE(sb, block);
  endpage = page + MFS_PAGES_PER_BLOCK(sb);
  for (; page < endpage; page++)
    {
      mfs_alloc_mark_page_deleted(sb, page);
    }
}

/****************************************************************************
 * Name: mfs_alloc_block_is_free
 *
 * Description:
 * Check whether every page in block is currently marked free.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   block - The block number to inspect.
 *
 * Returned Value:
 * true is returned if the whole block is free. false is returned
 * otherwise.
 *
 ****************************************************************************/

static bool mfs_alloc_block_is_free(FAR const struct mfs_sb_s *sb,
                                    mfs_t block)
{
  mfs_t page;
  mfs_t endpage;

  page    = MFS_BLOCK_TO_PAGE(sb, block);
  endpage = page + MFS_PAGES_PER_BLOCK(sb);
  for (; page < endpage; page++)
    {
      if (!mfs_bitmap_get(sb->freepages, page))
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Name: mfs_alloc_block_is_deleted
 *
 * Description:
 * Check whether every page in block is currently marked deleted.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   block - The block number to inspect.
 *
 * Returned Value:
 * true is returned if the whole block is pending deletion. false is
 * returned otherwise.
 *
 ****************************************************************************/

static bool mfs_alloc_block_is_deleted(FAR const struct mfs_sb_s *sb,
                                       mfs_t block)
{
  mfs_t page;
  mfs_t endpage;

  page    = MFS_BLOCK_TO_PAGE(sb, block);
  endpage = page + MFS_PAGES_PER_BLOCK(sb);
  for (; page < endpage; page++)
    {
      if (!mfs_bitmap_get(sb->delpages, page))
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Name: mfs_alloc_reclaim_block
 *
 * Description:
 * Erase one block if every page in it has been marked deleted, then
 * return it to the free bitmap.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   block - The block number to reclaim.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * validation or erase failure.
 *
 ****************************************************************************/

static int mfs_alloc_reclaim_block(FAR struct mfs_sb_s *sb, mfs_t block)
{
  int ret;

  ret = mfs_alloc_validate_block(sb, block);
  if (ret < 0)
    {
      ferr("mfs_alloc_validate_block failed: %d\n", ret);
      return ret;
    }

  if (!mfs_alloc_block_is_deleted(sb, block))
    {
      return OK;
    }

  ret = mfs_erase_blocks(sb, block, 1);
  if (ret < 0)
    {
      ferr("mfs_erase_blocks failed: %d\n", ret);
      return ret;
    }

  if (ret != 1)
    {
      ferr("short erase: %d\n", ret);
      return -EIO;
    }

  mfs_alloc_mark_block_free(sb, block);
  return OK;
}

/****************************************************************************
 * Name: mfs_alloc_page_erased
 *
 * Description:
 * Read a page and report whether it is still in the erased state.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   page - The page number to inspect.
 *   erased - The location to receive the erased-state result.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

static int mfs_alloc_page_erased(FAR struct mfs_sb_s *sb, mfs_t page,
                                 FAR bool *erased)
{
  ssize_t nread;
  size_t i;

  if (sb == NULL || erased == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  nread = mfs_read_page(sb, page, sb->rwbuf);
  if (nread < 0)
    {
      ferr("mfs_read_page failed: %zd\n", nread);
      return nread;
    }

  if (nread != 1)
    {
      ferr("short read: %zd\n", nread);
      return -EIO;
    }

  * erased = true;
  for (i = 0; i < MFS_PAGE_SIZE(sb); i++)
    {
      if (sb->rwbuf[i] != sb->erasestate)
        {
          * erased = false;
          break;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: mfs_alloc_child_path
 *
 * Description:
 * Allocate `parent/name` as one new relative path string.
 *
 * Input Parameters:
 *   parent - The parent path prefix.
 *   name - The child name to append.
 *
 * Returned Value:
 * A newly allocated path string is returned on success. NULL is returned
 * on invalid input or allocation failure.
 *
 ****************************************************************************/

static FAR char *mfs_alloc_child_path(FAR const char *parent,
                                      FAR const char *name)
{
  FAR char *path;
  size_t parentlen;
  size_t namelen;

  if (parent == NULL || name == NULL)
    {
      ferr("invalid args\n");
      return NULL;
    }

  parentlen = strlen(parent);
  namelen   = strlen(name);
  path      = kmm_malloc(parentlen + namelen + 2);
  if (path == NULL)
    {
      ferr("kmm_malloc failed\n");
      return NULL;
    }

  if (parentlen == 0)
    {
      memcpy(path, name, namelen + 1);
      return path;
    }

  memcpy(path, parent, parentlen);
  path[parentlen] = '/';
  memcpy(path + parentlen + 1, name, namelen + 1);
  return path;
}

/****************************************************************************
 * Name: mfs_alloc_reserve_live_tree
 *
 * Description:
 * Walk the live directory tree rooted at relpath and mark each reachable
 * page or block as allocated in the allocator bitmaps.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   relpath - The relative path that roots the subtree to reserve.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * traversal, allocation, or directory-read failure.
 *
 ****************************************************************************/

static int mfs_alloc_reserve_live_tree(FAR struct mfs_sb_s *sb,
                                       FAR const char *relpath)
{
  struct mfs_direntry_s direntry;
  struct mfs_dirloc_s dirloc;
  FAR struct fs_dirent_s *dir = NULL;
  FAR char *childpath = NULL;
  FAR char *dirpath = NULL;
  struct dirent entry;
  mfs_t location;
  int ret;
  int closeret;

  ret = mfs_dirent_traverse(sb, relpath, &location, &direntry, &dirloc);
  if (ret < 0)
    {
      ferr("mfs_dirent_traverse failed: %d\n", ret);
      return ret;
    }

  if (!S_ISDIR(direntry.mode))
    {
      mfs_alloc_mark_page_used(sb, location);
      return OK;
    }

  mfs_alloc_mark_block_used(sb, location);
  ret = mfs_dir_opendir(sb, relpath, &dir);
  if (ret < 0)
    {
      ferr("mfs_dir_opendir failed: %d\n", ret);
      return ret;
    }

  for (; ; )
    {
      ret = mfs_dir_readdir(sb, dir, &entry);
      if (ret == -ENOENT)
        {
          ret = OK;
          goto errout_with_dir;
        }

      if (ret < 0)
        {
          ferr("mfs_dir_readdir failed: %d\n", ret);
          goto errout_with_dir;
        }

      childpath = mfs_alloc_child_path(relpath, entry.d_name);
      if (childpath == NULL)
        {
          ferr("mfs_alloc_child_path failed\n");
          ret = -ENOMEM;
          goto errout_with_dir;
        }

      ret = mfs_alloc_reserve_live_tree(sb, childpath);
      kmm_free(childpath);
      childpath = NULL;
      if (ret < 0)
        {
          ferr("reserve child failed: %d\n", ret);
          goto errout_with_dir;
        }
    }

errout_with_dir:
  kmm_free(childpath);
  if (dir != NULL)
    {
      dirpath = dir->fd_path;
      closeret = mfs_dir_closedir(sb, dir);
      fs_heap_free(dirpath);
      if (ret >= 0)
        {
          ret = closeret;
        }
    }

  if (ret < 0)
    {
      ferr("failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: mfs_alloc_choose_start_page
 *
 * Description:
 * Choose a random starting block for future allocation scans. The first
 * usable data block found from that random point becomes the next
 * allocation position.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *
 * Returned Value:
 * Zero (OK) is returned on success. -ENOSPC is returned if no usable
 * data block exists. A negated errno value is returned on other
 * failures.
 *
 ****************************************************************************/

static int mfs_alloc_choose_start_page(FAR struct mfs_sb_s *sb)
{
  mfs_t firstblock;
  mfs_t nblocks;
  mfs_t start;
  mfs_t block;
  mfs_t i;
  int bad;

  if (sb == NULL)
    {
      ferr("invalid sb\n");
      return -EINVAL;
    }

  firstblock = mfs_first_data_block(sb);
  if (firstblock == MFS_LOCATION_INVALID)
    {
      ferr("no usable data block\n");
      return -ENOSPC;
    }

  nblocks = MFS_BLOCK_COUNT(sb) - firstblock;
  start   = firstblock + (arc4random() % nblocks);

  for (i = 0, block = start; i < nblocks; i++)
    {
      bad = mfs_is_bad_block(sb, block);
      if (bad < 0)
        {
          ferr("mfs_is_bad_block failed: %d\n", bad);
          return bad;
        }

      if (bad == 0)
        {
          sb->nextpage = MFS_BLOCK_TO_PAGE(sb, block);
          return OK;
        }

      block = mfs_next_data_block(sb, block);
    }

  ferr("no usable start block\n");
  return -ENOSPC;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mfs_alloc_init
 *
 * Description:
 * Initialize allocator state for the mounted volume. This builds the
 * free and pending-delete bitmaps, reserves the currently live file
 * system tree, and selects a random starting point for future scans.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *
 * Returned Value:
 * Zero (OK) is returned on success. -ENOMEM or -ENOSPC may be returned
 * for resource exhaustion. A negated errno value is returned on other
 * failures.
 *
 ****************************************************************************/

int mfs_alloc_init(FAR struct mfs_sb_s *sb)
{
  mfs_t firstblock;
  mfs_t block;
  mfs_t page;
  mfs_t endpage;
  bool erased;
  int ret;
  int bad;

  if (sb == NULL || sb->rwbuf == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  firstblock = mfs_first_data_block(sb);
  if (firstblock == MFS_LOCATION_INVALID)
    {
      ferr("no usable data block\n");
      return -ENOSPC;
    }

  sb->bitmapsize = (MFS_PAGE_COUNT(sb) + 7) / 8;
  sb->nextpage   = MFS_LOCATION_INVALID;
  sb->freepages  = kmm_zalloc(sb->bitmapsize);
  if (sb->freepages == NULL)
    {
      ferr("free bitmap allocation failed\n");
      ret = -ENOMEM;
      goto errout;
    }

  sb->delpages = kmm_zalloc(sb->bitmapsize);
  if (sb->delpages == NULL)
    {
      ferr("deleted bitmap allocation failed\n");
      ret = -ENOMEM;
      goto errout;
    }

  for (block = 0; block < MFS_BLOCK_COUNT(sb); block++)
    {
      bad = mfs_is_bad_block(sb, block);
      if (bad < 0)
        {
          ferr("mfs_is_bad_block failed: %d\n", bad);
          ret = bad;
          goto errout;
        }

      if (bad != 0 || block < firstblock)
        {
          mfs_alloc_mark_block_used(sb, block);
          continue;
        }

      page    = MFS_BLOCK_TO_PAGE(sb, block);
      endpage = page + MFS_PAGES_PER_BLOCK(sb);
      for (; page < endpage; page++)
        {
          ret = mfs_alloc_page_erased(sb, page, &erased);
          if (ret < 0)
            {
              ferr("mfs_alloc_page_erased failed: %d\n", ret);
              goto errout;
            }

          if (erased)
            {
              mfs_alloc_mark_page_free(sb, page);
            }
        }
    }

  ret = mfs_alloc_reserve_live_tree(sb, "");
  if (ret < 0)
    {
      ferr("mfs_alloc_reserve_live_tree failed: %d\n", ret);
      goto errout;
    }

  ret = mfs_alloc_choose_start_page(sb);
  if (ret < 0)
    {
      ferr("mfs_alloc_choose_start_page failed: %d\n", ret);
      goto errout;
    }

  return OK;

errout:
  ferr("failed: %d\n", ret);
  mfs_alloc_uninit(sb);
  return ret;
}

/****************************************************************************
 * Name: mfs_alloc_uninit
 *
 * Description:
 * Release allocator-owned bitmap state.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

void mfs_alloc_uninit(FAR struct mfs_sb_s *sb)
{
  if (sb == NULL)
    {
      return;
    }

  kmm_free(sb->freepages);
  kmm_free(sb->delpages);
  sb->freepages  = NULL;
  sb->delpages   = NULL;
  sb->bitmapsize = 0;
  sb->nextpage   = MFS_LOCATION_INVALID;
}

/****************************************************************************
 * Name: mfs_alloc_page
 *
 * Description:
 * Allocate one page using a circular scan that begins at the current
 * allocator position.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   page - The location to receive the allocated page number.
 *
 * Returned Value:
 * Zero (OK) is returned on success. -ENOSPC is returned if no free page
 * is available. A negated errno value is returned on other failures.
 *
 ****************************************************************************/

int mfs_alloc_page(FAR struct mfs_sb_s *sb, FAR mfs_t *page)
{
  mfs_t firstblock;
  mfs_t candidate;
  mfs_t npages;
  mfs_t i;

  if (sb == NULL || page == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  * page = MFS_LOCATION_INVALID;
  if (sb->freepages == NULL || sb->delpages == NULL ||
      sb->nextpage == MFS_LOCATION_INVALID)
    {
      ferr("allocator not initialized\n");
      return -EINVAL;
    }

  firstblock = mfs_first_data_block(sb);
  if (firstblock == MFS_LOCATION_INVALID)
    {
      ferr("no usable data block\n");
      return -ENOSPC;
    }

  npages = (MFS_BLOCK_COUNT(sb) - firstblock) *
           MFS_PAGES_PER_BLOCK(sb);
  candidate = sb->nextpage;

  for (i = 0; i < npages; i++)
    {
      if (mfs_bitmap_get(sb->freepages, candidate))
        {
          mfs_alloc_mark_page_used(sb, candidate);
          * page = candidate;
          sb->nextpage = mfs_next_data_page(sb, candidate);
          return OK;
        }

      candidate = mfs_next_data_page(sb, candidate);
    }

  ferr("no free page\n");
  return -ENOSPC;
}

/****************************************************************************
 * Name: mfs_alloc_block
 *
 * Description:
 * Allocate one whole directory block. The scan follows the allocator's
 * circular position, but accepts only blocks whose every page is free.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   block - The location to receive the allocated block number.
 *
 * Returned Value:
 * Zero (OK) is returned on success. -ENOSPC is returned if no free
 * block is available. A negated errno value is returned on other
 * failures.
 *
 ****************************************************************************/

int mfs_alloc_block(FAR struct mfs_sb_s *sb, FAR mfs_t *block)
{
  mfs_t firstblock;
  mfs_t candidate;
  mfs_t nblocks;
  mfs_t i;

  if (sb == NULL || block == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  * block = MFS_LOCATION_INVALID;
  if (sb->freepages == NULL || sb->delpages == NULL ||
      sb->nextpage == MFS_LOCATION_INVALID)
    {
      ferr("allocator not initialized\n");
      return -EINVAL;
    }

  firstblock = mfs_first_data_block(sb);
  if (firstblock == MFS_LOCATION_INVALID)
    {
      ferr("no usable data block\n");
      return -ENOSPC;
    }

  candidate = mfs_page_to_block(sb, sb->nextpage);
  if (candidate == MFS_LOCATION_INVALID)
    {
      ferr("invalid next page\n");
      return -EINVAL;
    }

  if (sb->nextpage != MFS_BLOCK_TO_PAGE(sb, candidate))
    {
      candidate = mfs_next_data_block(sb, candidate);
    }

  nblocks = MFS_BLOCK_COUNT(sb) - firstblock;
  for (i = 0; i < nblocks; i++)
    {
      if (mfs_alloc_block_is_free(sb, candidate))
        {
          mfs_alloc_mark_block_used(sb, candidate);
          * block = candidate;
          sb->nextpage = MFS_BLOCK_TO_PAGE(sb,
                                           mfs_next_data_block(sb,
                                                               candidate));
          return OK;
        }

      candidate = mfs_next_data_block(sb, candidate);
    }

  ferr("no free block\n");
  return -ENOSPC;
}

/****************************************************************************
 * Name: mfs_release_page
 *
 * Description:
 * Return a page to the free bitmap immediately. This is intended for
 * rollback of allocations that were not yet committed into directory
 * state.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   page - The page number to release.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_release_page(FAR struct mfs_sb_s *sb, mfs_t page)
{
  int ret;

  ret = mfs_alloc_validate_page(sb, page);
  if (ret < 0)
    {
      ferr("mfs_alloc_validate_page failed: %d\n", ret);
      return ret;
    }

  mfs_rwbuf_discard_page(sb, page);
  mfs_alloc_mark_page_free(sb, page);
  return OK;
}

/****************************************************************************
 * Name: mfs_report_page_deleted
 *
 * Description:
 * Mark one committed page as obsolete. If this makes every page in the
 * containing block obsolete, then the block is erased immediately and
 * returned to the free bitmap.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   page - The page number that became obsolete.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_report_page_deleted(FAR struct mfs_sb_s *sb, mfs_t page)
{
  mfs_t block;
  int ret;

  ret = mfs_alloc_validate_page(sb, page);
  if (ret < 0)
    {
      ferr("mfs_alloc_validate_page failed: %d\n", ret);
      return ret;
    }

  if (mfs_bitmap_get(sb->freepages, page))
    {
      ferr("page already free\n");
      return -EINVAL;
    }

  mfs_rwbuf_discard_page(sb, page);
  block = mfs_page_to_block(sb, page);
  mfs_alloc_mark_page_deleted(sb, page);
  ret = mfs_alloc_reclaim_block(sb, block);
  if (ret < 0)
    {
      ferr("mfs_alloc_reclaim_block failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: mfs_report_block_deleted
 *
 * Description:
 * Mark every page in a committed block as obsolete and reclaim the block
 * immediately once all pages in it are scheduled for deletion.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   block - The block number that became obsolete.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_report_block_deleted(FAR struct mfs_sb_s *sb, mfs_t block)
{
  int ret;

  ret = mfs_alloc_validate_block(sb, block);
  if (ret < 0)
    {
      ferr("mfs_alloc_validate_block failed: %d\n", ret);
      return ret;
    }

  mfs_rwbuf_discard_block(sb, block);
  mfs_alloc_mark_block_deleted(sb, block);
  ret = mfs_alloc_reclaim_block(sb, block);
  if (ret < 0)
    {
      ferr("mfs_alloc_reclaim_block failed: %d\n", ret);
    }

  return ret;
}
