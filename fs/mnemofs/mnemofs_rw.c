/****************************************************************************
 * fs/mnemofs/mnemofs_rw.c
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

#include <errno.h>
#include <string.h>

#include "mnemofs.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mfs_rwbuf_check_page
 *
 * Description:
 * Verify that page belongs to a good NAND block before it is accessed
 * through the shared read/write buffer.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   page - The page number to validate.
 *
 * Returned Value:
 * Zero (OK) is returned if page is valid and its block is not marked bad.
 * A negated errno value is returned on invalid input or if the page maps
 * to a bad block.
 *
 ****************************************************************************/

static int mfs_rwbuf_check_page(FAR struct mfs_sb_s *sb, mfs_t page)
{
  mfs_t block;
  int ret;

  if (sb == NULL || sb->mtd == NULL)
    {
      return -EINVAL;
    }

  if (page >= MFS_PAGE_COUNT(sb))
    {
      return -EINVAL;
    }

  block = page / MFS_PAGES_PER_BLOCK(sb);

  ret = mfs_is_bad_block(sb, block);
  if (ret < 0)
    {
      return ret;
    }

  return ret == 0 ? OK : -EIO;
}

/****************************************************************************
 * Name: mfs_rwbuf_load
 *
 * Description:
 * Load one page into the shared read/write buffer, syncing any dirty
 * contents already cached there first.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   page - The page number to load.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned if
 * validation, syncing, or the underlying read fails.
 *
 ****************************************************************************/

static int mfs_rwbuf_load(FAR struct mfs_sb_s *sb, mfs_t page)
{
  ssize_t nread;
  int ret;

  if (sb == NULL || sb->rwbuf == NULL)
    {
      return -EINVAL;
    }

  if (sb->rwvalid && sb->rwpage == page)
    {
      return OK;
    }

  ret = mfs_rwbuf_sync(sb);
  if (ret < 0)
    {
      return ret;
    }

  ret = mfs_rwbuf_check_page(sb, page);
  if (ret < 0)
    {
      return ret;
    }

  nread = MTD_BREAD(sb->mtd, page, 1, sb->rwbuf);
  if (nread < 0)
    {
      return (int)nread;
    }

  if (nread != 1)
    {
      return -EIO;
    }

  sb->rwpage = page;
  sb->rwvalid = true;
  sb->rwdirty = false;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mfs_is_bad_block
 *
 * Description:
 * Query the backing MTD device to learn whether blk is marked bad.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   blk - The block number to test.
 *
 * Returned Value:
 * Zero (OK) is returned if blk is usable. A positive non-zero value may
 * be returned if the MTD reports blk as bad. A negated errno value is
 * returned on invalid input or device failure.
 *
 ****************************************************************************/

int mfs_is_bad_block(FAR const struct mfs_sb_s *sb, mfs_t blk)
{
  int ret;

  if (sb == NULL || sb->mtd == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (blk >= MFS_BLOCK_COUNT(sb))
    {
      ferr("invalid block\n");
      return -EINVAL;
    }

  ret = MTD_ISBAD(sb->mtd, blk);
  if (ret < 0 && ret != -ENOSYS)
    {
      ferr("MTD_ISBAD failed: %d\n", ret);
    }

  return ret == -ENOSYS ? 0 : ret;
}

/****************************************************************************
 * Name: mfs_rwbuf_sync
 *
 * Description:
 * Flush dirty contents from the shared read/write buffer to its cached
 * page.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned if
 * the buffered page cannot be written back.
 *
 ****************************************************************************/

int mfs_rwbuf_sync(FAR struct mfs_sb_s *sb)
{
  ssize_t nwritten;

  if (sb == NULL)
    {
      ferr("invalid sb\n");
      return -EINVAL;
    }

  if (sb->rwbuf == NULL || !sb->rwvalid || !sb->rwdirty)
    {
      return OK;
    }

  nwritten = MTD_BWRITE(sb->mtd, sb->rwpage, 1, sb->rwbuf);
  if (nwritten < 0)
    {
      ferr("MTD_BWRITE failed: %zd\n", nwritten);
      return (int)nwritten;
    }

  if (nwritten != 1)
    {
      ferr("short write: %zd\n", nwritten);
      return -EIO;
    }

  sb->rwdirty = false;
  return OK;
}

/****************************************************************************
 * Name: mfs_rwbuf_invalidate
 *
 * Description:
 * Drop any cached page association from the shared read/write buffer.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

void mfs_rwbuf_invalidate(FAR struct mfs_sb_s *sb)
{
  if (sb == NULL)
    {
      return;
    }

  sb->rwpage = MFS_LOCATION_INVALID;
  sb->rwvalid = false;
  sb->rwdirty = false;
}

/****************************************************************************
 * Name: mfs_rwbuf_prepare_write
 *
 * Description:
 * Sync any dirty buffered page and then invalidate the cache so the next
 * write starts from a clean state.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned if
 * syncing the buffered page fails.
 *
 ****************************************************************************/

int mfs_rwbuf_prepare_write(FAR struct mfs_sb_s *sb)
{
  int ret;

  if (sb == NULL)
    {
      ferr("invalid sb\n");
      return -EINVAL;
    }

  ret = mfs_rwbuf_sync(sb);
  if (ret < 0)
    {
      ferr("mfs_rwbuf_sync failed: %d\n", ret);
      return ret;
    }

  mfs_rwbuf_invalidate(sb);
  return OK;
}

/****************************************************************************
 * Name: mfs_rwbuf_discard_page
 *
 * Description:
 * Invalidate the shared read/write buffer if it currently caches page.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   page - The page whose cached state should be discarded.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

void mfs_rwbuf_discard_page(FAR struct mfs_sb_s *sb, mfs_t page)
{
  if (sb == NULL || !sb->rwvalid)
    {
      return;
    }

  if (sb->rwpage == page)
    {
      mfs_rwbuf_invalidate(sb);
    }
}

/****************************************************************************
 * Name: mfs_rwbuf_discard_block
 *
 * Description:
 * Invalidate the shared read/write buffer if it currently caches any page
 * that belongs to block.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   block - The block whose cached state should be discarded.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

void mfs_rwbuf_discard_block(FAR struct mfs_sb_s *sb, mfs_t block)
{
  if (sb == NULL || block >= MFS_BLOCK_COUNT(sb) || !sb->rwvalid)
    {
      return;
    }

  if (sb->rwpage / MFS_PAGES_PER_BLOCK(sb) == block)
    {
      mfs_rwbuf_invalidate(sb);
    }
}

/****************************************************************************
 * Name: mfs_write_page
 *
 * Description:
 * Stage one page for writing through the shared read/write buffer. The
 * page becomes dirty and is written later by mfs_rwbuf_sync().
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   page - The page number to update.
 *   buffer - The page-sized data to stage.
 *
 * Returned Value:
 * One is returned on success. A negated errno value is returned if the
 * inputs are invalid, the page cannot be cached, or an earlier dirty page
 * cannot be synced.
 *
 ****************************************************************************/

ssize_t mfs_write_page(FAR struct mfs_sb_s *sb, mfs_t page,
                       FAR const uint8_t *buffer)
{
  int ret;

  if (sb == NULL || sb->mtd == NULL || sb->rwbuf == NULL || buffer == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (page >= MFS_PAGE_COUNT(sb))
    {
      ferr("invalid page\n");
      return -EINVAL;
    }

  if (!sb->rwvalid || sb->rwpage != page)
    {
      ret = mfs_rwbuf_sync(sb);
      if (ret < 0)
        {
          ferr("mfs_rwbuf_sync failed: %d\n", ret);
          return ret;
        }

      ret = mfs_rwbuf_check_page(sb, page);
      if (ret < 0)
        {
          ferr("mfs_rwbuf_check_page failed: %d\n", ret);
          return ret;
        }

      sb->rwpage = page;
      sb->rwvalid = true;
      sb->rwdirty = false;
    }

  if (buffer != sb->rwbuf)
    {
      memcpy(sb->rwbuf, buffer, MFS_PAGE_SIZE(sb));
    }

  sb->rwdirty = true;
  return 1;
}

/****************************************************************************
 * Name: mfs_read_page
 *
 * Description:
 * Load one page through the shared read/write buffer and copy it to the
 * caller if needed.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   page - The page number to read.
 *   buffer - The page-sized buffer that receives the data.
 *
 * Returned Value:
 * One is returned on success. A negated errno value is returned if the
 * inputs are invalid or the page cannot be loaded.
 *
 ****************************************************************************/

ssize_t mfs_read_page(FAR struct mfs_sb_s *sb, mfs_t page,
                      FAR uint8_t *buffer)
{
  int ret;

  if (sb == NULL || sb->mtd == NULL || sb->rwbuf == NULL || buffer == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (page >= MFS_PAGE_COUNT(sb))
    {
      ferr("invalid page\n");
      return -EINVAL;
    }

  ret = mfs_rwbuf_load(sb, page);
  if (ret < 0)
    {
      ferr("mfs_rwbuf_load failed: %d\n", ret);
      return ret;
    }

  if (buffer != sb->rwbuf)
    {
      memcpy(buffer, sb->rwbuf, MFS_PAGE_SIZE(sb));
    }

  return 1;
}

/****************************************************************************
 * Name: mfs_erase_blocks
 *
 * Description:
 * Erase a contiguous range of blocks and discard any cached page that
 * falls inside that range.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   startblk - The first block to erase.
 *   nblocks - The number of blocks to erase.
 *
 * Returned Value:
 * The underlying MTD erase result is returned on success. A negated
 * errno value is returned on invalid input.
 *
 ****************************************************************************/

int mfs_erase_blocks(FAR struct mfs_sb_s *sb, mfs_t startblk,
                     size_t nblocks)
{
  int ret;

  if (sb == NULL || sb->mtd == NULL || nblocks == 0)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (startblk >= MFS_BLOCK_COUNT(sb) ||
      nblocks > MFS_BLOCK_COUNT(sb) ||
      startblk > MFS_BLOCK_COUNT(sb) - nblocks)
    {
      ferr("invalid erase range\n");
      return -EINVAL;
    }

  if (sb->rwvalid)
    {
      mfs_t endblk = startblk + nblocks;
      mfs_t curblk = sb->rwpage / MFS_PAGES_PER_BLOCK(sb);

      if (curblk >= startblk && curblk < endblk)
        {
          mfs_rwbuf_invalidate(sb);
        }
    }

  ret = MTD_ERASE(sb->mtd, startblk, nblocks);
  if (ret < 0)
    {
      ferr("MTD_ERASE failed: %d\n", ret);
    }

  return ret;
}
