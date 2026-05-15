/****************************************************************************
 * fs/mnemofs/mnemofs_ctz.c
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
#include <nuttx/kmalloc.h>
#include <string.h>
#include <sys/endian.h>

#include "mnemofs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MFS_CTZ_PTR_SIZE sizeof(mfs_t)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int mfs_ctz_nptrs(mfs_t index);
static int mfs_ctz_read_ptr(FAR struct mfs_sb_s *sb, mfs_t page,
                            unsigned int pow, FAR uint8_t *scratch,
                            FAR mfs_t *ptrpage);
static void mfs_ctz_write_ptr(FAR struct mfs_sb_s *sb, FAR uint8_t *buffer,
                              unsigned int pow, mfs_t page);
static int mfs_ctz_follow(FAR struct mfs_sb_s *sb, mfs_t startidx,
                          mfs_t endidx, mfs_t startpage,
                          FAR uint8_t *scratch,
                          FAR mfs_t *endpage);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mfs_ctz_nptrs
 *
 * Description:
 * Return the number of backward pointers stored in CTZ unit index.
 *
 * Input Parameters:
 *   index - The CTZ unit index to inspect.
 *
 * Returned Value:
 * The number of pointers stored in the unit is returned.
 *
 ****************************************************************************/

static int mfs_ctz_nptrs(mfs_t index)
{
  int nptrs;

  if (index == 0)
    {
      return 0;
    }

  nptrs = 1;
  while ((index & 1) == 0)
    {
      nptrs++;
      index >>= 1;
    }

  return nptrs;
}

/****************************************************************************
 * Name: mfs_ctz_read_ptr
 *
 * Description:
 * Read one backward pointer from the pointer area of a CTZ unit.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   page - The CTZ unit page to read.
 *   pow - The pointer slot number to decode.
 *   scratch - A page-sized scratch buffer.
 *   ptrpage - The location to receive the decoded pointer target page.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned if
 * the inputs are invalid, the page cannot be read, or the decoded pointer
 * is outside the valid page range.
 *
 ****************************************************************************/

static int mfs_ctz_read_ptr(FAR struct mfs_sb_s *sb, mfs_t page,
                            unsigned int pow, FAR uint8_t *scratch,
                            FAR mfs_t *ptrpage)
{
  size_t offset;
  ssize_t nread;
  mfs_t raw;

  if (sb == NULL || scratch == NULL || ptrpage == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (page == MFS_LOCATION_INVALID || page >= MFS_PAGE_COUNT(sb))
    {
      ferr("invalid page\n");
      return -EINVAL;
    }

  if ((pow + 1) * MFS_CTZ_PTR_SIZE > MFS_PAGE_SIZE(sb))
    {
      ferr("invalid pointer slot\n");
      return -EINVAL;
    }

  nread = mfs_read_page(sb, page, scratch);
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

  offset = MFS_PAGE_SIZE(sb) - ((pow + 1) * MFS_CTZ_PTR_SIZE);
  memcpy(&raw, scratch + offset, sizeof(raw));
  * ptrpage = le32toh(raw);
  if (*ptrpage == MFS_LOCATION_INVALID || *ptrpage >= MFS_PAGE_COUNT(sb))
    {
      ferr("invalid pointer target\n");
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: mfs_ctz_write_ptr
 *
 * Description:
 * Encode one backward pointer into the pointer area of a CTZ unit buffer.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   buffer - The page-sized CTZ unit buffer to modify.
 *   pow - The pointer slot number to write.
 *   page - The target page to encode.
 *
 * Returned Value:
 * None.
 *
 ****************************************************************************/

static void mfs_ctz_write_ptr(FAR struct mfs_sb_s *sb, FAR uint8_t *buffer,
                              unsigned int pow, mfs_t page)
{
  size_t offset;
  mfs_t raw;

  DEBUGASSERT(sb != NULL && buffer != NULL);
  DEBUGASSERT((pow + 1) * MFS_CTZ_PTR_SIZE <= MFS_PAGE_SIZE(sb));

  offset = MFS_PAGE_SIZE(sb) - ((pow + 1) * MFS_CTZ_PTR_SIZE);
  raw = htole32(page);
  memcpy(buffer + offset, &raw, sizeof(raw));
}

/****************************************************************************
 * Name: mfs_ctz_follow
 *
 * Description:
 * Walk backward through a CTZ chain from startidx/startpage until endidx
 * is reached.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   startidx - The index of the known CTZ unit.
 *   endidx - The target earlier index to reach.
 *   startpage - The page of the known CTZ unit.
 *   scratch - A page-sized scratch buffer.
 *   endpage - The location to receive the page at endidx.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * invalid input or if any pointer read fails.
 *
 ****************************************************************************/

static int mfs_ctz_follow(FAR struct mfs_sb_s *sb, mfs_t startidx,
                          mfs_t endidx, mfs_t startpage,
                          FAR uint8_t *scratch,
                          FAR mfs_t *endpage)
{
  mfs_t diff;
  mfs_t page;
  mfs_t idx;
  unsigned int pow;
  unsigned int maxpow;
  int ret;

  if (sb == NULL || scratch == NULL || endpage == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (startpage == MFS_LOCATION_INVALID || startpage >= MFS_PAGE_COUNT(sb) ||
      startidx < endidx)
    {
      ferr("invalid traversal request\n");
      return -EINVAL;
    }

  page = startpage;
  idx  = startidx;
  while (idx > endidx)
    {
      diff   = idx - endidx;
      pow    = (sizeof(diff) * 8) - 1 - __builtin_clz(diff);
      maxpow = __builtin_ctz(idx);
      if (pow > maxpow)
        {
          pow = maxpow;
        }

      DEBUGASSERT(pow < (unsigned int)mfs_ctz_nptrs(idx));
      ret = mfs_ctz_read_ptr(sb, page, pow, scratch, &page);
      if (ret < 0)
        {
          ferr("mfs_ctz_read_ptr failed: %d\n", ret);
          return ret;
        }

      idx -= (mfs_t)1 << pow;
    }

  * endpage = page;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mfs_ctz_traverse
 *
 * Description:
 * Traverse a CTZ chain from one known unit index/page to an earlier
 * index.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   startidx - The index of the known CTZ unit.
 *   endidx - The earlier index to reach.
 *   startpage - The page of the known CTZ unit.
 *   endpage - The location to receive the resolved page.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * invalid input, allocation failure, or traversal failure.
 *
 ****************************************************************************/

int mfs_ctz_traverse(FAR struct mfs_sb_s *sb, mfs_t startidx, mfs_t endidx,
                     mfs_t startpage, FAR mfs_t *endpage)
{
  FAR uint8_t *scratch;
  int ret;

  if (sb == NULL || endpage == NULL || startpage == MFS_LOCATION_INVALID ||
      startpage >= MFS_PAGE_COUNT(sb) || startidx < endidx)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  scratch = kmm_malloc(MFS_PAGE_SIZE(sb));
  if (scratch == NULL)
    {
      ferr("kmm_malloc failed\n");
      return -ENOMEM;
    }

  ret = mfs_ctz_follow(sb, startidx, endidx, startpage, scratch, endpage);
  if (ret < 0)
    {
      ferr("mfs_ctz_follow failed: %d\n", ret);
    }

  kmm_free(scratch);
  return ret;
}

/****************************************************************************
 * Name: mfs_ctz_unit_data_area
 *
 * Description:
 * Return the number of payload bytes available in CTZ unit index after
 * its backward pointers are reserved.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   index - The CTZ unit index to inspect.
 *
 * Returned Value:
 * The number of payload bytes available in the unit is returned.
 *
 ****************************************************************************/

mfs_t mfs_ctz_unit_data_area(FAR struct mfs_sb_s *sb, mfs_t index)
{
  return (mfs_t)MFS_PAGE_SIZE(sb) - ((mfs_t)mfs_ctz_nptrs(index) * 4);
}

/****************************************************************************
 * Name: mfs_ctz_index_from_off
 *
 * Description:
 * Convert a logical file offset into the CTZ unit index and byte offset
 * inside that unit.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   offset - The logical file offset to translate.
 *   index - The location to receive the CTZ unit index.
 *   pageoff - The location to receive the byte offset within that unit.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * invalid input or overflow.
 *
 ****************************************************************************/

int mfs_ctz_index_from_off(FAR struct mfs_sb_s *sb, mfs_t offset,
                           FAR mfs_t *index,
                           FAR mfs_t *pageoff)
{
  mfs_t idx;
  mfs_t area;

  if (sb == NULL || index == NULL || pageoff == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  idx = 0;
  while (true)
    {
      area = mfs_ctz_unit_data_area(sb, idx);
      if (area == 0)
        {
          ferr("zero data area\n");
          return -EINVAL;
        }

      if (offset < area)
        {
          * index = idx;
          * pageoff = offset;
          return OK;
        }

      offset -= area;
      if (idx == UINT32_MAX)
        {
          ferr("offset overflow\n");
          return -EOVERFLOW;
        }

      idx++;
    }
}

/****************************************************************************
 * Name: mfs_ctz_fill_next_ptrs
 *
 * Description:
 * Populate the pointer area in a page-sized buffer as though the buffer
 * were the CTZ unit immediately after the known unit at `page`/`index`.
 *
 * For every pointer required by unit `index + 1`, this walks backward from
 * the known unit `index` to the target `(index + 1) - 2^pow`.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   page - The location of the known CTZ unit at `index`.
 *   index - The index of the known CTZ unit.
 *   buffer - Caller-supplied page-sized buffer to receive the speculative
 *   pointer bytes.
 *
 * Returned Value:
 * Zero (OK) is returned on success. A negated errno value is returned on
 * failure.
 *
 ****************************************************************************/

int mfs_ctz_fill_next_ptrs(FAR struct mfs_sb_s *sb, mfs_t page,
                           mfs_t index, FAR uint8_t *buffer)
{
  FAR uint8_t *scratch;
  mfs_t targetidx;
  mfs_t targetpage;
  mfs_t nextidx;
  unsigned int pow;
  unsigned int nptrs;
  int ret;

  if (sb == NULL || buffer == NULL)
    {
      ferr("invalid args\n");
      return -EINVAL;
    }

  if (page == MFS_LOCATION_INVALID || page >= MFS_PAGE_COUNT(sb))
    {
      ferr("invalid page\n");
      return -EINVAL;
    }

  if (index == UINT32_MAX)
    {
      ferr("index overflow\n");
      return -EOVERFLOW;
    }

  scratch = kmm_malloc(MFS_PAGE_SIZE(sb));
  if (scratch == NULL)
    {
      ferr("kmm_malloc failed\n");
      return -ENOMEM;
    }

  nextidx = index + 1;
  nptrs   = mfs_ctz_nptrs(nextidx);
  for (pow = 0; pow < nptrs; pow++)
    {
      targetidx = index - (((mfs_t)1 << pow) - 1);
      ret = mfs_ctz_follow(sb, index, targetidx, page, scratch, &targetpage);
      if (ret < 0)
        {
          ferr("mfs_ctz_follow failed: %d\n", ret);
          goto errout;
        }

      mfs_ctz_write_ptr(sb, buffer, pow, targetpage);
    }

  kmm_free(scratch);
  return OK;

errout:
  kmm_free(scratch);
  return ret;
}

/****************************************************************************
 * Name: mfs_page_to_block
 *
 * Description:
 * Convert a page number into the block that contains it.
 *
 * Input Parameters:
 *   sb - The mounted file system instance.
 *   page - The page number to translate.
 *
 * Returned Value:
 * The containing block number is returned on success.
 * MFS_LOCATION_INVALID is returned if the inputs are invalid.
 *
 ****************************************************************************/

mfs_t mfs_page_to_block(FAR const struct mfs_sb_s *sb, mfs_t page)
{
  if (sb == NULL || page == MFS_LOCATION_INVALID ||
      page >= MFS_PAGE_COUNT(sb))
    {
      return MFS_LOCATION_INVALID;
    }

  return page / MFS_PAGES_PER_BLOCK(sb);
}
