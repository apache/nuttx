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

#include <sys/param.h>

#include "mnemofs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mfs_isbadblk(FAR const struct mfs_sb_s * const sb, mfs_t blk)
{
  if (predict_false(blk > MFS_NBLKS(sb)))
    {
      return -EINVAL;
    }

  return MTD_ISBAD(MFS_MTD(sb), blk);
}

int mfs_markbadblk(FAR const struct mfs_sb_s * const sb, mfs_t blk)
{
  if (predict_false(blk > MFS_NBLKS(sb)))
    {
      return -EINVAL;
    }

  return MTD_MARKBAD(MFS_MTD(sb), blk);
}

/* NOTE: These functions do not update the block allocator's state nor do
 * they enforce it.
 */

ssize_t mfs_write_page(FAR const struct mfs_sb_s * const sb,
                       FAR const char *data, const mfs_t datalen,
                       const off_t page, const mfs_t pgoff)
{
  int ret = OK;

  if (predict_false(page > MFS_NPGS(sb) || pgoff >= MFS_PGSZ(sb)))
    {
      return -EINVAL;
    }

  memcpy(MFS_RWBUF(sb) + pgoff, data, MIN(datalen, MFS_PGSZ(sb) - pgoff));

  ret = MTD_BWRITE(MFS_MTD(sb), page, 1, MFS_RWBUF(sb));
  if (predict_false(ret < 0))
    {
      goto errout_with_reset;
    }

errout_with_reset:
  memset(MFS_RWBUF(sb), 0, MFS_PGSZ(sb));

  return ret;
}

ssize_t mfs_read_page(FAR const struct mfs_sb_s * const sb,
                      FAR char *data, const mfs_t datalen, const off_t page,
                      const mfs_t pgoff)
{
  int ret = OK;

  if (predict_false(page > MFS_NPGS(sb) || pgoff >= MFS_PGSZ(sb)))
    {
      return -EINVAL;
    }

  ret = MTD_BREAD(MFS_MTD(sb), page, 1, MFS_RWBUF(sb));
  if (predict_false(ret < 0))
    {
      goto errout_with_reset;
    }

  memcpy(data, MFS_RWBUF(sb) + pgoff, MIN(datalen, MFS_PGSZ(sb) - pgoff));

errout_with_reset:
  memset(MFS_RWBUF(sb), 0, MFS_PGSZ(sb));

  return ret;
}

int mfs_erase_blk(FAR const struct mfs_sb_s * const sb, const off_t blk)
{
  if (predict_false(blk > MFS_NBLKS(sb)))
    {
      return -EINVAL;
    }

  return MTD_ERASE(MFS_MTD(sb), blk, 1);
}

int mfs_erase_nblks(FAR const struct mfs_sb_s * const sb, const off_t blk,
                    const size_t n)
{
  if (predict_false(blk + n > MFS_NBLKS(sb)))
    {
      return -EINVAL;
    }

  return MTD_ERASE(MFS_MTD(sb), blk, n);
}
