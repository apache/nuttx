/****************************************************************************
 * fs/mnemofs/mnemofs_journal.c
 * Journal of mnemofs.
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
 * In mnemofs, the journal stores the path, depth and the new location of the
 * CTZ file called logs, and also the location of the master block. The first
 * n blocks of the journal store the logs, while the last two blocks contain
 * master nodes, and the blocks are called as master blocks. The two master
 * blocks are identical copies for backup.
 *
 * Due to LRU, and the structure of mnemofs, the first n blocks of the
 * journal get filled up much faster than the master blocks, and move more.
 * There will be certain point where the entire journal (the n+2 blocks)
 * move, but mostly, its the first n blocks that move.
 *
 * The first block starts with an 8 byte magic sequence, a 2 bytes long
 * number denoting number of blocks in the journal, and then follows up
 * with an array containing the block numbers of all blocks in the journal
 * including the first block. Then the logs start.
 *
 * The logs take up size in multiples of pages. There might be unitilzed
 * space at the end of a log.
 *
 * All logs are followed by a byte-long hash of the log.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <endian.h>
#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
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

int mfs_jrnl_init(FAR struct mfs_sb_s * const sb, mfs_t blk)
{
  /* TODO */

  return OK;
}

void mfs_jrnl_free(FAR struct mfs_sb_s * const sb)
{
  /* TODO */
}

int mfs_jrnl_fmt(FAR struct mfs_sb_s * const sb, mfs_t blk1, mfs_t blk2)
{
  /* TODO */

  return OK;
}
