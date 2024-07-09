/****************************************************************************
 * fs/mnemofs/mnemofs_blkalloc.c
 * Block Allocator for mnemofs
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

/* mnemofs block allocator takes some inspiration from littlefs's block
 * allocator.
 *
 * It has two primary jobs...provide a block and ensure wear levelling. The
 * block allocator of mnemofs tries to provide a block that will more or less
 * ensure wear levelling. We'll call the block allocator as BA.
 *
 * The block allocator starts at a random block in the device and starts a
 * circular allocation from there, ie. it allocated sequentially till it
 * reaches the end, at which point it cycles back to the beginning and then
 * continues allocating sequentially. If a page is requested it will check if
 * the page has been written to (being used). If a page is being written to
 * but all the pages in a block are ready to be erased, then the block is
 * erased and page is allocated. If none of these two conditions match, it
 * moves on to check the next page and so on. If the block that contains the
 * page is a bad block, the BA skips all the pages in the entire block.
 *
 * The BA can also grant a request for an entire block. If the BA is
 * currently in the middle of a block, it will skip the remaining pages till
 * it reaches the start of the next block. These pages won't be reflected as
 * being used, and can be allocated the next time the BA cycles back to these
 * pages. Even though skipped pages will be eventually utilized later anyway,
 * block allocation requests are made by very few critical data structures
 * in mnemofs, and they all do it in bulk, and thus skipped pages are
 * minimal.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <math.h>
#include <nuttx/kmalloc.h>
#include <stdbool.h>
#include <stdlib.h>

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

int mfs_ba_init(FAR struct mfs_sb_s * const sb)
{
  /* TODO */

  return OK;
}

void mfs_ba_free(FAR struct mfs_sb_s * const sb)
{
  /* TODO */
}
