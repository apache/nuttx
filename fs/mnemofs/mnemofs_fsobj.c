/****************************************************************************
 * fs/mnemofs/mnemofs_fsobj.c
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
 * In mnemofs, all the FS object methods (ie. methods in this file),
 * interface directly with the LRU. To these methods, only the methods
 * exposed by the LRU are visible, nothing else. The LRU will give them the
 * most updated data, which includes data from the flash, the updates from
 * the journal and the LRU deltas as well.
 *
 * TODO: The above menetioned concept.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/kmalloc.h>
#include <sys/stat.h>

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
 * Public Function Prototypes
 ****************************************************************************/

FAR const char * mfs_path2childname(FAR const char *relpath)
{
  /* TODO */

  return NULL;
}

mfs_t mfs_get_fsz(FAR struct mfs_sb_s * const sb,
                  FAR const struct mfs_path_s * const path,
                  const mfs_t depth)
{
  /* TODO */

  return 0;
}

int mfs_get_patharr(FAR struct mfs_sb_s *const sb,
                    FAR const char *relpath, FAR struct mfs_path_s **path,
                    FAR mfs_t *depth)
{
  /* TODO */

  return OK;
}

void mfs_free_patharr(FAR struct mfs_path_s *path)
{
  /* TODO */
}

bool mfs_obj_isempty(FAR struct mfs_sb_s * const sb,
                    FAR struct mfs_pitr_s * const pitr)
{
  /* TODO */

  return false;
}

void mfs_pitr_init(FAR struct mfs_sb_s * const sb,
                  FAR const struct mfs_path_s * const path,
                  const mfs_t depth, FAR struct mfs_pitr_s *pitr, bool child)
{
  /* TODO */
}

void mfs_pitr_free(FAR struct mfs_pitr_s * const pitr)
{
  /* TODO */
}

void mfs_pitr_adv(FAR struct mfs_sb_s * const sb,
                  FAR struct mfs_pitr_s * const pitr)
{
  /* TODO */
}

void mfs_pitr_adv_dirent(FAR struct mfs_pitr_s * const pitr,
                        FAR const struct mfs_dirent_s * const dirent)
{
  /* TODO */
}

void mfs_pitr_adv_off(FAR struct mfs_pitr_s * const pitr,
                      const mfs_t off)
{
  /* TODO */
}

void mfs_pitr_adv_tochild(FAR struct mfs_pitr_s * const pitr,
                          FAR const struct mfs_path_s * const path,
                          const mfs_t depth)
{
  /* TODO */
}

void mfs_pitr_reset(FAR struct mfs_pitr_s * const pitr)
{
  /* TODO */
}

void mfs_pitr_sync(FAR struct mfs_sb_s * const sb,
                    FAR struct mfs_pitr_s * const pitr,
                    FAR const struct mfs_path_s * const path,
                    const mfs_t depth)
{
  /* TODO */
}

int mfs_pitr_readdirent(FAR struct mfs_sb_s * const sb,
                        FAR struct mfs_pitr_s * const pitr,
                        FAR struct mfs_dirent_s **dirent)
{
  /* TODO */

  return OK;
}

void mfs_free_dirent(FAR struct mfs_dirent_s *dirent)
{
  /* TODO */
}

bool mfs_searchfopen(FAR const struct mfs_sb_s * const sb,
                      FAR const struct mfs_path_s * const path,
                      const mfs_t depth)
{
  /* TODO */

  return false;
}

int mfs_pitr_appendnew(FAR struct mfs_sb_s * const sb,
                      FAR struct mfs_path_s * const path,
                      const mfs_t depth,
                      FAR const struct mfs_pitr_s * const pitr,
                      FAR const char * const child_name,
                      const mode_t mode)
{
  /* TODO */

  return OK;
}

int mfs_pitr_appenddirent(FAR struct mfs_sb_s * const sb,
                          FAR struct mfs_path_s * const path,
                          const mfs_t depth,
                          FAR const struct mfs_pitr_s * const pitr,
                          FAR const struct mfs_dirent_s * const dirent)
{
  /* TODO */

  return OK;
}

int mfs_pitr_rmdirent(FAR struct mfs_sb_s * const sb,
                      FAR struct mfs_path_s * const path,
                      const mfs_t depth,
                      FAR struct mfs_pitr_s * const pitr,
                      FAR const struct mfs_dirent_s * const dirent)
{
  /* TODO */

  return OK;
}

int mfs_pitr_rm(FAR struct mfs_sb_s * const sb,
                FAR struct mfs_path_s * const path,
                const mfs_t depth)
{
  /* TODO */

  return OK;
}
