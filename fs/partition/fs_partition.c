/****************************************************************************
 * fs/partition/fs_partition.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/mount.h>

#include "partition.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef CODE int
  (*partition_parser_t)(FAR struct partition_state_s *state,
                        partition_handler_t handler,
                        FAR void *arg);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int parse_partition(FAR struct partition_state_s *state,
                           partition_handler_t handler,
                           FAR void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_PTABLE_PARTITION
int parse_ptable_partition(FAR struct partition_state_s *state,
                           partition_handler_t handler,
                           FAR void *arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const partition_parser_t g_parser[] =
{
#ifdef CONFIG_PTABLE_PARTITION
  parse_ptable_partition,
#endif
  NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: parse_partition
 *
 * Description:
 *   parse the partition table.
 *
 * Input Parameters:
 *   state   - The partition table state
 *   handler - The function to be called for each found partition
 *   arg     - A caller provided value to return with the handler
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on a failure
 *
 ****************************************************************************/

static int parse_partition(FAR struct partition_state_s *state,
                           partition_handler_t handler,
                           FAR void *arg)
{
  int i, ret = 0;

  for (i = 0; g_parser[i] != NULL; i++)
    {
      ret = g_parser[i](state, handler, arg);
      if (ret >= 0)
        {
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: parse_block_partition
 *
 * Description:
 *   parse the partition table on a block device.
 *
 * Input Parameters:
 *   path    - The block device to be parsed
 *   handler - The function to be called for each found partition
 *   arg     - A caller provided value to return with the handler
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on a failure
 *
 ****************************************************************************/

int parse_block_partition(FAR const char *path,
                          partition_handler_t handler,
                          FAR void *arg)
{
  struct partition_state_s state;
  struct geometry geo;
  int ret;

  ret = open_blockdriver(path, MS_RDONLY, &state.blk);
  if (ret < 0)
    {
      return ret;
    }

  ret = state.blk->u.i_bops->geometry(state.blk, &geo);
  if (ret >= 0)
    {
      state.mtd       = NULL;
      state.blocksize = geo.geo_sectorsize;
      state.erasesize = geo.geo_sectorsize;
      state.nblocks   = geo.geo_nsectors;

      ret = parse_partition(&state, handler, arg);
    }

  close_blockdriver(state.blk);
  return ret;
}

/****************************************************************************
 * Name: parse_mtd_partition
 *
 * Description:
 *   parse the partition table on a mtd device.
 *
 * Input Parameters:
 *   mtd     - The MTD device to be parsed
 *   handler - The function to be called for each found partition
 *   arg     - A caller provided value to return with the handler
 *
 * Returned Value:
 *   Zero on success; A negated errno value is returned on a failure
 *
 ****************************************************************************/

int parse_mtd_partition(FAR struct mtd_dev_s *mtd,
                        partition_handler_t handler,
                        FAR void *arg)
{
  struct partition_state_s state;
  struct mtd_geometry_s geo;
  int ret;

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)&geo);
  if (ret < 0)
    {
      return ret;
    }

  state.blk       = NULL;
  state.mtd       = mtd;
  state.blocksize = geo.blocksize;
  state.erasesize = geo.erasesize;
  state.nblocks   = geo.neraseblocks;
  state.nblocks  *= geo.erasesize / geo.blocksize;

  return parse_partition(&state, handler, arg);
}

