/****************************************************************************
 * fs/partition/fs_partition.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/mount.h>

#include <assert.h>

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
 * Private Data
 ****************************************************************************/

static const partition_parser_t g_parser[] =
{
#ifdef CONFIG_PTABLE_PARTITION
  parse_ptable_partition,
#endif

#ifdef CONFIG_GPT_PARTITION
  parse_gpt_partition,
#endif

#ifdef CONFIG_MBR_PARTITION
  parse_mbr_partition,
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
  int i;
  int ret = 0;

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

int read_partition_block(FAR struct partition_state_s *state,
                         FAR void *buffer, size_t startblock,
                         size_t nblocks)
{
  if (state->blk)
    {
      return state->blk->u.i_bops->read(state->blk,
                                        buffer, startblock, nblocks);
    }
  else
    {
      return state->mtd->bread(state->mtd, startblock, nblocks, buffer);
    }
}

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
  struct mtd_geometry_s mgeo;
  struct geometry geo;
  int ret;

  ret = open_blockdriver(path, MS_RDONLY, &state.blk);
  if (ret < 0)
    {
      return ret;
    }

  state.mtd = NULL;

  if (state.blk->u.i_bops->ioctl != NULL &&
      state.blk->u.i_bops->ioctl(state.blk, MTDIOC_GEOMETRY,
                                 (unsigned long)(uintptr_t)&mgeo) >= 0)
    {
      DEBUGASSERT(mgeo.blocksize);

      state.blocksize = mgeo.blocksize;
      state.erasesize = mgeo.erasesize;
      state.nblocks   = mgeo.neraseblocks;
      state.nblocks  *= mgeo.erasesize / mgeo.blocksize;

      ret = parse_partition(&state, handler, arg);
    }
  else
    {
      ret = state.blk->u.i_bops->geometry(state.blk, &geo);
      if (ret >= 0)
        {
          DEBUGASSERT(geo.geo_sectorsize);

          state.blocksize = geo.geo_sectorsize;
          state.erasesize = geo.geo_sectorsize;
          state.nblocks   = geo.geo_nsectors;

          ret = parse_partition(&state, handler, arg);
        }
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
  struct mtd_geometry_s mgeo;
  int ret;

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)(uintptr_t)&mgeo);
  if (ret < 0)
    {
      return ret;
    }

  DEBUGASSERT(mgeo.blocksize);

  state.blk       = NULL;
  state.mtd       = mtd;
  state.blocksize = mgeo.blocksize;
  state.erasesize = mgeo.erasesize;
  state.nblocks   = mgeo.neraseblocks;
  state.nblocks  *= mgeo.erasesize / mgeo.blocksize;

  return parse_partition(&state, handler, arg);
}
