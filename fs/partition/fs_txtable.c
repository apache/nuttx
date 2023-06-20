/****************************************************************************
 * fs/partition/fs_txtable.c
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

#include <string.h>
#include <stdio.h>
#include <ctype.h>

#include <nuttx/kmalloc.h>

#include "partition.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TXTABLE_MAGIC   "TXTABLE0"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: parse_txtable_partition
 *
 * Description:
 *   parse the TXTABLE partition txtable.
 *
 * Input Parameters:
 *   state   - The partition txtable state
 *   handler - The function to be called for each found partition
 *   arg     - A caller provided value to return with the handler
 *
 * Returned Value:
 *   A negated errno value is returned on a failure; Otherwise success
 *
 ****************************************************************************/

int parse_txtable_partition(FAR struct partition_state_s *state,
                            partition_handler_t handler,
                            FAR void *arg)
{
  FAR char *save_ptr;
  FAR char *token;
  FAR struct partition_s *part;
  size_t blkpererase;
  size_t lasteraseblk;
  int ret = OK;
  int i;
  int j;

  /* Allocate memory for table of parsed and raw */

  part = kmm_malloc(CONFIG_TXTABLE_PARTITION_MAX_NUM *
                    sizeof(struct partition_s) +
                    state->erasesize);
  if (part == NULL)
    {
      return -ENOMEM;
    }

  memset(part, 0, CONFIG_TXTABLE_PARTITION_MAX_NUM *
         sizeof(struct partition_s));

  token = (FAR char *)(part + CONFIG_TXTABLE_PARTITION_MAX_NUM);

  /* TXTABLE locate in the last erase block */

  blkpererase = state->erasesize / state->blocksize;
  lasteraseblk = state->nblocks - blkpererase;

  ret = read_partition_block(state, token, lasteraseblk, blkpererase);
  if (ret < 0)
    {
      goto out;
    }

  /* Parsing data of partition table */

  token = strtok_r(token, "\n", &save_ptr);
  if (strncmp(token, TXTABLE_MAGIC, strlen(TXTABLE_MAGIC)) != 0)
    {
      ret = -EFTYPE;
      goto out;
    }

  for (i = 0; i < CONFIG_TXTABLE_PARTITION_MAX_NUM; i++)
    {
      token = strtok_r(NULL, "\n", &save_ptr);
      if (token == NULL || !isalnum(token[0]))
        {
          break;
        }

      ret = sscanf(token, "%s %zx %zx",
                   part[i].name,
                   &part[i].nblocks,
                   &part[i].firstblock);
      if (ret < 0)
        {
          goto out;
        }

      part[i].index       = i;
      part[i].firstblock /= state->blocksize;
      part[i].nblocks    /= state->blocksize;
      part[i].blocksize   = state->blocksize;
    }

  for (j = 0; j < i; j++)
    {
      if (!part[j].firstblock && j > 0)
        {
          part[j].firstblock = part[j - 1].firstblock +
                               part[j - 1].nblocks;
        }

      if (!part[j].nblocks)
        {
          if (j + 1 < i)
            {
              if (part[j + 1].firstblock)
                {
                  part[j].nblocks = part[j + 1].firstblock -
                                    part[j].firstblock;
                }
            }
          else
            {
              part[j].nblocks = state->nblocks - part[j].firstblock -
                                blkpererase;
            }
        }

      /* Reserved for txtable */

      if (j + 1 == i &&
          part[j].firstblock + part[j].nblocks > lasteraseblk)
        {
          part[j].nblocks = lasteraseblk - part[j].firstblock;
        }

      /* Notify the caller */

      handler(&part[j], arg);
    }

out:
  kmm_free(part);
  return ret;
}
