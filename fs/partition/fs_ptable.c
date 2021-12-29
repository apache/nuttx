/****************************************************************************
 * fs/partition/fs_ptable.c
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

#include <nuttx/kmalloc.h>

#include "partition.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PTABLE_MAGIC_LEN            8
#define PTABLE_VERSION_LEN          8
#define PTABLE_NAME_LEN             16

#define PTABLE_MAGIC                "PTABLE0"
#define PTABLE_FLAG_END             (1 << 0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ptable_entry_s
{
  char name[PTABLE_NAME_LEN];
  uint64_t offset;
  uint64_t length;
  uint64_t flags;
  uint64_t reserve;
};

struct ptable_s
{
  char magic[PTABLE_MAGIC_LEN];
  char version[PTABLE_VERSION_LEN];
  struct ptable_entry_s entries[];
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: parse_ptable_partition
 *
 * Description:
 *   parse the PTABLE partition table.
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

int parse_ptable_partition(FAR struct partition_state_s *state,
                           partition_handler_t handler,
                           FAR void *arg)
{
  FAR struct ptable_entry_s *entry;
  FAR struct ptable_s *ptable;
  size_t blkpererase;
  size_t block;
  int ret = OK;

  /* Allocate one erase block memory */

  ptable = kmm_malloc(state->erasesize);
  if (ptable == NULL)
    {
      return -ENOMEM;
    }

  /* PTABLE locate in the first or last erase block */

  blkpererase = state->erasesize / state->blocksize;
  for (block = 0;
       block < state->nblocks;
       block += state->nblocks - blkpererase)
    {
      ret = read_partition_block(state, ptable, block, blkpererase);
      if (ret < 0)
        {
          goto out;
        }

      if (strcmp(ptable->magic, PTABLE_MAGIC) == 0)
        {
          break; /* Find the magic number */
        }
    }

  if (block >= state->nblocks)
    {
      ret = -EFTYPE;
      goto out;
    }

  entry = ptable->entries;
  while (!(entry->flags & PTABLE_FLAG_END))
    {
      struct partition_s part;

      /* Convert the entry to partition */

      strlcpy(part.name, entry->name, sizeof(part.name));
      part.index      = entry - ptable->entries;
      part.firstblock = entry->offset / state->blocksize;
      part.nblocks    = entry->length / state->blocksize;
      part.blocksize  = state->blocksize;

      /* Notify the caller */

      handler(&part, arg);

      /* Move to the next entry */

      entry++;
      if ((uintptr_t)entry - (uintptr_t)ptable >= state->erasesize)
        {
          break; /* Exit, at the end of erase block */
        }
    }

out:
  kmm_free(ptable);
  return ret;
}
