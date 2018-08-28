/****************************************************************************
 * fs/partition/fs_ptable.c
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

      strncpy(part.name, entry->name, sizeof(part.name));
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

