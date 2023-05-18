/****************************************************************************
 * fs/partition/fs_mbr.c
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

#include <debug.h>
#include <endian.h>
#include <string.h>

#include <nuttx/kmalloc.h>

#include "partition.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MBR_SIZE                   512
#define MBR_LBA_TO_BLOCK(lba, blk) ((le32toh(lba) * 512 + (blk) - 1) / (blk))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* These three have identical behaviour; use the second one if DOS FDISK gets
 * confused about extended/logical partitions starting past cylinder 1023.
 */

enum mbr_type_e
{
  DOS_EXTENDED_PARTITION = 5,
  LINUX_EXTENDED_PARTITION = 0x85,
  WIN98_EXTENDED_PARTITION = 0x0f,
};

/* Description of one partition table entry (D*S type) */

begin_packed_struct struct mbr_entry_s
{
  uint8_t boot_indicator;   /* Maybe marked as an active partition */
  uint8_t chs_begin[3];     /* Start of the partition in cylinders, heads and sectors */
  uint8_t type;             /* Filesystem type */
  uint8_t chs_end[3];       /* End of the partition in cylinders, heads and sectors */
  uint32_t partition_start; /* Start of the partition in LBA notation */
  uint32_t partition_size;  /* Start of the partition in LBA notation */
} end_packed_struct;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline int is_extended(uint8_t type)
{
  return (type == DOS_EXTENDED_PARTITION ||
          type == WIN98_EXTENDED_PARTITION ||
          type == LINUX_EXTENDED_PARTITION);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: parse_mbr_partition
 *
 * Description:
 *   parse the mbr(Master_boot_record) partition.
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

int parse_mbr_partition(FAR struct partition_state_s *state,
                        partition_handler_t handler,
                        FAR void *arg)
{
  struct partition_s pentry;
  FAR struct mbr_entry_s *table;
  FAR struct mbr_entry_s *extended = NULL;
  FAR uint8_t *buffer;
  int num;
  int ret;
  int i;

  num = (MBR_SIZE + state->blocksize - 1) / state->blocksize;
  buffer = kmm_malloc(num * state->blocksize);
  if (!buffer)
    {
      return -ENOMEM;
    }

  ret = read_partition_block(state, buffer, 0, num);
  if (ret < 0)
    {
      kmm_free(buffer);
      return ret;
    }

  if (buffer[0x1fe] != 0x55 || buffer[0x1ff] != 0xaa)
    {
      kmm_free(buffer);
      return -EINVAL;
    }

  memset(&pentry, 0, sizeof(pentry));
  table = (FAR struct mbr_entry_s *)&buffer[0x1be];
  for (i = 0; i < 4; i++)
    {
      pentry.firstblock = MBR_LBA_TO_BLOCK(table[i].partition_start,
                          state->blocksize);
      pentry.nblocks    = MBR_LBA_TO_BLOCK(table[i].partition_size,
                          state->blocksize);
      pentry.blocksize  = state->blocksize;

      if (pentry.nblocks != 0)
        {
          if (!is_extended(table[i].type))
            {
              handler(&pentry, arg);
              pentry.index++;
            }
          else if(!extended)
            {
              extended = &table[i];
            }
        }
      else
        {
          finfo("Skipping empty partition %d\n", i);
        }
    }

  if (extended)
    {
      uint32_t ebr_block;
      ebr_block = MBR_LBA_TO_BLOCK(extended->partition_start,
                                   state->blocksize);
      while (1)
        {
          ret = read_partition_block(state, buffer, ebr_block, num);
          if (ret < 0)
            {
              goto out;
            }

          if (buffer[0x1fe] != 0x55 || buffer[0x1ff] != 0xaa)
            {
              ferr("block %x doesn't contain an EBR signature\n",
                   ebr_block);
              ret = -EINVAL;
              goto out;
            }

          for (i = 0x1de; i < 0x1fe; ++i)
            {
              if (buffer[i])
                {
                  ferr("EBR's third or fourth partition non-empty\n");
                  ret = -EINVAL;
                  goto out;
                }
            }

          /* the first entry defines the extended partition */

          pentry.firstblock = ebr_block + MBR_LBA_TO_BLOCK(
                              table[0].partition_start, state->blocksize);
          pentry.nblocks = MBR_LBA_TO_BLOCK(table[0].partition_size,
                           state->blocksize);
          handler(&pentry, arg);
          pentry.index++;

          /* the second entry defines the start of the next ebr if != 0 */

          if (table[1].partition_start)
            {
              ebr_block = pentry.firstblock + MBR_LBA_TO_BLOCK(
                          table[1].partition_start, state->blocksize);
            }
          else
            {
              break;
            }
        }
    }

out:
  kmm_free(buffer);
  return ret;
}
