/****************************************************************************
 * fs/partition/fs_gpt.c
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

#include <crc32.h>
#include <ctype.h>
#include <debug.h>
#include <endian.h>
#include <inttypes.h>

#include <nuttx/kmalloc.h>

#include "partition.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPT_BLOCK_SIZE                  512
#define GPT_HEADER_SIGNATURE            0x5452415020494645ull
#define GPT_PARTNAME_MAX_SIZE           (72 / sizeof(uint16_t))
#define GPT_LBA_TO_BLOCK(lba, blk)      ((le64toh(lba) * 512 + (blk) -1) / (blk))
#define GPT_MIN(x, y)                   (((x) < (y)) ? (x) : (y))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct gpt_guid_s
{
  uint8_t b[16];
};

/* For limited backward compatibility, the space of the legacy MBR is still
 * reserved in the GPT specification, but it is now used in a way that
 * prevents MBR-based disk utilities from misrecognizing and possibly
 * overwriting GPT disks. This is referred to as a protective MBR.
 */

begin_packed_struct struct legacy_partition_s
{
  uint8_t  boot_ind;                     /* 0x80 - active */
  uint8_t  head;                         /* Starting head */
  uint8_t  sector;                       /* Starting sector */
  uint8_t  cyl;                          /* Starting cylinder */
  uint8_t  sys_ind;                      /* What partition type */
  uint8_t  end_head;                     /* End head */
  uint8_t  end_sector;                   /* End sector */
  uint8_t  end_cyl;                      /* End cylinder */
  uint32_t start_sect;                   /* Starting sector counting from 0 */
  uint32_t nr_sects;                     /* Nr of sectors in partition */
} end_packed_struct;

/* The partition table header defines the usable blocks on the disk.
 * It also defines the number and size of the partition entries that
 * make up the partition table (offsets 80 and 84 in the table).
 */

begin_packed_struct struct gpt_header_s
{
  uint64_t signature;                    /* EFI PART */
  uint32_t revision;                     /* Revision info */
  uint32_t header_size;                  /* Header size in little endian */
  uint32_t header_crc32;                 /* CRC32 of header (offset +0 up to header size) */
  uint32_t reserved1;                    /* Must be zero */
  uint64_t my_lba;                       /* Current LBA (location of this header copy) */
  uint64_t alternate_lba;                /* Backup LBA (location of the other header copy) */
  uint64_t first_usable_lba;             /* First usable LBA for partitions primary partition table last LBA + 1 */
  uint64_t last_usable_lba;              /* Last usable LBA secondary partition table first LBA − 1 */
  struct gpt_guid_s disk_guid;           /* Disk GUID in mixed endian */
  uint64_t partition_entry_lba;          /* Starting LBA of array of partition entries (always 2 in primary copy) */
  uint32_t num_partition_entries;        /* Number of partition entries in array */
  uint32_t sizeof_partition_entry;       /* Size of a single partition entry */
  uint32_t partition_entry_array_crc32;  /* CRC32 of partition entries array in little endian */

  /* The rest of the logical block is reserved by UEFI and must be zero.
   * EFI standard handles this by:
   *
   * uint8_t    reserved2[ BlockSize - 92 ];
   */
} end_packed_struct;

/* After the header, the Partition Entry Array describes partitions,
 * using a minimum size of 128 bytes for each entry block.
 */

/* The 64-bit partition table attributes are shared between 48-bit
 * common attributes for all partition types, and 16-bit
 * type-specific attributes
 */

begin_packed_struct struct gpt_entry_attributes_s
{
  uint64_t required_to_function:1;
  uint64_t reserved:47;
  uint64_t type_guid_specific:16;
} end_packed_struct;

begin_packed_struct struct gpt_entry_s
{
  struct gpt_guid_s partition_type_guid;          /* Partition type GUID */
  struct gpt_guid_s unique_partition_guid;        /* Unique partition GUID */
  uint64_t starting_lba;                          /* First LBA */
  uint64_t ending_lba;                            /* Last LBA */
  struct gpt_entry_attributes_s attributes;       /* Attribute flags */
  uint16_t partition_name[GPT_PARTNAME_MAX_SIZE]; /* Partition name */
} end_packed_struct;

begin_packed_struct struct gpt_ptable_s
{
  uint8_t mbr[512];
  union
  {
    struct gpt_header_s gpt_header;
    uint8_t gpt[512];
  } u;
} end_packed_struct;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct gpt_guid_s g_null_guid;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpt_last_lba
 *
 * Description:
 *   Return number of last logical block of device, 0 on error.
 *
 * Input Parameters:
 *   state   - The partition table state
 *
 * Returned Value:
 *   Returns last LBA value on success, 0 on error.
 *   This is stored (by sd and ide-geometry) in
 *   the part[0] entry for this disk, and is the number of
 *   physical sectors available on the disk.
 *
 ****************************************************************************/

static inline blkcnt_t gpt_last_lba(FAR struct partition_state_s *state)
{
  return (((uint64_t)state->nblocks) * state->blocksize + GPT_BLOCK_SIZE - 1)
         / GPT_BLOCK_SIZE - 1;
}

/****************************************************************************
 * Name: gpt_alloc_verify_entries()
 *
 * Description:
 *   reads and verifies partition entries from disk
 *
 * Input Parameters:
 *   state     - the handle of partition state
 *   gpt       - a GPT header ptr.
 *
 * Returned Value:
 *   Returns ptes on success,  NULL on error.
 *   Allocates space for PTEs based on information found in @gpt.
 *   Notes: remember to free pte when you're done!
 *
 ****************************************************************************/

static FAR struct gpt_entry_s *
gpt_alloc_verify_entries(FAR struct partition_state_s *state,
                         FAR struct gpt_header_s *gpt)
{
  FAR struct gpt_entry_s *pte;
  unsigned long from;
  unsigned long size;
  unsigned long blk;
  uint32_t crc;
  int ret;

  size = le32toh(gpt->num_partition_entries) *
         le32toh(gpt->sizeof_partition_entry);
  if (!size)
    {
      return NULL;
    }

  blk = (size + (state->blocksize -1)) / state->blocksize;
  pte = kmm_zalloc(blk * state->blocksize);
  if (!pte)
    {
      return NULL;
    }

  from = GPT_LBA_TO_BLOCK(gpt->partition_entry_lba,
                          state->blocksize);
  ret = read_partition_block(state, pte, from, blk);
  if (ret < 0)
    {
      kmm_free(pte);
      ferr("Read ptr from block failed:%d.\n", ret);
      return NULL;
    }

  /* Check the GUID Partition Table Entry Array CRC */

  crc = crc32part((FAR const uint8_t *)pte, size, ~0l) ^ ~0l;
  if (crc != le32toh(gpt->partition_entry_array_crc32))
    {
      ferr("GUID Partitition Entry Array CRC check failed.\n");
      kmm_free(pte);
      return NULL;
    }

  return pte;
}

/****************************************************************************
 * Name: gpt_header_is_valid
 *
 * Description:
 *   tests one GPT header for validity
 *
 * Input Parameters:
 *   state   - The partition table state
 *   gpt     - is a GPT header ptr.
 *   lba     - is the logical block address of the GPT header to test
 *
 * Returned Value:
 *   Returns 0 if valid,  a negative errno returned on error.
 *
 ****************************************************************************/

static int gpt_header_is_valid(FAR struct partition_state_s *state,
                               FAR struct gpt_header_s *gpt, blkcnt_t lba)
{
  uint32_t crc;
  uint32_t origcrc;
  blkcnt_t lastlba;

  /* Check the GPT header signature */

  if (le64toh(gpt->signature) != GPT_HEADER_SIGNATURE)
    {
      ferr("GUID Partition Table Header signature is wrong:"
            "0x%" PRIx64 " != 0x%llx\n",
            le64toh(gpt->signature), GPT_HEADER_SIGNATURE);
      return -EINVAL;
    }

  /* Check the GUID Partition Table CRC */

  origcrc = gpt->header_crc32;
  gpt->header_crc32 = 0;
  crc = crc32part((FAR const uint8_t *)gpt,
                  le32toh(gpt->header_size), ~0l) ^ ~0l;
  if (crc != le32toh(origcrc))
    {
      ferr("GUID Partition Table Header CRC is wrong: %" PRIx32
           " != %" PRIx32 "\n", crc, le32toh(origcrc));
      return -EINVAL;
    }

  gpt->header_crc32 = origcrc;

  /* Check that the my_lba entry points to the LBA that contains
   * the GUID Partition Table
   */

  if (le64toh(gpt->my_lba) != lba)
    {
      ferr("GPT: my_lba incorrect: %" PRIx64 " != %" PRIxOFF "\n",
           le64toh(gpt->my_lba), lba);
      return -EINVAL;
    }

  /* Check the first_usable_lba and last_usable_lba are within the disk. */

  lastlba = gpt_last_lba(state);
  if (le64toh(gpt->first_usable_lba) > lastlba)
    {
      ferr("GPT: first_usable_lba incorrect: %" PRId64 " > %" PRIdOFF "\n",
           le64toh(gpt->first_usable_lba), lastlba);
      return -EINVAL;
    }

  if (le64toh(gpt->last_usable_lba) > lastlba)
    {
      ferr("GPT: last_usable_lba incorrect: %" PRId64 " > %" PRIdOFF "\n",
           le64toh(gpt->last_usable_lba), lastlba);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: gpt_pte_is_valid()
 *
 * Description:
 *   tests one PTE for validity
 *
 * Input Parameters:
 *   pte is the pte to check
 *   lastlba is last lba of the disk
 *
 * Returned Value:
 *   Returns 1 if valid,  0 on error.
 *
 ****************************************************************************/

static inline int gpt_pte_is_valid(FAR const struct gpt_entry_s *pte,
                                   blkcnt_t lastlba)
{
  if (!memcmp(&pte->partition_type_guid, &g_null_guid,
              sizeof(g_null_guid)) ||
      le64toh(pte->starting_lba) > lastlba ||
      le64toh(pte->ending_lba) > lastlba)
    {
      return 0;
    }

  return 1;
}

static void gpt_part_set_name(FAR struct gpt_entry_s *pte,
                              FAR char *dest, size_t len)
{
  int i;

  if (--len > GPT_PARTNAME_MAX_SIZE)
    {
      len = GPT_PARTNAME_MAX_SIZE;
    }

  for (i = 0; i < len; i++)
    {
      uint8_t c = pte->partition_name[i];
      dest[i] = (c && !isprint(c)) ? '.' : c;
    }

  dest[i] = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: parse_gpt_partition
 *
 * Description:
 *   parse the gpt(EFI GUID Partition Table) partition.
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

int parse_gpt_partition(FAR struct partition_state_s *state,
                        partition_handler_t handler,
                        FAR void *arg)
{
  FAR struct legacy_partition_s *pmbr;
  FAR struct gpt_ptable_s *ptbl;
  FAR struct gpt_header_s *gpt;
  FAR struct gpt_entry_s *ptes;
  struct partition_s pentry;
  int nb_part;
  int count;
  int ret;

  /* Read GPT Ptable (LBA0 + LBA1) */

  count = (sizeof(struct gpt_ptable_s) + (state->blocksize - 1)) /
          state->blocksize;
  ptbl = kmm_malloc(count * state->blocksize);
  if (!ptbl)
    {
      return -ENOMEM;
    }

  ret = read_partition_block(state, ptbl, 0, count);
  if (ret < 0)
    {
      goto err;
    }

  /* Verify mbr is valid */

  pmbr = (FAR struct legacy_partition_s *)&ptbl->mbr[0x1be];
  if (pmbr->sys_ind != 0xee)
    {
      ret = -EINVAL;
      goto err;
    }

  /* Verify gpt header is valid */

  gpt = &(ptbl->u.gpt_header);
  ret = gpt_header_is_valid(state, gpt, 1);
  if (ret >= 0)
    {
      /* Verify gpt header is valid */

      ptes = gpt_alloc_verify_entries(state, gpt);
    }

  if (ret < 0 || !ptes)
    {
      /* Read and Verify backup gpt header is valid */

      finfo("Primary GPT is invalid, using alternate GPT.\n");

      count = (GPT_BLOCK_SIZE + state->blocksize - 1) / state->blocksize;
      ret = read_partition_block(state, ptbl,
                                 GPT_LBA_TO_BLOCK(gpt->alternate_lba,
                                 state->blocksize), count);
      if (ret < 0)
        {
          goto err;
        }

      gpt = (FAR struct gpt_header_s *)ptbl;
      ret = gpt_header_is_valid(state, gpt,
                                le64toh(gpt->alternate_lba));
      if (ret >= 0)
        {
          /* Verify gpt header is valid */

          ptes = gpt_alloc_verify_entries(state, gpt);
        }
    }

  if (ret < 0 || !ptes)
    {
      finfo("Alternate GPT is also invalid!!\n");
      goto err;
    }

  nb_part = le32toh(gpt->num_partition_entries);
  for (pentry.index = 0; pentry.index < nb_part; pentry.index++)
    {
      pentry.firstblock = GPT_LBA_TO_BLOCK(ptes[pentry.index].starting_lba,
                                           state->blocksize);
      pentry.nblocks = GPT_LBA_TO_BLOCK(ptes[pentry.index].ending_lba,
                                        state->blocksize) -
                       pentry.firstblock;
      pentry.blocksize = state->blocksize;
      gpt_part_set_name(&ptes[pentry.index], pentry.name,
                        sizeof(pentry.name));
      handler(&pentry, arg);
    }

  kmm_free(ptes);
err:
  kmm_free(ptbl);
  return ret;
}
