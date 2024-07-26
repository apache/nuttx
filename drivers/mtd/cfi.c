/****************************************************************************
 * drivers/mtd/cfi.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/param.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/arch.h>
#include <nuttx/nuttx.h>

#include "cfi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* type of port */

#define CFI_PORT_8BIT                      0x01
#define CFI_PORT_16BIT                     0x02
#define CFI_PORT_32BIT                     0X04
#define CFI_PORT_64BIT                     0x08

/* Common CFI value */

#define CFI_CMD                            0x98
#define CFI_QRY_OFFSET                     0x10
#define CFI_ERASE_BLOCK_REGION_INFO_OFFSET 0x2D

/* CFI command set number */

#define CFI_CMDSET_INTEL_EXTENDED          0x0001
#define CFI_CMDSET_AMD_STANDARD            0x0002
#define CFI_CMDSET_INTEL_STANDARD          0x0003
#define CFI_CMDSET_AMD_EXTENDED            0x0004
#define CFI_CMDSET_MITS_STANDARD           0x0100
#define CFI_CMDSET_MITS_EXTENDED           0x0101

/* Intel command value */

#define CFI_INTEL_CMD_RESET                0xFF
#define CFI_INTEL_CMD_CLEAR_STATUS         0x50
#define CFI_INTEL_CMD_BLOCK_ERASE          0x20
#define CFI_INTEL_CMD_ERASE_CONFIRM        0xD0
#define CFI_INTEL_CMD_WRITE                0x40
#define CFI_INTEL_CMD_WRITE_BUFFER         0xE8
#define CFI_INTEL_CMD_WRITE_CONFIRM        0xD0

/* Intel status value */

#define CFI_INTEL_STATUS_DONE              0x80
#define CFI_INTEL_STATUS_ESS               0x40 /* erase suspended */
#define CFI_INTEL_STATUS_ECLBS             0x20 /* clear block lock bit
                                                 * or erase error */
#define CFI_INTEL_STATUS_PSLBS             0X10 /* write or block lock bit
                                                 * set error */
#define CFI_INTEL_STATUS_VPENS             0x08 /* voltage range error */
#define CFI_INTEL_STATUS_PSS               0x04 /* write is suspended */
#define CFI_INTEL_STATUS_DPS               0x02 /* device protected error */

/* Amd command value */

#define CFI_AMD_CMD_RESET                  0xF0
#define CFI_AMD_CMD_UNLOCK1                0xAA
#define CFI_AMD_CMD_UNLOCK2                0x55
#define CFI_AMD_CMD_ERASE_START            0x80
#define CFI_AMD_CMD_ERASE_SECTOR           0x30
#define CFI_AMD_CMD_WRITE                  0xA0
#define CFI_AMD_CMD_WRITE_BUFFER           0x25
#define CFI_AMD_CMD_WRITE_CONFIRM          0x29

/* Amd status value */

#define CFI_AMD_STATUS_TOGGLE              0x40 /* DQ6 will toggle after a
                                                 * reading cycle during
                                                 * erasing */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Supports up to 64bits bank width */

typedef uint64_t cfiword_t;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* the offset of CFI query address is usually 0x55
 * Spansion S29WS-N CFI query fix is to try 0x555 if 0x55 fails
 */

static const uint32_t g_cfi_query_address[2] =
{
  0x55, 0x555
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cfi_get_block_addr
 *
 * Description:
 *   Get the start address of the block.
 *
 ****************************************************************************/

static uintptr_t cfi_get_block_addr(FAR struct cfi_dev_s *cfi,
                                    blkcnt_t block)
{
  uint8_t i;
  uintptr_t base = cfi->base_addr;

  for (i = 0; i < cfi->info.erase_region_num; i++)
    {
      blkcnt_t block_num = cfi_get_blocknum(cfi, i);
      size_t block_size = cfi_get_blocksize(cfi, i);

      if (block >= block_num)
        {
          block -= block_num;
          base += block_num * block_size;
          continue;
        }

      base += block * block_size;
      break;
    }

  return base;
}

/****************************************************************************
 * Name: cfi_addr_map
 *
 * Description:
 *   Get the address according to block number and offset, and needs to
 *   convert it according to bankwidth.
 *
 ****************************************************************************/

static uintptr_t cfi_addr_map(FAR struct cfi_dev_s *cfi, blkcnt_t block,
                              off_t offset)
{
  return cfi_get_block_addr(cfi, block) + cfi->bankwidth * offset;
}

/****************************************************************************
 * Name: cfi_make_cmd
 *
 * Description:
 *   Make command value according to bankwidth and device width. For example,
 *   a command 0x98 should be 0x00980098 for a flash with bankwidth = 4 and
 *   dev_width = 2.
 *
 ****************************************************************************/

static cfiword_t cfi_make_cmd(FAR struct cfi_dev_s *cfi, uint32_t cmd)
{
  cfiword_t result = 0;
  uint8_t i = cfi->bankwidth / cfi->dev_width;

  while (i--)
    {
      result = (result << (8 * cfi->dev_width)) | cmd;
    }

  return result;
}

/****************************************************************************
 * Name: cfi_add_byte
 *
 * Description:
 *   Add a byte to cfiword, which will be written into cfi-flash later.
 *
 ****************************************************************************/

static void cfi_add_byte(FAR struct cfi_dev_s *cfi, FAR cfiword_t *word,
                         uint8_t c)
{
  switch (cfi->bankwidth)
    {
      case CFI_PORT_8BIT:
        *word = c;
        break;
      case CFI_PORT_16BIT:
        *word = (*word >> 8) | (uint16_t)c << 8;
        break;
      case CFI_PORT_32BIT:
        *word = (*word >> 8) | (uint32_t)c << 24;
        break;
      default:
        *word = (*word >> 8) | (uint64_t)c << 56;
    }
}

/****************************************************************************
 * Name: cfi_write_data
 *
 * Description:
 *   Write a value to specified address.
 *
 ****************************************************************************/

static inline void cfi_write_data(FAR struct cfi_dev_s *cfi, uintptr_t addr,
                                  FAR const void *buffer)
{
  switch (cfi->bankwidth)
    {
      case CFI_PORT_8BIT:
        *(FAR volatile uint8_t *)addr = *(FAR const uint8_t *)buffer;
        break;
      case CFI_PORT_16BIT:
        *(FAR volatile uint16_t *)addr = *(FAR const uint16_t *)buffer;
        break;
      case CFI_PORT_32BIT:
        *(FAR volatile uint32_t *)addr = *(FAR const uint32_t *)buffer;
        break;
      default:
        *(FAR volatile uint64_t *)addr = *(FAR const uint64_t *)buffer;
    }
}

/****************************************************************************
 * Name: cfi_write_block_cmd
 *
 * Description:
 *   Write a command about specified block.
 *
 ****************************************************************************/

static void cfi_write_block_cmd(FAR struct cfi_dev_s *cfi, blkcnt_t block,
                                uint32_t cmd)
{
  cfiword_t val = cfi_make_cmd(cfi, cmd);

  cfi_write_data(cfi, cfi_addr_map(cfi, block, 0), &val);
}

/****************************************************************************
 * Name: cfi_write_address_cmd
 *
 * Description:
 *   Write a command to specified address.
 *
 ****************************************************************************/

static void cfi_write_address_cmd(FAR struct cfi_dev_s *cfi, uintptr_t addr,
                                  uint8_t cmd)
{
  cfiword_t val = cfi_make_cmd(cfi, cmd);

  cfi_write_data(cfi, cfi_addr_map(cfi, 0, addr), &val);
}

/****************************************************************************
 * Name: cfi_read_byte
 *
 * Description:
 *   Read an 8-bit value from addr.
 *
 ****************************************************************************/

static inline uint8_t cfi_read_byte(uintptr_t addr)
{
  return *(FAR volatile uint8_t *)addr;
}

/****************************************************************************
 * Name: cfi_read_data
 *
 * Description:
 *   Read a value to specified address.
 *
 ****************************************************************************/

static inline cfiword_t cfi_read_data(FAR struct cfi_dev_s *cfi,
                                      uintptr_t addr)
{
  switch (cfi->bankwidth)
    {
      case CFI_PORT_8BIT:
        return cfi_read_byte(addr);
      case CFI_PORT_16BIT:
        return *(FAR const volatile uint16_t *)addr;
      case CFI_PORT_32BIT:
        return *(FAR const volatile uint32_t *)addr;
      default:
        return *(FAR const volatile uint64_t *)addr;
    }
}

/****************************************************************************
 * Name: cfi_is_equal
 *
 * Description:
 *   Check whether the value corresponding to addr is equal to cmd.
 *
 ****************************************************************************/

static bool cfi_is_equal(FAR struct cfi_dev_s *cfi, uintptr_t addr,
                         uint8_t cmd)
{
  cfiword_t val = cfi_make_cmd(cfi, cmd);

  addr = cfi_addr_map(cfi, 0, addr);
  return cfi_read_data(cfi, addr) == val;
}

/****************************************************************************
 * Name: cfi_is_set
 *
 * Description:
 *   Check the status bit, which is set by cmd.
 *
 ****************************************************************************/

static bool cfi_is_set(FAR struct cfi_dev_s *cfi, uint8_t cmd)
{
  cfiword_t val = cfi_make_cmd(cfi, cmd);
  uintptr_t addr = cfi->base_addr;

  return (cfi_read_data(cfi, addr) & val) == val;
}

/****************************************************************************
 * Name: cfi_get_time_us
 *
 * Description:
 *   Check whether the value corresponding to offset is equal to cmd.
 *
 ****************************************************************************/

static inline uint32_t cfi_get_time_us(void)
{
  struct timespec ts;

  clock_systime_timespec(&ts);
  return 1000000 * ts.tv_sec + ts.tv_nsec / 1000;
}

/****************************************************************************
 * Name: cfi_is_timeout
 *
 * Description:
 *   Check whether the operation has timed out.
 *
 ****************************************************************************/

static bool cfi_is_timeout(uint32_t start_us, uint32_t tout_us)
{
  return (int32_t)(cfi_get_time_us() - start_us) >= tout_us;
}

/****************************************************************************
 * Name: cfi_intel_is_busy
 *
 * Description:
 *   Function of intel cfi command set.
 *   Check whether the cfi-flash is busy.
 *
 ****************************************************************************/

static bool cfi_intel_is_busy(FAR struct cfi_dev_s *cfi, blkcnt_t block)
{
  return !cfi_is_set(cfi, CFI_INTEL_STATUS_DONE);
}

/****************************************************************************
 * Name: cfi_intel_status_check
 *
 * Description:
 *   Function of intel cfi command set.
 *   Check the status register of the cfi-flash.
 *
 ****************************************************************************/

static int cfi_intel_status_check(FAR struct cfi_dev_s *cfi,
                                  blkcnt_t block, uint32_t tout)
{
  uint32_t start;

  /* wait for command completion */

  start = cfi_get_time_us();
  while (cfi_intel_is_busy(cfi, block))
    {
      if (!cfi_is_timeout(start, tout))
        {
          up_udelay(1);
          continue;
        }

      ferr("CFI flash operation has timed out.\n");

      /* Check each status bit */

      if (!cfi_is_equal(cfi, 0, CFI_INTEL_STATUS_DONE))
        {
          ferr("CFI flash operation failed.\n");
          if (cfi_is_set(cfi, CFI_INTEL_STATUS_ECLBS |
                         CFI_INTEL_STATUS_PSLBS))
            {
             ferr("CFI: command sequence error\n");
            }
          else if (cfi_is_set(cfi, CFI_INTEL_STATUS_ECLBS))
            {
             ferr("CFI: block erase error!\n");
            }
          else if (cfi_is_set(cfi, CFI_INTEL_STATUS_PSLBS))
            {
             ferr("CFI: locking error!\n");
            }
          else if (cfi_is_set(cfi, CFI_INTEL_STATUS_DPS))
            {
             ferr("CFI: block locked!\n");
            }
          else if (cfi_is_set(cfi, CFI_INTEL_STATUS_VPENS))
            {
             ferr("CFI: Vpp Low error!\n");
            }
        }

      cfi_reset(cfi);
      return -EAGAIN;
    }

  return 0;
}

/****************************************************************************
 * Name: cfi_intel_write_word
 *
 * Description:
 *   Function of intel cfi command set.
 *   Write data with the width of cfiword_t into cfi-flash.
 *
 ****************************************************************************/

static int cfi_intel_write_word(FAR struct cfi_dev_s *cfi,
                                uintptr_t addr, cfiword_t word)
{
  off_t offset = addr - cfi->base_addr;

  cfi_write_address_cmd(cfi, 0, CFI_INTEL_CMD_CLEAR_STATUS);
  cfi_write_address_cmd(cfi, offset / cfi->bankwidth, CFI_INTEL_CMD_WRITE);
  cfi_write_data(cfi, addr, &word);

  return cfi_intel_status_check(cfi, cfi_find_block(cfi, offset),
                                (1 << cfi->info.single_write_timeout_typ) *
                                (1 << cfi->info.single_write_timeout_max));
}

/****************************************************************************
 * Name: cfi_intel_write_buffer
 *
 * Description:
 *   Function of intel cfi command set.
 *   Write data from buffer into cfi-flash.
 *
 ****************************************************************************/

static int cfi_intel_write_buffer(FAR struct cfi_dev_s *cfi,
                                  uintptr_t addr, size_t nbytes,
                                  FAR const uint8_t *buffer)
{
  int ret;
  blkcnt_t block;
  off_t offset = addr - cfi->base_addr;

  block = cfi_find_block(cfi, offset);
  cfi_write_address_cmd(cfi, 0, CFI_INTEL_CMD_CLEAR_STATUS);
  cfi_write_block_cmd(cfi, block, CFI_INTEL_CMD_WRITE_BUFFER);
  ret = cfi_intel_status_check(cfi, block,
                               (1 << cfi->info.buffer_write_timeout_typ) *
                               (1 << cfi->info.buffer_write_timeout_max));
  if (ret < 0)
    {
      return ret;
    }

  addr = cfi->base_addr + offset;
  nbytes = nbytes / cfi->bankwidth;

  cfi_write_block_cmd(cfi, block, nbytes - 1);
  while (nbytes-- > 0)
    {
      cfi_write_data(cfi, addr, buffer);
      addr += cfi->bankwidth;
      buffer += cfi->bankwidth;
    }

  cfi_write_address_cmd(cfi, 0, CFI_INTEL_CMD_WRITE_CONFIRM);

  return cfi_intel_status_check(cfi, block,
                               (1 << cfi->info.buffer_write_timeout_typ) *
                               (1 << cfi->info.buffer_write_timeout_max));
}

/****************************************************************************
 * Name: cfi_intel_erase
 *
 * Description:
 *   Function of intel cfi command set.
 *   Erase one block.
 *
 ****************************************************************************/

static int cfi_intel_erase(FAR struct cfi_dev_s *cfi, blkcnt_t block)
{
  cfi_write_address_cmd(cfi, 0, CFI_INTEL_CMD_CLEAR_STATUS);
  cfi_write_block_cmd(cfi, block, CFI_INTEL_CMD_BLOCK_ERASE);
  cfi_write_address_cmd(cfi, 0, CFI_INTEL_CMD_ERASE_CONFIRM);
  return cfi_intel_status_check(cfi, block, 1000 *
                                (1 << cfi->info.block_erase_timeout_typ) *
                                (1 << cfi->info.block_erase_timeout_max));
}

/****************************************************************************
 * Name: cfi_amd_fix
 *
 * Description:
 *   Set the variables specific to AMD.
 *
 ****************************************************************************/

static void cfi_amd_fix(FAR struct cfi_dev_s *cfi)
{
  /* x8/x16 in x8 mode or x16/x32 in x16 mode */

  if ((cfi->dev_width == 1 && cfi->info.interface_desc == 0x02) ||
      (cfi->dev_width == 2 && cfi->info.interface_desc == 0x04))
    {
      cfi->unlock_addr1 = 0xaaa;
      cfi->unlock_addr2 = 0x555;
    }
  else
    {
      cfi->unlock_addr1 = 0x555;
      cfi->unlock_addr2 = 0x2aa;
    }
}

/****************************************************************************
 * Name: cfi_amd_unlock
 *
 * Description:
 *   Flash unlock command sequence specific to AMD.
 *
 ****************************************************************************/

static void cfi_amd_unlock(FAR struct cfi_dev_s *cfi)
{
  cfi_write_address_cmd(cfi, cfi->unlock_addr1, CFI_AMD_CMD_UNLOCK1);
  cfi_write_address_cmd(cfi, cfi->unlock_addr2, CFI_AMD_CMD_UNLOCK2);
}

/****************************************************************************
 * Name: cfi_amd_is_busy
 *
 * Description:
 *   Function of amd cfi command set.
 *   Check whether the cfi-flash is busy.
 *
 ****************************************************************************/

static bool cfi_amd_is_busy(FAR struct cfi_dev_s *cfi, blkcnt_t block)
{
  uintptr_t addr;

  addr = cfi_addr_map(cfi, block, 0);

  /* status bit DQ6 will toggle after a reading cycle during erasing */

  return ((cfi_read_byte(addr) ^ cfi_read_byte(addr)) &
          CFI_AMD_STATUS_TOGGLE) != 0;
}

/****************************************************************************
 * Name: cfi_amd_status_check
 *
 * Description:
 *   Function of amd cfi command set.
 *   Check the status register of the cfi-flash.
 *
 ****************************************************************************/

static int cfi_amd_status_check(FAR struct cfi_dev_s *cfi,
                                blkcnt_t block, uint32_t tout)
{
  uint32_t start;

  /* wait for command completion */

  start = cfi_get_time_us();
  while (cfi_amd_is_busy(cfi, block))
    {
      if (cfi_is_timeout(start, tout))
        {
          ferr("CFI flash operation has timed out.\n");
          cfi_reset(cfi);
          return -EAGAIN;
        }

      up_udelay(1);
    }

  return 0;
}

/****************************************************************************
 * Name: cfi_amd_write_word
 *
 * Description:
 *   Function of amd cfi command set.
 *   Write data with the width of cfiword_t into cfi-flash.
 *
 ****************************************************************************/

static int cfi_amd_write_word(FAR struct cfi_dev_s *cfi,
                              uintptr_t addr, cfiword_t word)
{
  off_t offset = addr - cfi->base_addr;

  cfi_amd_unlock(cfi);
  cfi_write_address_cmd(cfi, cfi->unlock_addr1, CFI_AMD_CMD_WRITE);
  cfi_write_data(cfi, addr, &word);

  return cfi_amd_status_check(cfi, cfi_find_block(cfi, offset),
                              (1 << cfi->info.single_write_timeout_typ) *
                              (1 << cfi->info.single_write_timeout_max));
}

/****************************************************************************
 * Name: cfi_amd_write_buffer
 *
 * Description:
 *   Function of amd cfi command set.
 *   Write data from buffer into cfi-flash.
 *
 ****************************************************************************/

static int cfi_amd_write_buffer(FAR struct cfi_dev_s *cfi, uintptr_t addr,
                                size_t nbytes, FAR const uint8_t *buffer)
{
  blkcnt_t block;
  off_t offset = addr - cfi->base_addr;

  block = cfi_find_block(cfi, offset);
  cfi_amd_unlock(cfi);
  cfi_write_block_cmd(cfi, block, CFI_AMD_CMD_WRITE_BUFFER);

  nbytes = nbytes / cfi->bankwidth;
  cfi_write_block_cmd(cfi, block, nbytes - 1);

  while (nbytes-- > 0)
    {
      cfi_write_data(cfi, addr, buffer);
      addr += cfi->bankwidth;
      buffer += cfi->bankwidth;
    }

  cfi_write_address_cmd(cfi, 0, CFI_AMD_CMD_WRITE_CONFIRM);
  return cfi_amd_status_check(cfi, block,
                              (1 << cfi->info.buffer_write_timeout_max) *
                              (1 << cfi->info.buffer_write_timeout_max));
}

/****************************************************************************
 * Name: cfi_amd_erase
 *
 * Description:
 *   Function of amd cfi command set.
 *   Erase one block.
 *
 ****************************************************************************/

static int cfi_amd_erase(FAR struct cfi_dev_s *cfi, blkcnt_t block)
{
  cfi_amd_unlock(cfi);
  cfi_write_address_cmd(cfi, cfi->unlock_addr1, CFI_AMD_CMD_ERASE_START);
  cfi_amd_unlock(cfi);
  cfi_write_block_cmd(cfi, block, CFI_AMD_CMD_ERASE_SECTOR);
  return cfi_amd_status_check(cfi, block, 1000 *
                              (1 << cfi->info.block_erase_timeout_typ) *
                              (1 << cfi->info.block_erase_timeout_max));
}

/****************************************************************************
 * Name: cfi_write_word
 *
 * Description:
 *   Write data with the width of cfiword_t into cfi-flash.
 *
 ****************************************************************************/

static int cfi_write_word(FAR struct cfi_dev_s *cfi, uintptr_t addr,
                          cfiword_t word)
{
  switch (cfi->info.p_id)
    {
      case CFI_CMDSET_INTEL_EXTENDED:
      case CFI_CMDSET_INTEL_STANDARD:
        return cfi_intel_write_word(cfi, addr, word);
      case CFI_CMDSET_AMD_STANDARD:
      case CFI_CMDSET_AMD_EXTENDED:
        return cfi_amd_write_word(cfi, addr, word);
      case CFI_CMDSET_MITS_STANDARD:
      case CFI_CMDSET_MITS_EXTENDED:
      default:
        return -EINVAL;
    }
}

/****************************************************************************
 * Name: cfi_write_buffer
 *
 * Description:
 *   Write data from buffer into cfi-flash.
 *
 ****************************************************************************/

static int cfi_write_buffer(FAR struct cfi_dev_s *cfi, uintptr_t addr,
                            size_t nbytes, FAR const uint8_t *buffer)
{
  switch (cfi->info.p_id)
    {
      case CFI_CMDSET_INTEL_EXTENDED:
      case CFI_CMDSET_INTEL_STANDARD:
        return cfi_intel_write_buffer(cfi, addr, nbytes, buffer);
      case CFI_CMDSET_AMD_STANDARD:
      case CFI_CMDSET_AMD_EXTENDED:
        return cfi_amd_write_buffer(cfi, addr, nbytes, buffer);
      case CFI_CMDSET_MITS_STANDARD:
      case CFI_CMDSET_MITS_EXTENDED:
      default:
        return -EINVAL;
    }
}

/****************************************************************************
 * Name: cfi_erase_one
 *
 * Description:
 *   Erase one block.
 *
 ****************************************************************************/

static int cfi_erase_one(FAR struct cfi_dev_s *cfi, blkcnt_t block)
{
  switch (cfi->info.p_id)
    {
      case CFI_CMDSET_INTEL_EXTENDED:
      case CFI_CMDSET_INTEL_STANDARD:
        return cfi_intel_erase(cfi, block);
      case CFI_CMDSET_AMD_STANDARD:
      case CFI_CMDSET_AMD_EXTENDED:
        return cfi_amd_erase(cfi, block);
      case CFI_CMDSET_MITS_STANDARD:
      case CFI_CMDSET_MITS_EXTENDED:
      default:
        return -EINVAL;
    }
}

static ssize_t cfi_write_unalign(FAR struct cfi_dev_s *cfi, off_t offset,
                                 size_t nbytes, FAR const uint8_t *buffer)
{
  uint8_t i;
  int ret;
  size_t delta;
  uintptr_t wp;
  cfiword_t word = 0;

  /* get lower aligned address */

  wp = offset & ~(cfi->bankwidth - 1);

  /* handle unaligned start */

  if ((delta = offset - wp) == 0)
    {
      return 0;
    }

  for (i = 0; i < delta; i++)
    {
      cfi_add_byte(cfi, &word, cfi_read_byte(wp + i));
    }

  delta = 0;
  for (; i < cfi->bankwidth && nbytes > 0; i++)
    {
      cfi_add_byte(cfi, &word, *buffer++);
      nbytes--;
      delta++;
    }

  for (; i < cfi->bankwidth; i++)
    {
      cfi_add_byte(cfi, &word, cfi_read_byte(wp + i));
    }

  ret = cfi_write_word(cfi, wp, word);
  return ret < 0 ? ret : delta;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cfi_get_blocksize
 *
 * Description:
 *   Get the size of block in the specified region.
 *
 ****************************************************************************/

size_t cfi_get_blocksize(FAR struct cfi_dev_s *cfi, uint8_t region)
{
  return cfi->dev_num * (cfi->info.erase_region_info[region] >> 16) * 256;
}

/****************************************************************************
 * Name: cfi_get_blocknum
 *
 * Description:
 *   Get the number of blocks in the specified region.
 *
 ****************************************************************************/

blkcnt_t cfi_get_blocknum(FAR struct cfi_dev_s *cfi, uint8_t region)
{
  return ((uint16_t)(cfi->info.erase_region_info[region])) + 1;
}

/****************************************************************************
 * Name: cfi_find_block
 *
 * Description:
 *   Get the block number corresponding to offset.
 *
 ****************************************************************************/

blkcnt_t cfi_find_block(FAR struct cfi_dev_s *cfi, off_t offset)
{
  uint8_t i;
  blkcnt_t ret = 0;

  for (i = 0; i < cfi->info.erase_region_num; i++)
    {
      blkcnt_t block_num = cfi_get_blocknum(cfi, i);
      size_t block_size = cfi_get_blocksize(cfi, i);

      if (block_num * block_size <= offset)
        {
          offset -= block_num * block_size;
          ret += block_num;
        }
      else
        {
          ret += offset / block_size;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: cfi_get_total_blocknum
 *
 * Description:
 *   Get total number of erase blocks, assuming that all erase blocks are the
 *   same size.
 *
 ****************************************************************************/

blkcnt_t cfi_get_total_blocknum(FAR struct cfi_dev_s *cfi)
{
  uint8_t i;
  blkcnt_t blocknum = 0;

  for (i = 0; i < cfi->info.erase_region_num; i++)
    {
      blocknum += cfi_get_blocknum(cfi, i) * cfi_get_blocksize(cfi, i) /
                  cfi_get_blocksize(cfi, 0);
    }

  return blocknum;
}

/****************************************************************************
 * Name: cfi_check
 *
 * Description:
 *   Check whether the device is a cfi device, if so, then get the cfi data.
 *
 ****************************************************************************/

int cfi_check(FAR struct cfi_dev_s *cfi)
{
  uint8_t i;
  uint8_t j;

  /* try different single device width which we don't know now */

  for (cfi->dev_width = 1; cfi->dev_width <= cfi->bankwidth;
       cfi->dev_width <<= 1)
    {
      cfi_reset(cfi);
      for (i = 0; i < nitems(g_cfi_query_address); i++)
        {
          cfi_write_address_cmd(cfi, g_cfi_query_address[i], CFI_CMD);

          /* try to get into cfi mode and read cfi data */

          if (cfi_is_equal(cfi, CFI_QRY_OFFSET, 'Q') &&
              cfi_is_equal(cfi, CFI_QRY_OFFSET + 1, 'R') &&
              cfi_is_equal(cfi, CFI_QRY_OFFSET + 2, 'Y'))
            {
              FAR struct cfi_info_s *info = &(cfi->info);
              FAR uint8_t *ptr = (FAR uint8_t *)info;

              for (j = CFI_QRY_OFFSET;
                   j < CFI_ERASE_BLOCK_REGION_INFO_OFFSET; j++)
                {
                  *ptr++ = cfi_read_byte(cfi_addr_map(cfi, 0, j));
                }

              while (j < CFI_ERASE_BLOCK_REGION_INFO_OFFSET + 4 *
                     info->erase_region_num)
                {
                  *ptr++ = cfi_read_byte(cfi_addr_map(cfi, 0, j++));
                }

              cfi->cfi_offset = g_cfi_query_address[i];
              cfi->dev_num = cfi->bankwidth / cfi->dev_width;
              cfi->page_size = (1 << info->max_write_bytes_num) *
                               cfi->dev_num;

              /* fix amd feature */

              if (info->p_id == CFI_CMDSET_AMD_STANDARD ||
                  info->p_id == CFI_CMDSET_AMD_EXTENDED)
                {
                  cfi_amd_fix(cfi);
                }

              /* exit cfi mode, enter into read array mode */

              cfi_reset(cfi);
              return 0;
            }
        }
    }

  /* cfi flash not found */

  return -EINVAL;
}

/****************************************************************************
 * Name: cfi_reset
 *
 * Description:
 *   Get into read array mode.
 *
 ****************************************************************************/

void cfi_reset(FAR struct cfi_dev_s *cfi)
{
  switch (cfi->info.p_id)
    {
      case CFI_CMDSET_INTEL_EXTENDED:
      case CFI_CMDSET_INTEL_STANDARD:
        cfi_write_address_cmd(cfi, 0, CFI_INTEL_CMD_CLEAR_STATUS);
        cfi_write_address_cmd(cfi, 0, CFI_INTEL_CMD_RESET);
        break;
      case CFI_CMDSET_AMD_STANDARD:
      case CFI_CMDSET_AMD_EXTENDED:
        cfi_write_address_cmd(cfi, 0, CFI_AMD_CMD_RESET);
    }
}

/****************************************************************************
 * Name: cfi_erase
 *
 * Description:
 *   Erase multi blocks.
 *
 ****************************************************************************/

int cfi_erase(FAR struct cfi_dev_s *cfi, blkcnt_t startblock,
              blkcnt_t blockcnt)
{
  blkcnt_t i;
  int ret = 0;

  for (i = startblock; i < startblock + blockcnt; i++)
    {
      ret = cfi_erase_one(cfi, i);
      if (ret < 0)
        {
          break;
        }
    }

  cfi_reset(cfi);
  return ret;
}

/****************************************************************************
 * Name: cfi_read
 *
 * Description:
 *   Read data from cfi-flash to buffer.
 *
 ****************************************************************************/

int cfi_read(FAR struct cfi_dev_s *cfi, off_t offset, size_t nbytes,
             FAR uint8_t *buffer)
{
  uintptr_t base = cfi->base_addr;

  memcpy(buffer, (FAR const uint8_t *)base + offset, nbytes);
  return 0;
}

/****************************************************************************
 * Name: cfi_write
 *
 * Description:
 *   Write data from buffer to cfi-flash.
 *
 ****************************************************************************/

int cfi_write(FAR struct cfi_dev_s *cfi, off_t offset, size_t nbytes,
              FAR const uint8_t *buffer)
{
  ssize_t ret;

  offset = cfi->base_addr + offset;
  ret = cfi_write_unalign(cfi, offset, nbytes, buffer);
  if (ret < 0)
    {
      goto out;
    }

  offset = ALIGN_UP(offset, cfi->bankwidth);
  nbytes -= ret;
  buffer += ret;

  /* handle the aligned part */

  /* if the device support buffer write */

  if (cfi->info.max_write_bytes_num != 0)
    {
      while (nbytes >= cfi->bankwidth)
        {
          size_t size = cfi->page_size - offset % cfi->page_size;

          if (size > nbytes)
            {
              size = nbytes;
            }

          ret = cfi_write_buffer(cfi, offset, size, buffer);
          if (ret < 0)
            {
              goto out;
            }

          offset += size;
          buffer += size;
          nbytes -= size;
        }
    }
  else
    {
      while (nbytes >= cfi->bankwidth)
        {
          cfiword_t word = 0;
          uint8_t i;

          for (i = 0; i < cfi->bankwidth; i++)
            {
              cfi_add_byte(cfi, &word, *buffer++);
            }

          ret = cfi_write_word(cfi, offset, word);
          if (ret < 0)
            {
              goto out;
            }

          offset += cfi->bankwidth;
          nbytes -= cfi->bankwidth;
        }
    }

  if (nbytes == 0)
    {
      goto out;
    }

  /* handle unaligned tail bytes */

  ret = cfi_write_unalign(cfi, offset, nbytes, buffer);

out:
  cfi_reset(cfi);
  return ret < 0 ? ret : 0;
}
