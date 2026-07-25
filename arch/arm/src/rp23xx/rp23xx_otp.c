/****************************************************************************
 * arch/arm/src/rp23xx/rp23xx_otp.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
 * The RP2350 one-time-programmable memory exposed through the NuttX efuse
 * interface.
 *
 * The OTP is an array of 4096 rows of 24 bits.  Software normally uses the
 * ECC interpretation, where 16 of those bits carry data and the remaining 8
 * carry a Hamming code, giving single-bit correction and double-bit
 * detection.  The chip maps that interpretation into the address space at
 * RP23XX_OTP_DATA_BASE, one halfword per row, so a read is just a halfword
 * load.  This driver uses that window, and therefore presents the OTP as a
 * flat bit address space of 4096 * 16 bits, which is what the efuse field
 * descriptors index into.
 *
 * Reading is free of side effects.  Programming is not: a bit can only ever
 * go from zero to one, and with ECC in use a row can only be programmed
 * once, because the Hamming code is computed over the whole 16 bit value.
 * Programming goes through the bootrom, which owns the programming voltage
 * and timing, and is compiled out unless CONFIG_RP23XX_OTP_WRITE is set.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/efuse/efuse.h>

#include "arm_internal.h"
#include "rp23xx_otp.h"
#include "hardware/rp23xx_memorymap.h"
#include "hardware/rp23xx_otp.h"

#ifdef CONFIG_RP23XX_OTP_WRITE
/* Only the programming path needs the bootrom lookup helpers */

#  include "rp23xx_rom.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Software lock bits, per page of 64 rows.  The two bit field encodes
 * 0 = read and write, 1 = read only, 2 and 3 = inaccessible.  NuttX runs in
 * the Secure state on this part, so the SEC field is the one that applies.
 */

#define OTP_SW_LOCK_SEC_SHIFT    0
#define OTP_SW_LOCK_NO_WRITE     1
#define OTP_SW_LOCK_NO_READ      2

/* Bootrom otp_access() command word: row in bits 15:0, then the direction
 * and the ECC translation.
 */

#define OTP_CMD_ROW_MASK         0x0000ffff
#define OTP_CMD_IS_WRITE         (1 << 16)
#define OTP_CMD_IS_ECC           (1 << 17)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The bootrom entry point used for programming */

typedef int (*rom_otp_access_fn)(FAR uint8_t *buf, uint32_t buf_len,
                                 uint32_t cmd);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int rp23xx_otp_read_field(FAR struct efuse_lowerhalf_s *lower,
                                 FAR const efuse_desc_t *field[],
                                 FAR uint8_t *data, size_t bit_size);
static int rp23xx_otp_write_field(FAR struct efuse_lowerhalf_s *lower,
                                  FAR const efuse_desc_t *field[],
                                  FAR const uint8_t *data, size_t bit_size);
static int rp23xx_otp_ioctl(FAR struct efuse_lowerhalf_s *lower, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct efuse_ops_s g_rp23xx_otp_ops =
{
  .read_field  = rp23xx_otp_read_field,
  .write_field = rp23xx_otp_write_field,
  .ioctl       = rp23xx_otp_ioctl,
};

static struct efuse_lowerhalf_s g_rp23xx_otp_lower =
{
  .ops = &g_rp23xx_otp_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_otp_page_lock
 *
 * Description:
 *   Return the Secure software lock state of the page holding a row.
 *
 ****************************************************************************/

static uint32_t rp23xx_otp_page_lock(uint32_t row)
{
  uint32_t page = row / RP23XX_OTP_PAGE_ROWS;
  uint32_t lock = getreg32(RP23XX_OTP_SW_LOCK(page));

  return (lock & RP23XX_OTP_SW_LOCK_SEC_MASK) >> OTP_SW_LOCK_SEC_SHIFT;
}

/****************************************************************************
 * Name: rp23xx_otp_read_row
 *
 * Description:
 *   Read one row through the ECC window.  Reading a page that is locked
 *   against reads raises a bus fault, so the lock is checked first and
 *   reported as an error instead.
 *
 * Input Parameters:
 *   row   - Row number, 0 to RP23XX_OTP_NUM_ROWS - 1
 *   value - Receives the 16 bits of data held in the row
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int rp23xx_otp_read_row(uint32_t row, FAR uint16_t *value)
{
  if (row >= RP23XX_OTP_NUM_ROWS)
    {
      return -EINVAL;
    }

  if (rp23xx_otp_page_lock(row) >= OTP_SW_LOCK_NO_READ)
    {
      finfo("OTP row %" PRIu32 " is in a page locked against reads\n", row);
      return -EPERM;
    }

  *value = getreg16(RP23XX_OTP_DATA_BASE + row * sizeof(uint16_t));
  return OK;
}

/****************************************************************************
 * Name: rp23xx_otp_field_bits
 *
 * Description:
 *   Total number of bits described by a NULL terminated field list.
 *
 ****************************************************************************/

static size_t rp23xx_otp_field_bits(FAR const efuse_desc_t *field[])
{
  size_t bits = 0;
  int i;

  for (i = 0; field[i] != NULL; i++)
    {
      bits += field[i]->bit_count;
    }

  return bits;
}

/****************************************************************************
 * Name: rp23xx_otp_check_field
 *
 * Description:
 *   Verify that every descriptor lies inside the OTP bit address space.
 *
 ****************************************************************************/

static int rp23xx_otp_check_field(FAR const efuse_desc_t *field[])
{
  int i;

  for (i = 0; field[i] != NULL; i++)
    {
      if ((size_t)field[i]->bit_offset + field[i]->bit_count >
          RP23XX_OTP_TOTAL_BITS)
        {
          ferr("ERROR: field %d [%u,+%u) is outside the OTP\n", i,
               field[i]->bit_offset, field[i]->bit_count);
          return -EINVAL;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: rp23xx_otp_read_field
 *
 * Description:
 *   Read the bits named by the field list.  The bits are packed towards the
 *   start of the caller's buffer: the first bit of the first descriptor
 *   lands in bit 0 of data[0], the next in bit 1, and so on across
 *   descriptor boundaries.
 *
 ****************************************************************************/

static int rp23xx_otp_read_field(FAR struct efuse_lowerhalf_s *lower,
                                 FAR const efuse_desc_t *field[],
                                 FAR uint8_t *data, size_t bit_size)
{
  uint32_t cached_row = UINT32_MAX;
  uint16_t cached_val = 0;
  size_t written = 0;
  size_t request;
  int ret;
  int i;

  if (field == NULL || data == NULL)
    {
      return -EINVAL;
    }

  ret = rp23xx_otp_check_field(field);
  if (ret < 0)
    {
      return ret;
    }

  request = rp23xx_otp_field_bits(field);
  if (bit_size != 0 && bit_size < request)
    {
      request = bit_size;
    }

  memset(data, 0, (request + 7) / 8);

  for (i = 0; field[i] != NULL && written < request; i++)
    {
      size_t bit;

      for (bit = 0; bit < field[i]->bit_count && written < request;
           bit++, written++)
        {
          size_t   flat = field[i]->bit_offset + bit;
          uint32_t row  = flat / RP23XX_OTP_ROW_BITS;

          if (row != cached_row)
            {
              ret = rp23xx_otp_read_row(row, &cached_val);
              if (ret < 0)
                {
                  return ret;
                }

              cached_row = row;
            }

          if ((cached_val & (1u << (flat % RP23XX_OTP_ROW_BITS))) != 0)
            {
              data[written / 8] |= 1u << (written % 8);
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: rp23xx_otp_write_field
 *
 * Description:
 *   Program the bits named by the field list, taking the data in the same
 *   packed layout that rp23xx_otp_read_field() produces.
 *
 *   This is destructive and irreversible.  Each affected row is programmed
 *   as a whole through the bootrom, because the ECC bits are computed over
 *   the whole row, which also means a row that has already been programmed
 *   cannot be modified: such a write is rejected here rather than left to
 *   corrupt the row's ECC.
 *
 ****************************************************************************/

static int rp23xx_otp_write_field(FAR struct efuse_lowerhalf_s *lower,
                                  FAR const efuse_desc_t *field[],
                                  FAR const uint8_t *data, size_t bit_size)
{
#ifndef CONFIG_RP23XX_OTP_WRITE
  /* Programming is not built in.  Refuse rather than silently doing
   * nothing, so a caller cannot mistake this for a successful burn.
   */

  return -EPERM;
#else
  rom_otp_access_fn otp_access;
  size_t consumed = 0;
  size_t request;
  int ret;
  int i;

  if (field == NULL || data == NULL)
    {
      return -EINVAL;
    }

  ret = rp23xx_otp_check_field(field);
  if (ret < 0)
    {
      return ret;
    }

  otp_access = (rom_otp_access_fn)rom_func_lookup(ROM_FUNC_OTP_ACCESS);
  if (otp_access == NULL)
    {
      ferr("ERROR: the bootrom has no OTP access function\n");
      return -ENOTSUP;
    }

  request = rp23xx_otp_field_bits(field);
  if (bit_size != 0 && bit_size < request)
    {
      request = bit_size;
    }

  /* Walk the field list a row at a time, building the value to program */

  for (i = 0; field[i] != NULL && consumed < request; i++)
    {
      size_t bit = 0;

      while (bit < field[i]->bit_count && consumed < request)
        {
          size_t   flat    = field[i]->bit_offset + bit;
          uint32_t row     = flat / RP23XX_OTP_ROW_BITS;
          uint16_t value = 0;
          uint16_t current;

          /* Collect every bit of this field that falls in this row */

          while (bit < field[i]->bit_count && consumed < request &&
                 (field[i]->bit_offset + bit) / RP23XX_OTP_ROW_BITS == row)
            {
              size_t pos = (field[i]->bit_offset + bit) %
                           RP23XX_OTP_ROW_BITS;

              if ((data[consumed / 8] & (1u << (consumed % 8))) != 0)
                {
                  value |= 1u << pos;
                }

              bit++;
              consumed++;
            }

          if (rp23xx_otp_page_lock(row) >= OTP_SW_LOCK_NO_WRITE)
            {
              ferr("ERROR: OTP row %" PRIu32 " is locked against writes\n",
                   row);
              return -EPERM;
            }

          ret = rp23xx_otp_read_row(row, &current);
          if (ret < 0)
            {
              return ret;
            }

          if ((uint16_t)(current | value) == current)
            {
              /* Every bit requested is already programmed, so there is
               * nothing to do for this row.
               */

              continue;
            }

          /* Anything else asked of a row that already holds data would need
           * its ECC recomputed, and the array can only set bits, never clear
           * them.  Refuse here rather than handing the bootrom a write that
           * would leave the row's ECC inconsistent.  Note this catches a
           * field covering the whole row as well as a partial one: any
           * non-blank row is off limits.
           */

          if (current != 0)
            {
              ferr("ERROR: OTP row %" PRIu32 " already programmed (0x%04x), "
                   "its ECC cannot be recomputed\n", row, current);
              return -EROFS;
            }

          ret = otp_access((FAR uint8_t *)&value, sizeof(value),
                           (row & OTP_CMD_ROW_MASK) | OTP_CMD_IS_WRITE |
                           OTP_CMD_IS_ECC);
          if (ret < 0)
            {
              ferr("ERROR: bootrom refused to program row %" PRIu32 ": %d\n",
                   row, ret);
              return -EIO;
            }
        }
    }

  return OK;
#endif /* CONFIG_RP23XX_OTP_WRITE */
}

/****************************************************************************
 * Name: rp23xx_otp_ioctl
 ****************************************************************************/

static int rp23xx_otp_ioctl(FAR struct efuse_lowerhalf_s *lower, int cmd,
                            unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rp23xx_otp_initialize
 *
 * Description:
 *   Register the OTP as an efuse character device.
 *
 ****************************************************************************/

int rp23xx_otp_initialize(FAR const char *devpath)
{
  FAR void *handle;

  handle = efuse_register(devpath, &g_rp23xx_otp_lower);
  if (handle == NULL)
    {
      ferr("ERROR: failed to register the OTP at %s\n", devpath);
      return -ENODEV;
    }

  return OK;
}
