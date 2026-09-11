/****************************************************************************
 * arch/arm/src/stm32h5/stm32_efuse.c
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
 * The STM32H5 one-time-programmable area exposed through the NuttX efuse
 * interface.
 *
 * All the hardware access -- including the ICACHE/ECC NMI handling a blank
 * OTP word needs -- lives in stm32_otp_word_read16()/write16()
 * (stm32h563xx_flash.c).  This file is just the efuse_ops_s adapter: it
 * packs/unpacks the field descriptors' bit ranges into and out of those
 * two word-level primitives, the same shape as stm32_flash_edata_*() is
 * to the MTD driver in stm32_edata.c.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/efuse/efuse.h>

#include "stm32_flash.h"
#include "stm32_efuse.h"

#ifdef CONFIG_STM32H5_EFUSE

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_efuse_read_field(FAR struct efuse_lowerhalf_s *lower,
                                  FAR const efuse_desc_t *field[],
                                  FAR uint8_t *data, size_t bit_size);
static int stm32_efuse_write_field(FAR struct efuse_lowerhalf_s *lower,
                                   FAR const efuse_desc_t *field[],
                                   FAR const uint8_t *data, size_t bit_size);
static int stm32_efuse_ioctl(FAR struct efuse_lowerhalf_s *lower, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct efuse_ops_s g_stm32_efuse_ops =
{
  .read_field  = stm32_efuse_read_field,
  .write_field = stm32_efuse_write_field,
  .ioctl       = stm32_efuse_ioctl,
};

static struct efuse_lowerhalf_s g_stm32_efuse_lower =
{
  .ops = &g_stm32_efuse_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_efuse_field_bits
 *
 * Description:
 *   Total number of bits described by a NULL terminated field list.
 *
 ****************************************************************************/

static size_t stm32_efuse_field_bits(FAR const efuse_desc_t *field[])
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
 * Name: stm32_efuse_check_field
 *
 * Description:
 *   Verify that every descriptor lies inside the OTP bit address space.
 *
 ****************************************************************************/

static int stm32_efuse_check_field(FAR const efuse_desc_t *field[])
{
  int i;

  for (i = 0; field[i] != NULL; i++)
    {
      if ((size_t)field[i]->bit_offset + field[i]->bit_count >
          STM32_OTP_TOTAL_BITS)
        {
          ferr("ERROR: field %d [%u,+%u) is outside the OTP\n", i,
               field[i]->bit_offset, field[i]->bit_count);
          return -EINVAL;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_efuse_read_field
 *
 * Description:
 *   Read the bits named by the field list.  The bits are packed towards
 *   the start of the caller's buffer: the first bit of the first
 *   descriptor lands in bit 0 of data[0], the next in bit 1, and so on
 *   across descriptor boundaries.
 *
 ****************************************************************************/

static int stm32_efuse_read_field(FAR struct efuse_lowerhalf_s *lower,
                                  FAR const efuse_desc_t *field[],
                                  FAR uint8_t *data, size_t bit_size)
{
  uint32_t cached_word = UINT32_MAX;
  uint16_t cached_val = 0;
  size_t written = 0;
  size_t request;
  int ret;
  int i;

  if (field == NULL || data == NULL)
    {
      return -EINVAL;
    }

  ret = stm32_efuse_check_field(field);
  if (ret < 0)
    {
      return ret;
    }

  request = stm32_efuse_field_bits(field);
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
          uint32_t word = flat / STM32_OTP_WORD_BITS;

          if (word != cached_word)
            {
              ret = stm32_otp_word_read16(word, &cached_val);
              if (ret < 0 && ret != -ENODATA)
                {
                  return ret;
                }

              cached_word = word;
            }

          if ((cached_val & (1u << (flat % STM32_OTP_WORD_BITS))) != 0)
            {
              data[written / 8] |= 1u << (written % 8);
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Name: stm32_efuse_write_field
 *
 * Description:
 *   Program the bits named by the field list, taking the data in the same
 *   packed layout that stm32_efuse_read_field() produces.
 *
 *   This is destructive and irreversible.  Programming happens a whole
 *   16-bit word at a time, via stm32_otp_word_write16(), which is also
 *   where a word that already holds a conflicting value is rejected.
 *
 ****************************************************************************/

static int stm32_efuse_write_field(FAR struct efuse_lowerhalf_s *lower,
                                   FAR const efuse_desc_t *field[],
                                   FAR const uint8_t *data, size_t bit_size)
{
#ifndef CONFIG_STM32H5_OTP_WRITE
  /* Programming is not built in.  Refuse rather than silently doing
   * nothing, so a caller cannot mistake this for a successful burn.
   */

  return -EPERM;
#else
  uint32_t word = UINT32_MAX;
  uint16_t value = 0;
  bool dirty = false;
  size_t consumed = 0;
  size_t request;
  int ret;
  int i;

  if (field == NULL || data == NULL)
    {
      return -EINVAL;
    }

  ret = stm32_efuse_check_field(field);
  if (ret < 0)
    {
      return ret;
    }

  request = stm32_efuse_field_bits(field);
  if (bit_size != 0 && bit_size < request)
    {
      request = bit_size;
    }

  for (i = 0; field[i] != NULL && consumed < request; i++)
    {
      size_t bit;

      for (bit = 0; bit < field[i]->bit_count && consumed < request;
           bit++, consumed++)
        {
          size_t   flat     = field[i]->bit_offset + bit;
          uint32_t new_word = flat / STM32_OTP_WORD_BITS;
          size_t   wordbit  = flat % STM32_OTP_WORD_BITS;

          if (new_word != word)
            {
              if (dirty)
                {
                  ret = stm32_otp_word_write16(word, value);
                  if (ret < 0)
                    {
                      return ret;
                    }
                }

              /* Seed "value" with the word's current contents, so bits
               * this field does not touch are preserved -- blank reads
               * back as 0xffff, which is exactly the starting point a
               * never-written word needs.
               */

              ret = stm32_otp_word_read16(new_word, &value);
              if (ret < 0 && ret != -ENODATA)
                {
                  return ret;
                }

              word  = new_word;
              dirty = false;
            }

          if ((data[consumed / 8] & (1u << (consumed % 8))) == 0)
            {
              value &= ~(1u << wordbit);
              dirty = true;
            }
        }
    }

  if (dirty)
    {
      ret = stm32_otp_word_write16(word, value);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
#endif /* CONFIG_STM32H5_OTP_WRITE */
}

/****************************************************************************
 * Name: stm32_efuse_ioctl
 ****************************************************************************/

static int stm32_efuse_ioctl(FAR struct efuse_lowerhalf_s *lower, int cmd,
                             unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_efuse_initialize
 ****************************************************************************/

int stm32_efuse_initialize(FAR const char *devpath)
{
  FAR void *handle;

  handle = efuse_register(devpath, &g_stm32_efuse_lower);
  if (handle == NULL)
    {
      ferr("ERROR: failed to register the OTP at %s\n", devpath);
      return -ENODEV;
    }

  return OK;
}

#endif /* CONFIG_STM32H5_EFUSE */
