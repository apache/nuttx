/****************************************************************************
 * arch/xtensa/src/esp32/esp32_efuse_utils.c
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
#include <errno.h>
#include <assert.h>
#include <string.h>
#include <nuttx/efuse/esp_efuse.h>

#include "xtensa.h"
#include "esp32_efuse.h"
#include "esp32_clockconfig.h"
#include "hardware/efuse_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EFUSE_CONF_WRITE   0x5a5a /* eFuse_pgm_op_ena, force no rd/wr dis. */
#define EFUSE_CONF_READ    0x5aa5 /* eFuse_read_op_ena, release force. */
#define EFUSE_CMD_PGM      0x02   /* Command to program. */
#define EFUSE_CMD_READ     0x01   /* Command to read. */

#define MIN(a, b)          ((a) < (b) ? (a) : (b))

/* Range addresses to read blocks */

const esp_efuse_range_addr_t range_read_addr_blocks[] =
{
  {
    /* range address of EFUSE_BLK0 */

    EFUSE_BLK0_RDATA0_REG, EFUSE_BLK0_RDATA6_REG
  },
  {
    /* range address of EFUSE_BLK1 */

    EFUSE_BLK1_RDATA0_REG, EFUSE_BLK1_RDATA7_REG
  },
  {
    /* range address of EFUSE_BLK2 */

    EFUSE_BLK2_RDATA0_REG, EFUSE_BLK2_RDATA7_REG
  },
  {
    /* range address of EFUSE_BLK3 */

    EFUSE_BLK3_RDATA0_REG, EFUSE_BLK3_RDATA7_REG
  }
};

/* Range addresses to write blocks */

const esp_efuse_range_addr_t range_write_addr_blocks[] =
{
  {
    /* range address of EFUSE_BLK0 */

    EFUSE_BLK0_WDATA0_REG, EFUSE_BLK0_WDATA6_REG
  },
  {
    /* range address of EFUSE_BLK1 */

    EFUSE_BLK1_WDATA0_REG, EFUSE_BLK1_WDATA7_REG
  },
  {
    /* range address of EFUSE_BLK2 */

    EFUSE_BLK2_WDATA0_REG, EFUSE_BLK2_WDATA7_REG
  },
  {
    /* range address of EFUSE_BLK3 */

    EFUSE_BLK3_WDATA0_REG, EFUSE_BLK3_WDATA7_REG
  }
};

void esp_efuse_write_reg(uint32_t blk, uint32_t num_reg, uint32_t value);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int esp_efuse_set_timing(void)
{
  uint32_t apb_freq_mhz = esp_clk_apb_freq() / 1000000;
  uint32_t clk_sel0;
  uint32_t clk_sel1;
  uint32_t dac_clk_div;

  if (apb_freq_mhz <= 26)
    {
      clk_sel0 = 250;
      clk_sel1 = 255;
      dac_clk_div = 52;
    }
  else
    {
      if (apb_freq_mhz <= 40)
        {
          clk_sel0 = 160;
          clk_sel1 = 255;
          dac_clk_div = 80;
        }
      else
        {
          clk_sel0 = 80;
          clk_sel1 = 128;
          dac_clk_div = 100;
        }
    }

  REG_SET_FIELD(EFUSE_DAC_CONF_REG, EFUSE_DAC_CLK_DIV, dac_clk_div);
  REG_SET_FIELD(EFUSE_CLK_REG, EFUSE_CLK_SEL0, clk_sel0);
  REG_SET_FIELD(EFUSE_CLK_REG, EFUSE_CLK_SEL1, clk_sel1);
  return OK;
}

/* efuse read operation: copies data from physical efuses to efuse read
 * registers.
 */

void esp_efuse_clear_program_regs(void)
{
  putreg32(EFUSE_CONF_READ, EFUSE_CONF_REG);
}

/* Burn values written to the efuse write registers */

void esp_efuse_burn_efuses(void)
{
  esp_efuse_set_timing();

  /* Permanently update values written to the efuse write registers */

  putreg32(EFUSE_CONF_WRITE, EFUSE_CONF_REG);
  putreg32(EFUSE_CMD_PGM, EFUSE_CMD_REG);

  while (getreg32(EFUSE_CMD_REG) != 0)
    {
    };

  putreg32(EFUSE_CONF_READ, EFUSE_CONF_REG);
  putreg32(EFUSE_CMD_READ, EFUSE_CMD_REG);

  while (getreg32(EFUSE_CMD_REG) != 0)
    {
    };
}

/* Reset efuse */

void esp_efuse_reset(void)
{
}

/* return mask with required the number of ones with shift */

static uint32_t get_mask(uint32_t bit_count, uint32_t shift)
{
  uint32_t mask;

  if (bit_count != 32)
    {
      mask = (1 << bit_count) - 1;
    }
  else
    {
      mask = 0xffffffff;
    }

  return mask << shift;
}

/* return the register number in the array
 * return -1 if all registers for field was selected
 */

static int get_reg_num(int bit_start, int bit_count, int i_reg)
{
  int num_reg = i_reg + bit_start / 32;

  if (num_reg > (bit_start + bit_count - 1) / 32)
    {
      return -1;
    }

  return num_reg;
}

/* returns the starting bit number in the register */

static int get_starting_bit_num_in_reg(int bit_start, int i_reg)
{
  return (i_reg == 0) ? bit_start % 32 : 0;
}

/* Returns the number of bits in the register */

static int get_count_bits_in_reg(int bit_start, int bit_count, int i_reg)
{
  int ret_count = 0;
  int num_reg = 0;
  int last_used_bit = (bit_start + bit_count - 1);

  for (int num_bit = bit_start; num_bit <= last_used_bit; ++num_bit)
    {
      ++ret_count;
      if ((((num_bit + 1) % 32) == 0) || (num_bit == last_used_bit))
        {
          if (i_reg == num_reg++)
            {
              return ret_count;
            }

          ret_count = 0;
        }
    }

  return 0;
}

/* get the length of the field in bits */

int esp_efuse_get_field_size(const esp_efuse_desc_t *field[])
{
  int bits_counter = 0;

  if (field != NULL)
    {
      int i = 0;

      while (field[i] != NULL)
        {
          bits_counter += field[i]->bit_count;
          ++i;
        }
    }

  return bits_counter;
}

/* check range of bits for any coding scheme */

static bool check_range_of_bits(esp_efuse_block_t blk,
                                int offset_in_bits, int size_bits)
{
  int max_num_bit = offset_in_bits + size_bits;

  if (max_num_bit > 256)
    {
      return false;
    }

  return true;
}

/* Returns the number of array elements for placing these bits in an array
 * with the length of each element equal to size_of_base.
 */

int esp_efuse_get_number_of_items(int bits, int size_of_base)
{
  return  bits / size_of_base + (bits % size_of_base > 0 ? 1 : 0);
}

/* fill efuse register from array */

static uint32_t fill_reg(int bit_start_in_reg, int bit_count_in_reg,
                         uint8_t *blob, int *filled_bits_blob)
{
  uint32_t reg_to_write = 0;
  uint32_t temp_blob_32;
  int shift_reg;
  int shift_bit = (*filled_bits_blob) % 8;

  if (shift_bit != 0)
    {
      temp_blob_32 = blob[(*filled_bits_blob) / 8] >> shift_bit;
      shift_bit = ((8 - shift_bit) < bit_count_in_reg) ?
                   (8 - shift_bit) : bit_count_in_reg;

      reg_to_write = temp_blob_32 & get_mask(shift_bit, 0);
      (*filled_bits_blob) += shift_bit;
      bit_count_in_reg -= shift_bit;
    }

  shift_reg = shift_bit;

  while (bit_count_in_reg > 0)
    {
      temp_blob_32 = blob[(*filled_bits_blob) / 8];
      shift_bit = (bit_count_in_reg > 8) ? 8 : bit_count_in_reg;
      reg_to_write |= (temp_blob_32 & get_mask(shift_bit, 0)) << shift_reg;
      (*filled_bits_blob) += shift_bit;
      bit_count_in_reg -= shift_bit;
      shift_reg += 8;
    };

  return reg_to_write << bit_start_in_reg;
}

/* This function processes the field by calling the passed function */

int esp_efuse_process(const esp_efuse_desc_t *field[], void *ptr,
                      size_t ptr_size_bits, efuse_func_proc_t func_proc)
{
  int err = OK;
  int bits_counter = 0;
  int field_len;
  int req_size;
  int i = 0;

  /* get and check size */

  field_len = esp_efuse_get_field_size(field);
  req_size = (ptr_size_bits == 0) ? field_len : \
              MIN(ptr_size_bits, field_len);

  while (err == OK && req_size > bits_counter && field[i] != NULL)
    {
      int i_reg = 0;
      int num_reg;

      if (check_range_of_bits(field[i]->efuse_block, field[i]->bit_start,
                              field[i]->bit_count) == false)
        {
          minfo("Range of data does not match the coding scheme");
          err = -EINVAL;
        }

      while (err == OK && req_size > bits_counter &&
             (num_reg = get_reg_num(field[i]->bit_start,
                                    field[i]->bit_count, i_reg)) != -1)
        {
          int start_bit = get_starting_bit_num_in_reg(field[i]->bit_start,
                                                      i_reg);
          int num_bits = get_count_bits_in_reg(field[i]->bit_start,
                                               field[i]->bit_count,
                                               i_reg);

          if ((bits_counter + num_bits) > req_size)
            {
              /* Limits the length of the field */

              num_bits = req_size - bits_counter;
            }

          minfo("EFUSE_BLK%d__DATA%d_REG is used %d bits starting"
                " with %d bit", (int)field[i]->efuse_block, num_reg,
                num_bits, start_bit);

          err = func_proc(num_reg, field[i]->efuse_block, start_bit,
                          num_bits, ptr, &bits_counter);
          ++i_reg;
        }

      i++;
    }

  DEBUGASSERT(bits_counter <= req_size);
  return err;
}

/* Fill registers from array for writing */

int esp_efuse_write_blob(uint32_t num_reg, uint32_t block, int bit_start,
                         int bit_count, void *arr_in, int *bits_counter)
{
  uint32_t reg_to_write = fill_reg(bit_start, bit_count, (uint8_t *) arr_in,
                                   bits_counter);

  esp_efuse_write_reg(block, num_reg, reg_to_write);

  return OK;
}

/* Read efuse register */

uint32_t esp_efuse_read_reg(uint32_t blk, uint32_t num_reg)
{
  DEBUGASSERT(blk >= 0 && blk < EFUSE_BLK_MAX);
  uint32_t value;
  uint32_t max_num_reg = (range_read_addr_blocks[blk].end -
                          range_read_addr_blocks[blk].start) /
                          sizeof(uint32_t);

  DEBUGASSERT(num_reg <= max_num_reg);

  value = getreg32(range_read_addr_blocks[blk].start + num_reg * 4);
  return value;
}

/* Read efuse register and write this value to array. */

int esp_efuse_fill_buff(uint32_t num_reg, uint32_t efuse_block,
                        int bit_start, int bit_count, void *arr_out,
                        int *bits_counter)
{
  uint8_t *blob = (uint8_t *) arr_out;
  uint32_t reg = esp_efuse_read_reg(efuse_block, num_reg);
  uint64_t reg_of_aligned_bits = (reg >> bit_start) & get_mask(bit_count, 0);
  int sum_shift = 0;
  int shift_bit = (*bits_counter) % 8;

  if (shift_bit != 0)
    {
      blob[(*bits_counter) / 8] |= (uint8_t)(reg_of_aligned_bits << \
                                   shift_bit);
      shift_bit = ((8 - shift_bit) < bit_count) ? (8 - shift_bit) : \
                  bit_count;
      (*bits_counter) += shift_bit;
      bit_count -= shift_bit;
    }

  while (bit_count > 0)
    {
      sum_shift += shift_bit;
      blob[(*bits_counter) / 8] |= (uint8_t)(reg_of_aligned_bits >> \
                                   sum_shift);
      shift_bit = (bit_count > 8) ? 8 : bit_count;
      (*bits_counter) += shift_bit;
      bit_count -= shift_bit;
    };

  return OK;
}

/* Write efuse register */

void esp_efuse_write_reg(uint32_t blk, uint32_t num_reg, uint32_t value)
{
  unsigned int max_num_reg;
  uint32_t addr_wr_reg;
  uint32_t reg_to_write;

  DEBUGASSERT(blk >= 0 && blk < EFUSE_BLK_MAX);

  max_num_reg = (range_read_addr_blocks[blk].end -
                 range_read_addr_blocks[blk].start) /
                 sizeof(uint32_t);

  DEBUGASSERT(num_reg <= max_num_reg);

  addr_wr_reg = range_write_addr_blocks[blk].start + num_reg * 4;
  reg_to_write = getreg32(addr_wr_reg) | value;

  /* The register can be written in parts so we combine the new value
   * with the one already available.
   */

  putreg32(reg_to_write, addr_wr_reg);
}

/* Read value from EFUSE, writing it into an array */

int esp_efuse_read_field_blob(const esp_efuse_desc_t *field[], void *dst,
                              size_t dst_size_bits)
{
  int err = OK;

  if (field == NULL || dst == NULL || dst_size_bits == 0)
    {
      err = -EINVAL;
    }
  else
    {
      memset((uint8_t *)dst, 0,
             esp_efuse_get_number_of_items(dst_size_bits, 8));

      err = esp_efuse_process(field, dst, dst_size_bits,
                              esp_efuse_fill_buff);
    }

  return err;
}

/* Write array to EFUSE */

int esp_efuse_write_field_blob(const esp_efuse_desc_t *field[],
                               const void *src, size_t src_size_bits)
{
  int  err = OK;

  if (field == NULL || src == NULL || src_size_bits == 0)
    {
      err = -EINVAL;
    }
  else
    {
      err = esp_efuse_process(field, (void *)src, src_size_bits,
                              esp_efuse_write_blob);
    }

  return err;
}

/* This function reads the key from the efuse block, starting at the offset
 * and the required size.
 */

int esp_efuse_read_block(uint32_t blk, void *dst_key, size_t offset_in_bits,
                         size_t size_bits)
{
  int err = OK;

  if (blk == EFUSE_BLK0 || blk >= EFUSE_BLK_MAX || dst_key == NULL ||
      size_bits == 0)
    {
      err = -EINVAL;
    }
  else
    {
      const esp_efuse_desc_t field_desc[] =
        {
          {
            blk, offset_in_bits, size_bits
          },
        };

      const esp_efuse_desc_t *field[] =
        {
          &field_desc[0],
          NULL
        };

      err = esp_efuse_read_field_blob(field, dst_key, size_bits);
    }

  return err;
}

/* Write a key to a block */

int esp_efuse_write_block(uint32_t blk, const void *src_key,
                          size_t offset_in_bits, size_t size_bits)
{
  int err = OK;

  if (blk == EFUSE_BLK0 || blk >= EFUSE_BLK_MAX || \
      src_key == NULL || size_bits == 0)
    {
      err = -EINVAL;
    }
  else
    {
      const esp_efuse_desc_t field_desc[] =
        {
          {
            blk, offset_in_bits, size_bits
          },
        };

      const esp_efuse_desc_t *field[] =
        {
          &field_desc[0],
          NULL
        };

      err = esp_efuse_write_field_blob(field, src_key, size_bits);
    }

  return err;
}

