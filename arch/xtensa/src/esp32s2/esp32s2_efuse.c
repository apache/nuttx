/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_efuse.c
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
#include <nuttx/efuse/efuse.h>
#include <arch/esp32s2/chip.h>

#include "xtensa.h"
#include "hardware/esp32s2_soc.h"
#include "hardware/esp32s2_efuse.h"

#include "esp32s2_clockconfig.h"
#include "esp32s2_efuse.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EFUSE_CONF_WRITE   0x5a5a /* eFuse_pgm_op_ena, force no rd/wr dis. */
#define EFUSE_CONF_READ    0x5aa5 /* eFuse_read_op_ena, release force. */
#define EFUSE_CMD_PGM      0x02   /* Command to program. */
#define EFUSE_CMD_READ     0x01   /* Command to read. */
#define EFUSE_MAX_BLK_LEN  256    /* Max length of efuse block. */

#define MIN(a, b)          ((a) < (b) ? (a) : (b))

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_start_efuse_rdreg[] =
{
  EFUSE_RD_WR_DIS_REG,
  EFUSE_RD_MAC_SPI_SYS_0_REG,
  EFUSE_RD_SYS_DATA_PART1_0_REG,
  EFUSE_RD_USR_DATA0_REG,
  EFUSE_RD_KEY0_DATA0_REG,
  EFUSE_RD_KEY1_DATA0_REG,
  EFUSE_RD_KEY2_DATA0_REG,
  EFUSE_RD_KEY3_DATA0_REG,
  EFUSE_RD_KEY4_DATA0_REG,
  EFUSE_RD_KEY5_DATA0_REG,
  EFUSE_RD_SYS_DATA_PART2_0_REG
};

static uint32_t g_start_efuse_wrreg[] =
{
  EFUSE_PGM_DATA0_REG,
  EFUSE_PGM_CHECK_VALUE0_REG
};

/****************************************************************************
 * Private Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s2_efuse_set_timing
 *
 * Description:
 *   Modify both EFUSE_CLK_REG and EFUSE_DAC_CONF_REG
 *   for match ABP frequency in Hertz.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

static int esp32s2_efuse_set_timing(void)
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

  modifyreg32(EFUSE_DAC_CONF_REG, EFUSE_DAC_CLK_DIV, dac_clk_div);
  modifyreg32(EFUSE_CLK_REG, EFUSE_DAC_CLK_DIV, clk_sel0);
  modifyreg32(EFUSE_CLK_REG, EFUSE_DAC_CLK_DIV, clk_sel1);
  return OK;
}

/****************************************************************************
 * Name: esp32s2_efuse_get_mask
 *
 * Description:
 *   Return mask with required the number of ones with shift.
 *
 * Input Parameters:
 *   bit_count    - The number of bits required
 *   shift        - The shift of programmed as, '1' or '0'
 *
 * Returned Value:
 *   The mask with required the number of ones with shift.
 *
 ****************************************************************************/

static uint32_t esp32s2_efuse_get_mask(uint32_t bit_count, uint32_t shift)
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

/****************************************************************************
 * Name: esp32s2_efuse_get_reg_num
 *
 * Description:
 *   Returns the number of bits in the register.
 *
 * Input Parameters:
 *   bit_offset   - Start bit in block
 *   bit_count    - The number of bits required
 *   i_reg        - The register number in the block
 *
 * Returned Value:
 *   The register number in the array.
 *
 ****************************************************************************/

static int esp32s2_efuse_get_reg_num(int bit_offset,
                                     int bit_count, int i_reg)
{
  uint32_t bit_start = (bit_offset % EFUSE_MAX_BLK_LEN);
  int num_reg = i_reg + bit_start / 32;

  if (num_reg > (bit_start + bit_count - 1) / 32)
    {
      return -1;
    }

  return num_reg;
}

/****************************************************************************
 * Name: esp32s2_efuse_get_count_bits_in_reg
 *
 * Description:
 *   Returns the number of bits in the register.
 *
 * Input Parameters:
 *   bit_offset   - Start bit in block
 *   bit_count    - The number of bits required
 *   i_reg        - The register number in the block
 *
 * Returned Value:
 *   The number of bits in the register.
 *
 ****************************************************************************/

static int esp32s2_efuse_get_count_bits_in_reg(int bit_offset,
                                               int bit_count, int i_reg)
{
  int ret_count = 0;
  int num_reg = 0;
  int bit_start = (bit_offset % EFUSE_MAX_BLK_LEN);
  int last_used_bit = (bit_start + bit_count - 1);

  for (int num_bit = bit_start; num_bit <= last_used_bit; ++num_bit)
    {
      ++ret_count;
      if ((((num_bit + 1) % 32) == 0) || (num_bit == last_used_bit))
        {
          if (i_reg == num_reg)
            {
              return ret_count;
            }

          ++num_reg;
          ret_count = 0;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: esp32s2_efuse_get_field_size
 *
 * Description:
 *   Get the length of the field in bits.
 *
 * Input Parameters:
 *   field   - Pointer to the structure describing the efuse field
 *
 * Returned Value:
 *   The length of the field in bits.
 *
 ****************************************************************************/

static int esp32s2_efuse_get_field_size(const efuse_desc_t *field[])
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

/****************************************************************************
 * Name: esp32s2_efuse_check_range_of_bits
 *
 * Description:
 *   Check range of bits for any coding scheme.
 *
 * Input Parameters:
 *   offset_in_bits   - The bit offset related to beginning of efuse
 *   size_bits        - The length of bit field
 *
 * Returned Value:
 *   True is returned if the bits offset matched. Otherwise false.
 *
 ****************************************************************************/

static bool esp32s2_efuse_check_range_of_bits(int offset_in_bits,
                                              int size_bits)
{
  int blk_offset = offset_in_bits % EFUSE_MAX_BLK_LEN;
  int max_num_bit = blk_offset + size_bits;

  if (max_num_bit > EFUSE_MAX_BLK_LEN)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: esp32s2_efuse_get_number_of_items
 *
 * Description:
 *   Returns the number of array elements for placing these bits in an array
 *   with the length of each element equal to size_of_base.
 *
 * Input Parameters:
 *   bits               - The number of bits required
 *   size_of_base       - The base of bits required
 *
 * Returned Value:
 *   The number of array elements.
 *
 ****************************************************************************/

static int esp32s2_efuse_get_number_of_items(int bits, int size_of_base)
{
  return bits / size_of_base + (bits % size_of_base > 0 ? 1 : 0);
}

/****************************************************************************
 * Name: esp32s2_efuse_fill_reg
 *
 * Description:
 *   Fill efuse register from array.
 *
 * Input Parameters:
 *   bit_start_in_reg       - Start bit in block
 *   bit_count_in_reg       - The number of bits required to write
 *   blob                   - A pointer that will contain the value
 *   filled_bits_blob       - A pointer that will contain the bits counter
 *
 * Returned Value:
 *   The value to write efuse register.
 *
 ****************************************************************************/

static uint32_t esp32s2_efuse_fill_reg(int bit_start_in_reg,
                                       int bit_count_in_reg,
                                       uint8_t *blob, int *filled_bits_blob)
{
  uint32_t reg_to_write = 0;
  uint32_t temp_blob_32;
  int shift_reg;
  int shift_bit = (*filled_bits_blob) % 8;

  if (shift_bit != 0)
    {
      temp_blob_32 = blob[(*filled_bits_blob) / 8] >> shift_bit;
      shift_bit = MIN((8 - shift_bit), bit_count_in_reg);

      reg_to_write = temp_blob_32 & esp32s2_efuse_get_mask(shift_bit, 0);
      (*filled_bits_blob) += shift_bit;
      bit_count_in_reg -= shift_bit;
    }

  shift_reg = shift_bit;

  while (bit_count_in_reg > 0)
    {
      temp_blob_32 = blob[(*filled_bits_blob) / 8];
      shift_bit = MIN(bit_count_in_reg, 8);
      reg_to_write |= (temp_blob_32 & \
                       esp32s2_efuse_get_mask(shift_bit, 0)) << shift_reg;
      (*filled_bits_blob) += shift_bit;
      bit_count_in_reg -= shift_bit;
      shift_reg += 8;
    };

  return reg_to_write << bit_start_in_reg;
}

/****************************************************************************
 * Name: esp32s2_efuse_process
 *
 * Description:
 *   Processes the field by calling the passed function.
 *
 * Input Parameters:
 *   field            - A pointer to describing the fields of efuse
 *   ptr              - A pointer to array that will contain the result
 *   ptr_size_bits    - The number of bits required to read
 *   func_proc        - A callback for handle the efuse field register
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

static int esp32s2_efuse_process(const efuse_desc_t *field[], void *ptr,
                                 size_t ptr_size_bits,
                                 efuse_func_proc_t func_proc)
{
  int err = OK;
  int bits_counter = 0;
  int field_len;
  int req_size;
  int i = 0;

  /* get and check size */

  field_len = esp32s2_efuse_get_field_size(field);
  req_size = (ptr_size_bits == 0) ? field_len : \
              MIN(ptr_size_bits, field_len);

  while (err == OK && req_size > bits_counter && field[i] != NULL)
    {
      int i_reg = 0;
      int num_reg;

      if (esp32s2_efuse_check_range_of_bits(field[i]->bit_offset,
                              field[i]->bit_count) == false)
        {
          minfo("Range of data does not match the coding scheme");
          err = -EINVAL;
        }

      while (err == OK && req_size > bits_counter &&
             (num_reg = esp32s2_efuse_get_reg_num(field[i]->bit_offset,
                                    field[i]->bit_count, i_reg)) != -1)
        {
          int num_bits = esp32s2_efuse_get_count_bits_in_reg(
                                               field[i]->bit_offset,
                                               field[i]->bit_count,
                                               i_reg);
          int bit_offset = field[i]->bit_offset;

          if ((bits_counter + num_bits) > req_size)
            {
              /* Limits the length of the field */

              num_bits = req_size - bits_counter;
            }

          err = func_proc(num_reg, bit_offset, num_bits, ptr, &bits_counter);
          ++i_reg;
        }

      i++;
    }

  DEBUGASSERT(bits_counter <= req_size);
  return err;
}

/****************************************************************************
 * Name: esp32s2_efuse_write_reg
 *
 * Description:
 *   Write value to efuse register.
 *
 * Input Parameters:
 *   blk          - Block number of eFuse
 *   num_reg      - The register number in the block
 *   value        - Value to write
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void esp32s2_efuse_write_reg(uint32_t blk, uint32_t num_reg,
                                    uint32_t value)
{
  uint32_t addr_wr_reg;
  uint32_t reg_to_write;
  uint32_t blk_start = g_start_efuse_wrreg[blk];

  DEBUGASSERT(blk >= 0 && blk < EFUSE_BLK_MAX);

  DEBUGASSERT(num_reg <= 7);

  /* The block 0 and register 7 doesn't exist */

  if (blk == 0 && num_reg == 7)
    {
      merr("Block 0 Register 7 doesn't exist!\n");
      return;
    }

  addr_wr_reg = blk_start + num_reg * 4;
  reg_to_write = getreg32(addr_wr_reg) | value;

  /* The register can be written in parts so we combine the new value
   * with the one already available.
   */

  putreg32(reg_to_write, addr_wr_reg);
}

/****************************************************************************
 * Name: esp32s2_efuse_write_blob
 *
 * Description:
 *   Fill registers from array for writing.
 *
 * Input Parameters:
 *   num_reg      - The register number in the block
 *   bit_offset   - Start bit in block
 *   bit_count    - The number of bits required to read
 *   arr_in       - A pointer to array that will contain the value of writing
 *   bits_counter - A pointer that will contain the bits counter of writing
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

static int esp32s2_efuse_write_blob(uint32_t num_reg, int bit_offset,
                                    int bit_count, void *arr_in,
                                    int *bits_counter)
{
  uint32_t block = (bit_offset / EFUSE_MAX_BLK_LEN);
  uint32_t bit_start = (bit_offset % EFUSE_MAX_BLK_LEN);
  uint32_t reg_to_write = esp32s2_efuse_fill_reg(bit_start, bit_count,
                                                 (uint8_t *) arr_in,
                                                 bits_counter);

  esp32s2_efuse_write_reg(block, num_reg, reg_to_write);

  return OK;
}

/****************************************************************************
 * Name: esp32s2_efuse_read_reg
 *
 * Description:
 *   Read efuse register.
 *
 * Input Parameters:
 *   blk          - Block number of eFuse
 *   num_reg      - The register number in the block
 *
 * Returned Value:
 *   Return the value in the efuse register.
 *
 ****************************************************************************/

static uint32_t esp32s2_efuse_read_reg(uint32_t blk, uint32_t num_reg)
{
  DEBUGASSERT(blk >= 0 && blk < EFUSE_BLK_MAX);
  uint32_t value;
  uint32_t blk_start = g_start_efuse_rdreg[blk];

  DEBUGASSERT(num_reg <= 7);

  value = getreg32(blk_start + num_reg * 4);
  return value;
}

/****************************************************************************
 * Name: esp32s2_efuse_fill_buff
 *
 * Description:
 *   Read efuse register and write this value to array.
 *
 * Input Parameters:
 *   num_reg      - The register number in the block
 *   bit_offset   - Start bit in block
 *   bit_count    - The number of bits required to read
 *   arr_out      - A pointer to array that will contain the result
 *   bits_counter - A pointer that will contain the bits counter of reading
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

static int esp32s2_efuse_fill_buff(uint32_t num_reg, int bit_offset,
                                   int bit_count, void *arr_out,
                                   int *bits_counter)
{
  uint8_t *blob = (uint8_t *) arr_out;
  uint32_t efuse_block = (bit_offset / EFUSE_MAX_BLK_LEN);
  uint32_t bit_start = (bit_offset % EFUSE_MAX_BLK_LEN);
  uint32_t reg = esp32s2_efuse_read_reg(efuse_block, num_reg);
  uint64_t reg_of_aligned_bits = (reg >> bit_start) & \
                                  esp32s2_efuse_get_mask(bit_count, 0);
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s2_efuse_read_field
 *
 * Description:
 *   Read value from EFUSE, writing it into an array.
 *
 * Input Parameters:
 *   field          - A pointer to describing the fields of efuse
 *   dst            - A pointer to array that contains the data for reading
 *   dst_size_bits  - The number of bits required to read
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32s2_efuse_read_field(const efuse_desc_t *field[], void *dst,
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
             esp32s2_efuse_get_number_of_items(dst_size_bits, 8));

      err = esp32s2_efuse_process(field, dst, dst_size_bits,
                                  esp32s2_efuse_fill_buff);
    }

  return err;
}

/****************************************************************************
 * Name: esp32s2_efuse_write_field
 *
 * Description:
 *   Write array to EFUSE.
 *
 * Input Parameters:
 *   field          - A pointer to describing the fields of efuse
 *   src            - A pointer to array that contains the data for writing
 *   src_size_bits  - The number of bits required to write
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32s2_efuse_write_field(const efuse_desc_t *field[],
                              const void *src, size_t src_size_bits)
{
  int  err = OK;

  if (field == NULL || src == NULL || src_size_bits == 0)
    {
      err = -EINVAL;
    }
  else
    {
      err = esp32s2_efuse_process(field, (void *)src, src_size_bits,
                                  esp32s2_efuse_write_blob);
    }

  return err;
}

/****************************************************************************
 * Name: esp32s2_efuse_burn_efuses
 *
 * Description:
 *   Burn values written to the efuse write registers.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s2_efuse_burn_efuses(void)
{
  esp32s2_efuse_set_timing();

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

