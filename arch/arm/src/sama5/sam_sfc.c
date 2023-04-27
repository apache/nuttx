/****************************************************************************
 * arch/arm/src/sama5/sam_sfc.c
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <sys/param.h>

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/efuse/efuse.h>
#include <nuttx/efuse/sama5_sfc_fuses.h>
#include <nuttx/wdog.h>

#include "arm_internal.h"
#include "sam_sfc.h"
#include "hardware/sam_sfc.h"

#ifdef CONFIG_SAMA5_SFC

#ifdef ATSAMA5D4
#warning SAMA5 SFC functions have NOT been checked on a board using SAMA5D4
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SAM_SFC_DR_LEN 32             /* Each data register is 32 bits */

#define SFC_WDOG_DELAY MSEC2TICK(100) /* exact value not important.
                                       * This is to prevent getting stuck
                                       * while burning fuses. */

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/* This structure describes the state of the upper half driver */

struct sama5_sfc_upperhalf_s
{
  mutex_t   lock;                  /* Supports mutual exclusion */
  char *path;                      /* Registration path */
  struct efuse_lowerhalf_s *lower; /* Pointer to efuse_lowerhalf_s */
};

/****************************************************************************
 * Name: sam_sfc_func_proc_t
 *
 * Description:
 *   This is type of function that will handle the sfc efuse field register.
 *
 * Input Parameters:
 *   num_reg          - The register number.
 *   bit_start        - Start bit in the register.
 *   bit_count        - The number of bits used in the register.
 *   arr              - A pointer to an array or variable.
 *   bits_counter     - Counter bits.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

typedef int (*sam_sfc_func_proc_t)(uint32_t num_reg,
                                   int bit_start,
                                   int bit_count,
                                   void *arr, int *bits_counter);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sama5_sfc_lower_ioctl(struct efuse_lowerhalf_s *lower,
                                 int cmd, unsigned long arg);
static int sama5_sfc_lower_read(struct efuse_lowerhalf_s *lower,
                                const efuse_desc_t *field[],
                                uint8_t *data,
                                size_t bits_len);
static int sama5_sfc_lower_write(struct efuse_lowerhalf_s *lower,
                                 const efuse_desc_t *field[],
                                 const uint8_t *data,
                                 size_t bits_len);
static int sam_sfc_get_field_size(const efuse_desc_t *field[]);
static bool sam_sfc_check_range_of_bits(int offset_in_bits, int size_bits);
static int sam_sfc_get_number_of_items(int bits, int size_of_base);
static int sam_sfc_fill_buff(uint32_t num_reg, int bit_offset,
                             int bit_count, void *arr_out,
                             int *bits_counter);
static int sam_sfc_process(const efuse_desc_t *field[], void *ptr,
                           size_t ptr_size_bits,
                           sam_sfc_func_proc_t func_proc);
static int sam_sfc_get_reg_num(int bit_offset, int bit_count, int i_reg);
static int sam_sfc_get_count_bits_in_reg(int bit_offset, int bit_count,
                                         int i_reg);
static uint32_t sam_sfc_read_reg(uint32_t num_reg);
static void sam_sfc_mask_read(void);
static uint32_t sam_sfc_get_mask(uint32_t bit_count, uint32_t shift);
static int sama5_sfc_burn_efuses(const efuse_desc_t *field[],
                                 const void *src, size_t src_size_bits);
static int  sama5_sfc_write_reg(uint32_t num_reg, uint32_t value);
static int sama5_sfc_write_blob(uint32_t num_reg, int bit_offset,
                                int bit_count, void *arr_in,
                                int *bits_counter);
static uint32_t sama5_sfc_fill_reg(int bit_start_in_reg,
                                   int bit_count_in_reg, uint8_t *blob,
                                   int *filled_bits_blob);
static void sfc_timeout(wdparm_t arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* EFUSE lower-half driver methods */

static const struct efuse_ops_s sama5_sfc_lower_ops =
{
  .read_field   = sama5_sfc_lower_read,
  .write_field  = sama5_sfc_lower_write,
  .ioctl        = sama5_sfc_lower_ioctl,
};

static struct efuse_lowerhalf_s g_sama5_sfc_lowerhalf_s =
{
  .ops = &sama5_sfc_lower_ops,
};

static uint32_t g_start_sam_sfc_reg[SFC_DR_END] =
{
  SAM_SFC_DR(0),
  SAM_SFC_DR(1),
  SAM_SFC_DR(2),
  SAM_SFC_DR(3),
  SAM_SFC_DR(4),
  SAM_SFC_DR(5),
  SAM_SFC_DR(6),
  SAM_SFC_DR(7),
  SAM_SFC_DR(8),
  SAM_SFC_DR(9),
  SAM_SFC_DR(10),
  SAM_SFC_DR(11),
  SAM_SFC_DR(12),
  SAM_SFC_DR(13),
  SAM_SFC_DR(14),
#ifdef ATSAMA5D2
  SAM_SFC_DR(15),
#endif
};

struct wdog_s wdog; /* watchdog timer for efuse burning */

bool waiting; /* Waiting for efuse burning to be done  */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sama5_sfc_lower_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 * Description:
 *   Initialize the efuse driver. The efuse is initialized
 *   and registered as 'devpath'.
 *
 * Input Parameters:
 *   lower        - A pointer the publicly visible representation of
 *                  the "lower-half" driver state structure
 *   cmd          - The ioctl command value
 *   arg          - The optional argument that accompanies the 'cmd'
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -ENOTTY.
 *
 ****************************************************************************/

static int sama5_sfc_lower_ioctl(struct efuse_lowerhalf_s *lower,
                                 int cmd,
                                 unsigned long arg)
{
  switch (cmd)
    {
      /* We don't have proprietary EFUSE ioctls */

      case EFUSEIOC_SAMA5_MASK:
        {
          minfo("Masking fuses register \n");
          sam_sfc_mask_read();
          return OK;
        }
        break;
      default:
        {
          minfo("Unrecognized cmd: %d\n", cmd);
          return -ENOTTY;
        }
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: sam_sfc_get_field_size
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

static int sam_sfc_get_field_size(const efuse_desc_t *field[])
{
  int bits_counter = 0;
  int i;

  if (field != NULL)
    {
      i = 0;

      while (field[i] != NULL)
        {
          bits_counter += field[i]->bit_count;
          ++i;
        }
    }

  return bits_counter;
}

/****************************************************************************
 * Name: sam_sfc_check_range_of_bits
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

static bool sam_sfc_check_range_of_bits(int offset_in_bits, int size_bits)
{
  int bit_offset = offset_in_bits % SAM_SFC_EFUSE_MAX_LEN;
  int max_num_bit = bit_offset + size_bits;

  if (max_num_bit > SAM_SFC_EFUSE_MAX_LEN)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: sam_sfc_get_number_of_items
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

static int sam_sfc_get_number_of_items(int bits, int size_of_base)
{
  return bits / size_of_base + (bits % size_of_base > 0 ? 1 : 0);
}

/****************************************************************************
 * Name: sam_sfc_get_reg_num
 *
 * Description:
 *   Returns the number of bits in the register.
 *
 * Input Parameters:
 *   bit_offset   - Start bit in register
 *   bit_count    - The number of bits required
 *   i_reg        - The register number
 *
 * Returned Value:
 *   The register number in the array.
 *
 ****************************************************************************/

static int sam_sfc_get_reg_num(int bit_offset, int bit_count, int i_reg)
{
  uint32_t bit_start = (bit_offset % SAM_SFC_EFUSE_MAX_LEN);
  int num_reg = i_reg + bit_start / SAM_SFC_DR_LEN;

  if (num_reg > (bit_start + bit_count - 1) / SAM_SFC_DR_LEN)
    {
      return -1;
    }

  return num_reg;
}

/****************************************************************************
 * Name: sam_sfc_get_count_bits_in_reg
 *
 * Description:
 *   Returns the number of bits in the register.
 *
 * Input Parameters:
 *   bit_offset   - Start bit in register
 *   bit_count    - The number of bits required
 *   i_reg        - The register number
 *
 * Returned Value:
 *   The number of bits in the register.
 *
 ****************************************************************************/

static int sam_sfc_get_count_bits_in_reg(int bit_offset, int bit_count,
                                         int i_reg)
{
  int ret_count = 0;
  int num_reg = 0;
  int bit_start = (bit_offset % SAM_SFC_DR_LEN);
  int last_used_bit = (bit_start + bit_count - 1);
  int num_bit;

  for (num_bit = bit_start; num_bit <= last_used_bit; ++num_bit)
    {
      ++ret_count;
      if ((((num_bit + 1) % SAM_SFC_DR_LEN) == 0) ||
          (num_bit == last_used_bit))
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
 * Name: sam_sfc_process
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

static int sam_sfc_process(const efuse_desc_t *field[], void *ptr,
                           size_t ptr_size_bits,
                           sam_sfc_func_proc_t func_proc)
{
  int err = OK;
  int bits_counter = 0;
  int field_len;
  int req_size;
  int i = 0;
  int i_reg;
  int num_reg;
  int num_bits;
  int bit_offset;

  /* get and check size */

  field_len = sam_sfc_get_field_size(field);
  req_size = (ptr_size_bits == 0) ? field_len : \
              MIN(ptr_size_bits, field_len);

  while (err == OK && req_size > bits_counter && field[i] != NULL)
    {
      i_reg = 0;

      if (sam_sfc_check_range_of_bits(field[i]->bit_offset,
                              field[i]->bit_count) == false)
        {
          minfo("Range of data does not match the coding scheme");
          err = -EINVAL;
        }

      while (err == OK && req_size > bits_counter &&
             (num_reg = sam_sfc_get_reg_num(field[i]->bit_offset,
              field[i]->bit_count, i_reg)) != -1)
        {
          num_bits = sam_sfc_get_count_bits_in_reg(field[i]->bit_offset,
                                                   field[i]->bit_count,
                                                   i_reg);
          bit_offset = field[i]->bit_offset;

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
 * Name: sama5_sfc_fill_reg
 *
 * Description:
 *   Fill efuse register from array.
 *
 * Input Parameters:
 *   bit_start_in_reg       - Start bit in register
 *   bit_count_in_reg       - The number of bits required to write
 *   blob                   - A pointer that will contain the value
 *   filled_bits_blob       - A pointer that will contain the bits counter
 *
 * Returned Value:
 *   The value to write efuse register.
 *
 ****************************************************************************/

static uint32_t sama5_sfc_fill_reg(int bit_start_in_reg,
                                   int bit_count_in_reg, uint8_t *blob,
                                   int *filled_bits_blob)
{
  uint32_t reg_to_write = 0;
  uint32_t temp_blob_32;
  int shift_reg;
  int shift_bit = (*filled_bits_blob) % 8;

  if (shift_bit != 0)
    {
      temp_blob_32 = blob[(*filled_bits_blob) / 8] >> shift_bit;
      shift_bit = MIN((8 - shift_bit), bit_count_in_reg);

      reg_to_write = temp_blob_32 & sam_sfc_get_mask(shift_bit, 0);
      (*filled_bits_blob) += shift_bit;
      bit_count_in_reg -= shift_bit;
    }

  shift_reg = shift_bit;

  while (bit_count_in_reg > 0)
    {
      temp_blob_32 = blob[(*filled_bits_blob) / 8];
      shift_bit = MIN(bit_count_in_reg, 8);
      reg_to_write |= (temp_blob_32 & \
                       sam_sfc_get_mask(shift_bit, 0)) << shift_reg;
      (*filled_bits_blob) += shift_bit;
      bit_count_in_reg -= shift_bit;
      shift_reg += 8;
    };

  return reg_to_write << bit_start_in_reg;
}

/****************************************************************************
 * Name: sfc_timeout
 *
 * Description:
 *   Called if the efuse burn watchdog times out
 *
 * Input Parameters: none
 *
 * Returned Value: none
 *
 ****************************************************************************/

static void sfc_timeout(wdparm_t arg)
{
  waiting = false;
}

/****************************************************************************
 * Name: sama5_sfc_write_reg
 *
 * Description:
 *   Write value to be written to efuse register, preceded by magic number
 *   to allow the write to occur.
 *
 * Input Parameters:
 *   num_reg      - The register number in the block
 *   value        - Value to write
 *
 * Returned Value:
 *   Success or error code.
 *
 ****************************************************************************/

static int sama5_sfc_write_reg(uint32_t num_reg, uint32_t value)
{
  uint32_t regval;
  int ret;

  DEBUGASSERT(num_reg < SFC_DR_END);

  /* The register can be written in parts so we combine the new value
   * with the one already available, if the new value can actually
   * be programmed
   */

  regval = getreg32(g_start_sam_sfc_reg[num_reg]);

  if (regval == value)
    {
      /* it's the same! */

      return OK;
    }

  /* Write Key Code value then the value to be burned */

  putreg32(SAM_SFC_KEYCODE, SAM_SFC_KR);
  putreg32(regval | value, g_start_sam_sfc_reg[num_reg]);

  waiting = true;
  ret = wd_start(&wdog, SFC_WDOG_DELAY, sfc_timeout, (wdparm_t)0);
  if (ret < 0)
    {
      merr("ERROR: wd_start failed: %d\n", ret);
    }

  while (waiting)
    {
      if (getreg32(SAM_SFC_SR) & SAM_SFC_SR_PGMC)
        {
          break;
        }
    }

  wd_cancel(&wdog);

  if (!waiting)
    {
      /* we got here because of a watchdog timeout */

      merr("ERROR: efuse burning timed out\n");
      return -ETIMEDOUT;
    }

  regval = getreg32(SAM_SFC_SR);
  if (regval & SAM_SFC_SR_PGMF)
    {
      /* There was an internal error burning the fuses */

      merr("ERROR: Error burning efuses\n");

      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: sama5_sfc_write_blob
 *
 * Description:
 *   Fill register from array and write.
 *
 * Input Parameters:
 *   num_reg      - The register number
 *   bit_offset   - Start bit in register
 *   bit_count    - The number of bits required to read
 *   arr_in       - A pointer to array that will contain the value of writing
 *   bits_counter - A pointer that will contain the bits counter of writing
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -ERROR.
 *
 ****************************************************************************/

static int sama5_sfc_write_blob(uint32_t num_reg, int bit_offset,
                                int bit_count, void *arr_in,
                                int *bits_counter)
{
  uint32_t curval;
  uint32_t mask;

  uint32_t bit_start = (bit_offset % SAM_SFC_DR_LEN);
  uint32_t reg_to_write = sama5_sfc_fill_reg(bit_start, bit_count,
                                                   (uint8_t *)arr_in,
                                                   bits_counter);

  curval = getreg32(g_start_sam_sfc_reg[num_reg]);

  /* We cannot burn a 1 back to a 0 */

  mask = sam_sfc_get_mask(bit_count, bit_start);

  if (!(curval & ~reg_to_write & mask))
    {
      return sama5_sfc_write_reg(num_reg, reg_to_write);
    }
  else
    {
      merr("ERROR: requested value cannot be burned\n");
      return -EINVAL;
    }
}

/****************************************************************************
 * Name: sama5_sfc_burn_efuses
 *
 * Description:
 *   Write data fields to SFC EFUSE.
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

static int sama5_sfc_burn_efuses(const efuse_desc_t *field[],
                                 const void *src, size_t src_size_bits)
{
  int  err = OK;

  if (field == NULL || src == NULL || src_size_bits == 0)
    {
      err = -EINVAL;
    }
  else
    {
      err = sam_sfc_process(field, (void *)src, src_size_bits,
                            sama5_sfc_write_blob);
    }

  return err;
}

/****************************************************************************
 * Name: sama5_sfc_lower_read
 *
 * Description:
 *   Read value from EFUSE, writing it into an array.
 *
 * Input Parameters:
 *   lower          - A pointer the publicly visible representation of
 *                    the "lower-half" driver state structure
 *   field          - A pointer to describing the fields of efuse
 *   data           - A pointer to array that contains the data for reading
 *   bits_len       - The number of bits required to read
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

static int sama5_sfc_lower_read(struct efuse_lowerhalf_s *lower,
                                const efuse_desc_t *field[],
                                uint8_t *data,
                                size_t bits_len)
{
  int err = OK;
  int num_registers;

  if (field == NULL || data == NULL || bits_len == 0)
    {
      err = -EINVAL;
    }
  else
    {
      num_registers = sam_sfc_get_number_of_items(bits_len, 8);
      memset((uint8_t *)data, 0, num_registers);

      err = sam_sfc_process(field, data, bits_len, sam_sfc_fill_buff);
    }

  return err;
}

/****************************************************************************
 * Name: sama5_sfc_lower_write
 *
 * Description:
 *   Write array to EFUSE.
 *
 * Input Parameters:
 *   lower          - A pointer the publicly visible representation of
 *                    the "lower-half" driver state structure
 *   field          - A pointer to describing the fields of efuse
 *   data           - A pointer to array that contains the data for writing
 *   bits_len       - The number of bits required to write
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

static ssize_t sama5_sfc_lower_write(struct efuse_lowerhalf_s *lower,
                                     const efuse_desc_t *field[],
                                     const uint8_t *data,
                                     size_t bits_len)
{
  /* Write the blob data to the field */

  return sama5_sfc_burn_efuses(field, data, bits_len);
}

/****************************************************************************
 * Name: sam_sfc_read_reg
 *
 * Description:
 *   Read efuse register.
 *
 * Input Parameters:
 *   num_reg      - The register number
 *
 * Returned Value:
 *   Return the value in the efuse register.
 *
 ****************************************************************************/

static uint32_t sam_sfc_read_reg(uint32_t num_reg)
{
  uint32_t value;

  DEBUGASSERT(num_reg < SFC_DR_END);

  value = getreg32(g_start_sam_sfc_reg[num_reg]);

  return value;
}

/****************************************************************************
 * Name: sam_sfc_mask_read
 *
 * Description:
 *   Read efuse register.
 *
 ****************************************************************************/

static void sam_sfc_mask_read(void)
{
  uint32_t regval;

  regval = getreg32(SAM_SFC_MR);
  regval |= SAM_SFC_MR_MASK;

  putreg32(regval, SAM_SFC_MR);
}

/****************************************************************************
 * Name: sam_sfc_get_mask
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

static uint32_t sam_sfc_get_mask(uint32_t bit_count, uint32_t shift)
{
  uint32_t mask;

  if (bit_count != SAM_SFC_DR_LEN)
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
 * Name: sam_sfc_fill_buff
 *
 * Description:
 *   Read efuse register and write this value to array.
 *
 * Input Parameters:
 *   num_reg      - The register number
 *   bit_offset   - Start bit in register
 *   bit_count    - The number of bits required to read
 *   arr_out      - A pointer to array that will contain the result
 *   bits_counter - A pointer that will contain the bits counter of reading
 *
 * Returned Value:
 *   Zero (OK) is returned on success.
 *
 ****************************************************************************/

static int sam_sfc_fill_buff(uint32_t num_reg, int bit_offset,
                             int bit_count, void *arr_out, int *bits_counter)
{
  uint8_t *blob = (uint8_t *)arr_out;
  uint32_t bit_start = (bit_offset % SAM_SFC_DR_LEN);
  uint32_t reg_val = sam_sfc_read_reg(num_reg);
  uint64_t reg_of_aligned_bits = (reg_val >> bit_start) & \
                                  sam_sfc_get_mask(bit_count, 0);
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
 * Name: sama5_sfc_initialize
 *
 * Description:
 *   Initialize the sfc efuse driver. The efuse is initialized
 *   and registered as 'devpath'.
 *
 * Input Parameters:
 *   devpath        - The full path to the efuse device.
 *                    This should be of the form /dev/efuse
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -EEXIST (error).
 *
 ****************************************************************************/

int sama5_sfc_initialize(const char *devpath)
{
  struct sama5_sfc_upperhalf_s *upper = NULL;
  struct efuse_lowerhalf_s *lower;
  int ret = OK;

  DEBUGASSERT(devpath != NULL);

  lower = &g_sama5_sfc_lowerhalf_s;

  /* Register the efuse upper driver */

  upper = efuse_register(devpath, lower);

  if (upper == NULL)
    {
      /* The actual cause of the failure may have been a failure to allocate
       * perhaps a failure to register the efuser driver (such as if the
       * 'devpath' were not unique).  We know here but we return EEXIST to
       * indicate the failure (implying the non-unique devpath).
       */

      ret = -EEXIST;
    }

  return ret;
}

#endif /* CONFIG_SAMA5_SFC */
