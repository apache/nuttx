/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_efuse.c
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
 * Included Files
 ****************************************************************************/

#include <stdlib.h>
#include <debug.h>
#include <errno.h>
#include <assert.h>
#include <string.h>
#include <sys/param.h>
#include <nuttx/irq.h>
#include <nuttx/efuse/efuse.h>

#include "espressif/esp_efuse.h"

#include "esp_efuse.h"
#include "esp_efuse_utility.h"

#ifdef CONFIG_ARCH_CHIP_ESP32
#include "xtensa.h"
#include "soc/syscon_reg.h"
#include "esp_efuse_table.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EFUSE_MAX_BLK_LEN  256   /* Max length of efuse block. */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_efuse_lowerhalf_s
{
  const struct efuse_ops_s *ops; /* Lower half operations */
  void *upper;                   /* Pointer to efuse_upperhalf_s */
};

#ifdef CONFIG_ESPRESSIF_EFUSE

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/* "Lower half" driver methods */

static int esp_efuse_lowerhalf_read(struct efuse_lowerhalf_s *lower,
                                    const efuse_desc_t *field[],
                                    uint8_t *data, size_t bits_len);
static int esp_efuse_lowerhalf_write(struct efuse_lowerhalf_s *lower,
                                     const efuse_desc_t *field[],
                                     const uint8_t *data,
                                     size_t bits_len);
static int esp_efuse_lowerhalf_ioctl(struct efuse_lowerhalf_s *lower,
                                     int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* "Lower half" driver methods */

static const struct efuse_ops_s g_esp_efuse_ops =
{
  .read_field   = esp_efuse_lowerhalf_read,
  .write_field  = esp_efuse_lowerhalf_write,
  .ioctl        = esp_efuse_lowerhalf_ioctl,
};

/* EFUSE lower-half */

static struct esp_efuse_lowerhalf_s g_esp_efuse_lowerhalf =
{
  .ops = &g_esp_efuse_ops,
  .upper = NULL,
};

/****************************************************************************
 * Private functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_efuse_lowerhalf_read
 *
 * Description:
 *   Read value from EFUSE, writing it into an array.
 *   The field[0]->bit_offset received from the upper half represents
 *   the bit offset taking into consideration that each block is 256 bits.
 *   This is necessary as we have multiple blocks of 256 bits.
 *
 *   Example: To read data from USER_DATA (EFUSE_BLK3), from bit 16 onwards,
 *   then bit_offset should be 3*256 + 16.
 *
 * Input Parameters:
 *   lower    - A pointer the publicly visible representation of
 *              the "lower-half" driver state structure
 *   field    - A pointer to describing the fields of efuse
 *   data     - A pointer to array that contains the data for reading
 *   bits_len - The number of bits required to read
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_efuse_lowerhalf_read(struct efuse_lowerhalf_s *lower,
                                    const efuse_desc_t *field[],
                                    uint8_t *data, size_t bits_len)
{
  int ret = OK;
  uint8_t blk_num = field[0]->bit_offset / EFUSE_MAX_BLK_LEN;

  esp_efuse_desc_t recv =
    {
      .efuse_block = blk_num,
      .bit_start = field[0]->bit_offset - blk_num * EFUSE_MAX_BLK_LEN,
      .bit_count = field[0]->bit_count
    };

  const esp_efuse_desc_t *desc[] =
    {
      &recv,
      NULL
    };

  minfo("read from blk_num: %d, bit_start: %d, bit_count: %d\n",
        blk_num, recv.bit_start, recv.bit_count);

  /* Read the requested field */

  ret = esp_efuse_read_field_blob((const esp_efuse_desc_t**)&desc,
                                  data,
                                  bits_len);

  return ret;
}

/****************************************************************************
 * Name: esp_efuse_lowerhalf_write
 *
 * Description:
 *   Write array to EFUSE.
 *
 *   The field[0]->bit_offset received from the upper half represents
 *   the bit offset taking into consideration that each block is 256 bits.
 *   This is necessary as we have multiple blocks of 256 bits.
 *
 *   Example: To write data to USER_DATA (EFUSE_BLK3), from bit 16 onwards,
 *   then bit_offset should be 3*256 + 16.
 *
 * Input Parameters:
 *   lower    - A pointer the publicly visible representation of
 *              the "lower-half" driver state structure
 *   field    - A pointer to describing the fields of efuse
 *   data     - A pointer to array that contains the data for writing
 *   bits_len - The number of bits required to write
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_efuse_lowerhalf_write(struct efuse_lowerhalf_s *lower,
                                     const efuse_desc_t *field[],
                                     const uint8_t *data,
                                     size_t bits_len)
{
  irqstate_t flags;
  int ret = OK;
  uint8_t blk_num = field[0]->bit_offset / EFUSE_MAX_BLK_LEN;
  esp_efuse_desc_t recv =
    {
      .efuse_block = blk_num,
      .bit_start = field[0]->bit_offset - blk_num * EFUSE_MAX_BLK_LEN,
      .bit_count = field[0]->bit_count
    };

  const esp_efuse_desc_t *desc[] =
    {
      &recv,
      NULL
    };

  minfo("write to blk_num: %d, bit_start: %d, bit_count: %d\n",
        blk_num, recv.bit_start, recv.bit_count);

  flags = enter_critical_section();

  ret = esp_efuse_write_field_blob((const esp_efuse_desc_t**)&desc,
                                   data,
                                   bits_len);

  leave_critical_section(flags);

  if (ret != OK)
    {
      return ERROR;
    }

  return ret;
}

/****************************************************************************
 * Name: esp_efuse_lowerhalf_ioctl
 *
 * Description:
 *   Any ioctl commands that are not recognized by the "upper-half"
 *   driver are forwarded to the lower half driver through this method.
 *
 * Input Parameters:
 *   lower   - A pointer the publicly visible representation of the
 *             "lower-half" driver state structure.
 *   cmd     - The ioctl command value
 *   arg     - The optional argument that accompanies the ioctl command.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

static int esp_efuse_lowerhalf_ioctl(struct efuse_lowerhalf_s *lower,
                                     int cmd, unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      /* We don't have proprietary EFUSE ioctls */

      default:
        {
          minfo("Unrecognized cmd: %d\n", cmd);
          ret = -ENOTTY;
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_efuse_initialize
 *
 * Description:
 *   Initialize the efuse driver. The efuse is initialized
 *   and registered as 'devpath'
 *
 * Input Parameters:
 *   devpath        - The full path to the efuse device.
 *                    This should be of the form /dev/efuse
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp_efuse_initialize(const char *devpath)
{
  struct esp_efuse_lowerhalf_s *lower = NULL;
  int ret = OK;

  DEBUGASSERT(devpath != NULL);

  lower = &g_esp_efuse_lowerhalf;

  /* Register the efuse upper driver */

  lower->upper = efuse_register(devpath,
                                (struct efuse_lowerhalf_s *)lower);

  if (lower->upper == NULL)
    {
      /* The actual cause of the failure may have been a failure to allocate
       * perhaps a failure to register the efuse driver (such as if the
       * 'devpath' were not unique).  We know here but we return EEXIST to
       * indicate the failure (implying the non-unique devpath).
       */

      ret = -EEXIST;
    }

#ifdef CONFIG_ESPRESSIF_EFUSE_VIRTUAL
  mwarn("Virtual E-Fuses are enabled\n");
  esp_efuse_utility_update_virt_blocks();
#endif

  return ret;
}
#endif

/****************************************************************************
 * Name: esp_efuse_hal_chip_revision
 *
 * Description:
 *   Returns the chip version in the format: Major * 100 + Minor.
 *   This function must be compiled even when CONFIG_ESPRESSIF_EFUSE is not
 *   defined, because it required used by esp32_start.c.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The chip version as an unsigned 32-bit integer.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_ESP32
uint32_t esp_efuse_hal_chip_revision(void)
{
  uint8_t eco_bit0;
  uint8_t eco_bit1;
  uint8_t eco_bit2;
  uint8_t minor_chip_version;
  uint32_t combine_value;
  uint32_t chip_ver = 0;

  esp_efuse_read_field_blob(ESP_EFUSE_CHIP_VER_REV1,
                            &eco_bit0,
                            ESP_EFUSE_CHIP_VER_REV1[0]->bit_count);
  esp_efuse_read_field_blob(ESP_EFUSE_CHIP_VER_REV2,
                            &eco_bit1,
                            ESP_EFUSE_CHIP_VER_REV2[0]->bit_count);
  esp_efuse_read_field_blob(ESP_EFUSE_WAFER_VERSION_MINOR,
                            &minor_chip_version,
                            ESP_EFUSE_WAFER_VERSION_MINOR[0]->bit_count);

  eco_bit2 = (getreg32(SYSCON_DATE_REG) & 0x80000000) >> 31;
  combine_value = (eco_bit2 << 2) | (eco_bit1 << 1) | eco_bit0;

  switch (combine_value)
  {
    case 0:
        chip_ver = 0;
        break;
    case 1:
        chip_ver = 1;
        break;
    case 3:
        chip_ver = 2;
        break;
    case 7:
        chip_ver = 3;
        break;
    default:
        chip_ver = 0;
        break;
  }

  return (chip_ver * 100) + minor_chip_version;
}
#endif
