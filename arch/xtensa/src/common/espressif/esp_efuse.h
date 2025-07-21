/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_efuse.h
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

#ifndef __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_EFUSE_H
#define __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_EFUSE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/efuse/efuse.h>

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP_EFUSE_BLK_LEN 256

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* E-Fuse block and bit definitions can be found on the Technical Reference
 * Manual or on the ESP-IDF documentation.
 */

#ifdef CONFIG_ARCH_CHIP_ESP32
typedef enum
{
  ESP_EFUSE_BLK0              = 0,

  ESP_EFUSE_BLK1              = 1,
  ESP_EFUSE_BLK_KEY0          = 1,
  ESP_EFUSE_BLK_ENCRYPT_FLASH = 1,

  ESP_EFUSE_BLK2              = 2,
  ESP_EFUSE_BLK_KEY1          = 2,
  ESP_EFUSE_BLK_SECURE_BOOT   = 2,

  ESP_EFUSE_BLK3              = 3,
  ESP_EFUSE_BLK_KEY2          = 3,
  ESP_EFUSE_BLK_KEY_MAX       = 4,

  ESP_EFUSE_BLK_MAX           = 4,
} esp_efuse_blk_num_t;
#else
typedef enum
{
  ESP_EFUSE_BLK0                 = 0,
  ESP_EFUSE_BLK1                 = 1,

  ESP_EFUSE_BLK2                 = 2,
  ESP_EFUSE_BLK_SYS_DATA_PART1   = 2,

  ESP_EFUSE_BLK3                 = 3,
  ESP_EFUSE_BLK_USER_DATA        = 3,

  ESP_EFUSE_BLK4                 = 4,
  ESP_EFUSE_BLK_KEY0             = 4,

  ESP_EFUSE_BLK5                 = 5,
  ESP_EFUSE_BLK_KEY1             = 5,

  ESP_EFUSE_BLK6                 = 6,
  ESP_EFUSE_BLK_KEY2             = 6,

  ESP_EFUSE_BLK7                 = 7,
  ESP_EFUSE_BLK_KEY3             = 7,

  ESP_EFUSE_BLK8                 = 8,
  ESP_EFUSE_BLK_KEY4             = 8,

  ESP_EFUSE_BLK9                 = 9,
  ESP_EFUSE_BLK_KEY5             = 9,
  ESP_EFUSE_BLK_KEY_MAX          = 10,

  ESP_EFUSE_BLK10                = 10,
  ESP_EFUSE_BLK_SYS_DATA_PART2   = 10,

  ESP_EFUSE_BLK_MAX
} esp_efuse_blk_num_t;
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_efuse_hal_chip_revision
 *
 * Description:
 *   Returns the chip version in the format: Major * 100 + Minor.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The chip version as an unsigned 32-bit integer.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_ESP32
uint32_t esp_efuse_hal_chip_revision(void);
#endif

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

#ifdef CONFIG_ESPRESSIF_EFUSE
int esp_efuse_initialize(const char *devpath);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_EFUSE_H */
