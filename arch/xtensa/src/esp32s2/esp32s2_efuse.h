/****************************************************************************
 * arch/xtensa/src/esp32s2/esp32s2_efuse.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_EFUSE_H
#define __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_EFUSE_H

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
 * Public Types
 ****************************************************************************/

/* Type of eFuse blocks for ESP32S2 */

typedef enum
{
    EFUSE_BLK0                 = 0,   /**< Number of eFuse BLOCK0. REPEAT_DATA */

    EFUSE_BLK1                 = 1,   /**< Number of eFuse BLOCK1. MAC_SPI_8M_SYS */

    EFUSE_BLK2                 = 2,   /**< Number of eFuse BLOCK2. SYS_DATA_PART1 */
    EFUSE_BLK_SYS_DATA_PART1   = 2,   /**< Number of eFuse BLOCK2. SYS_DATA_PART1 */

    EFUSE_BLK3                 = 3,   /**< Number of eFuse BLOCK3. USER_DATA */
    EFUSE_BLK_USER_DATA        = 3,   /**< Number of eFuse BLOCK3. USER_DATA */

    EFUSE_BLK4                 = 4,   /**< Number of eFuse BLOCK4. KEY0 */
    EFUSE_BLK_KEY0             = 4,   /**< Number of eFuse BLOCK4. KEY0 */

    EFUSE_BLK5                 = 5,   /**< Number of eFuse BLOCK5. KEY1 */
    EFUSE_BLK_KEY1             = 5,   /**< Number of eFuse BLOCK5. KEY1 */

    EFUSE_BLK6                 = 6,   /**< Number of eFuse BLOCK6. KEY2 */
    EFUSE_BLK_KEY2             = 6,   /**< Number of eFuse BLOCK6. KEY2 */

    EFUSE_BLK7                 = 7,   /**< Number of eFuse BLOCK7. KEY3 */
    EFUSE_BLK_KEY3             = 7,   /**< Number of eFuse BLOCK7. KEY3 */

    EFUSE_BLK8                 = 8,   /**< Number of eFuse BLOCK8. KEY4 */
    EFUSE_BLK_KEY4             = 8,   /**< Number of eFuse BLOCK8. KEY4 */

    EFUSE_BLK9                 = 9,   /**< Number of eFuse BLOCK9. KEY5 */
    EFUSE_BLK_KEY5             = 9,   /**< Number of eFuse BLOCK9. KEY5 */
    EFUSE_BLK_KEY_MAX          = 10,

    EFUSE_BLK10                = 10,  /**< Number of eFuse BLOCK10. SYS_DATA_PART2 */
    EFUSE_BLK_SYS_DATA_PART2   = 10,  /**< Number of eFuse BLOCK10. SYS_DATA_PART2 */

    EFUSE_BLK_MAX
} esp_efuse_block_t;

/****************************************************************************
 * Name: efuse_func_proc_t
 *
 * Description:
 *   This is type of function that will handle the efuse field register.
 *
 * Input Parameters:
 *   num_reg          - The register number in the block.
 *   bit_start        - Start bit in the register.
 *   bit_count        - The number of bits used in the register.
 *   arr              - A pointer to an array or variable.
 *   bits_counter     - Counter bits.
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

typedef int (*efuse_func_proc_t) (uint32_t num_reg,
                                  int bit_start,
                                  int bit_count,
                                  void *arr, int *bits_counter);

/****************************************************************************
 * Public Functions Prototypes
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
                             size_t dst_size_bits);

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
                              const void *src, size_t src_size_bits);

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

void esp32s2_efuse_burn_efuses(void);

/****************************************************************************
 * Name: esp32s2_efuse_initialize
 *
 * Description:
 *   Initialize the efuse driver. The efuse is initialized
 *   and registered as 'devpath'.
 *
 * Input Parameters:
 *   devpath        - The full path to the efuse device.
 *                    This should be of the form /dev/efuse
 *
 * Returned Value:
 *   Zero (OK) is returned on success. Otherwise -1 (ERROR).
 *
 ****************************************************************************/

int esp32s2_efuse_initialize(const char *devpath);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S2_ESP32S2_EFUSE_H */
