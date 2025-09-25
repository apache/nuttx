/****************************************************************************
 * arch/xtensa/src/common/espressif/esp_spiflash.h
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

#ifndef __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_SPIFLASH_H
#define __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_SPIFLASH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <nuttx/mtd/mtd.h>

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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_spiflash_read
 *
 * Description:
 *   Read data from Flash.
 *
 * Parameters:
 *   address - Source address of the data in Flash.
 *   buffer  - Pointer to the destination buffer.
 *   length  - Length of data in bytes.
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

int esp_spiflash_read(uint32_t address, void *buffer, uint32_t length);

/****************************************************************************
 * Name: esp_spiflash_erase
 *
 * Description:
 *   Erase data from Flash.
 *
 * Parameters:
 *   address - Start address of the data in Flash.
 *   length  - Length of the data to erase.
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

int esp_spiflash_erase(uint32_t address, uint32_t length);

/****************************************************************************
 * Name: esp_spiflash_write
 *
 * Description:
 *   Write data to Flash.
 *
 * Parameters:
 *   address - Destination address in Flash.
 *   buffer  - Pointer to the source buffer.
 *   length  - Length of data, in bytes.
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

int esp_spiflash_write(uint32_t address, const void *buffer,
                       uint32_t length);

/****************************************************************************
 * Name: esp_spiflash_init
 *
 * Description:
 *   Initialize ESP SPI flash driver.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int esp_spiflash_init(void);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_COMMON_ESPRESSIF_ESP_SPIFLASH_H */
