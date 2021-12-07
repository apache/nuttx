/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_spiflash.h
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

#ifndef __ARCH_RISCV_SRC_ESP32C3_ESP32C3_SPIFLASH_H
#define __ARCH_RISCV_SRC_ESP32C3_ESP32C3_SPIFLASH_H

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
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: spi_flash_read_encrypted
 *
 * Description:
 *
 *   Read data from Encrypted Flash.
 *
 *   If flash encryption is enabled, this function will transparently
 *   decrypt data as it is read.
 *   If flash encryption is not enabled, this function behaves the same as
 *   spi_flash_read().
 *
 *   See esp_flash_encryption_enabled() for a function to check if flash
 *   encryption is enabled.
 *
 * Parameters:
 *   addr   - source address of the data in Flash.
 *   buffer - pointer to the destination buffer
 *   size   - length of data
 *
 * Returned Values: esp_err_t
 *
 ****************************************************************************/

int spi_flash_read_encrypted(uint32_t addr, void *buffer, uint32_t size);

/****************************************************************************
 * Name: esp32c3_spiflash_unmask_cpuint
 *
 * Description:
 *   Unmask CPU interrupt and keep this interrupt work when read, write,
 *   erase SPI Flash.
 *
 *   By default, all CPU interrupts are masked.
 *
 * Input Parameters:
 *   cpuint - CPU interrupt ID
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_spiflash_unmask_cpuint(int cpuint);

/****************************************************************************
 * Name: esp32c3_spiflash_unmask_cpuint
 *
 * Description:
 *   Mask CPU interrupt and disable this interrupt when read, write,
 *   erase SPI Flash.
 *
 *   By default, all CPU interrupts are masked.
 *
 * Input Parameters:
 *   cpuint - CPU interrupt ID
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32c3_spiflash_mask_cpuint(int cpuint);

/****************************************************************************
 * Name: esp32c3_spiflash_init
 *
 * Description:
 *   Initialize ESP32-C3 SPI flash driver.
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int esp32c3_spiflash_init(void);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP32C3_SPIFLASH_H */
