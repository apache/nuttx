/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_spiflash.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_SPIFLASH_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_SPIFLASH_H

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

/**
 * Structure holding SPI flash access critical sections management functions.
 *
 * Flash API uses two types of functions for flash access management:
 * 1) Functions which prepare/restore flash cache and interrupts before
 *    calling appropriate ROM functions (spi_flash_write, spi_flash_read,
 *    spi_flash_erase_sector and spi_flash_erase_range):
 *   - 'start' function should disable flash cache and non-IRAM interrupts
 *      and is invoked before the call to one of ROM functions from
 *      "struct spiflash_guard_funcs".
 *   - 'end' function should restore state of flash cache and non-IRAM
 *      interrupts and is invoked after the call to one of ROM
 *      functions from "struct spiflash_guard_funcs".
 *    These two functions are not reentrant.
 * 2) Functions which synchronizes access to internal data used by flash API.
 *    These functions are mostly intended to synchronize access to flash API
 *    internal data in multithreaded environment and use OS primitives:
 *   - 'op_lock' locks access to flash API internal data.
 *   - 'op_unlock' unlocks access to flash API internal data.
 *   These two functions are reentrant and can be used around the outside of
 *   multiple calls to 'start' & 'end', in order to create atomic multi-part
 *   flash operations.
 *
 * Structure and corresponding guard functions should not reside
 * in flash. For example structure can be placed in DRAM and functions
 * in IRAM sections.
 */

struct spiflash_guard_funcs
{
  void (*start)(void);      /* critical section start function */
  void (*end)(void);        /* critical section end function */
  void (*op_lock)(void);    /* flash access API lock function */
  void (*op_unlock)(void);  /* flash access API unlock function */

  /* checks flash write addresses */

  bool (*address_is_safe)(size_t addr, size_t size);

  void (*yield)(void);      /* yield to the OS during flash erase */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: spi_flash_read
 *
 * Description:
 *   Read data from Flash.
 *
 * Parameters:
 *   address - source address of the data in Flash.
 *   buffer  - pointer to the destination buffer
 *   length  - length of data
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

int spi_flash_read(uint32_t address, void *buffer, uint32_t length);

/****************************************************************************
 * Name: spi_flash_erase_sector
 *
 * Description:
 *   Erase the Flash sector.
 *
 * Parameters:
 *   sector - Sector number, the count starts at sector 0, 4KB per sector.
 *
 * Returned Values: esp_err_t
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

int spi_flash_erase_sector(uint32_t sector);

/****************************************************************************
 * Name: spi_flash_erase_range
 *
 * Description:
 *   Erase a range of flash sectors
 *
 * Parameters:
 *   start_address - Address where erase operation has to start.
 *                   Must be 4kB-aligned
 *   size          - Size of erased range, in bytes. Must be divisible by
 *                   4kB.
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

int spi_flash_erase_range(uint32_t start_address, uint32_t size);

/****************************************************************************
 * Name: spi_flash_write
 *
 * Description:
 *   Write data to Flash.
 *
 * Parameters:
 *   dest_addr - Destination address in Flash.
 *   src       - Pointer to the source buffer.
 *   size      - Length of data, in bytes.
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

int spi_flash_write(uint32_t dest_addr, const void *buffer, uint32_t size);

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
#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_SPIFLASH_H */
