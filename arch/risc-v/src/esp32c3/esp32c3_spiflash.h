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
 * Name: esp32c3_spiflash_mtd
 *
 * Description:
 *   Get ESP32-C3 SPI Flash MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP32-C3 SPI Flash MTD pointer.
 *
 ****************************************************************************/

struct mtd_dev_s *esp32c3_spiflash_mtd(void);

/****************************************************************************
 * Name: esp32c3_spiflash_alloc_mtdpart
 *
 * Description:
 *   Alloc ESP32-C3 SPI Flash MTD
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP32-C3 SPI Flash MTD data pointer if success or NULL if fail
 *
 ****************************************************************************/

FAR struct mtd_dev_s *esp32c3_spiflash_alloc_mtdpart(void);

/****************************************************************************
 * Name: esp32c3_spiflash_encrypt_mtd
 *
 * Description:
 *   Get ESP32-C3 SPI Flash encryption MTD.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   ESP32-C3 SPI Flash encryption MTD pointer.
 *
 ****************************************************************************/

struct mtd_dev_s *esp32c3_spiflash_encrypt_mtd(void);

/****************************************************************************
 * Name: spi_flash_write_encrypted
 *
 * Description:
 *   Write data encrypted to Flash.
 *
 *   Flash encryption must be enabled for this function to work.
 *
 *   Flash encryption must be enabled when calling this function.
 *   If flash encryption is disabled, the function returns
 *   ESP_ERR_INVALID_STATE.  Use esp_flash_encryption_enabled()
 *   function to determine if flash encryption is enabled.
 *
 *   Both dest_addr and size must be multiples of 16 bytes. For
 *   absolute best performance, both dest_addr and size arguments should
 *   be multiples of 32 bytes.
 *
 * Input Parameters:
 *   dest_addr - Destination address in Flash. Must be a multiple of 16
 *               bytes.
 *   src       - Pointer to the source buffer.
 *   size      - Length of data, in bytes. Must be a multiple of 16 bytes.
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

int spi_flash_write_encrypted(uint32_t dest_addr, const void *src,
                              uint32_t size);

/****************************************************************************
 * Name: spi_flash_write
 *
 * Description:
 *
 *   Write data to Flash.
 *
 *   Note: For fastest write performance, write a 4 byte aligned size at a
 *   4 byte aligned offset in flash from a source buffer in DRAM. Varying
 *   any of these parameters will still work, but will be slower due to
 *   buffering.
 *
 *   Writing more than 8KB at a time will be split into multiple
 *   write operations to avoid disrupting other tasks in the system.
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

int spi_flash_write(uint32_t dest_addr, const void *src, uint32_t size);

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
 *   src   - source address of the data in Flash.
 *   dest  - pointer to the destination buffer
 *   size  - length of data
 *
 * Returned Values: esp_err_t
 *
 ****************************************************************************/

int spi_flash_read_encrypted(uint32_t src, void *dest, uint32_t size);

/****************************************************************************
 * Name: spi_flash_read
 *
 * Description:
 *   Read data from Flash.
 *
 *   Note: For fastest read performance, all parameters should be
 *   4 byte aligned. If source address and read size are not 4 byte
 *   aligned, read may be split into multiple flash operations. If
 *   destination buffer is not 4 byte aligned, a temporary buffer will
 *   be allocated on the stack.
 *
 *   Reading more than 16KB of data at a time will be split
 *   into multiple reads to avoid disruption to other tasks in the
 *   system. Consider using spi_flash_mmap() to read large amounts
 *   of data.
 *
 * Parameters:
 *   src_addr - source address of the data in Flash.
 *   dest     - pointer to the destination buffer
 *   size     - length of data
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

int spi_flash_read(uint32_t src_addr, void *dest, uint32_t size);

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

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_ESP32C3_ESP32C3_SPIFLASH_H */
