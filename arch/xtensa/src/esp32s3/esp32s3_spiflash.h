/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_spiflash.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_SPIFLASH_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_SPIFLASH_H

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

/* SPI Flash map request data */

struct spiflash_map_req_s
{
  /* Request mapping SPI Flash base address */

  uint32_t  src_addr;

  /* Request mapping SPI Flash size */

  uint32_t  size;

  /* Mapped memory pointer */

  void      *ptr;

  /* Mapped started MMU page index */

  uint32_t  start_page;

  /* Mapped MMU page count */

  uint32_t  page_cnt;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_mmap
 *
 * Description:
 *   Mapped SPI Flash address to ESP32-S3's address bus, so that software
 *   can read SPI Flash data by reading data from memory access.
 *
 *   If SPI Flash hardware encryption is enable, the read from mapped
 *   address is decrypted.
 *
 * Input Parameters:
 *   req - SPI Flash mapping requesting parameters
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

int esp32s3_mmap(struct spiflash_map_req_s *req);

/****************************************************************************
 * Name: esp32s3_ummap
 *
 * Description:
 *   Unmap SPI Flash address in ESP32-S3's address bus, and free resource.
 *
 * Input Parameters:
 *   req - SPI Flash mapping requesting parameters
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp32s3_ummap(const struct spiflash_map_req_s *req);

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
 * Name: esp32s3_spiflash_init
 *
 * Description:
 *   Initialize ESP32-S3 SPI flash driver.
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int esp32s3_spiflash_init(void);

/****************************************************************************
 * Name: esp32s3_flash_encryption_enabled
 *
 * Description:
 *   Check if ESP32-S3 enables SPI Flash encryption.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   True: SPI Flash encryption is enable, False if not.
 *
 ****************************************************************************/

bool esp32s3_flash_encryption_enabled(void);

/****************************************************************************
 * Name: esp32s3_get_flash_address_mapped_as_text
 *
 * Description:
 *   Get flash address which is currently mapped as text
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   flash address which is currently mapped as text
 *
 ****************************************************************************/

uint32_t esp32s3_get_flash_address_mapped_as_text(void);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_SPIFLASH_H */
