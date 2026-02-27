/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_spiflash.c
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

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/nuttx.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>
#include <nuttx/mutex.h>
#include <sys/types.h>
#include <inttypes.h>
#include <sched/sched.h>

#include "esp_private/esp_flash_internal.h"
#include "esp_flash.h"
#include "esp_flash_encrypt.h"
#include "esp_private/cache_utils.h"
#include "hal/efuse_hal.h"
#include "bootloader_flash_priv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_EFUSE_VIRTUAL_KEEP_IN_FLASH
#define ENCRYPTION_IS_VIRTUAL (!efuse_hal_flash_encryption_enabled())
#else
#define ENCRYPTION_IS_VIRTUAL 0
#endif

#ifndef ALIGN_OFFSET
#define ALIGN_OFFSET(num, align) ((num) & ((align) - 1))
#endif

#ifndef ROUND_DOWN
#define ROUND_DOWN(x, align) ((unsigned long)(x) & ~((unsigned long)align - 1))
#endif

#ifndef ROUND_UP
#define ROUND_UP(x, align) \
        (((unsigned long)(x) + ((unsigned long)align - 1)) & \
        ~((unsigned long)align - 1))
#endif

 #define FLASH_BUFFER_SIZE          32
 #define FLASH_ERASE_VALUE          0xff
 #define SPIFLASH_OP_TASK_STACKSIZE 768

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if CONFIG_ESPRESSIF_SECURE_FLASH_ENC_ENABLED
static uint8_t write_aux_buf[FLASH_SECTOR_SIZE] =
{
  0
};
static uint8_t erase_aux_buf[FLASH_SECTOR_SIZE] =
{
  0
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: flash_esp32_read_check_enc
 *
 * Description:
 *   Read data from flash, automatically handling encryption if enabled.
 *   This function checks if flash encryption is enabled and uses the
 *   appropriate read function (encrypted or raw).
 *
 * Input Parameters:
 *   address - Source address of the data in flash.
 *   buffer  - Pointer to the destination buffer.
 *   length  - Length of data in bytes.
 *
 * Returned Value:
 *   OK on success; -EIO on failure.
 *
 ****************************************************************************/

static int flash_esp32_read_check_enc(uint32_t address, void *buffer,
                                      size_t length)
{
  int ret;

  if (esp_flash_encryption_enabled())
    {
      finfo("Flash read ENCRYPTED - address 0x%lx size 0x%x",
            address, length);
      ret = esp_flash_read_encrypted(NULL, address, buffer, length);
    }
  else
    {
      finfo("Flash read RAW - address 0x%lx size 0x%x", address, length);
      ret = esp_flash_read(NULL, buffer, address, length);
    }

  if (ret != OK)
    {
      ferr("ERROR: failed to read: ret=%d", ret);
      return -EIO;
    }

  return OK;
}

/****************************************************************************
 * Name: flash_esp32_write_check_enc
 *
 * Description:
 *   Write data to flash, automatically handling encryption if enabled.
 *   This function checks if flash encryption is enabled and uses the
 *   appropriate write function (encrypted or raw).
 *
 * Input Parameters:
 *   address - Destination address in flash.
 *   buffer  - Pointer to the source buffer.
 *   length  - Length of data in bytes.
 *
 * Returned Value:
 *   OK on success; -EIO on failure.
 *
 ****************************************************************************/

static int flash_esp32_write_check_enc(uint32_t address, const void *buffer,
                                       size_t length)
{
  int ret;

  if (esp_flash_encryption_enabled() && !ENCRYPTION_IS_VIRTUAL)
    {
      finfo("Flash write ENCRYPTED - address 0x%lx size 0x%x",
            address, length);
      ret = esp_flash_write_encrypted(NULL, address, buffer, length);
    }
  else
    {
      finfo("Flash write RAW - address 0x%lx size 0x%x",
            address, length);
      ret = esp_flash_write(NULL, buffer, address, length);
    }

  if (ret != 0)
    {
      ferr("ERROR: failed to write: ret=%d", ret);
      return -EIO;
    }

  return OK;
}

#if CONFIG_ESPRESSIF_SECURE_FLASH_ENC_ENABLED
/****************************************************************************
 * Name: aligned_flash_write
 *
 * Description:
 *   Write data to flash with proper alignment handling. This function
 *   ensures that writes are aligned according to flash encryption
 *   requirements. When flash encryption is enabled, writes must be
 *   aligned to 32 bytes (or FLASH_SECTOR_SIZE if erase is required).
 *   For unaligned writes, the function reads the existing data, merges
 *   it with the new data, and writes back the aligned chunk.
 *
 * Input Parameters:
 *   dest_addr - Destination address in flash.
 *   src       - Pointer to the source buffer.
 *   size      - Length of data in bytes.
 *   erase     - If true, erase the region before writing (required when
 *               flash encryption is enabled).
 *
 * Returned Value:
 *   true on success; false on failure.
 *
 ****************************************************************************/

static bool aligned_flash_write(size_t dest_addr, const void *src,
                                size_t size, bool erase)
{
  bool flash_enc_enabled = esp_flash_encryption_enabled();
  size_t alignment;
  size_t write_addr = dest_addr;
  size_t bytes_remaining = size;
  size_t src_offset = 0;

  /* When flash encryption is enabled, write alignment is 32 bytes, however
   * to avoid inconsistences the region may be erased right before writing,
   * thus the alignment is set to the erase required alignment
   * (FLASH_SECTOR_SIZE).
   * When flash encryption is not enabled, regular write alignment
   * is 4 bytes.
   */

  alignment = flash_enc_enabled ? (erase ? FLASH_SECTOR_SIZE : 32) : 4;

  if (IS_ALIGNED(dest_addr, alignment) && IS_ALIGNED((uintptr_t)src, 4) &&
      IS_ALIGNED(size, alignment))
    {
      /* A single write operation is enough when all parameters are aligned */

      if (flash_enc_enabled && erase)
        {
          if (esp_flash_erase_region(NULL, dest_addr, size) != OK)
            {
              ferr("ERROR: erase failed at 0x%08x", (uintptr_t)dest_addr);
              return false;
            }
        }

      return flash_esp32_write_check_enc(dest_addr, (void *)src, size) == OK;
    }

  finfo("forcing unaligned write dest_addr: "
        "0x%08x src: 0x%08x size: 0x%x erase: %c",
        (uintptr_t)dest_addr, (uintptr_t)src, size, erase ? 't' : 'f');

  while (bytes_remaining > 0)
    {
      size_t aligned_curr_addr = ROUND_DOWN(write_addr, alignment);
      size_t curr_buf_off = write_addr - aligned_curr_addr;
      size_t chunk_len = MIN(bytes_remaining,
                             FLASH_SECTOR_SIZE - curr_buf_off);

      /* Read data before modifying */

      if (flash_esp32_read_check_enc(aligned_curr_addr, write_aux_buf,
                                     ROUND_UP(chunk_len, alignment)) != OK)
        {
          ferr("ERROR: flash read failed at 0x%08x",
               (uintptr_t)aligned_curr_addr);
          return false;
        }

      /* Erase if needed */

      if (flash_enc_enabled && erase)
        {
          if (esp_flash_erase_region(NULL, aligned_curr_addr,
                                     ROUND_UP(
                                      chunk_len, FLASH_SECTOR_SIZE)) != OK)
            {
              ferr("ERROR: flash erase failed at 0x%08x",
                   (uintptr_t)aligned_curr_addr);
              return false;
            }
        }

      /* Merge data into buffer */

      memcpy(&write_aux_buf[curr_buf_off],
             &((const uint8_t *)src)[src_offset],
             chunk_len);

      /* Write back aligned chunk */

      if (flash_esp32_write_check_enc(aligned_curr_addr, write_aux_buf,
                                      ROUND_UP(chunk_len, alignment)) != OK)
        {
          ferr("ERROR: flash write failed at 0x%08x",
               (uintptr_t)aligned_curr_addr);
          return false;
        }

      write_addr += chunk_len;
      src_offset += chunk_len;
      bytes_remaining -= chunk_len;
    }

  return true;
}

/****************************************************************************
 * Name: erase_partial_sector
 *
 * Description:
 *   Erase a partial sector while preserving data outside the erase region.
 *   This function reads the full sector, erases it, then writes back the
 *   preserved data at the head and tail of the sector.
 *
 * Input Parameters:
 *   addr        - Base address of the sector to erase.
 *   sector_size - Size of the sector in bytes.
 *   erase_start - Offset from sector start where erase begins.
 *   erase_end   - Offset from sector start where erase ends.
 *
 * Returned Value:
 *   true on success; false on failure.
 *
 ****************************************************************************/

static bool erase_partial_sector(size_t addr, size_t sector_size,
                                 size_t erase_start, size_t erase_end)
{
  /* Read full sector before erasing */

  if (flash_esp32_read_check_enc(addr, erase_aux_buf, sector_size) != OK)
    {
      ferr("ERROR: flash read failed at 0x%08x", (uintptr_t)addr);
      return false;
    }

  /* Erase full sector */

  if (esp_flash_erase_region(NULL, addr, sector_size) != OK)
    {
      ferr("ERROR: flash erase failed at 0x%08x", (uintptr_t)addr);
      return false;
    }

  /* Write back preserved head data up to erase_start */

  if (erase_start > 0)
    {
      if (!aligned_flash_write(addr, erase_aux_buf, erase_start, false))
        {
          ferr("ERROR: flash write failed at 0x%08x", (uintptr_t)addr);
          return false;
        }
    }

  /* Write back preserved tail data from erase_end up to sector end */

  if (erase_end < sector_size)
    {
      if (!aligned_flash_write(addr + erase_end, &erase_aux_buf[erase_end],
                               sector_size - erase_end, false))
        {
          ferr("ERROR: flash write failed at 0x%08x",
               (uintptr_t)(addr + erase_end));
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Name: aligned_flash_erase
 *
 * Description:
 *   Erase a region of flash with proper sector alignment handling.
 *   This function handles both aligned and unaligned erase operations.
 *   For unaligned operations, it preserves data outside the erase region
 *   by reading sectors, erasing them, and writing back the preserved data.
 *
 * Input Parameters:
 *   addr - Start address of the region to erase.
 *   size - Length of the region to erase in bytes.
 *
 * Returned Value:
 *   true on success; false on failure.
 *
 ****************************************************************************/

static bool aligned_flash_erase(size_t addr, size_t size)
{
  const size_t sector_size = FLASH_SECTOR_SIZE;
  const size_t start_addr = ROUND_DOWN(addr, sector_size);
  const size_t end_addr = ROUND_UP(addr + size, sector_size);
  const size_t total_len = end_addr - start_addr;
  size_t current_addr;

  if (IS_ALIGNED(addr, FLASH_SECTOR_SIZE) && \
      IS_ALIGNED(size, FLASH_SECTOR_SIZE))
    {
      /* A single erase operation is enough when all parameters are aligned */

      return esp_flash_erase_region(NULL, addr, size) == OK;
    }

  finfo("forcing unaligned erase on sector offset: "
        "0x%08x Length: 0x%x total_len: 0x%x",
        (uintptr_t)addr, (int)size, total_len);

  current_addr = start_addr;

  while (current_addr < end_addr)
    {
      bool preserve_head = (addr > current_addr);
      bool preserve_tail = ((addr + size) < (current_addr + sector_size));

      if (preserve_head || preserve_tail)
        {
          size_t erase_start = preserve_head ? (addr - current_addr) : 0;
          size_t erase_end =
            MIN(current_addr + sector_size, addr + size) - current_addr;

          finfo("partial sector erase: 0x%08x to: 0x%08x length: 0x%x",
                (uintptr_t)(current_addr + erase_start),
                (uintptr_t)(current_addr + erase_end),
                erase_end - erase_start);

          if (!erase_partial_sector(current_addr, sector_size, erase_start,
                                    erase_end))
            {
              return false;
            }

          current_addr += sector_size;
        }
      else
        {
          /* Full sector erase is safe, erase the next consecutive full
           * sectors
           */

          size_t contiguous_size =
            ROUND_DOWN(addr + size, sector_size) - current_addr;

          finfo("sectors erased from: 0x%08x length: 0x%x",
                (uintptr_t)current_addr, contiguous_size);

          if (esp_flash_erase_region(
                NULL, current_addr, contiguous_size) != OK)
            {
              ferr("ERROR: flash erase failed at 0x%08x",
                   (uintptr_t)current_addr);
              return false;
            }

          current_addr += contiguous_size;
        }
    }

  return true;
}
#endif /* CONFIG_ESP_FLASH_ENCRYPTION */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_spiflash_read
 *
 * Description:
 *   Read data from flash.
 *
 * Parameters:
 *   address - Source address of the data in flash.
 *   buffer  - Pointer to the destination buffer.
 *   length  - Length of data in bytes.
 *
 * Returned Values:
 *   Zero (OK) is returned or a negative error.
 *
 ****************************************************************************/

int esp_spiflash_read(uint32_t address, void *buffer,
                      uint32_t length)
{
  int ret = OK;

  ret = flash_esp32_read_check_enc(address, buffer, length);
  if (ret != 0)
    {
      ferr("esp_flash_read failed %d", ret);
      ret = ERROR;
    }

  return ret;
}

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
                       uint32_t length)
{
  int ret = OK;
#if CONFIG_ESPRESSIF_SECURE_FLASH_ENC_ENABLED
  bool erase = false;
#endif

#if CONFIG_ESPRESSIF_SECURE_FLASH_ENC_ENABLED
  if (esp_flash_encryption_enabled())
    {
      /* Ensuring flash region has been erased before writing in order to
       * avoid inconsistences when hardware flash encryption is enabled.
       */

      erase = true;
    }

  if (!aligned_flash_write(address, buffer, length, erase))
    {
      ferr("flash erase before write failed\n");
      ret = ERROR;
    }
#else
    ret = flash_esp32_write_check_enc(address, buffer, length);
#endif

  if (ret != OK)
    {
      ferr("ERROR: write failed: ret=%d", ret);
      ret = ERROR;
    }

  return ret;
}

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

int esp_spiflash_erase(uint32_t address, uint32_t length)
{
  int ret = OK;

#if CONFIG_ESPRESSIF_SECURE_FLASH_ENC_ENABLED
  if (!aligned_flash_erase(address, length))
    {
      ret = -EIO;
    }

  if (esp_flash_encryption_enabled())
    {
      uint8_t erased_val_buf[FLASH_BUFFER_SIZE];
      uint32_t bytes_remaining = length;
      uint32_t offset = address;
      uint32_t bytes_written = MIN(sizeof(erased_val_buf), length);

      memset(erased_val_buf, FLASH_ERASE_VALUE, sizeof(erased_val_buf));

      /* When hardware flash encryption is enabled, force expected erased
       * value (0xFF) into flash when erasing a region.
       * This is handled on this implementation because MCUboot's state
       * machine relies on erased valued data (0xFF) read from a
       * previously erased region that was not written yet, however when
       * hardware flash encryption is enabled, the flash read always
       * decrypts what's being read from flash, thus a region that was
       * erased would not be read as what MCUboot expected (0xFF).
       */

      while (bytes_remaining != 0)
        {
          if (!aligned_flash_write(offset, erased_val_buf, bytes_written,
                                   false))
            {
              ferr("ERROR: aligned_flash_write failed during erase\n");
              ret = ERROR;
              break;
            }

          offset += bytes_written;
          bytes_remaining -= bytes_written;
        }
    }
#else
  ret = esp_flash_erase_region(NULL, address, length);
#endif

  if (ret != OK)
    {
      ferr("ERROR: erase failed: ret=%d", ret);
      ret = ERROR;
    }

  return ret;
}
