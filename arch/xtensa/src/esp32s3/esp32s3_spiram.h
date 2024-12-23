/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_spiram.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_SPIRAM_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_SPIRAM_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: cache_dbus_mmu_map
 *
 * Description:
 *   Set Ext-SRAM-Cache mmu mapping.
 *
 * Input Parameters:
 *   vaddr - Virtual address in CPU address space
 *   paddr - Physical address in Ext-SRAM
 *   num   - Pages to be set
 *
 * Returned Value:
 *   0 if success or a negative value if fail.
 *
 ****************************************************************************/

int cache_dbus_mmu_map(int vaddr, int paddr, int num);

/* Initialize spiram interface/hardware. Normally called from
 * cpu_start.c.
 *
 *  Return ESP_OK on success
 */

int esp_spiram_init(void);

/* Configure Cache/MMU for access to external SPI RAM.
 *
 * Normally this function is called from cpu_start, if
 * CONFIG_SPIRAM_BOOT_INIT option is enabled. Applications which need to
 * enable SPI RAM at run time can disable CONFIG_SPIRAM_BOOT_INIT, and
 * call this function later.
 *
 * Attention this function must be called with flash cache disabled.
 */

int esp_spiram_init_cache(void);

/* Memory test for SPI RAM. Should be called after SPI RAM is
 * initialized and (in case of a dual-core system) the app CPU is online.
 * This test overwrites the memory with crap, so do not call after e.g. the
 * heap allocator has stored important stuff in SPI RAM.
 *
 *  Return true on success, false on failed memory test
 */

int esp_spiram_test(void);

/* Add the initialized SPI RAM to the heap allocator. */

int esp_spiram_add_to_heapalloc(void);

/* Get the available physical size of the attached SPI RAM chip
 *
 * Note if ECC is enabled, the available physical size would be smaller
 * than the physical size. See `CONFIG_ESP32S3_SPIRAM_ECC_ENABLE`
 *
 *  Return Size in bytes, or 0 if no external RAM chip support compiled in.
 */

size_t esp_spiram_get_size(void);

/* Force a writeback of the data in the SPI RAM cache. This is to be
 * called whenever cache is disabled, because disabling cache on the ESP32
 * discards the data in the SPI RAM cache.
 *
 * This is meant for use from within the SPI flash code.
 */

void esp_spiram_writeback_cache(void);

/* If SPI RAM(PSRAM) has been initialized
 *
 *  Return
 *          - true SPI RAM has been initialized successfully
 *          - false SPI RAM hasn't been initialized or initialized failed
 */

bool esp_spiram_is_initialized(void);

/* Get psram CS IO
 *
 * This interface should be called after PSRAM is enabled, otherwise it will
 * return an invalid value -1/0xff.
 *
 *  Return psram CS IO or -1/0xff if psram not enabled
 */

uint8_t esp_spiram_get_cs_io(void);

/* Reserve a pool of internal memory for specific DMA/internal
 * allocations
 *
 * size Size of reserved pool in bytes
 *
 *  Return
 *          - ESP_OK on success
 *          - ESP_ERR_NO_MEM when no memory available for pool
 */

int esp_spiram_reserve_dma_pool(size_t size);

/* If SPI RAM(PSRAM) has been initialized
 *
 *  Return
 *          - true SPI RAM has been initialized successfully
 *          - false SPI RAM hasn't been initialized or initialized failed
 */

bool esp_spiram_is_initialized(void);

#if defined(CONFIG_ESP32S3_SPIRAM_FETCH_INSTRUCTIONS)

extern uint8_t _instruction_reserved_start[];
extern uint8_t _instruction_reserved_end[];

/* Get the start page number of the instruction in SPI flash
 *
 *  Return start page number
 */

uint32_t instruction_flash_start_page_get(void);

/* Get the end page number of the instruction in SPI flash
 *
 *  Return end page number
 */

uint32_t instruction_flash_end_page_get(void);

/* Get the offset of instruction from SPI flash to SPI RAM
 *
 *  Return instruction offset
 */

int instruction_flash2spiram_offset(void);

#endif

#if defined(CONFIG_SPIRAM_RODATA)

extern uint8_t _rodata_reserved_start[];
extern uint8_t _rodata_reserved_end[];

/* Get the start page number of the rodata in SPI flash
 *
 *  Return start page number
 */

uint32_t rodata_flash_start_page_get(void);

/* Get the end page number of the rodata in SPI flash
 *
 *  Return end page number
 */

uint32_t rodata_flash_end_page_get(void);

/* Get the offset number of rodata from SPI flash to SPI RAM
 *
 *  Return rodata offset
 */

int rodata_flash2spiram_offset(void);

#endif

/* Get allocable virtual start address
 *
 *  Return Allocable virtual start address
 */

uint32_t esp_spiram_allocable_vaddr_start(void);

/* Get allocable virtual end address
 *
 *  Return Allocable virtual end address
 */

uint32_t esp_spiram_allocable_vaddr_end(void);

#ifdef __cplusplus
}
#endif

#endif
