/****************************************************************************
 * arch/xtensa/src/esp32/esp32_himem.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_HIMEM_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_HIMEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Indicates that a mapping will only be read from. Note that this is unused
 * for now.
 */

#define ESP_HIMEM_MAPFLAG_RO 1

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/* Allocate a block in high memory
 *
 * params:
 *   size Size of the to-be-allocated block, in bytes. Note that this
 *        needs to be a multiple of the external RAM mmu block size
 *        (32K).
 *   [out] handle_out Handle to be returned
 *
 * returns:
 *   - ESP_OK if successful
 *   - ESP_ERR_NO_MEM if out of memory
 *   - ESP_ERR_INVALID_SIZE if size is not a multiple of 32K
 */

int esp_himem_alloc(size_t size, esp_himem_handle_t *handle_out);

/* Description: Allocate a memory region to map blocks into
 *
 * This allocates a contiguous CPU memory region that can be used to map
 * blocks of physical memory into.
 *
 * params:
 *   size Size of the range to be allocated. Note this needs to be a multiple
 *        of the external RAM mmu block size (32K).
 *   [out] handle_out Handle to be returned
 *
 * returns:
 *   - ESP_OK if successful
 *   - ESP_ERR_NO_MEM if out of memory or address space
 *   - ESP_ERR_INVALID_SIZE if size is not a multiple of 32K
 */

int esp_himem_alloc_map_range(size_t size, esp_himem_rangehandle_t
                              *handle_out);

/* Description: Map a block of high memory into the CPUs address space
 *
 * This effectively makes the block available for read/write operations.
 *
 * Note: The region to be mapped needs to have offsets and sizes that are
 *       aligned to the SPI RAM MMU block size (32K)
 *
 * params:
 *   handle       Handle to the block of memory, as given by esp_himem_alloc
 *   range        Range handle to map the memory in
 *   ram_offset   Offset into the block of physical memory of the block to
 *                map
 *   range_offset Offset into the address range where the block will be
 *                mapped,
 *   len          Length of region to map
 *   flags        One of ESP_HIMEM_MAPFLAG_*
 *   [out] out_ptr Pointer to variable to store resulting memory pointer in
 *
 * returns:
 *   - ESP_OK if the memory could be mapped
 *   - ESP_ERR_INVALID_ARG if offset, range or len aren't MMU-block-aligned
 *     (32K)
 *   - ESP_ERR_INVALID_SIZE if the offsets/lengths don't fit in the allocated
 *       memory or range
 *   - ESP_ERR_INVALID_STATE if a block in the selected ram offset/length is
 *       already mapped, or if a block in the selected range offset/length
 *       already has a mapping.
 */

int esp_himem_map(esp_himem_handle_t handle,
                  esp_himem_rangehandle_t range,
                  size_t ram_offset,
                  size_t range_offset,
                  size_t len,
                  int flags,
                  void **out_ptr);

/* Description: Free a block of physical memory
 *
 * This clears out the associated handle making the memory available for
 * re-allocation again.
 * This will only succeed if none of the memory blocks currently have a
 *   mapping.
 *
 * params:
 *   handle Handle to the block of memory, as given by esp_himem_alloc
 *
 * returns:
 *   - ESP_OK if the memory is successfully freed
 *   - ESP_ERR_INVALID_ARG if the handle still is (partially) mapped
 */

int esp_himem_free(esp_himem_handle_t handle);

/* Description: Free a mapping range
 *
 * This clears out the associated handle making the range available for
 * re-allocation again.
 * This will only succeed if none of the range blocks currently are used for
 * a mapping.
 *
 * params:
 *   handle Handle to the range block, as given by esp_himem_alloc_map_range
 *
 * returns:
 *   - ESP_OK if the memory is successfully freed
 *   - ESP_ERR_INVALID_ARG if the handle still is (partially) mapped to
 */

int esp_himem_free_map_range(esp_himem_rangehandle_t handle);

/* Description: Unmap a region
 *
 * params:
 *   range Range handle
 *   ptr   Pointer returned by esp_himem_map
 *   len   Length of the block to be unmapped. Must be aligned to the SPI RAM
 *         MMU blocksize (32K)
 * returns:
 *   - ESP_OK if the memory is successfully unmapped,
 *   - ESP_ERR_INVALID_ARG if ptr or len are invalid.
 */

int esp_himem_unmap(esp_himem_rangehandle_t range, void *ptr,
                    size_t len);

/* Description: Get total amount of memory under control of himem API
 *
 * returns:
 *   Amount of memory, in bytes
 */

size_t esp_himem_get_phys_size(void);

/* Description: Get free amount of memory under control of himem API
 *
 * returns:
 *   Amount of free memory, in bytes
 */

size_t esp_himem_get_free_size(void);

/* Description: Get amount of SPI memory address space needed for
 *              bankswitching
 *
 * Note: This is also weakly defined in esp32/spiram.c and returns 0 there,
 *       so if no other function in this file is used, no memory is reserved.
 *
 * return:
 *   Amount of reserved area, in bytes
 */

size_t esp_himem_reserved_area_size(void);

#ifdef __cplusplus
}
#endif
#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_HIMEM_H */
