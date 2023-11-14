/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_himem.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_HIMEM_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_HIMEM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>

#include <nuttx/himem/himem.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_himem_reserved_area_size
 *
 * Description:
 *   Get amount of SPI memory address space needed for bankswitching.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Amount of reserved area, in bytes.
 *
 ****************************************************************************/

size_t esp_himem_reserved_area_size(void);

/****************************************************************************
 * Name: esp_himem_get_phys_size
 *
 * Description:
 *   Get total amount of memory under control of himem API.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Amount of memory, in bytes.
 *
 ****************************************************************************/

size_t esp_himem_get_phys_size(void);

/****************************************************************************
 * Name: esp_himem_get_free_size
 *
 * Description:
 *   Get free amount of memory under control of himem API.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Amount of memory, in bytes.
 *
 ****************************************************************************/

size_t esp_himem_get_free_size(void);

/****************************************************************************
 * Name: esp_himem_alloc
 *
 * Description:
 *   Allocate a block in high memory.
 *
 * Input Parameters:
 *   size       - Size of the to-be-allocated block, in bytes. Note that
 *                this needs to be a multiple of the external RAM mmu block
 *                size(64K).
 *   handle_out - Handle to be returned
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int esp_himem_alloc(size_t size, esp_himem_handle_t *handle_out);

/****************************************************************************
 * Name: esp_himem_free
 *
 * Description:
 *   Free a block of physical memory, this clears out the associated handle
 *   making the memory available for re-allocation again, this will only
 *   succeed if none of the memory blocks currently have a mapping.
 *
 * Input Parameters:
 *   handle - Handle to the block of memory, as given by esp_himem_alloc.
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int esp_himem_free(esp_himem_handle_t handle);

/****************************************************************************
 * Name: esp_himem_alloc_map_range
 *
 * Description:
 *   Allocate a memory region to map blocks into, this allocates a
 *   contiguous CPU memory region that can be used to map blocks of
 *   physical memory into.
 *
 * Input Parameters:
 *   size       - Size of the range to be allocated. Note this needs to be a
 *                multiple of the external RAM mmu block size (64K).
 *   handle_out - Handle to be returned
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int esp_himem_alloc_map_range(size_t size, esp_himem_rangehandle_t
                              *handle_out);

/****************************************************************************
 * Name: esp_himem_free_map_range
 *
 * Description:
 *   Free a mapping range, this clears out the associated handle making the
 *   range available for re-allocation again, This will only succeed if none
 *   of the range blocks currently are used for a mapping.
 *
 * Input Parameters:
 *   handle - Handle to the range block, as given by
 *            esp_himem_alloc_map_range
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int esp_himem_free_map_range(esp_himem_rangehandle_t handle);

/****************************************************************************
 * Name: esp_himem_map
 *
 * Description:
 *   Map a block of high memory into the CPUs address space, this effectively
 *   makes the block available for read/write operations.
 *
 * Input Parameters:
 *   handle       - Handle to the block of memory, as given by
 *                  esp_himem_alloc
 *   range        - Range handle to map the memory in
 *   ram_offset   - Offset into the block of physical memory of the block to
 *                  map
 *   range_offset - Offset into the address range where the block will be
 *                  mapped
 *   len          - Length of region to map
 *   flags        - One of ESP_HIMEM_MAPFLAG_*
 *   out_ptr      - Pointer to variable to store resulting memory pointer in
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int esp_himem_map(esp_himem_handle_t handle,
                  esp_himem_rangehandle_t range,
                  size_t ram_offset,
                  size_t range_offset,
                  size_t len,
                  int flags,
                  void **out_ptr);

/****************************************************************************
 * Name: esp_himem_unmap
 *
 * Description:
 *   Unmap a region.
 *
 * Input Parameters:
 *   range - Range handle
 *   ptr   - Pointer returned by esp_himem_map
 *   len   - Length of the block to be unmapped, must be aligned to the
 *           SPI RAM MMU blocksize (64K)
 *
 * Returned Value:
 *   OK if success or a negative value if fail.
 *
 ****************************************************************************/

int esp_himem_unmap(esp_himem_rangehandle_t range, void *ptr, size_t len);

#ifdef __cplusplus
}
#endif
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_HIMEM_H */
