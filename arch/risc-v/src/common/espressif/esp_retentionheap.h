/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_retentionheap.h
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

#ifndef __ARCH_RISC_V_SRC_COMMON_ESPRESSIF_ESP_RETENTIONHEAP_H
#define __ARCH_RISC_V_SRC_COMMON_ESPRESSIF_ESP_RETENTIONHEAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_ESPRESSIF_RETENTION_HEAP)

/****************************************************************************
 * Name: esp_retentionheap_find_region
 *
 * Description:
 *   Find address of retention heap.
 *
 * Input Parameters:
 *   rstart - Starting address of the region
 *   rend   - Ending address of the region
 *
 * Returned Value:
 *   True if region found; false otherwise
 *
 ****************************************************************************/

bool esp_retentionheap_find_region(uintptr_t *rstart, uintptr_t *rend);

/****************************************************************************
 * Name: esp_retentionheap_initialize
 *
 * Description:
 *   Initialize the retention heap.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void esp_retentionheap_initialize(void);

/****************************************************************************
 * Name: esp_retentionheap_malloc
 *
 * Description:
 *   Allocate memory from the retention heap.
 *
 * Input Parameters:
 *   size - Size to allocate memory in bytes
 *
 * Returned Value:
 *   Valid address reference on success; NULL, otherwise.
 *
 ****************************************************************************/

void *esp_retentionheap_malloc(size_t size);

/****************************************************************************
 * Name: esp_retentionheap_calloc
 *
 * Description:
 *   Calculates the size of the allocation and allocate memory from
 *   the retention heap.
 *
 * Input Parameters:
 *   n         - Number of elements to allocate.
 *   elem_size - Size of each element, in bytes.
 *
 * Returned Value:
 *   Valid address reference on success; NULL, otherwise.
 *
 ****************************************************************************/

void *esp_retentionheap_calloc(size_t n, size_t elem_size);

/****************************************************************************
 * Name: esp_retentionheap_memalign
 *
 * Description:
 *   memalign requests more than enough space from malloc, finds a region
 *   within that chunk that meets the alignment request and then frees any
 *   leading or trailing space.
 *
 *   The alignment argument must be a power of two (not checked). 8-byte
 *   alignment is guaranteed by normal malloc calls.
 *
 * Input Parameters:
 *   alignment - Required byte alignment; must be a power of two and usually
 *               at least sizeof(void *).
 *   size      - Number of bytes to allocate.
 *
 * Returned Value:
 *   Valid address reference on success; NULL, otherwise.
 *
 ****************************************************************************/

void *esp_retentionheap_memalign(size_t alignment, size_t size);

/****************************************************************************
 * Name: esp_retentionheap_realloc
 *
 * Description:
 *   Reallocate memory from the retention heap.
 *
 * Input Parameters:
 *   mem  - The address to reallocate
 *   size - New requested size
 *
 * Returned Value:
 *   Valid address reference on success; NULL, otherwise.
 *
 ****************************************************************************/

void *esp_retentionheap_realloc(void *ptr, size_t size);

/****************************************************************************
 * Name: esp_retentionheap_free
 *
 * Description:
 *   Free memory from the retention heap.
 *
 * Input Parameters:
 *   mem - The address to free
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_retentionheap_free(void *mem);

/****************************************************************************
 * Name: esp_retentionheap_heapmember
 *
 * Description:
 *   Check if an address lies in the retention heap.
 *
 * Parameters:
 *   mem - The address to check
 *
 * Return Value:
 *   True if the address is a member of the RTC heap. false if not
 *
 ****************************************************************************/

bool esp_retentionheap_heapmember(void *mem);

/****************************************************************************
 * Name: esp_retentionheap_malloc_size
 *
 * Description:
 *   Get size of the allocated space in retention heap
 *
 * Input Parameters:
 *   mem - The address to check
 *
 * Returned Value:
 *   Size of allocated space related to address.
 *
 ****************************************************************************/

size_t esp_retentionheap_malloc_size(void *mem);

#endif /* CONFIG_ESPRESSIF_RETENTION_HEAP */

#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_RISC_V_SRC_COMMON_ESPRESSIF_ESP_RETENTIONHEAP_H */
