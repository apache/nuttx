/****************************************************************************
 * arch/xtensa/src/esp32/esp32_iramheap.h
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

#ifndef __ARCH_XTENSA_SRC_ESP32_ESP32_IRAMHEAP_H
#define __ARCH_XTENSA_SRC_ESP32_ESP32_IRAMHEAP_H

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct mallinfo; /* Forward reference, see malloc.h */

/****************************************************************************
 * Name: esp32_iramheap_initialize
 *
 * Description:
 *   Initialize the IRAM heap.
 *
 ****************************************************************************/

void esp32_iramheap_initialize(void);

/****************************************************************************
 * Name: esp32_iramheap_malloc
 *
 * Description:
 *   Allocate memory from the IRAM heap.
 *
 ****************************************************************************/

void *esp32_iramheap_malloc(size_t size);

/****************************************************************************
 * Name: esp32_iramheap_calloc
 *
 * Description:
 *   Calculates the size of the allocation and allocate memory from
 *   the IRAM heap.
 *
 ****************************************************************************/

void *esp32_iramheap_calloc(size_t n, size_t elem_size);

/****************************************************************************
 * Name: esp32_iramheap_realloc
 *
 * Description:
 *   Reallocate memory from the IRAM heap.
 *
 ****************************************************************************/

void *esp32_iramheap_realloc(void *ptr, size_t size);

/****************************************************************************
 * Name: esp32_iramheap_zalloc
 *
 * Description:
 *   Allocate and zero memory from the IRAM heap.
 *
 ****************************************************************************/

void *esp32_iramheap_zalloc(size_t size);

/****************************************************************************
 * Name: esp32_iramheap_free
 *
 * Description:
 *   Free memory from the IRAM heap.
 *
 ****************************************************************************/

void esp32_iramheap_free(void *mem);

/****************************************************************************
 * Name: esp32_iramheap_memalign
 *
 * Description:
 *   memalign requests more than enough space from malloc, finds a region
 *   within that chunk that meets the alignment request and then frees any
 *   leading or trailing space.
 *
 *   The alignment argument must be a power of two (not checked). 8-byte
 *   alignment is guaranteed by normal malloc calls.
 *
 ****************************************************************************/

void *esp32_iramheap_memalign(size_t alignment, size_t size);

/****************************************************************************
 * Name: esp32_iramheap_heapmember
 *
 * Description:
 *   Check if an address lies in the IRAM heap.
 *
 * Parameters:
 *   mem - The address to check
 *
 * Return Value:
 *   true if the address is a member of the IRAM heap. false if not
 *
 ****************************************************************************/

bool esp32_iramheap_heapmember(void *mem);

/****************************************************************************
 * Name: esp32_iramheap_mallinfo
 *
 * Description:
 *   mallinfo returns a copy of updated current heap information for the
 *   user heap.
 *
 ****************************************************************************/

int esp32_iramheap_mallinfo(struct mallinfo *info);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_XTENSA_SRC_ESP32_ESP32_IRAMHEAP_H */
