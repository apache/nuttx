/****************************************************************************
 * arch/xtensa/src/esp32/esp32_iramheap.c
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
#include <nuttx/mm/mm.h>
#include <malloc.h>

#include "esp32_iramheap.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mm_heap_s *g_iramheap;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_iramheap_initialize
 *
 * Description:
 *   Initialize the IRAM heap.
 *
 ****************************************************************************/

void esp32_iramheap_initialize(void)
{
  void  *start;
  size_t size;

  /* These values come from the linker scripts. Check boards/xtensa/esp32. */

  extern uint8_t *_siramheap;
  extern uint8_t *_eiramheap;

  start = (void *)&_siramheap;
  size  = (size_t)((uintptr_t)&_eiramheap - (uintptr_t)&_siramheap);
  g_iramheap = mm_initialize("iramheap", start, size);
}

/****************************************************************************
 * Name: esp32_iramheap_malloc
 *
 * Description:
 *   Allocate memory from the IRAM heap.
 *
 ****************************************************************************/

void *esp32_iramheap_malloc(size_t size)
{
  return mm_malloc(g_iramheap, size);
}

/****************************************************************************
 * Name: esp32_iramheap_calloc
 *
 * Description:
 *   Calculates the size of the allocation and allocate memory from
 *   the IRAM heap.
 *
 ****************************************************************************/

void *esp32_iramheap_calloc(size_t n, size_t elem_size)
{
  return mm_calloc(g_iramheap, n, elem_size);
}

/****************************************************************************
 * Name: esp32_iramheap_realloc
 *
 * Description:
 *   Reallocate memory from the IRAM heap.
 *
 ****************************************************************************/

void *esp32_iramheap_realloc(void *ptr, size_t size)
{
  return mm_realloc(g_iramheap, ptr, size);
}

/****************************************************************************
 * Name: esp32_iramheap_zalloc
 *
 * Description:
 *   Allocate and zero memory from the IRAM heap.
 *
 ****************************************************************************/

void *esp32_iramheap_zalloc(size_t size)
{
  return mm_zalloc(g_iramheap, size);
}

/****************************************************************************
 * Name: esp32_iramheap_free
 *
 * Description:
 *   Free memory from the IRAM heap.
 *
 ****************************************************************************/

void esp32_iramheap_free(void *mem)
{
  mm_free(g_iramheap, mem);
}

/****************************************************************************
 * Name: esp32_iramheap_memalign
 *
 * Description:
 *   memalign requests more than enough space from malloc, finds a region
 *   within that chunk that meets the alignment request and then frees any
 *   leading or trailing space.
 *
 *   The alignment argument must be a power of two (not checked).  8-byte
 *   alignment is guaranteed by normal malloc calls.
 *
 ****************************************************************************/

void *esp32_iramheap_memalign(size_t alignment, size_t size)
{
  return mm_memalign(g_iramheap, alignment, size);
}

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
 *   true if the address is a member of the IRAM heap.  false if not
 *
 ****************************************************************************/

bool esp32_iramheap_heapmember(void *mem)
{
  return mm_heapmember(g_iramheap, mem);
}

/****************************************************************************
 * Name: esp32_iramheap_mallinfo
 *
 * Description:
 *   mallinfo returns a copy of updated current heap information for the
 *   user heap.
 *
 ****************************************************************************/

int esp32_iramheap_mallinfo(struct mallinfo *info)
{
  return mm_mallinfo(g_iramheap, info);
}
