/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_rtcheap.c
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

#include "esp32s3_rtcheap.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mm_heap_s *g_rtcheap;

/****************************************************************************
 * Extern values declaration
 ****************************************************************************/

/* These values come from the linker scripts. Check boards/xtensa/esp32s3.  */

extern uint8_t _srtcheap[];
extern uint8_t _ertcheap[];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_rtcheap_initialize
 *
 * Description:
 *   Initialize the RTC heap.
 *
 ****************************************************************************/

void esp32s3_rtcheap_initialize(void)
{
  void *start;
  size_t size;

  start = (void *)_srtcheap;
  size  = _ertcheap - _srtcheap;
  g_rtcheap = mm_initialize("rtcheap", start, size);
}

/****************************************************************************
 * Name: esp32s3_rtcheap_malloc
 *
 * Description:
 *   Allocate memory from the RTC heap.
 *
 * Parameters:
 *   size - Size (in bytes) of the memory region to be allocated.
 *
 ****************************************************************************/

void *esp32s3_rtcheap_malloc(size_t size)
{
  return mm_malloc(g_rtcheap, size);
}

/****************************************************************************
 * Name: esp32s3_rtcheap_calloc
 *
 * Description:
 *   Calculates the size of the allocation and allocate memory from
 *   the RTC heap.
 *
 ****************************************************************************/

void *esp32s3_rtcheap_calloc(size_t n, size_t elem_size)
{
  return mm_calloc(g_rtcheap, n, elem_size);
}

/****************************************************************************
 * Name: esp32s3_rtcheap_realloc
 *
 * Description:
 *   Reallocate memory from the RTC heap.
 *
 ****************************************************************************/

void *esp32s3_rtcheap_realloc(void *ptr, size_t size)
{
  return mm_realloc(g_rtcheap, ptr, size);
}

/****************************************************************************
 * Name: esp32s3_rtcheap_zalloc
 *
 * Description:
 *   Allocate and zero memory from the RTC heap.
 *
 ****************************************************************************/

void *esp32s3_rtcheap_zalloc(size_t size)
{
  return mm_zalloc(g_rtcheap, size);
}

/****************************************************************************
 * Name: esp32s3_rtcheap_free
 *
 * Description:
 *   Free memory from the RTC heap.
 *
 ****************************************************************************/

void esp32s3_rtcheap_free(void *mem)
{
  mm_free(g_rtcheap, mem);
}

/****************************************************************************
 * Name: esp32s3_rtcheap_memalign
 *
 * Description:
 *   memalign requests more than enough space from malloc, finds a region
 *   within that chunk that meets the alignment request and then frees any
 *   leading or trailing space.
 *
 *   The alignment argument must be a power of two (not checked).  8-byte
 *   alignment is guaranteed by normal malloc calls.
 *
 * Parameters:
 *   alignment - Requested alignment.
 *   size - Size (in bytes) of the memory region to be allocated.
 *
 ****************************************************************************/

void *esp32s3_rtcheap_memalign(size_t alignment, size_t size)
{
  return mm_memalign(g_rtcheap, alignment, size);
}

/****************************************************************************
 * Name: esp32s3_rtcheap_heapmember
 *
 * Description:
 *   Check if an address lies in the RTC heap.
 *
 * Parameters:
 *   mem - The address to check.
 *
 * Return Value:
 *   True if the address is a member of the RTC heap.  False if not.
 *
 ****************************************************************************/

bool esp32s3_rtcheap_heapmember(void *mem)
{
  return mm_heapmember(g_rtcheap, mem);
}

/****************************************************************************
 * Name: esp32s3_rtcheap_mallinfo
 *
 * Description:
 *   mallinfo returns a copy of updated current heap information for the
 *   user heap.
 *
 * Parameters:
 *   info - Where memory information will be copied.
 *
 ****************************************************************************/

struct mallinfo esp32s3_rtcheap_mallinfo(void)
{
  return mm_mallinfo(g_rtcheap);
}
