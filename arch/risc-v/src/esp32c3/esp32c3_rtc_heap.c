/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_rtc_heap.c
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
#include <nuttx/fs/procfs.h>
#include <nuttx/mm/mm.h>
#include <malloc.h>

#include "esp32c3_rtc_heap.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mm_heap_s g_rtc_heap;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_rtc_heap_initialize
 *
 * Description:
 *   Initialize the RTC heap.
 *
 ****************************************************************************/

void esp32c3_rtc_heap_initialize(void)
{
  void  *start;
  size_t size;

  /* These values come from the linker scripts (esp32c3.ld and
   * esp32c3.template.ld.)  Check boards/risc-v/esp32c3.
   */

  extern uint8_t *_srtc_heap;
  extern uint8_t *_ertc_heap;

  start = (FAR void *)&_srtc_heap;
  size  = (size_t)((uintptr_t)&_ertc_heap - (uintptr_t)&_srtc_heap);
  mm_initialize(&g_rtc_heap, start, size);

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
  static struct procfs_meminfo_entry_s g_rtc_procfs;

  g_rtc_procfs.name = "rtc_heap";
  g_rtc_procfs.mallinfo = (void *)mm_mallinfo;
  g_rtc_procfs.user_data = &g_rtc_heap;
  procfs_register_meminfo(&g_rtc_procfs);
#endif
}

/****************************************************************************
 * Name: esp32c3_rtc_heap_malloc
 *
 * Description:
 *   Allocate memory from the RTC heap.
 *
 ****************************************************************************/

void *esp32c3_rtc_heap_malloc(size_t size)
{
  return mm_malloc(&g_rtc_heap, size);
}

/****************************************************************************
 * Name: esp32c3_rtc_heap_calloc
 *
 * Description:
 *   Calculates the size of the allocation and allocate memory from
 *   the RTC heap.
 *
 ****************************************************************************/

void *esp32c3_rtc_heap_calloc(size_t n, size_t elem_size)
{
  return mm_calloc(&g_rtc_heap, n, elem_size);
}

/****************************************************************************
 * Name: esp32c3_rtc_heap_realloc
 *
 * Description:
 *   Reallocate memory from the RTC heap.
 *
 ****************************************************************************/

void *esp32c3_rtc_heap_realloc(void *ptr, size_t size)
{
  return mm_realloc(&g_rtc_heap, ptr, size);
}

/****************************************************************************
 * Name: esp32c3_rtc_heap_zalloc
 *
 * Description:
 *   Allocate and zero memory from the RTC heap.
 *
 ****************************************************************************/

void *esp32c3_rtc_heap_zalloc(size_t size)
{
  return mm_zalloc(&g_rtc_heap, size);
}

/****************************************************************************
 * Name: esp32c3_rtc_heap_free
 *
 * Description:
 *   Free memory from the RTC heap.
 *
 ****************************************************************************/

void esp32c3_rtc_heap_free(FAR void *mem)
{
  mm_free(&g_rtc_heap, mem);
}

/****************************************************************************
 * Name: esp32c3_rtc_heap_memalign
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

void *esp32c3_rtc_heap_memalign(size_t alignment, size_t size)
{
  return mm_memalign(&g_rtc_heap, alignment, size);
}

/****************************************************************************
 * Name: esp32c3_rtc_heap_heapmember
 *
 * Description:
 *   Check if an address lies in the RTC heap.
 *
 * Parameters:
 *   mem - The address to check
 *
 * Return Value:
 *   true if the address is a member of the RTC heap.  false if not
 *
 ****************************************************************************/

bool esp32c3_rtc_heap_heapmember(FAR void *mem)
{
  return mm_heapmember(&g_rtc_heap, mem);
}

/****************************************************************************
 * Name: esp32c3_rtc_heap_mallinfo
 *
 * Description:
 *   mallinfo returns a copy of updated current heap information for the
 *   user heap.
 *
 ****************************************************************************/

int esp32c3_rtc_heap_mallinfo(FAR struct mallinfo *info)
{
  return mm_mallinfo(&g_rtc_heap, info);
}
