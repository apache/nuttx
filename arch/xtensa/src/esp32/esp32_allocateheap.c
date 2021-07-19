/****************************************************************************
 * arch/xtensa/src/esp32/esp32_allocateheap.c
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

#include <sys/types.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <arch/esp32/memory_layout.h>

#ifdef CONFIG_ESP32_SPIRAM_BANKSWITCH_ENABLE
#include <nuttx/himem/himem.h>
#include "esp32_himem.h"
#endif

#include "xtensa.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   For the kernel build (CONFIG_BUILD_KERNEL=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function provides the
 *   size of the unprotected, user-space heap.
 *
 *   If a protected kernel-space heap is provided, the kernel heap must be
 *   allocated (and protected) by an analogous up_allocate_kheap().
 *
 ****************************************************************************/

void up_allocate_heap(FAR void **heap_start, size_t *heap_size)
{
  board_autoled_on(LED_HEAPALLOCATE);

  *heap_start = (FAR void *)&_sheap;
  DEBUGASSERT(HEAP_REGION1_END > (uintptr_t)*heap_start);
  *heap_size = (size_t)(HEAP_REGION1_END - (uintptr_t)*heap_start);
}

/****************************************************************************
 * Name: xtensa_add_region
 *
 * Description:
 *   Memory may be added in non-contiguous chunks.  Additional chunks are
 *   added by calling this function.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void xtensa_add_region(void)
{
  void  *start;
  size_t size;
  int availregions;
  int nregions = CONFIG_MM_REGIONS - 1;

#ifdef CONFIG_SMP
  availregions = 3;
#  ifdef CONFIG_BOARD_LATE_INITIALIZE
  availregions++;
#  else
  minfo("A ~3KB heap region can be added to the heap by enabling"
        " CONFIG_BOARD_LATE_INITIALIZE\n");
#  endif
#else
  availregions = 2;
#endif

#ifdef CONFIG_ESP32_SPIRAM
  availregions++;
#endif

  if (nregions < availregions)
    {
      mwarn("Some memory regions are left unused!\n");
      mwarn("Increase CONFIG_MM_NREGIONS to add them to the heap\n");
    }

#ifndef CONFIG_SMP
  start = (FAR void *)(HEAP_REGION2_START + XTENSA_IMEM_REGION_SIZE);
  size  = (size_t)(uintptr_t)&_eheap - (size_t)start;
  umm_addregion(start, size);

#else
#ifdef CONFIG_ESP32_QEMU_IMAGE
  start = (FAR void *)HEAP_REGION2_START;
  size  = (size_t)(uintptr_t)&_eheap - (size_t)start;
  umm_addregion(start, size);
#else
  start = (FAR void *)HEAP_REGION2_START;
  size  = (size_t)(HEAP_REGION2_END - HEAP_REGION2_START);
  umm_addregion(start, size);

  start = (FAR void *)HEAP_REGION3_START + XTENSA_IMEM_REGION_SIZE;
  size  = (size_t)(uintptr_t)&_eheap - (size_t)start;
  umm_addregion(start, size);
#endif
#endif

#ifndef CONFIG_ESP32_QEMU_IMAGE
  start = (FAR void *)HEAP_REGION0_START;
  size  = (size_t)(HEAP_REGION0_END - HEAP_REGION0_START);
  umm_addregion(start, size);
#endif

#ifdef CONFIG_ESP32_SPIRAM
#  if defined(CONFIG_HEAP2_BASE) && defined(CONFIG_HEAP2_SIZE)
#    ifdef CONFIG_XTENSA_EXTMEM_BSS
      start = (FAR void *)(&_ebss_extmem);
      size = CONFIG_HEAP2_SIZE - (size_t)(&_ebss_extmem - &_sbss_extmem);
#    else
      start = (FAR void *)CONFIG_HEAP2_BASE;
      size = CONFIG_HEAP2_SIZE;
#    endif
#  ifdef CONFIG_ESP32_SPIRAM_BANKSWITCH_ENABLE
    size -= esp_himem_reserved_area_size();
#  endif
    umm_addregion(start, size);
#  endif
#endif
}
#endif

