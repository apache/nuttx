/****************************************************************************
 * arch/xtensa/src/esp32/esp32_allocateheap.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <arch/esp32/memory_layout.h>

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
  start = (FAR void *)HEAP_REGION2_START;
  size  = (size_t)(HEAP_REGION2_END - HEAP_REGION2_START);
  umm_addregion(start, size);

  start = (FAR void *)HEAP_REGION3_START + XTENSA_IMEM_REGION_SIZE;
  size  = (size_t)(uintptr_t)&_eheap - (size_t)start;
  umm_addregion(start, size);
#endif

  start = (FAR void *)HEAP_REGION0_START;
  size  = (size_t)(HEAP_REGION0_END - HEAP_REGION0_START);
  umm_addregion(start, size);

#if defined(CONFIG_ESP32_SPIRAM)
  /* Check for any additional memory regions */

#  if defined(CONFIG_HEAP2_BASE) && defined(CONFIG_HEAP2_SIZE)
    umm_addregion((FAR void *)CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE);
#  endif
#endif
}
#endif

