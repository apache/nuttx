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
#include <nuttx/board.h>
#include <nuttx/mm/mm.h>
#include <nuttx/userspace.h>
#include <arch/board/board.h>
#ifdef CONFIG_MM_KERNEL_HEAP
#include <arch/board/board_memorymap.h>
#endif
#include <arch/esp32/memory_layout.h>

#include "xtensa.h"
#ifdef CONFIG_ESP32_SPIRAM
#include "esp32_himem.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_MM_KERNEL_HEAP
#  if defined(CONFIG_ESP32_SPIRAM) && defined(CONFIG_ARCH_HAVE_HEAP2)
#    define MM_USER_HEAP_EXTRAM
#  else
#    define MM_USER_HEAP_IRAM
#  endif

#  define MM_ADDREGION kmm_addregion
#else
#  define MM_ADDREGION umm_addregion
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   For the kernel build (CONFIG_BUILD_KERNEL=y) with both kernel and
 *   userspace heaps (CONFIG_MM_KERNEL_HEAP=y), this function provides the
 *   size of the unprotected, userspace heap.
 *
 *   If a protected kernel space heap is provided, the kernel heap must be
 *   allocated (and protected) by an analogous up_allocate_kheap().
 *
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  uintptr_t ubase;
  uintptr_t utop;
  size_t    usize;

#ifdef CONFIG_MM_KERNEL_HEAP
#  ifdef CONFIG_BUILD_PROTECTED
  ubase = USERSPACE->us_dataend;
  utop  = USERSPACE->us_heapend;
  usize = utop - ubase;
#    ifdef CONFIG_ESP32_USER_DATA_EXTMEM
  usize -= esp_himem_reserved_area_size();
#    endif

#  elif defined(CONFIG_BUILD_FLAT)
#    ifdef MM_USER_HEAP_EXTRAM
#      ifdef CONFIG_XTENSA_EXTMEM_BSS
  ubase = (uintptr_t)_ebss_extmem;
  usize = CONFIG_HEAP2_SIZE - (size_t)(_ebss_extmem - _sbss_extmem);
#      else
  ubase = CONFIG_HEAP2_BASE;
  usize = CONFIG_HEAP2_SIZE;
#      endif
  usize -= esp_himem_reserved_area_size();
  utop  = ubase + usize;

#    elif defined(MM_USER_HEAP_IRAM)
  ubase = ESP32_IMEM_START + XTENSA_IMEM_REGION_SIZE;
  utop  = (uintptr_t)_eheap;
  usize = utop - ubase;
#    endif /* MM_USER_HEAP_EXTRAM */

#  endif /* CONFIG_BUILD_PROTECTED */

#else /* !CONFIG_MM_KERNEL_HEAP */
  ubase = (uintptr_t)_sheap;
  utop  = HEAP_REGION1_END;
  usize = utop - ubase;
#endif /* CONFIG_MM_KERNEL_HEAP */

  minfo("Heap: start=%" PRIxPTR " end=%" PRIxPTR " size=%zu\n",
        ubase, utop, usize);

  DEBUGASSERT(utop > ubase);

  board_autoled_on(LED_HEAPALLOCATE);

  /* Return the userspace heap settings */

  *heap_start = (void *)ubase;
  *heap_size  = usize;
}

/****************************************************************************
 * Name: up_allocate_kheap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   For the kernel build (CONFIG_BUILD_PROTECTED=y) with both kernel and
 *   userspace heaps (CONFIG_MM_KERNEL_HEAP=y), this function allocates
 *   (and protects) the kernel space heap.
 *
 *   For Flat build (CONFIG_BUILD_FLAT=y), this function enables a separate
 *   (although unprotected) heap for the kernel.
 *
 ****************************************************************************/

#ifdef CONFIG_MM_KERNEL_HEAP
void up_allocate_kheap(void **heap_start, size_t *heap_size)
{
  uintptr_t kbase;
  uintptr_t ktop;
  size_t    ksize;

#ifdef CONFIG_BUILD_PROTECTED
  /* These values come from the linker scripts (kernel-space.ld and
   * protected_memory.ld).
   * Check boards/xtensa/esp32.
   */

  kbase = (uintptr_t)_sheap;
  ktop  = KDRAM_0_END;
  ksize = ktop - kbase;
#elif defined(CONFIG_BUILD_FLAT)
  kbase = (uintptr_t)_sheap;
  ktop  = HEAP_REGION1_END;
  ksize = ktop - kbase;
#endif

  minfo("Heap: start=%" PRIxPTR " end=%" PRIxPTR " size=%zu\n",
        kbase, ktop, ksize);

  DEBUGASSERT(ktop > kbase);

  board_autoled_on(LED_HEAPALLOCATE);

  *heap_start = (void *)kbase;
  *heap_size  = ksize;
}
#endif /* CONFIG_MM_KERNEL_HEAP */

/****************************************************************************
 * Name: xtensa_add_region
 *
 * Description:
 *   Memory may be added in non-contiguous chunks. Additional chunks are
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

#ifdef CONFIG_ESP32_SPIRAM_COMMON_HEAP
  availregions++;
#endif

  if (nregions < availregions)
    {
      mwarn("Some memory regions are left unused!\n");
      mwarn("Increase CONFIG_MM_REGIONS to add them to the heap\n");
    }

#ifdef CONFIG_SMP
  start = (void *)HEAP_REGION2_START;
  size  = (size_t)(HEAP_REGION2_END - HEAP_REGION2_START);
  MM_ADDREGION(start, size);
#endif

#ifndef MM_USER_HEAP_IRAM
  /* Skip internal heap region if CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP is
   * enabled.
   */

  start = (void *)ESP32_IMEM_START + XTENSA_IMEM_REGION_SIZE;
  size  = (size_t)(uintptr_t)_eheap - (size_t)start;
  MM_ADDREGION(start, size);
#endif

#ifndef CONFIG_ESP32_BLE
  start = (void *)HEAP_REGION0_START;
  size  = (size_t)(HEAP_REGION0_END - HEAP_REGION0_START);
  MM_ADDREGION(start, size);
#endif

#ifdef CONFIG_ESP32_SPIRAM_COMMON_HEAP
#ifdef CONFIG_XTENSA_EXTMEM_BSS
  start = (void *)(_ebss_extmem);
  size  = CONFIG_HEAP2_SIZE - (size_t)(_ebss_extmem - _sbss_extmem);
#else
  start = (void *)CONFIG_HEAP2_BASE;
  size  = CONFIG_HEAP2_SIZE;
#endif
  size -= esp_himem_reserved_area_size();

  MM_ADDREGION(start, size);
#endif
}
#endif

