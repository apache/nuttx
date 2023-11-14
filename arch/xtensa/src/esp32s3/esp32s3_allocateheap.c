/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_allocateheap.c
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

#include <debug.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/mm/mm.h>
#include <nuttx/userspace.h>
#include <arch/board/board.h>
#ifdef CONFIG_MM_KERNEL_HEAP
#include <arch/board/board_memorymap.h>
#endif

#include <arch/esp32s3/memory_layout.h>
#include "xtensa.h"
#include "hardware/esp32s3_rom_layout.h"
#ifdef CONFIG_ESP32S3_SPIRAM
#  include "esp32s3_spiram.h"
#  include "esp32s3_himem.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_MM_KERNEL_HEAP
#  if defined(CONFIG_ESP32S3_SPIRAM)
#    define MM_USER_HEAP_EXTRAM
#  else
#    define MM_USER_HEAP_IRAM
#  endif

#  define MM_ADDREGION kmm_addregion
#else
#  define MM_ADDREGION umm_addregion
#endif

#ifndef ALIGN_DOWN
#  define ALIGN_DOWN(num, align)  ((num) & ~((align) - 1))
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

  /* Align the heap top address to 256 bytes to match the PMS split address
   * requirement.
   */

  utop  = ALIGN_DOWN(ets_rom_layout_p->dram0_rtos_reserved_start, 256);

#  elif defined(CONFIG_BUILD_FLAT)
#    ifdef MM_USER_HEAP_EXTRAM
  ubase = (uintptr_t)esp_spiram_allocable_vaddr_start();
  utop  = (uintptr_t)(esp_spiram_allocable_vaddr_end() -
                      esp_himem_reserved_area_size());
#    elif defined(MM_USER_HEAP_IRAM)
  ubase = (uintptr_t)_sheap + XTENSA_IMEM_REGION_SIZE;
  utop  = (uintptr_t)HEAP_REGION1_END;
#    endif /* MM_USER_HEAP_EXTRAM */

#  endif /* CONFIG_BUILD_PROTECTED */

#else /* !CONFIG_MM_KERNEL_HEAP */

  /* Skip internal heap region if CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP is
   * enabled.
   */

  ubase = (uintptr_t)(_sheap) + XTENSA_IMEM_REGION_SIZE;
  utop  = (uintptr_t)ets_rom_layout_p->dram0_rtos_reserved_start;
#endif /* CONFIG_MM_KERNEL_HEAP */

  usize = utop - ubase;

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
   * Check boards/xtensa/esp32s3.
   */

  kbase = (uintptr_t)_sheap;
  ktop  = KDRAM_END;
#elif defined(CONFIG_BUILD_FLAT)
#  ifdef MM_USER_HEAP_IRAM
  /* Skip internal heap region if CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP is
   * enabled.
   */

  kbase = (uintptr_t)HEAP_REGION2_START;
  ktop  = (uintptr_t)ets_rom_layout_p->dram0_rtos_reserved_start;
#  else
  kbase = (uintptr_t)_sheap + XTENSA_IMEM_REGION_SIZE;
  ktop  = (uintptr_t)ets_rom_layout_p->dram0_rtos_reserved_start;
#  endif /* MM_USER_HEAP_IRAM */

#endif

  ksize = ktop - kbase;

  minfo("Heap: start=%" PRIxPTR " end=%" PRIxPTR " size=%zu\n",
        kbase, ktop, ksize);

  DEBUGASSERT(ktop > kbase);

  board_autoled_on(LED_HEAPALLOCATE);

  *heap_start = (void *)kbase;
  *heap_size  = ksize;
}
#endif /* CONFIG_BUILD_PROTECTED && CONFIG_MM_KERNEL_HEAP */

/****************************************************************************
 * Name: xtensa_add_region
 *
 * Description:
 *   RAM may be added in non-contiguous chunks. This routine adds all chunks
 *   that may be used for heap.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void xtensa_add_region(void)
{
  void  *start;
  void  *end;
  size_t size = 0;
  int availregions = 1;
  int nregions = CONFIG_MM_REGIONS - 1;

#if defined(CONFIG_ESP32S3_SPIRAM_COMMON_HEAP) && !defined(MM_USER_HEAP_EXTRAM)
  availregions++;
#endif

  if (nregions < availregions)
    {
      mwarn("Some memory regions are left unused!\n");
      mwarn("Increase CONFIG_MM_REGIONS to add them to the heap\n");
    }

#if defined(CONFIG_ESP32S3_SPIRAM_COMMON_HEAP) && !defined(MM_USER_HEAP_EXTRAM)
  start = (void *)esp_spiram_allocable_vaddr_start();
  end = (void *)(esp_spiram_allocable_vaddr_end() -
                 esp_himem_reserved_area_size());
  size  = (size_t)(end - start);
#endif

  if (size)
    {
      DEBUGASSERT(end > start);
      MM_ADDREGION(start, size);
      minfo("Heap: start=%" PRIxPTR " end=%" PRIxPTR " size=%zu\n",
            (uintptr_t)start, (uintptr_t)end, size);
    }
}
#endif
