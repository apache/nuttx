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

#include "xtensa.h"
#include "hardware/esp32s3_rom_layout.h"
#ifdef CONFIG_ESP32S3_SPIRAM
#  include "esp32s3_spiram.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  uintptr_t ubase = USERSPACE->us_dataend;

  /* Align the heap top address to 256 bytes to match the PMS split address
   * requirement.
   */

  uintptr_t utop  = ALIGN_DOWN(ets_rom_layout_p->dram0_rtos_reserved_start,
                               256);
  size_t    usize = utop - ubase;

  minfo("Heap: start=%" PRIxPTR " end=%" PRIxPTR " size=%zu\n",
        ubase, utop, usize);

  board_autoled_on(LED_HEAPALLOCATE);

  /* Return the userspace heap settings */

  *heap_start = (void *)ubase;
  *heap_size  = usize;

  /* Allow user-mode access to the user heap memory in PMP
   * is already done in esp32s3_userspace().
   */

#else
  /* These values come from the linker scripts (esp32s3_sections.ld and
   * flat_memory.ld).
   * Check boards/xtensa/esp32s3.
   */

  board_autoled_on(LED_HEAPALLOCATE);

  *heap_start = _sheap;
  *heap_size  = ets_rom_layout_p->dram0_rtos_reserved_start -
                (uintptr_t)_sheap;
#endif /* CONFIG_BUILD_PROTECTED && CONFIG_MM_KERNEL_HEAP */
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

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP) && \
    defined(__KERNEL__)
void up_allocate_kheap(void **heap_start, size_t *heap_size)
{
  /* These values come from the linker scripts (kernel-space.ld and
   * protected_memory.ld).
   * Check boards/xtensa/esp32s3.
   */

  uintptr_t kbase = (uintptr_t)_sheap;
  uintptr_t ktop  = KDRAM_END;
  size_t    ksize = ktop - kbase;

  minfo("Heap: start=%" PRIxPTR " end=%" PRIxPTR " size=%zu\n",
        kbase, ktop, ksize);

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
#ifdef CONFIG_ESP32S3_SPIRAM
  void  *start;
  size_t size;

  start = (void *)esp_spiram_allocable_vaddr_start();
  size  = (size_t)(esp_spiram_allocable_vaddr_end() -
                   esp_spiram_allocable_vaddr_start());
  umm_addregion(start, size);
#endif
}
#endif
