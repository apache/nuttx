/****************************************************************************
 * arch/risc-v/src/esp32c3/esp32c3_allocateheap.c
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

#include "esp32c3.h"
#include "hardware/esp32c3_rom_layout.h"

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
  extern const struct esp32c3_rom_layout_s *ets_rom_layout_p;

  uintptr_t ubase = USERSPACE->us_dataend;
  uintptr_t utop  = ets_rom_layout_p->dram0_rtos_reserved_start;
  size_t    usize = utop - ubase;

  minfo("Heap: start=%" PRIxPTR " end=%" PRIxPTR " size=%zu\n",
        ubase, utop, usize);

  board_autoled_on(LED_HEAPALLOCATE);

  /* Return the userspace heap settings */

  *heap_start = (void *)ubase;
  *heap_size  = usize;

  /* Allow user-mode access to the user heap memory in PMP
   * is already done in esp32c3_userspace().
   */

#else
  /* These values come from the linker scripts (esp32c3.ld and
   * flat.template.ld).
   * Check boards/risc-v/esp32c3.
   */

  extern uint8_t _sheap[];
  extern const struct esp32c3_rom_layout_s *ets_rom_layout_p;

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
   * protected.template.ld).
   * Check boards/risc-v/esp32c3.
   */

  extern uint8_t _sheap[];

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
 * Name: riscv_addregion
 *
 * Description:
 *   RAM may be added in non-contiguous chunks.  This routine adds all chunks
 *   that may be used for heap.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void riscv_addregion(void)
{
}
#endif

