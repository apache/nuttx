/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_allocateheap.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/debug.h>
#include <sys/types.h>

#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/mm/mm.h>

#include "riscv_internal.h"
#include "rom/rom_layout.h"
#ifdef CONFIG_ESPRESSIF_RETENTION_HEAP
#  include "esp_retentionheap.h"
#endif
#if defined(CONFIG_ESPRESSIF_SPIRAM)
#  include "esp_psram.h"
#  include "esp_private/esp_psram_extram.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

uintptr_t _heap_start;

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
 *   If a protected kernel heap is provided, the kernel heap must be
 *   allocated (and protected) by an analogous up_allocate_kheap().
 *
 * Input Parameters:
 *   None.
 *
 * Output Parameters:
 *   heap_start - Address of the beginning of the (initial) memory region.
 *   heap_size  - The size (in bytes) if the (initial) memory region.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  /* These values come from the linker scripts
   * (<chip>_<legacy/mcuboot>_sections.ld and <chip>_flat_memory.ld).
   * Check boards/risc-v/espressif.
   */

#ifdef CONFIG_ESPRESSIF_RETENTION_HEAP
  uintptr_t rstart;
  uintptr_t rend;
#endif
#if defined(CONFIG_MM_KERNEL_HEAP) && \
    defined(CONFIG_ESPRESSIF_SPIRAM) && \
    defined(CONFIG_ESPRESSIF_SPIRAM_USER_HEAP)
  uintptr_t ubase;
  uintptr_t utop;
#endif

  board_autoled_on(LED_HEAPALLOCATE);

#if defined(CONFIG_MM_KERNEL_HEAP) && \
    defined(CONFIG_ESPRESSIF_SPIRAM) && \
    defined(CONFIG_ESPRESSIF_SPIRAM_USER_HEAP)
  DEBUGASSERT(esp_psram_is_initialized());
  ubase = esp_psram_extram_vaddr_start();
  utop  = esp_psram_extram_vaddr_end();

  *heap_start = (void *)ubase;
  *heap_size  = utop - ubase;
#else
  *heap_start = (void *)g_idle_topstack;
#ifdef CONFIG_ESPRESSIF_RETENTION_HEAP
  esp_retentionheap_find_region(&rstart, &rend);
  *heap_size  = (uintptr_t) rstart - g_idle_topstack;
#else
  *heap_size  = (uintptr_t)ets_rom_layout_p->dram0_rtos_reserved_start -
                           g_idle_topstack;
#endif
#endif
  _heap_start = (uintptr_t)*heap_start;
}

/****************************************************************************
 * Name: up_allocate_kheap
 *
 * Description:
 *   For the kernel builds (CONFIG_BUILD_PROTECTED=y or
 *   CONFIG_BUILD_KERNEL=y) there may be both kernel- and user-space heaps
 *   as determined by CONFIG_MM_KERNEL_HEAP=y.  This function allocates (and
 *   protects) the kernel-space heap.
 *
 *   For Flat build (CONFIG_BUILD_FLAT=y), this function enables a separate
 *   (although unprotected) heap for the kernel.
 *
 ****************************************************************************/

#ifdef CONFIG_MM_KERNEL_HEAP
void up_allocate_kheap(void **heap_start, size_t *heap_size)
{
  uintptr_t kbase = g_idle_topstack;
  uintptr_t ktop;

#ifdef CONFIG_ESPRESSIF_RETENTION_HEAP
  uintptr_t rstart;
  uintptr_t rend;

  esp_retentionheap_find_region(&rstart, &rend);
  ktop = rstart;
#else
  ktop = (uintptr_t)ets_rom_layout_p->dram0_rtos_reserved_start;
#endif

  DEBUGASSERT(ktop > kbase);

  board_autoled_on(LED_HEAPALLOCATE);

  *heap_start = (void *)kbase;
  *heap_size  = ktop - kbase;
}
#endif

/****************************************************************************
 * Name: riscv_addregion
 *
 * Description:
 *   RAM may be added in non-contiguous chunks. This routine adds all chunks
 *   that may be used for heap.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void riscv_addregion(void)
{
#if defined(CONFIG_ESP32P4_SELECTS_REV_LESS_V3)
  /* ESP32-P4 rev < v3 has non-contiguous SRAM: sram_low + sram_high.
   * The primary heap is in sram_low. Add sram_high as a second region.
   */

  extern uint8_t _sram_high_heap_start[];
  extern uint8_t _sram_high_heap_end[];

  size_t region_size = _sram_high_heap_end - _sram_high_heap_start;

  if (region_size > 0)
    {
#ifdef CONFIG_MM_KERNEL_HEAP
      kmm_addregion(_sram_high_heap_start, region_size);
#else
      kumm_addregion(_sram_high_heap_start, region_size);
#endif
    }
#endif

#if !defined(CONFIG_MM_KERNEL_HEAP)
#  if defined(CONFIG_ESPRESSIF_SPIRAM_USER_HEAP)
  if (esp_psram_is_initialized())
    {
      uintptr_t start = esp_psram_extram_vaddr_start();
      uintptr_t end = esp_psram_extram_vaddr_end();

      if (end > start)
        {
          kumm_addregion((void *)start, end - start);
        }
    }
#  endif
#endif
}
#endif
