/****************************************************************************
 * arch/arm/src/rtl8721dx/ameba_allocateheap.c
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
#include <nuttx/nuttx.h>

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/userspace.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/ameba_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Terminology.
 * In the flat build (CONFIG_BUILD_FLAT=y), there is only a single heap
 * accessed with the standard allocations (malloc/free).  This heap is
 * referred to as the user heap.  Only the flat build with a single SRAM
 * region is supported on the RTL8721Dx.
 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* NuttX heap = all remaining KM4 on-chip SRAM.
 *
 * The KM4 image2 linker script (ameba_img2_all.ld) lays out this region for
 * us: __bdram_heap_buffer_start__ is the first byte after NuttX's .bss, and
 * __bdram_heap_buffer_size__ is an ABSOLUTE symbol whose VALUE is the number
 * of bytes from there to the end of the KM4 SRAM region
 * (__km4_bd_ram_end__).  Using these gives NuttX every SRAM byte the SDK did
 * not reserve, instead of a fixed compile-time buffer, and tracks the
 * chip/board RAM automatically.
 *
 * Boards with PSRAM can add it as a second region via arm_addregion() using
 * __psram_heap_buffer_start__/__psram_heap_buffer_size__ (zero on
 * PKE8721DAF, which has no PSRAM).
 */

extern uint8_t __bdram_heap_buffer_start__[];
extern uint8_t __bdram_heap_buffer_size__[];   /* ABS symbol: value == bytes */

#define NUTTX_HEAP_BASE ((void *)__bdram_heap_buffer_start__)
#define NUTTX_HEAP_SIZE ((size_t)((uintptr_t)__bdram_heap_buffer_size__))

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   For the flat build, this returns the location and size of the single
 *   heap: all KM4 SRAM remaining after NuttX's .data/.bss, as computed by
 *   the SDK linker script.
 *
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  *heap_start = NUTTX_HEAP_BASE;
  *heap_size  = NUTTX_HEAP_SIZE;

#if defined(CONFIG_MM_KERNEL_HEAP)
  *heap_size -= CONFIG_MM_KERNEL_HEAPSIZE;
#endif
}

/****************************************************************************
 * Name: up_allocate_kheap
 *
 * Description:
 *   For the kernel build (CONFIG_BUILD_PROTECTED/KERNEL=y) with both kernel-
 *   and user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function allocates
 *   the kernel-space heap.
 *
 ****************************************************************************/

#if defined(CONFIG_MM_KERNEL_HEAP)
void up_allocate_kheap(void **heap_start, size_t *heap_size)
{
  /* Carve the kernel heap off the top of the SRAM heap region. */

  *heap_start = (void *)((uintptr_t)NUTTX_HEAP_BASE +
                         (NUTTX_HEAP_SIZE - CONFIG_MM_KERNEL_HEAPSIZE));
  *heap_size  = CONFIG_MM_KERNEL_HEAPSIZE;
}
#endif

/****************************************************************************
 * Name: arm_addregion
 *
 * Description:
 *   Memory may be added in non-contiguous chunks.  Additional chunks are
 *   added by calling this function.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void arm_addregion(void)
{
}
#endif
