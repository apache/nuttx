/****************************************************************************
 * arch/arm/src/common/arm_allocateheap.c
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

#include <sys/types.h>
#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/userspace.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Terminology.  In the flat build (CONFIG_BUILD_FLAT=y), there is only a
 * single heap access with the standard allocations (malloc/free).  This
 * heap is referred to as the user heap.  In the protected build
 * (CONFIG_BUILD_PROTECTED=y) where an MPU is used to protect a region of
 * otherwise flat memory, there will be two allocators:  One that allocates
 * protected (kernel) memory and one that allocates unprotected (user)
 * memory.  These are referred to as the kernel and user heaps,
 * respectively.
 *
 * The ARMv7 has no MPU but does have an MMU.  With this MMU, it can support
 * the kernel build (CONFIG_BUILD_KERNEL=y).  In this configuration, there
 * is one kernel heap but multiple user heaps:  One per task group.  However,
 * in this case, we need only be concerned about initializing the single
 * kernel heap here.
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap/up_allocate_kheap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   - For the normal "flat" build, this function returns the size of the
 *     single heap.
 *   - For the protected build (CONFIG_BUILD_PROTECTED=y) with both kernel-
 *     and user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function
 *     provides the size of the unprotected, user-space heap.
 *   - For the kernel build (CONFIG_BUILD_KERNEL=y), this function provides
 *     the size of the protected, kernel-space heap.
 *
 *   If a protected kernel-space heap is provided, the kernel heap must be
 *   allocated by an analogous up_allocate_kheap(). A custom version of this
 *   file is needed if memory protection of the kernel heap is required.
 *
 *   The following memory map is assumed for the flat build:
 *
 *     .data region.  Size determined at link time.
 *     .bss  region  Size determined at link time.
 *     IDLE thread stack.  Size determined by CONFIG_IDLETHREAD_STACKSIZE.
 *     Heap.  Extends to the end of SRAM.
 *
 *   The following memory map is assumed for the kernel build:
 *
 *     Kernel .data region.  Size determined at link time.
 *     Kernel .bss  region  Size determined at link time.
 *     Kernel IDLE thread stack.  Size determined by
 *       CONFIG_IDLETHREAD_STACKSIZE.
 *     Padding for alignment
 *     User .data region.  Size determined at link time.
 *     User .bss region  Size determined at link time.
 *     Kernel heap.  Size determined by CONFIG_MM_KERNEL_HEAPSIZE.
 *     User heap.  Extends to the end of SRAM.
 *
 ****************************************************************************/

#ifndef CONFIG_BUILD_KERNEL
void weak_function up_allocate_heap(void **heap_start, size_t *heap_size)
{
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

#ifdef CONFIG_BUILD_PROTECTED
  uintptr_t base = (uintptr_t)USERSPACE->us_bssend;
#else
  uintptr_t base = g_idle_topstack;
#endif

#ifdef CONFIG_ARCH_PGPOOL_PBASE
  size_t end = CONFIG_ARCH_PGPOOL_PBASE;
#else
  size_t end = CONFIG_RAM_END;
#endif

#ifdef CONFIG_MM_KERNEL_HEAP
  base += CONFIG_MM_KERNEL_HEAPSIZE;
#endif

  /* Return the user-space heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (void *)base;
  *heap_size  = end - base;
}
#endif

/****************************************************************************
 * Name: up_allocate_kheap
 *
 * Description:
 *   For the kernel build (CONFIG_BUILD_PROTECTED/KERNEL=y) with both kernel-
 *   and user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function allocates
 *   the kernel-space heap.  A custom version of this function is needed if
 *   memory protection of the kernel heap is required.
 *
 ****************************************************************************/

#ifdef CONFIG_MM_KERNEL_HEAP
void weak_function up_allocate_kheap(void **heap_start, size_t *heap_size)
{
  /* Get the unaligned size and position of the kernel-space heap.
   * This heap begins after .bss section at an offset.
   * If not kernel build, have to work with CONFIG_MM_KERNEL_HEAPSIZE.
   * And have to subject alignment.
   */

#ifdef CONFIG_BUILD_FLAT
  uintptr_t base = g_idle_topstack;
  uintptr_t size = CONFIG_MM_KERNEL_HEAPSIZE;

#elif defined(CONFIG_BUILD_PROTECTED)
  uintptr_t base = (uintptr_t)USERSPACE->us_bssend;
  uintptr_t size = CONFIG_MM_KERNEL_HEAPSIZE;
  DEBUGASSERT(base < (uintptr_t)CONFIG_RAM_END);

#elif defined(CONFIG_ARCH_PGPOOL_PBASE)
  /* CONFIG_BUILD_KERNEL && CONFIG_ARCH_PGPOOL_PBASE */

  uintptr_t base = g_idle_topstack;
  uintptr_t size = CONFIG_ARCH_PGPOOL_PBASE - g_idle_topstack;
# else
  /* CONFIG_BUILD_KERNEL && !CONFIG_ARCH_PGPOOL_PBASE */

  uintptr_t base = g_idle_topstack;
  uintptr_t size = CONFIG_RAM_END - ubase;
#endif

  /* Return the kernel-space heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (void *)base;
  *heap_size  = size;
}
#endif
