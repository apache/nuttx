/****************************************************************************
 * arch/x86_64/src/common/x86_64_allocateheap.c
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
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "x86_64_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IDLE_STACK_SIZE CONFIG_IDLETHREAD_STACKSIZE

#if CONFIG_IDLETHREAD_STACKSIZE % 16 != 0
#  error CONFIG_IDLETHREAD_STACKSIZE must be aligned to 16
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static const uintptr_t g_idle_stackalloc = (uintptr_t)_ebss +
  CONFIG_IDLETHREAD_STACKSIZE * CONFIG_SMP_NCPUS;

const uintptr_t g_idle_topstack[CONFIG_SMP_NCPUS] =
{
  (uintptr_t)g_idle_stackalloc + (1 * IDLE_STACK_SIZE) - 16,
#if CONFIG_SMP_NCPUS > 1
  (uintptr_t)g_idle_stackalloc + (2 * IDLE_STACK_SIZE) - 16,
#endif
#if CONFIG_SMP_NCPUS > 2
  (uintptr_t)g_idle_stackalloc + (3 * IDLE_STACK_SIZE) - 16,
#endif
#if CONFIG_SMP_NCPUS > 3
  (uintptr_t)g_idle_stackalloc + (4 * IDLE_STACK_SIZE) - 16,
#endif
#if CONFIG_SMP_NCPUS > 4
  (uintptr_t)g_idle_stackalloc + (5 * IDLE_STACK_SIZE) - 16,
#endif
#if CONFIG_SMP_NCPUS > 5
#  error missing logic
#endif
};

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

#ifdef CONFIG_BUILD_KERNEL
void up_allocate_kheap(void **heap_start, size_t *heap_size)
#else
void up_allocate_heap(void **heap_start, size_t *heap_size)
#endif /* CONFIG_BUILD_KERNEL */
{
  uintptr_t hstart;
  uintptr_t topstack;

  board_autoled_on(LED_HEAPALLOCATE);

  topstack = g_idle_topstack[CONFIG_SMP_NCPUS - 1] + 8;

  /* Calculate the end of .bss section */

  hstart = (topstack + PAGE_SIZE - 1) & PAGE_MASK;
  *heap_start = (void *)hstart;

  /* The size is the rest of the RAM minus page pool */

#ifdef CONFIG_ARCH_PGPOOL_PBASE
  *heap_size = (size_t)(CONFIG_ARCH_PGPOOL_PBASE -
                        (hstart - X86_64_LOAD_OFFSET - 1));
#else
  *heap_size = (size_t)(X86_64_PGPOOL_BASE -
                        (hstart - X86_64_LOAD_OFFSET - 1));
#endif
}
