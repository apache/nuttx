/****************************************************************************
 * arch/tricore/src/common/tricore_allocateheap.c
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
#include <nuttx/userspace.h>

#include <sys/types.h>
#include <nuttx/debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "tricore_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
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
 *   - For the normal "flat" build, this function returns the size of the
 *     single heap.
 *   - For the protected build (CONFIG_BUILD_PROTECTED=y) with both kernel-
 *     and user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function
 *     provides the size of the user-space heap.
 *
 ****************************************************************************/

void weak_function up_allocate_heap(void **heap_start, size_t *heap_size)
{
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section.
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend;
  uintptr_t utop  = (uintptr_t)USERSPACE->us_heapend;

  DEBUGASSERT(ubase <= utop);

  /* Return the user-space heap settings */

  *heap_start = (void *)ubase;
  *heap_size  = utop - ubase;
#else

  /* Return the heap settings */

  *heap_start = _sheap;
  *heap_size = (uintptr_t)_eheap - (uintptr_t)_sheap;

#if defined(CONFIG_MM_KERNEL_HEAP)
  *heap_start += CONFIG_MM_KERNEL_HEAPSIZE;
  *heap_size -= CONFIG_MM_KERNEL_HEAPSIZE;
#endif

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
void weak_function up_allocate_kheap(void **heap_start, size_t *heap_size)
{
  /* Get the unaligned size and position of the kernel-space heap.
   * This heap begins after the kernel-space idle stack.
   */

#if defined(CONFIG_BUILD_PROTECTED)
  DEBUGASSERT((uintptr_t)_sheap + CONFIG_MM_KERNEL_HEAPSIZE
              <= (uintptr_t)_eheap);

  /* Return the kernel heap settings */

  *heap_start = _sheap;
  *heap_size = (uintptr_t)_eheap - (uintptr_t)_sheap;
#else
  *heap_start = _sheap;
  *heap_size = CONFIG_MM_KERNEL_HEAPSIZE;
#endif
}
#endif
