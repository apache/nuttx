/****************************************************************************
 * arch/arm64/src/fvp-v8r/fvp_v8r_allocateheap.c
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
#include <arch/board/board_memorymap.h>

#include "arm64_internal.h"

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
 * The ARMv8 has no MPU but does have an MMU.  With this MMU, it can support
 * the kernel build (CONFIG_BUILD_KERNEL=y).  In this configuration, there
 * is one kernel heap but multiple user heaps:  One per task group.  However,
 * in this case, we need only be concerned about initializing the single
 * kernel heap here.
 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
void up_allocate_kheap(void **heap_start, size_t *heap_size)
{
  *heap_start = g_idle_topstack;
  *heap_size = KSRAM_END - (size_t)g_idle_topstack;
}
#endif
