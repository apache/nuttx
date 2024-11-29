/****************************************************************************
 * arch/risc-v/src/bl808/bl808_allocateheap.c
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
#include <nuttx/kmalloc.h>
#include <nuttx/userspace.h>
#include <nuttx/arch.h>
#include <arch/board/board_memorymap.h>
#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define KRAM_END    KSRAM_END

/****************************************************************************
 * Public Data
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
 *   For the kernel build (CONFIG_BUILD_PROTECTED=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function provides the
 *   size of the unprotected, user-space heap.
 *
 *   If a protected kernel-space heap is provided, the kernel heap must be
 *   allocated (and protected) by an analogous up_allocate_kheap().
 *
 *   The following memory map is assumed for the flat build:
 *
 *   .data region.  Size determined at link time.
 *   .bss  region  Size determined at link time.
 *   IDLE thread stack.  Size determined by CONFIG_IDLETHREAD_STACKSIZE.
 *   Heap.  Extends to the end of User SRAM.
 *
 *   The following memory map is assumed for the protect build.
 *   The kernel and user space have it's own dedicated heap space.
 *
 *   User .data region         Size determined at link time
 *   User .bss region          Size determined at link time
 *   User heap                 Extends to the end of User SRAM
 *   Kernel .data region       Size determined at link time
 *   Kernel .bss  region       Size determined at link time
 *   Kernel IDLE thread stack  Size determined by CONFIG_IDLETHREAD_STACKSIZE
 *   Kernel heap               Size determined by CONFIG_MM_KERNEL_HEAPSIZE
 *
 ****************************************************************************/

void up_allocate_kheap(void **heap_start, size_t *heap_size)
{
  /* Return the heap settings */

  *heap_start = (void *)g_idle_topstack;
  *heap_size = KRAM_END - g_idle_topstack;
}
