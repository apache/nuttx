/****************************************************************************
 * arch/arm/src/rtl8720c/ameba_heap.c
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
#include <assert.h>
#include <string.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define _START_HEAP                     ((uintptr_t)&__bss_end__)
#define _END_HEAP                       ((uintptr_t)&__sram_end__)
#ifdef CONFIG_HEAP_COLORATION
#  define song_heap_color(start, size)  memset(start, HEAP_COLOR, size)
#else
#  define song_heap_color(start, size)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t __stack;
extern uint32_t __bss_end__;
extern uint32_t __sram_end__;
/* g_idle_topstack: _sbss is the start of the BSS region as defined by the
 * linker script. _ebss lies at the end of the BSS region. The idle task
 * stack starts at the end of BSS and is of size CONFIG_IDLETHREAD_STACKSIZE.
 * The IDLE thread is the thread that the system boots on and, eventually,
 * becomes the IDLE, do nothing task that runs only when there is nothing
 * else to run.  The heap continues from there until the end of memory.
 * g_idle_topstack is a read-only variable the provides this computed
 * address.
 */

const uintptr_t weak_data g_idle_topstack =
  ((uintptr_t)&__stack);

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
 *     .data region.  Size determined at link time.
 *     .bss  region  Size determined at link time.
 *     IDLE thread stack.  Size determined by CONFIG_IDLETHREAD_STACKSIZE.
 *     Heap.  Extends to the end of SRAM.
 *
 *   The following memory map is assumed for the kernel build:
 *
 *     Kernel .data region.  Size determined at link time.
 *     Kernel .bss  region  Size determined at link time.
 *     Kernel IDLE thread stack. Size determined by
 *                               CONFIG_IDLETHREAD_STACKSIZE.
 *     Kernel heap.  Size determined by CONFIG_MM_KERNEL_HEAPSIZE.
 *     Padding for alignment
 *     User .data region.  Size determined at link time.
 *     User .bss region  Size determined at link time.
 *     User heap.  Extends to the end of SRAM.
 *
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  /* Return the heap settings */

  *heap_start = (void *)_START_HEAP;
  *heap_size  = _END_HEAP - _START_HEAP;

  /* Colorize the heap for debug */

  song_heap_color(*heap_start, *heap_size);
}

