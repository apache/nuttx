/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_allocateheap.c
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
#include <nuttx/kmalloc.h>
#include <nuttx/userspace.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "mpu.h"
#include "chip.h"
#include "stm32wb_mpuinit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Internal SRAM is available in all members of the STM32WB family. The
 * following definitions must be provided to specify the size and
 * location of internal (system) SRAM1, SRAM2a and SRAM2b:
 *
 * SRAM1_START   0x20000000
 * SRAM1_END
 * SRAM2A_START  0x20030000
 * SRAM2A_END
 * SRAM2B_START  0x20038000
 * SRAM2B_END
 */

/* Set the range of system SRAM1 */

#define SRAM1_START     STM32WB_SRAM1_BASE
#define SRAM1_END       (SRAM1_START + STM32WB_SRAM1_SIZE)

/* Set the range of SRAM2a as well, requires a second memory region */

#ifdef CONFIG_STM32WB_SRAM2A_HEAP
#  define SRAM2A_START  (STM32WB_SRAM2A_BASE + CONFIG_STM32WB_SRAM2A_USER_BASE_OFFSET)
#  define SRAM2A_END    (SRAM2A_START + CONFIG_STM32WB_SRAM2A_USER_SIZE)
#endif

/* Set the range of SRAM2b as well, requires a third memory region */

#ifdef CONFIG_STM32WB_SRAM2B_HEAP
#  define SRAM2B_START  STM32WB_SRAM2B_BASE
#  define SRAM2B_END    (SRAM2B_START + CONFIG_STM32WB_SRAM2B_USER_SIZE)
#endif

/* Some sanity checking.  If multiple memory regions are defined, verify
 * that CONFIG_MM_REGIONS is set to match the number of memory regions
 * that we have been asked to add to the heap.
 */

#ifdef CONFIG_STM32WB_SRAM2A_HEAP
#  if SRAM2A_END > STM32WB_SRAM2A_BASE + STM32WB_SRAM2A_SIZE
#    error "SRAM2a heap memory region is out of it's physical address space"
#  endif
#endif

#ifdef CONFIG_STM32WB_SRAM2B_HEAP
#  if SRAM2B_END > STM32WB_SRAM2B_BASE + STM32WB_SRAM2B_SIZE
#    error "SRAM2b heap memory region is out of it's physical address space"
#  endif
#endif

#if CONFIG_MM_REGIONS < defined(CONFIG_STM32WB_SRAM2A_HEAP) + \
                        defined(CONFIG_STM32WB_SRAM2B_HEAP) + 1
#  error "You need more memory manager regions to support selected heap components"
#endif

#if CONFIG_MM_REGIONS > defined(CONFIG_STM32WB_SRAM2A_HEAP) + \
                        defined(CONFIG_STM32WB_SRAM2B_HEAP) + 1
#  warning "CONFIG_MM_REGIONS large enough but I do not know what some of the region(s) are"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_heap_color
 *
 * Description:
 *   Set heap memory to a known, non-zero state to checking heap usage.
 *
 ****************************************************************************/

#ifdef CONFIG_HEAP_COLORATION
static inline void up_heap_color(void *start, size_t size)
{
  memset(start, HEAP_COLOR, size);
}
#else
#  define up_heap_color(start,size)
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
 *     Kernel IDLE thread stack.  Size determined by
 *                                    CONFIG_IDLETHREAD_STACKSIZE.
 *     Padding for alignment
 *     User .data region.  Size determined at link time.
 *     User .bss region  Size determined at link time.
 *     Kernel heap.  Size determined by CONFIG_MM_KERNEL_HEAPSIZE.
 *     User heap.  Extends to the end of SRAM.
 *
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend +
                    CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = SRAM1_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)SRAM1_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the SRAM1_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(usize);
  DEBUGASSERT((SRAM1_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = SRAM1_END - usize;

  /* Return the user-space heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (void *)ubase;
  *heap_size  = usize;

  /* Colorize the heap for debug */

  up_heap_color((void *)ubase, usize);

  /* Allow user-mode access to the user heap memory */

  stm32wb_mpu_uheap((uintptr_t)ubase, usize);
#else

  /* Return the heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (void *)g_idle_topstack;
  *heap_size  = SRAM1_END - g_idle_topstack;

  /* Colorize the heap for debug */

  up_heap_color(*heap_start, *heap_size);
#endif
}

/****************************************************************************
 * Name: up_allocate_kheap
 *
 * Description:
 *   For the kernel build (CONFIG_BUILD_PROTECTED=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function allocates
 *   (and protects) the kernel-space heap.
 *
 ****************************************************************************/

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
void up_allocate_kheap(void **heap_start, size_t *heap_size)
{
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend +
                    CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = SRAM1_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)SRAM1_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the SRAM1_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(usize);
  DEBUGASSERT((SRAM1_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = SRAM1_END - usize;

  /* Return the kernel heap settings (i.e., the part of the heap region
   * that was not dedicated to the user heap).
   */

  *heap_start = (void *)USERSPACE->us_bssend;
  *heap_size  = ubase - (uintptr_t)USERSPACE->us_bssend;
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
  /* SRAM2a and SRAM2b memories contain a secure section, which cannot be
   * read nor written by CPU1. The secure start address for each memory can
   * be read from the option bytes SBRSA (SRAM2a) and SNBRSA (SRAM2b).
   * The beginning of the SRAM2a memory is used for communication in between
   * the 2 cores. SRAM2b is all secure for any CPU2 firmware supporting the
   * Thread protocol. So the available memory for the user application may
   * vary between different versions of the RF stack and can be obtained
   * from the release notes for STM32WB coprocessor wireless binaries.
   */

#ifdef CONFIG_STM32WB_SRAM2A_HEAP

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Allow user-mode access to the SRAM2a heap */

  stm32wb_mpu_uheap((uintptr_t)SRAM2A_START, SRAM2A_END - SRAM2A_START);

#endif

  /* Colorize the heap for debug */

  up_heap_color((void *)SRAM2A_START, SRAM2A_END - SRAM2A_START);

  /* Add the SRAM2a user heap region. */

  kumm_addregion((void *)SRAM2A_START, SRAM2A_END - SRAM2A_START);

#endif /* CONFIG_STM32WB_SRAM2A_HEAP */

#ifdef CONFIG_STM32WB_SRAM2B_HEAP

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Allow user-mode access to the SRAM2b heap */

  stm32wb_mpu_uheap((uintptr_t)SRAM2B_START, SRAM2B_END - SRAM2B_START);

#endif

  /* Colorize the heap for debug */

  up_heap_color((void *)SRAM2B_START, SRAM2B_END - SRAM2B_START);

  /* Add the SRAM2b user heap region. */

  kumm_addregion((void *)SRAM2B_START, SRAM2B_END - SRAM2B_START);

#endif /* CONFIG_STM32WB_SRAM2B_HEAP */
}
#endif
