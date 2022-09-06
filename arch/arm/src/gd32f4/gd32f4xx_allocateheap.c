/****************************************************************************
 * arch/arm/src/gd32f4/gd32f4xx_allocateheap.c
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
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/userspace.h>

#include <arch/board/board.h>

#include "chip.h"

#include "mpu.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The GD32F4xx all contains System SRAM and tightly-coupled memory
 * SRAM (TCMSRAM) on the chip.
 * The following definitions must be provided to specify the size and
 * location of System SRAM:
 *
 * CONFIG_RAM_END                  :  SRAM end address
 *
 * The TCMSRAM is different to SRAM, and it be accessed only by the data
 * bus of the Cortex®-M4 core, and can not be used for DMA. When used DMA,
 * then the following should be defined to exclude TCMSRAM from the heap:
 *
 * CONFIG_GD32F4_TCMEXCLUDE        : Exclude TCMSRAM from the HEAP
 *
 * In addition to internal SRAM, external RAM may also be available through
 * the EXMC. When external RAM is want to be used, then the following
 * definitions should to be provided:
 *
 * CONFIG_GD32F4_EXMC=y            : Enable the EXMC
 * CONFIG_GD32F4_EXTERNAL_RAM=y    : Indicates that via the EXMC, external
 *                                   RAM can be used.
 * CONFIG_HEAP2_BASE               : External RAM base address
 * CONFIG_HEAP2_SIZE               : External RAM size
 * CONFIG_MM_REGIONS               : Must be set to the value match to how
 *                                   many RAMs you have used.
 */

#if !defined(CONFIG_GD32F4_EXMC)
#  undef CONFIG_GD32F4_EXTERNAL_RAM
#endif

/* The heap is in one contiguous block starting at g_idle_topstack and
 * extending through CONFIG_RAM_END.
 */

#if defined(CONFIG_GD32F4_GD32F4XX)

/* Set the end of system SRAM */

#  if defined(CONFIG_GD32F4_GD32F450)
#    if defined(CONFIG_GD32F4_GD32F450XI)
#      define SRAM_END 0x20070000
#    else
#      define SRAM_END 0x20030000
#    endif
#  else
#    define SRAM_END 0x20020000
#  endif

/* Set the range of TCMSRAM */

#  define TCMSRAM_START 0x10000000
#  define TCMSRAM_END   0x10010000

/* There are 4 possible SRAM configuration case:
 *
 * Case 0.     System SRAM
 *             CONFIG_MM_REGIONS define as 1
 *             CONFIG_GD32F4_TCMEXCLUDE defined, not use TCMSRAM
 *             CONFIG_GD32F4_EXTERNAL_RAM NOT defined
 *
 * Case 1.     System SRAM and TCM SRAM
 *             CONFIG_MM_REGIONS define as 2
 *             CONFIG_GD32F4_TCMEXCLUDE NOT defined, use TCMSRAM
 *             CONFIG_GD32F4_EXTERNAL_RAM NOT defined
 *
 * Case 2.     System SRAM and EXMC SRAM
 *             CONFIG_MM_REGIONS define as 2
 *             CONFIG_GD32F4_TCMEXCLUDE defined, not use TCMSRAM
 *             CONFIG_GD32F4_EXTERNAL_RAM defined
 *
 * Case 3.     System SRAM, CCM SRAM, and EXMC SRAM
 *             CONFIG_MM_REGIONS define as 3
 *             CONFIG_GD32F4_TCMEXCLUDE NOT defined, use TCMSRAM
 *             CONFIG_GD32F4_EXTERNAL_RAM defined
 *
 * Make sure that all definitions are consistent before doing anything else
 */

#  if CONFIG_MM_REGIONS < 1

#    error "There is at least one memory region"
#    define CONFIG_MM_REGIONS 1
#    undef CONFIG_GD32F4_EXTERNAL_RAM
#    undef CONFIG_GD32F4_TCMEXCLUDE
#    define CONFIG_GD32F4_TCMEXCLUDE 1

#  elif CONFIG_MM_REGIONS < 2
/* Only one memory region.  Force Case 0 */

#    warning "EXMC SRAM (and TCMSRAM) excluded from the heap"
#    undef CONFIG_GD32F4_EXTERNAL_RAM
#    undef CONFIG_GD32F4_TCMEXCLUDE
#    define CONFIG_GD32F4_TCMEXCLUDE 1

#  elif CONFIG_MM_REGIONS < 3
/* Two memory regions.  Case 1 or 2 */

#    if !defined(CONFIG_GD32F4_TCMEXCLUDE) && defined(CONFIG_GD32F4_EXTERNAL_RAM)
#      error "Can not support both TCM SRAM and EXMC SRAM, when CONFIG_MM_REGIONS is 2 " 
#      undef CONFIG_GD32F4_TCMEXCLUDE
#      define CONFIG_GD32F4_TCMEXCLUDE 1  
#    endif

/* Case 1, TCMSRAM is used. In this case, DMA should not be used  */

#    if !defined(CONFIG_GD32F4_TCMEXCLUDE)

#      ifdef CONFIG_ARCH_DMA
#        error "TCMSRAM is included in the heap AND DMA is enabled"
#      endif
#    endif

#  elif CONFIG_MM_REGIONS == 3
/* Three memory regions.  Case 3 */

#    ifdef CONFIG_ARCH_DMA
#      error "TCM SRAM is included in the heap AND DMA is enabled"
#    endif

#  else
#    error "CONFIG_MM_REGIONS > 3 but there no more region(s) are other than SRAM, TCMSRAM and EXMC RAM"
#    undef CONFIG_MM_REGIONS
#    define CONFIG_MM_REGIONS 3

#  endif /* CONFIG_MM_REGIONS */

#else
#  error "Unsupported GD32 chip"
#endif

/* If EXMC SRAM is going to be used as heap, then verify that the starting
 * address and size of the external SRAM region has been provided in the
 * configuration (as CONFIG_HEAP2_BASE and CONFIG_HEAP2_SIZE).
 */

#ifdef CONFIG_GD32F4_EXTERNAL_RAM
#  if !defined(CONFIG_HEAP2_BASE) || !defined(CONFIG_HEAP2_SIZE)
#    error "When use EXMC RAM CONFIG_HEAP2_BASE and CONFIG_HEAP2_SIZE must be provided"
#    undef CONFIG_GD32F4_EXTERNAL_RAM
#  endif
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
 *     Kernel .data region       Size determined at link time
 *     Kernel .bss  region       Size determined at link time
 *     Kernel IDLE thread stack  Size determined by
 *                                CONFIG_IDLETHREAD_STACKSIZE
 *     Padding for alignment
 *     User .data region         Size determined at link time
 *     User .bss region          Size determined at link time
 *     Kernel heap               Size determined by
 *                                CONFIG_MM_KERNEL_HEAPSIZE
 *     User heap                 Extends to the end of SRAM
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
  size_t    usize = SRAM_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)SRAM_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the SRAM_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(usize);

  usize = (1 << log2);
  ubase = SRAM_END - usize;

  /* Return the user-space heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (void *)ubase;
  *heap_size  = usize;

  /* Colorize the heap for debug */

  up_heap_color((void *)ubase, usize);

  /* Allow user-mode access to the user heap memory */

  gd32_mpu_uheap((uintptr_t)ubase, usize);
#else

  /* Return the heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (void *)g_idle_topstack;
  *heap_size  = SRAM_END - g_idle_topstack;

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
  size_t    usize = SRAM_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)SRAM_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the SRAM_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(usize);

  usize = (1 << log2);
  ubase = SRAM_END - usize;

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
#ifndef CONFIG_GD32F4_TCMEXCLUDE
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Allow user-mode access to the TCMSRAM heap */

  gd32_mpu_uheap((uintptr_t)TCMSRAM_START, TCMSRAM_END - TCMSRAM_START);

#endif

  /* Colorize the heap for debug */

  up_heap_color((void *)TCMSRAM_START, TCMSRAM_END - TCMSRAM_START);

  /* Add the TCMSRAM user heap region. */

  kumm_addregion((void *)TCMSRAM_START, TCMSRAM_END - TCMSRAM_START);
#endif

#ifdef CONFIG_GD32F4_EXTERNAL_RAM
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Allow user-mode access to the EXMC SRAM user heap memory */

  gd32_mpu_uheap((uintptr_t)CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE);

#endif

  /* Colorize the heap for debug */

  up_heap_color((void *)CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE);

  /* Add the external EXMC SRAM user heap region. */

  kumm_addregion((void *)CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE);
#endif
}
#endif
