/****************************************************************************
 * arch/arm/src/stm32l4/up_allocateheap.c
 *
 *   Copyright (C) 2011-2013, 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include "chip.h"
#include "mpu.h"
#include "up_arch.h"
#include "up_internal.h"
#include "stm32l4_mpuinit.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Internal SRAM is available in all members of the STM32L4 family. The
 * following definitions must be provided to specify the size and
 * location of internal (system) SRAM1 and SRAM2:
 *
 * SRAM1_START   0x20000000
 * SRAM1_END
 * SRAM2_START   0x10000000
 * SRAM2_END
 *
 * In addition to internal SRAM, memory may also be available through the FSMC.
 * In order to use FSMC SRAM, the following additional things need to be
 * present in the NuttX configuration file:
 *
 * CONFIG_STM32L4_FSMC=y      : Enables the FSMC
 * CONFIG_STM32L4_FSMC_SRAM=y : Indicates that SRAM is available via the
 *                              FSMC (as opposed to an LCD or FLASH).
 * CONFIG_HEAP2_BASE          : The base address of the SRAM in the FSMC
 *                              address space
 * CONFIG_HEAP2_SIZE          : The size of the SRAM in the FSMC
 *                              address space
 * CONFIG_MM_REGIONS          : Must be set to a large enough value to
 *                              include the additional regions.
 */

#ifndef CONFIG_STM32L4_FSMC
#  undef CONFIG_STM32L4_FSMC_SRAM
#endif

/* STM32L4[7,8]6xx have 128 Kib in two banks, both accessible to DMA:
 *
 *   1) 96 KiB of System SRAM beginning at address 0x2000:0000 - 0x2001:8000
 *   2) 32 KiB of System SRAM beginning at address 0x1000:0000 - 0x1000:8000
 *
 * STM32L496xx have 320 Kib in two banks, both accessible to DMA:
 *
 *   1) 256 KiB of System SRAM beginning at address 0x2000:0000 - 0x2004:0000
 *   2) 64 KiB of System SRAM beginning at address 0x1000:0000 - 0x1001:0000
 *
 * STM32L4Rxxx have 640 Kib in three banks:
 *
 *   1) 192 KiB of System SRAM beginning at address 0x2000:0000 - 0x2003:0000
 *   2) 64 KiB of System SRAM beginning at address 0x1000:0000 - 0x1001:0000
 *   3) 384 KiB of System SRAM beginning at address 0x2004:0000 - 0x200A:0000
 *
 * In addition, external FSMC SRAM may be available.
 */

/* Set the range of system SRAM */

#define SRAM1_START  STM32L4_SRAM_BASE
#define SRAM1_END    (SRAM1_START + STM32L4_SRAM1_SIZE)

/* Set the range of SRAM2 as well, requires a second memory region */

#define SRAM2_START  STM32L4_SRAM2_BASE
#define SRAM2_END    (SRAM2_START + STM32L4_SRAM2_SIZE)

/* Set the range of SRAM3, requiring a third memory region */

#ifdef STM32L4_SRAM3_SIZE
#  define SRAM3_START  STM32L4_SRAM3_BASE
#  define SRAM3_END    (SRAM3_START + STM32L4_SRAM3_SIZE)
#  define CONFIG_STM32L4_SRAM3_HEAP
#endif

#if defined(CONFIG_STM32L4_SRAM3_HEAP)
/* TODO: better check here */
#  if CONFIG_MM_REGIONS < 3
#    error you need at least 3 memory manager regions to support SRAM2 and SRAM3
#  endif
#endif

#if defined(CONFIG_STM32L4_SRAM2_HEAP) && defined(CONFIG_STM32L4_FSMC_SRAM_HEAP)
#  if CONFIG_MM_REGIONS < 3
#    error you need at least 3 memory manager regions to support SRAM2 and FSMC
#  endif
#endif

#if defined(CONFIG_STM32L4_SRAM2_HEAP) || defined(CONFIG_STM32L4_FSMC_SRAM_HEAP)
#  if CONFIG_MM_REGIONS < 2
#    error you need at least 2 memory manager regions to support SRAM2 or FSMC
#  endif
#endif

#if CONFIG_MM_REGIONS < 1
#  warning you have no heap; malloc() will fail.  are you sure?
#endif

/* If FSMC SRAM is going to be used as heap, then verify that the starting
 * address and size of the external SRAM region has been provided in the
 * configuration (as CONFIG_HEAP2_BASE and CONFIG_HEAP2_SIZE).
 */

#ifdef CONFIG_STM32L4_FSMC_SRAM
#  if !defined(CONFIG_HEAP2_BASE) || !defined(CONFIG_HEAP2_SIZE)
#    error "CONFIG_HEAP2_BASE and CONFIG_HEAP2_SIZE must be provided"
#    undef CONFIG_STM32L4_FSMC_SRAM
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
static inline void up_heap_color(FAR void *start, size_t size)
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
 *     Kernel IDLE thread stack.  Size determined by CONFIG_IDLETHREAD_STACKSIZE.
 *     Padding for alignment
 *     User .data region.  Size determined at link time.
 *     User .bss region  Size determined at link time.
 *     Kernel heap.  Size determined by CONFIG_MM_KERNEL_HEAPSIZE.
 *     User heap.  Extends to the end of SRAM.
 *
 ****************************************************************************/

void up_allocate_heap(FAR void **heap_start, size_t *heap_size)
{
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend + CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = SRAM1_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)SRAM1_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the SRAM1_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(ubase, usize);
  DEBUGASSERT((SRAM1_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = SRAM1_END - usize;

  /* Return the user-space heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void *)ubase;
  *heap_size  = usize;

  /* Colorize the heap for debug */

  up_heap_color((FAR void *)ubase, usize);

  /* Allow user-mode access to the user heap memory */

   stm32l4_mpu_uheap(ubase, usize);
#else

  /* Return the heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void *)g_idle_topstack;
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
void up_allocate_kheap(FAR void **heap_start, size_t *heap_size)
{
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend + CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = SRAM1_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)SRAM1_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the SRAM1_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(ubase, usize);
  DEBUGASSERT((SRAM1_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = SRAM1_END - usize;

  /* Return the kernel heap settings (i.e., the part of the heap region
   * that was not dedicated to the user heap).
   */

  *heap_start = (FAR void *)USERSPACE->us_bssend;
  *heap_size  = ubase - (uintptr_t)USERSPACE->us_bssend;
}
#endif

/****************************************************************************
 * Name: up_addregion
 *
 * Description:
 *   Memory may be added in non-contiguous chunks.  Additional chunks are
 *   added by calling this function.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void up_addregion(void)
{

#ifdef CONFIG_STM32L4_SRAM2_HEAP

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Allow user-mode access to the SRAM2 heap */

  stm32l4_mpu_uheap((uintptr_t)SRAM2_START, SRAM2_END-SRAM2_START);

#endif

  /* Colorize the heap for debug */

  up_heap_color((FAR void *)SRAM2_START, SRAM2_END-SRAM2_START);

  /* Add the SRAM2 user heap region. */

  kumm_addregion((FAR void *)SRAM2_START, SRAM2_END-SRAM2_START);

#endif /* SRAM2 */

#ifdef CONFIG_STM32L4_SRAM3_HEAP

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Allow user-mode access to the SRAM3 heap */

  stm32l4_mpu_uheap((uintptr_t)SRAM3_START, SRAM3_END-SRAM3_START);

#endif

  /* Colorize the heap for debug */

  up_heap_color((FAR void *)SRAM3_START, SRAM3_END-SRAM3_START);

  /* Add the SRAM2 user heap region. */

  kumm_addregion((FAR void *)SRAM3_START, SRAM3_END-SRAM3_START);

#endif /* SRAM3 */

#ifdef CONFIG_STM32L4_FSMC_SRAM_HEAP
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Allow user-mode access to the FSMC SRAM user heap memory */

   stm32l4_mpu_uheap((uintptr_t)CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE);

#endif

  /* Colorize the heap for debug */

  up_heap_color((FAR void *)CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE);

  /* Add the external FSMC SRAM user heap region. */

  kumm_addregion((FAR void *)CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE);
#endif
}
#endif
