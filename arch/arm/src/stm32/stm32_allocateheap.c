/****************************************************************************
 * arch/arm/src/stm32/up_allocateheap.c
 *
 *   Copyright (C) 2011-2013 Gregory Nutt. All rights reserved.
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
#include <nuttx/kmalloc.h>
#include <nuttx/userspace.h>

#include <arch/board/board.h>

#include "chip.h"
#include "mpu.h"
#include "up_arch.h"
#include "up_internal.h"
#include "stm32_mpuinit.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/
/* Internal SRAM is available in all members of the STM32 family. The
 * following definitions must be provided to specify the size and
 * location of internal(system) SRAM:
 *
 * CONFIG_RAM_END             : End address (+1) of SRAM (F1 family only, the
 *                            : F4 family uses the a priori end of SRAM)
 *
 * The F4 family also contains internal CCM SRAM.  This SRAM is different
 * because it cannot be used for DMA.  So if DMA needed, then the following
 * should be defined to exclude CCM SRAM from the heap:
 *
 * CONFIG_STM32_CCMEXCLUDE    : Exclude CCM SRAM from the HEAP
 *
 * In addition to internal SRAM, SRAM may also be available through the FSMC.
 * In order to use FSMC SRAM, the following additional things need to be
 * present in the NuttX configuration file:
 *
 * CONFIG_STM32_FSMC=y        : Enables the FSMC
 * CONFIG_STM32_FSMC_SRAM=y   : Indicates that SRAM is available via the
 *                              FSMC (as opposed to an LCD or FLASH).
 * CONFIG_HEAP2_BASE          : The base address of the SRAM in the FSMC
 *                              address space
 * CONFIG_HEAP2_SIZE          : The size of the SRAM in the FSMC
 *                              address space
 * CONFIG_MM_REGIONS          : Must be set to a large enough value to
 *                              include the FSMC SRAM (as determined by
 *                              the rules provided below)
 */

#ifndef CONFIG_STM32_FSMC
#  undef CONFIG_STM32_FSMC_SRAM
#endif

/* The STM32L15xxx family has only internal SRAM.  The heap is in one contiguous
 * block starting at g_idle_topstack and extending through CONFIG_RAM_END.
 */

#if defined(CONFIG_STM32_STM32L15XX)

   /* Set the end of system SRAM */

#  define SRAM1_END CONFIG_RAM_END

   /* There is no FSMC (Other EnergyLite STM32's do have an FSMC, but not the STM32L15X */

#  undef CONFIG_STM32_FSMC_SRAM

   /* The STM32L EnergyLite family has no CCM SRAM */

#  undef CONFIG_STM32_CCMEXCLUDE
#  define CONFIG_STM32_CCMEXCLUDE 1

   /* Only one memory region can be support (internal SRAM) */

#  if CONFIG_MM_REGIONS > 1
#    error "CONFIG_MM_REGIONS > 1.  The STM32L15X has only one memory region."
#  endif

/* For the STM312F10xxx family, all internal SRAM is in one contiguous block
 * starting at g_idle_topstack and extending through CONFIG_RAM_END (my apologies
 * for the bad naming).  In addition, external FSMC SRAM may be available.
 */

#elif defined(CONFIG_STM32_STM32F10XX)

   /* Set the end of system SRAM */

#  define SRAM1_END CONFIG_RAM_END

   /* Check if external FSMC SRAM is provided */

#  if CONFIG_STM32_FSMC_SRAM
#    if CONFIG_MM_REGIONS < 2
#      warning "FSMC SRAM not included in the heap"
#      undef CONFIG_STM32_FSMC_SRAM
#    elif CONFIG_MM_REGIONS > 2
#      error "CONFIG_MM_REGIONS > 2 but I don't know what some of the region(s) are"
#      undef CONFIG_MM_REGIONS
#      define CONFIG_MM_REGIONS 2
#    endif
#  elif CONFIG_MM_REGIONS > 1
#    error "CONFIG_MM_REGIONS > 1 but I don't know what the other region(s) are"
#  endif

   /* The STM32 F1 has no CCM SRAM */

#  undef CONFIG_STM32_CCMEXCLUDE
#  define CONFIG_STM32_CCMEXCLUDE 1

/* Members of the STM32F30xxx family has a variable amount of SRAM from 24
 * to 40Kb plus 8KB if CCM SRAM.  No external RAM is supported (the F3 family
 * has no FSMC).
 *
 * As a complication, CCM SRAM cannot be used for DMA.  So, if STM32 DMA is
 * enabled, CCM SRAM should probably be excluded from the heap.
 */

#elif defined(CONFIG_STM32_STM32F30XX)

   /* Set the end of system SRAM */

#  define SRAM1_END CONFIG_RAM_END

   /* Set the range of CCM SRAM as well (although we may not use it) */

#  define SRAM2_START 0x10000000
#  define SRAM2_END   0x10002000

   /* There is no FSMC */

#  undef CONFIG_STM32_FSMC_SRAM

   /* There are 2 possible SRAM configurations:
    *
    * Configuration 1. System SRAM (only)
    *                  CONFIG_MM_REGIONS == 1
    *                  CONFIG_STM32_CCMEXCLUDE defined
    * Configuration 2. System SRAM and CCM SRAM
    *                  CONFIG_MM_REGIONS == 2
    *                  CONFIG_STM32_CCMEXCLUDE NOT defined
    */

#    if CONFIG_MM_REGIONS < 2

       /* Only one memory region.  Force Configuration 1 */

#      ifndef CONFIG_STM32_CCMEXCLUDE
#        warning "CCM SRAM excluded from the heap"
#        define CONFIG_STM32_CCMEXCLUDE 1
#      endif

   /* CONFIG_MM_REGIONS may be 2 if CCM SRAM is included in the head */

#    elif CONFIG_MM_REGIONS >= 2
#      if CONFIG_MM_REGIONS > 2
#         error "No more than two memory regions can be supported (CONFIG_MM_REGIONS)"
#         undef CONFIG_MM_REGIONS
#         define CONFIG_MM_REGIONS 2
#      endif

     /* Two memory regions is okay if CCM SRAM is not disabled. */

#      ifdef CONFIG_STM32_CCMEXCLUDE

         /* Configuration 1: CONFIG_MM_REGIONS should have been 2 */

#        error "CONFIG_MM_REGIONS >= 2 but but CCM SRAM is excluded (CONFIG_STM32_CCMEXCLUDE)"
#        undef CONFIG_MM_REGIONS
#        define CONFIG_MM_REGIONS 1
#      else

         /* Configuration 2: DMA should be disabled */

#        ifdef CONFIG_ARCH_DMA
#          warning "CCM SRAM is included in the heap AND DMA is enabled"
#        endif
#      endif
#    endif

/* Most members of both the STM32F20xxx and STM32F40xxx families have 128Kib
 * in two banks:
 *
 *   1) 112KiB of System SRAM beginning at address 0x2000:0000
 *   2)  16KiB of System SRAM beginning at address 0x2001:c000
 *
 * The STM32F401 family is an exception and has only 96Kib total on one bank:
 *
 *   3)  96KiB of System SRAM beginning at address 0x2000:0000
 *
 * Members of the STM32F40xxx family have an additional 64Kib of CCM RAM
 * for a total of 192KB.
 *
 *   4)  64Kib of CCM SRAM beginning at address 0x1000:0000
 *
 * The STM32F427/437/429/439 parts have another 64KiB of System SRAM for a total
 * of 256KiB.
 *
 *   5)  64Kib of System SRAM beginning at address 0x2002:0000
 *
 * As determined by the linker script, g_heapbase lies in the 112KiB memory
 * region and that extends to 0x2001:0000.  But the  first and second memory
 * regions are contiguous and treated as one in this logic that extends to
 * 0x2002:0000 (or 0x2003:0000 for the F427/F437/F429/F439).
 *
 * As a complication, CCM SRAM cannot be used for DMA.  So, if STM32 DMA is enabled,
 * CCM SRAM should probably be excluded from the heap or the application must take
 * extra care to ensure that DMA buffers are not allocated in CCM SRAM.
 *
 * In addition, external FSMC SRAM may be available.
 */

#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)

   /* The STM32 F2 and the STM32 F401 have no CCM SRAM */

#  if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F401)
#    undef CONFIG_STM32_CCMEXCLUDE
#    define CONFIG_STM32_CCMEXCLUDE 1
#  endif

   /* Set the end of system SRAM */

#  if defined(CONFIG_STM32_STM32F401)
#    define SRAM1_END 0x20018000
#  elif defined(CONFIG_STM32_STM32F427) || defined(CONFIG_STM32_STM32F429)
#    define SRAM1_END 0x20030000
#  else
#    define SRAM1_END 0x20020000
#  endif

   /* Set the range of CCM SRAM as well (although we may not use it) */

#  define SRAM2_START 0x10000000
#  define SRAM2_END   0x10010000

   /* There are 4 possible SRAM configurations:
    *
    * Configuration 1. System SRAM (only)
    *                  CONFIG_MM_REGIONS == 1
    *                  CONFIG_STM32_FSMC_SRAM NOT defined
    *                  CONFIG_STM32_CCMEXCLUDE defined
    * Configuration 2. System SRAM and CCM SRAM
    *                  CONFIG_MM_REGIONS == 2
    *                  CONFIG_STM32_FSMC_SRAM NOT defined
    *                  CONFIG_STM32_CCMEXCLUDE NOT defined
    * Configuration 3. System SRAM and FSMC SRAM
    *                  CONFIG_MM_REGIONS == 2
    *                  CONFIG_STM32_FSMC_SRAM defined
    *                  CONFIG_STM32_CCMEXCLUDE defined
    * Configuration 4. System SRAM, CCM SRAM, and FSMC SRAM
    *                  CONFIG_MM_REGIONS == 3
    *                  CONFIG_STM32_FSMC_SRAM defined
    *                  CONFIG_STM32_CCMEXCLUDE NOT defined
    *
    * Let's make sure that all definitions are consistent before doing
    * anything else
    */

#  if defined(CONFIG_STM32_FSMC_SRAM)

   /* Configuration 3 or 4. External SRAM is available.  CONFIG_MM_REGIONS
    * should be at least 2.
    */

#    if CONFIG_MM_REGIONS < 2

       /* Only one memory region.  Force Configuration 1 */

#      warning "FSMC SRAM (and CCM SRAM) excluded from the heap"
#      undef CONFIG_STM32_FSMC_SRAM
#      undef CONFIG_STM32_CCMEXCLUDE
#      define CONFIG_STM32_CCMEXCLUDE 1

   /* CONFIG_MM_REGIONS may be 3 if CCM SRAM is included in the head */

#    elif CONFIG_MM_REGIONS > 2

       /* More than two memory regions.  This is okay if CCM SRAM is not
        * disabled.
        */

#      if defined(CONFIG_STM32_CCMEXCLUDE)

         /* Configuration 3: CONFIG_MM_REGIONS should have been 2 */

#        error "CONFIG_MM_REGIONS > 2 but I don't know what some of the region(s) are"
#        undef CONFIG_MM_REGIONS
#        define CONFIG_MM_REGIONS 2
#      else

         /* Configuration 4: DMA should be disabled and CONFIG_MM_REGIONS
          * should be 3.
          */

#        ifdef CONFIG_ARCH_DMA
#          warning "CCM SRAM is included in the heap AND DMA is enabled"
#        endif

#        if CONFIG_MM_REGIONS != 3
#          error "CONFIG_MM_REGIONS > 3 but I don't know what some of the region(s) are"
#          undef CONFIG_MM_REGIONS
#          define CONFIG_MM_REGIONS 3
#        endif
#      endif

   /* CONFIG_MM_REGIONS is exactly 2.  We cannot support both CCM SRAM and
    * FSMC SRAM.
    */

#    elif !defined(CONFIG_STM32_CCMEXCLUDE)
#      error "CONFIG_MM_REGIONS == 2, cannot support both CCM SRAM and FSMC SRAM"
#      undef CONFIG_STM32_CCMEXCLUDE
#      define CONFIG_STM32_CCMEXCLUDE 1
#    endif

#  elif !defined(CONFIG_STM32_CCMEXCLUDE)

   /* Configuration 2: FSMC SRAM is not used, but CCM SRAM is requested.  DMA
    * should be disabled and CONFIG_MM_REGIONS should be 2.
    */

#    ifdef CONFIG_ARCH_DMA
#      warning "CCM SRAM is included in the heap AND DMA is enabled"
#    endif

#    if CONFIG_MM_REGIONS < 2
#      error "CCM SRAM excluded from the heap because CONFIG_MM_REGIONS < 2"
#      undef CONFIG_STM32_CCMEXCLUDE
#      define CONFIG_STM32_CCMEXCLUDE 1
#    elif CONFIG_MM_REGIONS > 2
#      error "CONFIG_MM_REGIONS > 2 but I don't know what some of the region(s) are"
#      undef CONFIG_MM_REGIONS
#      define CONFIG_MM_REGIONS 2
#    endif
#  endif

#else
#  error "Unsupported STM32 chip"
#endif

/* If FSMC SRAM is going to be used as heap, then verify that the starting
 * address and size of the external SRAM region has been provided in the
 * configuration (as CONFIG_HEAP2_BASE and CONFIG_HEAP2_SIZE).
 */

#ifdef CONFIG_STM32_FSMC_SRAM
#  if !defined(CONFIG_HEAP2_BASE) || !defined(CONFIG_HEAP2_SIZE)
#    error "CONFIG_HEAP2_BASE and CONFIG_HEAP2_SIZE must be provided"
#    undef CONFIG_STM32_FSMC_SRAM
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

#ifdef CONFIG_DEBUG_HEAP
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
 *   For the kernel build (CONFIG_NUTTX_KERNEL=y) with both kernel- and
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
#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
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

  log2  = (int)mpu_log2regionfloor(usize);
  DEBUGASSERT((SRAM1_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = SRAM1_END - usize;

  /* Return the user-space heap settings */

  board_led_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void*)ubase;
  *heap_size  = usize;

  /* Colorize the heap for debug */

  up_heap_color((FAR void*)ubase, usize);

  /* Allow user-mode access to the user heap memory */

   stm32_mpu_uheap((uintptr_t)ubase, usize);
#else

  /* Return the heap settings */

  board_led_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void*)g_idle_topstack;
  *heap_size  = SRAM1_END - g_idle_topstack;

  /* Colorize the heap for debug */

  up_heap_color(*heap_start, *heap_size);
#endif
}

/****************************************************************************
 * Name: up_allocate_kheap
 *
 * Description:
 *   For the kernel build (CONFIG_NUTTX_KERNEL=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function allocates
 *   (and protects) the kernel-space heap.
 *
 ****************************************************************************/

#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
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

  log2  = (int)mpu_log2regionfloor(usize);
  DEBUGASSERT((SRAM1_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = SRAM1_END - usize;

  /* Return the kernel heap settings (i.e., the part of the heap region
   * that was not dedicated to the user heap).
   */

  *heap_start = (FAR void*)USERSPACE->us_bssend;
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
#ifndef CONFIG_STM32_CCMEXCLUDE
#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Allow user-mode access to the STM32F20xxx/STM32F40xxx CCM SRAM heap */

  stm32_mpu_uheap((uintptr_t)SRAM2_START, SRAM2_END-SRAM2_START);

#endif

  /* Colorize the heap for debug */

  up_heap_color((FAR void*)SRAM2_START, SRAM2_END-SRAM2_START);

  /* Add the STM32F20xxx/STM32F40xxx CCM SRAM user heap region. */

  kumm_addregion((FAR void*)SRAM2_START, SRAM2_END-SRAM2_START);
#endif

#ifdef CONFIG_STM32_FSMC_SRAM
#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Allow user-mode access to the FSMC SRAM user heap memory */

   stm32_mpu_uheap((uintptr_t)CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE);

#endif

  /* Colorize the heap for debug */

  up_heap_color((FAR void*)CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE);

  /* Add the external FSMC SRAM user heap region. */

  kumm_addregion((FAR void*)CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE);
#endif
}
#endif
