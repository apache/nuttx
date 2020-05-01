/****************************************************************************
 * arch/arm/src/stm32f7/stm32_allocateheap.c
 *
 *   Copyright (C) 2015, 2017 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <nuttx/userspace.h>

#include <arch/stm32f7/chip.h>
#include <arch/board/board.h>

#include "mpu.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include "hardware/stm32_memorymap.h"
#include "stm32_mpuinit.h"
#include "stm32_dtcm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Internal SRAM is available in all members of the STM32 family. The
 * following definitions must be provided to specify the size and
 * location of internal(system) SRAM:
 *
 * CONFIG_RAM_END               : End address (+1) of SRAM (F1 family only,
 *                              : the F4 family uses the a priori end of
 *                              : SRAM)
 *
 * In addition to internal SRAM, external RAM may also be available through
 * the FMC.  In order to use FMC RAM, the following additional things need
 * to be present in the NuttX configuration file:
 *
 * CONFIG_STM32F7_FMC=y         : Enables the FMC
 * CONFIG_STM32F7_FMC_S[D]RAM=y : SRAM and/or SDRAM is available via the FMC.
 *                                Either of these autoselects CONFIG_ARCH_HAVE_HEAP2
 *                                which is what we are interested in here.
 * CONFIG_HEAP2_BASE            : The base address of the external RAM in the FMC
 *                                address space
 * CONFIG_HEAP2_SIZE            : The size of the external RAM in the FMC
 *                                address space
 * CONFIG_MM_REGIONS            : Must be set to a large enough value to
 *                                include the FMC external RAM (as determined by
 *                                the rules provided below)
 */

/* Set the start and end of SRAM1 and SRAM2 */

#define SRAM1_START  STM32_SRAM1_BASE
#define SRAM1_END    (SRAM1_START + STM32F7_SRAM1_SIZE)

#define SRAM2_START  STM32_SRAM2_BASE
#define SRAM2_END    (SRAM2_START + STM32F7_SRAM2_SIZE)

/* The STM32 F7 has DTCM memory */

#undef HAVE_DTCM
#define HAVE_DTCM 1
#if !defined(DTCM_START) || !defined(DTCM_END)
#  undef HAVE_DTCM
#endif

/* DTCM to be excluded from the main heap. */

#ifdef CONFIG_STM32F7_DTCMEXCLUDE
#  undef HAVE_DTCM
#endif

/* We can't possibly have FMC external RAM if the FMC is not enabled */

#ifndef CONFIG_STM32F7_FMC
#  ifdef CONFIG_ARCH_HAVE_HEAP2
#    error CONFIG_ARCH_HAVE_HEAP2 but not CONFIG_STM32F7_FMC! Kconfig flawed?
#  endif
#  undef CONFIG_ARCH_HAVE_HEAP2
#endif

/* If FMC external RAM is going to be used as heap, then verify that the starting
 * address and size of the external SRAM region has been provided in the
 * configuration (as CONFIG_HEAP2_BASE and CONFIG_HEAP2_SIZE).
 */

#ifdef CONFIG_ARCH_HAVE_HEAP2
#  if !defined(CONFIG_HEAP2_BASE) || !defined(CONFIG_HEAP2_SIZE)
#    error CONFIG_HEAP2_BASE and CONFIG_HEAP2_SIZE must be provided
#    undef CONFIG_ARCH_HAVE_HEAP2
#  endif
#endif

#ifdef CONFIG_ARCH_HAVE_HEAP2
#  if CONFIG_HEAP2_BASE == 0 || CONFIG_HEAP2_SIZE == 0
#    warning "CONFIG_HEAP2_BASE or CONFIG_HEAP2_SIZE are zero. No HEAP2 enabled!"
#    undef CONFIG_ARCH_HAVE_HEAP2
#  endif
#endif

/* There are 5 possible heap configurations:
 *
 * Configuration 1. System SRAM1 (only)
 *                  CONFIG_MM_REGIONS == 1
 *                  CONFIG_ARCH_HAVE_HEAP2 NOT defined
 * Configuration 2. System SRAM1 and SRAM2
 *                  CONFIG_MM_REGIONS == 2
 *                  CONFIG_ARCH_HAVE_HEAP2 NOT defined
 * Configuration 3. System SRAM1 and SRAM2 and DTCM
 *                  CONFIG_MM_REGIONS == 3
 *                  CONFIG_ARCH_HAVE_HEAP2 undefined
 *                  HAVE_DTCM defined
 * Configuration 4. System SRAM1 and SRAM2 and FMC RAM
 *                  CONFIG_MM_REGIONS == 3
 *                  CONFIG_ARCH_HAVE_HEAP2 defined
 *                  HAVE_DTCM undefined
 * Configuration 5. System SRAM1 and SRAM2 and DTCM and FMC RAM
 *                  CONFIG_MM_REGIONS == 4
 *                  CONFIG_ARCH_HAVE_HEAP2 defined
 *                  HAVE_DTCM defined
 *
 * Let's make sure that all definitions are consistent before doing
 * anything else
 */

#if CONFIG_MM_REGIONS < 2
#  ifdef CONFIG_ARCH_HAVE_HEAP2
#    warning "FMC external RAM excluded from the heap"
#    undef CONFIG_ARCH_HAVE_HEAP2
#  endif
#  ifdef HAVE_DTCM
#    warning "DTCM excluded from the heap"
#    undef HAVE_DTCM
#  endif
#  warning "SRAM2 excluded from the heap"
#elif CONFIG_MM_REGIONS < 3
#  ifdef CONFIG_ARCH_HAVE_HEAP2
#    warning "FMC external RAM excluded from the heap"
#    undef CONFIG_ARCH_HAVE_HEAP2
#  endif
#  ifdef HAVE_DTCM
#    warning "DTCM excluded from the heap"
#    undef HAVE_DTCM
#  endif
#elif CONFIG_MM_REGIONS < 4
#  if defined(CONFIG_ARCH_HAVE_HEAP2) && defined(HAVE_DTCM)
#    warning "CONFIG_MM_REGIONS == 3 but have both FMC external RAM and DTCM. DTCM excluded from the heap."
#    undef  HAVE_DTCM
#  elif !defined(CONFIG_ARCH_HAVE_HEAP2) && !defined(HAVE_DTCM)
#    error  "CONFIG_MM_REGIONS == 3 but I do not know what some of the region(s) are"
#    undef  CONFIG_MM_REGIONS
#    define CONFIG_MM_REGIONS 2
#  endif
#elif CONFIG_MM_REGIONS < 5
#  if !defined(CONFIG_ARCH_HAVE_HEAP2) && !defined(HAVE_DTCM)
#    error  "CONFIG_MM_REGIONS == 4 but I do not know what some of the region(s) are"
#    undef  CONFIG_MM_REGIONS
#    define CONFIG_MM_REGIONS 2
#  elif !defined(CONFIG_ARCH_HAVE_HEAP2) || !defined(HAVE_DTCM)
#    error  "CONFIG_MM_REGIONS == 4 but I do not know what some of the region(s) are"
#    undef  CONFIG_MM_REGIONS
#    define CONFIG_MM_REGIONS 3
#  endif
#else
#  error "CONFIG_MM_REGIONS > 4 but I do not know what some of the region(s) are"
#  undef CONFIG_MM_REGIONS
#  if defined(CONFIG_ARCH_HAVE_HEAP2) && defined(HAVE_DTCM)
#    define CONFIG_MM_REGIONS 4
#  elif defined(CONFIG_ARCH_HAVE_HEAP2) || defined(HAVE_DTCM)
#    define CONFIG_MM_REGIONS 3
#  else
#    define CONFIG_MM_REGIONS 2
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

  log2  = (int)mpu_log2regionfloor(usize);
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

   stm32_mpu_uheap((uintptr_t)ubase, usize);
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

  log2  = (int)mpu_log2regionfloor(usize);
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
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Allow user-mode access to the SRAM2 heap */

  stm32_mpu_uheap((uintptr_t)SRAM2_START, SRAM2_END-SRAM2_START);

#endif

  /* Colorize the heap for debug */

  up_heap_color((FAR void *)SRAM2_START, SRAM2_END-SRAM2_START);

  /* Add the SRAM2 user heap region. */

  kumm_addregion((FAR void *)SRAM2_START, SRAM2_END-SRAM2_START);

#ifdef HAVE_DTCM
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Allow user-mode access to the DTCM heap */

  stm32_mpu_uheap((uintptr_t)DTCM_START, DTCM_END-DTCM_START);

#endif

  /* Colorize the heap for debug */

  up_heap_color((FAR void *)DTCM_START, DTCM_END-DTCM_START);

  /* Add the DTCM user heap region. */

  kumm_addregion((FAR void *)DTCM_START, DTCM_END-DTCM_START);
#endif

#ifdef CONFIG_ARCH_HAVE_HEAP2
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Allow user-mode access to the FMC RAM user heap memory */

   stm32_mpu_uheap((uintptr_t)CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE);

#endif

  /* Colorize the heap for debug */

  up_heap_color((FAR void *)CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE);

  /* Add the external FMC RAM user heap region. */

  kumm_addregion((FAR void *)CONFIG_HEAP2_BASE, CONFIG_HEAP2_SIZE);
#endif
}
#endif
