/****************************************************************************
 * arch/arm/src/stm32h7/up_allocateheap.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <arch/stm32h7/chip.h>
#include <arch/board/board.h>

#include "mpu.h"
#include "up_arch.h"
#include "up_internal.h"

#include "chip/stm32_memorymap.h"
// TODO: #include "stm32_mpuinit.h"
// TODO: #include "stm32_dtcm.h"

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
 * CONFIG_STM32H7_FMC=y         : Enables the FMC
 * CONFIG_STM32H7_FMC_S[D]RAM=y : SRAM and/or SDRAM is available via the FMC.
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

/* Set the start and end of the SRAMs */

#define SRAM_START   STM32_SRAM_BASE
#define SRAM_END     (SRAM_START + STM32H7_SRAM_SIZE)

#define SRAM123_START STM32_SRAM123_BASE
#define SRAM123_END   (SRAM123_START + STM32H7_SRAM123_SIZE)

#define SRAM4_START  STM32_SRAM4_BASE
#define SRAM4_END    (SRAM4_START + STM32H7_SRAM4_SIZE)

/* The STM32 H7 has DTCM memory */

#undef HAVE_DTCM
#define HAVE_DTCM 1
#if !defined(DTCM_START) || !defined(DTCM_END)
#  undef HAVE_DTCM
#endif

/* DTCM to be excluded from the main heap. */

#ifdef CONFIG_STM32H7_DTCMEXCLUDE
#  undef HAVE_DTCM
#endif

/* There are <x> possible heap configurations:
 *
 * Configuration 1. System SRAM (only)
 *                  CONFIG_MM_REGIONS == 1
 * Configuration 2. System SRAM and SRAM123
 *                  CONFIG_MM_REGIONS == 2
 * Configuration 3. System SRAM and SRAM123 and DTCM
 *                  CONFIG_MM_REGIONS == 3
 *                  HAVE_DTCM defined
 * Configuration 4. System SRAM and SRAM123 and DTCM
 *                  CONFIG_MM_REGIONS == 3
 *                  HAVE_DTCM defined
 *
 * TODO ....
 *
 * Let's make sure that all definitions are consistent before doing
 * anything else
 */

// TODO: Check configurations ....

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
  size_t    usize = SRAM_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)SRAM_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the SRAM_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(ubase, usize);
  DEBUGASSERT((SRAM_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = SRAM_END - usize;

  /* Return the user-space heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void *)ubase;
  *heap_size  = usize;

  /* Colorize the heap for debug */

  up_heap_color((FAR void *)ubase, usize);

  /* Allow user-mode access to the user heap memory */

   stm32_mpu_uheap(ubase, usize);
#else

  /* Return the heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void *)g_idle_topstack;
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
void up_allocate_kheap(FAR void **heap_start, size_t *heap_size)
{
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend + CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = SRAM_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)SRAM_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the SRAM_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(ubase, usize);
  DEBUGASSERT((SRAM_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = SRAM_END - usize;

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
    // TODO ....
}
#endif
