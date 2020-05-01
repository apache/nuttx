/****************************************************************************
 * arch/arm/src/sam34/sam_allocateheap.c
 *
 *   Copyright (C) 2010, 2013, 2015 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include "mpu.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include "chip.h"
#include "sam_mpuinit.h"
#include "sam_periphclks.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* All SAM's have SRAM0.  The SAM3U family also have SRAM1 and possibly
 * NFCSRAM.  NFCSRAM may not be used, however, if NAND support is enabled.
 * In addition, the SAM3U and SAM4S have external SRAM at CS0 (EXTSRAM0).
 * Support for external SRAM at CS1-3 is not fully implemented.
 */

#undef HAVE_SRAM1_REGION     /* Assume no internal SRAM1 */
#undef HAVE_NFCSRAM_REGION   /* Assume no NFC SRAM */
#undef HAVE_EXTSRAM0_REGION  /* Assume no external SRAM at CS0 */
#undef HAVE_EXTSRAM1_REGION  /* Assume no external SRAM at CS1 */
#undef HAVE_EXTSRAM2_REGION  /* Assume no external SRAM at CS2 */
#undef HAVE_EXTSRAM3_REGION  /* Assume no external SRAM at CS3 */

/* Check if external SRAM is supported and, if so, it is is intended
 * to be used as heap.
 */

#if !defined(CONFIG_SAM34_EXTSRAM0) || !defined(CONFIG_SAM34_EXTSRAM0HEAP)
#  undef CONFIG_SAM34_EXTSRAM0SIZE
#  define CONFIG_SAM34_EXTSRAM0SIZE 0
#endif

#if !defined(CONFIG_SAM34_EXTSRAM1) || !defined(CONFIG_SAM34_EXTSRAM1HEAP)
#  undef CONFIG_SAM34_EXTSRAM1SIZE
#  define CONFIG_SAM34_EXTSRAM1SIZE 0
#endif

#if !defined(CONFIG_SAM34_EXTSRAM2) || !defined(CONFIG_SAM34_EXTSRAM2HEAP)
#  undef CONFIG_SAM34_EXTSRAM2SIZE
#  define CONFIG_SAM34_EXTSRAM2SIZE 0
#endif

#if !defined(CONFIG_SAM34_EXTSRAM3) || !defined(CONFIG_SAM34_EXTSRAM3HEAP)
#  undef CONFIG_SAM34_EXTSRAM3SIZE
#  define CONFIG_SAM34_EXTSRAM3SIZE 0
#endif

/* SAM3U, SAM3X, and SAM3A Unique memory configurations */

#if defined(CONFIG_ARCH_CHIP_SAM3U) || defined(CONFIG_ARCH_CHIP_SAM3X) || \
    defined(CONFIG_ARCH_CHIP_SAM3A)
#  ifdef CONFIG_SAM34_NAND
#    undef SAM34_NFCSRAM_SIZE
#    define SAM34_NFCSRAM_SIZE 0
#  endif

#  if SAM34_SRAM1_SIZE > 0
#    if CONFIG_MM_REGIONS > 1
#      define HAVE_SRAM1_REGION 1
#    else
#      warning "CONFIG_MM_REGIONS < 2: SRAM1 not included in HEAP"
#    endif
#  endif

#  if SAM34_NFCSRAM_SIZE > 0
#    if CONFIG_MM_REGIONS > 2
#      define HAVE_NFCSRAM_REGION
#    else
#      warning "CONFIG_MM_REGIONS < 3: NFC SRAM not included in HEAP"
#    endif

#    if CONFIG_SAM34_EXTSRAM0SIZE > 0
#      if CONFIG_MM_REGIONS > 3
#        define HAVE_EXTSRAM0_REGION 1
#      else
#        warning "CONFIG_MM_REGIONS < 4: External SRAM not included in HEAP"
#      endif
#    endif

#  elif CONFIG_SAM34_EXTSRAM0SIZE > 0
#    if CONFIG_MM_REGIONS > 2
#       define HAVE_EXTSRAM0_REGION 1
#    else
#      warning "CONFIG_MM_REGIONS < 3: External SRAM not included in HEAP"
#    endif
#  endif
#else

/* The SAM4S and SAM4L may have only internal SRAM0 and external SRAM0 */

#  if CONFIG_SAM34_EXTSRAM0SIZE > 0
#    if CONFIG_MM_REGIONS > 1
#       define HAVE_EXTSRAM0_REGION 1
#    else
#      warning "CONFIG_MM_REGIONS < 2: External SRAM not included in HEAP"
#    endif
#  endif
#endif

/* Check common SRAM0 configuration */

#if CONFIG_RAM_END > (SAM_INTSRAM0_BASE+SAM34_SRAM0_SIZE)
#  error "CONFIG_RAM_END is beyond the end of SRAM0"
#  undef CONFIG_RAM_END
#  define CONFIG_RAM_END (SAM_INTSRAM0_BASE+SAM34_SRAM0_SIZE)
#elif CONFIG_RAM_END < (SAM_INTSRAM0_BASE+SAM34_SRAM0_SIZE)
#  warning "CONFIG_RAM_END is before end of SRAM0... not all of SRAM0 used"
#endif

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
  size_t    usize = CONFIG_RAM_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)CONFIG_RAM_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the CONFIG_RAM_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(usize);
  DEBUGASSERT((CONFIG_RAM_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = CONFIG_RAM_END - usize;

  /* Return the user-space heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void *)ubase;
  *heap_size  = usize;

  /* Allow user-mode access to the user heap memory */

   sam_mpu_uheap((uintptr_t)ubase, usize);
#else

  /* Return the heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void *)g_idle_topstack;
  *heap_size  = CONFIG_RAM_END - g_idle_topstack;
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
  size_t    usize = CONFIG_RAM_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)CONFIG_RAM_END);

  /* Adjust that size to account for MPU alignment requirements.
   * NOTE that there is an implicit assumption that the CONFIG_RAM_END
   * is aligned to the MPU requirement.
   */

  log2  = (int)mpu_log2regionfloor(usize);
  DEBUGASSERT((CONFIG_RAM_END & ((1 << log2) - 1)) == 0);

  usize = (1 << log2);
  ubase = CONFIG_RAM_END - usize;

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
  /* The SAM3U also have SRAM1 and NFCSRAM,  We will add these as regions
   * the first two additional memory regions if we have them.
   */

#ifdef HAVE_SRAM1_REGION
  /* Allow user access to the heap memory */

  sam_mpu_uheap(SAM_INTSRAM1_BASE, SAM34_SRAM1_SIZE);

  /* Add the region */

  kumm_addregion((FAR void *)SAM_INTSRAM1_BASE, SAM34_SRAM1_SIZE);

#endif /* HAVE_SRAM1_REGION */

#ifdef HAVE_NFCSRAM_REGION
#if defined(CONFIG_ARCH_CHIP_SAM3X) || defined(CONFIG_ARCH_CHIP_SAM3A)
  /* In the 3X/3A family I note that clocking must appled to the SMC module
   * in order for the NFCS SRAM to be functional.  I don't recall such an
   * issue with the 3U.
   */

  sam_smc_enableclk();
#endif

  /* Allow user access to the heap memory */

  sam_mpu_uheap(SAM_NFCSRAM_BASE, SAM34_NFCSRAM_SIZE);

  /* Add the region */

  kumm_addregion((FAR void *)SAM_NFCSRAM_BASE, SAM34_NFCSRAM_SIZE);

#endif /* HAVE_NFCSRAM_REGION */

#ifdef HAVE_EXTSRAM0_REGION
  /* Allow user access to the heap memory */

  sam_mpu_uheap(SAM_EXTCS0_BASE, CONFIG_SAM34_EXTSRAM0SIZE);

  /* Add the region */

  kumm_addregion((FAR void *)SAM_EXTCS0_BASE, CONFIG_SAM34_EXTSRAM0SIZE);

#endif /* HAVE_EXTSRAM0_REGION */

#ifdef HAVE_EXTSRAM1_REGION
  /* Allow user access to the heap memory */

  sam_mpu_uheap(SAM_EXTCS1_BASE, CONFIG_SAM34_EXTSRAM1SIZE);

  /* Add the region */

  kumm_addregion((FAR void *)SAM_EXTCS1_BASE, CONFIG_SAM34_EXTSRAM1SIZE);

#endif /* HAVE_EXTSRAM0_REGION */

#ifdef HAVE_EXTSRAM2_REGION
  /* Allow user access to the heap memory */

  sam_mpu_uheap(SAM_EXTCS2_BASE, CONFIG_SAM34_EXTSRAM2SIZE);

  /* Add the region */

  kumm_addregion((FAR void *)SAM_EXTCS2_BASE, CONFIG_SAM34_EXTSRAM2SIZE);

#endif /* HAVE_EXTSRAM0_REGION */

#ifdef HAVE_EXTSRAM3_REGION
  /* Allow user access to the heap memory */

  sam_mpu_uheap(SAM_EXTCS3_BASE, CONFIG_SAM34_EXTSRAM3SIZE);

  /* Add the region */

  kumm_addregion((FAR void *)SAM_EXTCS3_BASE, CONFIG_SAM34_EXTSRAM3SIZE);

#endif /* HAVE_EXTSRAM0_REGION */
}
#endif /* CONFIG_MM_REGIONS > 1 */
