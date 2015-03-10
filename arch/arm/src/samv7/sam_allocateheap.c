/****************************************************************************
 * arch/arm/src/samv7/sam_allocateheap.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <arch/samv7/chip.h>
#include <arch/board/board.h>

#include "mpu.h"
#include "up_arch.h"
#include "up_internal.h"

#include "sam_mpuinit.h"
#include "sam_periphclks.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/
/* All SAM's have SRAM0.and possibly NFCSRAM.  NFCSRAM may not be used,
 * however, if NAND support is enabled.  External memory may also be
 * available.
 *
 * REVISIT: Support for external SRAM at CS1-3 is not fully implemented.
 */

#undef HAVE_NFCSRAM_REGION   /* Assume no available NFC SRAM */
#undef HAVE_EXTSRAM0_REGION  /* Assume no external SRAM at CS0 */
#undef HAVE_EXTSRAM1_REGION  /* Assume no external SRAM at CS1 */
#undef HAVE_EXTSRAM2_REGION  /* Assume no external SRAM at CS2 */
#undef HAVE_EXTSRAM3_REGION  /* Assume no external SRAM at CS3 */

/* Check if external SRAM is supported and, if so, it is is intended
 * to be used as heap.
 */

#if !defined(CONFIG_SAMV7_EXTSRAM0) || !defined(CONFIG_SAMV7_EXTSRAM0HEAP)
#  undef CONFIG_SAMV7_EXTSRAM0SIZE
#  define CONFIG_SAMV7_EXTSRAM0SIZE 0
#endif

#if !defined(CONFIG_SAMV7_EXTSRAM1) || !defined(CONFIG_SAMV7_EXTSRAM1HEAP)
#  undef CONFIG_SAMV7_EXTSRAM1SIZE
#  define CONFIG_SAMV7_EXTSRAM1SIZE 0
#endif

#if !defined(CONFIG_SAMV7_EXTSRAM2) || !defined(CONFIG_SAMV7_EXTSRAM2HEAP)
#  undef CONFIG_SAMV7_EXTSRAM2SIZE
#  define CONFIG_SAMV7_EXTSRAM2SIZE 0
#endif

#if !defined(CONFIG_SAMV7_EXTSRAM3) || !defined(CONFIG_SAMV7_EXTSRAM3HEAP)
#  undef CONFIG_SAMV7_EXTSRAM3SIZE
#  define CONFIG_SAMV7_EXTSRAM3SIZE 0
#endif

/* NFCSRAM is not available is NAND is supported */

#ifdef CONFIG_SAMV7_NAND
#  undef SAMV7_NFCSRAM_SIZE
#  define SAMV7_NFCSRAM_SIZE 0
#endif

#if SAMV7_NFCSRAM_SIZE > 0
#  if CONFIG_MM_REGIONS > 2
#    define HAVE_NFCSRAM_REGION
#  else
#    warning "CONFIG_MM_REGIONS < 3: NFC SRAM not included in HEAP"
#  endif

#  if CONFIG_SAMV7_EXTSRAM0SIZE > 0
#    if CONFIG_MM_REGIONS > 3
#      define HAVE_EXTSRAM0_REGION 1
#    else
#      warning "CONFIG_MM_REGIONS < 4: External SRAM not included in HEAP"
#    endif
#  endif

#elif CONFIG_SAMV7_EXTSRAM0SIZE > 0
#  if CONFIG_MM_REGIONS > 2
#     define HAVE_EXTSRAM0_REGION 1
#  else
#    warning "CONFIG_MM_REGIONS < 3: External SRAM not included in HEAP"
#  endif
#endif

/* Check internal SRAM configuration */

#ifdef CONFIG_ARMV7M_DTCM
#  define SRAM_BASE SAM_DTCM_BASE
#else
#  define SRAM_BASE SAM_SRAM_BASE
#endif

#if CONFIG_RAM_END > (SRAM_BASE+SAMV7_SRAM_SIZE)
#  error "CONFIG_RAM_END is beyond the end of SRAM"
#  undef CONFIG_RAM_END
#  define CONFIG_RAM_END (SRAM_BASE+SAMV7_SRAM_SIZE)
#elif CONFIG_RAM_END < (SRAM_BASE+SAMV7_SRAM_SIZE)
#  warning "CONFIG_RAM_END is before end of SRAM... not all of SRAM used"
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

  board_led_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void*)ubase;
  *heap_size  = usize;

  /* Allow user-mode access to the user heap memory */

   sam_mpu_uheap((uintptr_t)ubase, usize);
#else

  /* Return the heap settings */

  board_led_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void*)g_idle_topstack;
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

  *heap_start = (FAR void*)USERSPACE->us_bssend;
  *heap_size  = ubase - (uintptr_t)USERSPACE->us_bssend;
}
#endif

/************************************************************************
 * Name: up_addregion
 *
 * Description:
 *   Memory may be added in non-contiguous chunks.  Additional chunks are
 *   added by calling this function.
 *
 ************************************************************************/

#if CONFIG_MM_REGIONS > 1
void up_addregion(void)
{
#ifdef HAVE_NFCSRAM_REGION
  /* Clocking may need to be applied to the SMC module in order for the
   * NFCS SRAM to be functional. 
   */

  sam_smc_enableclk();

  /* Allow user access to the heap memory */

  sam_mpu_uheap(SAM_NFCSRAM_BASE, SAMV7_NFCSRAM_SIZE);

  /* Add the region */

  kumm_addregion((FAR void*)SAM_NFCSRAM_BASE, SAMV7_NFCSRAM_SIZE);

#endif /* HAVE_NFCSRAM_REGION */

#ifdef HAVE_EXTSRAM0_REGION
  /* Allow user access to the heap memory */

  sam_mpu_uheap(SAM_EXTCS0_BASE, CONFIG_SAMV7_EXTSRAM0SIZE);

  /* Add the region */

  kumm_addregion((FAR void*)SAM_EXTCS0_BASE, CONFIG_SAMV7_EXTSRAM0SIZE);

#endif /* HAVE_EXTSRAM0_REGION */

#ifdef HAVE_EXTSRAM1_REGION
  /* Allow user access to the heap memory */

  sam_mpu_uheap(SAM_EXTCS1_BASE, CONFIG_SAMV7_EXTSRAM1SIZE);

  /* Add the region */

  kumm_addregion((FAR void*)SAM_EXTCS1_BASE, CONFIG_SAMV7_EXTSRAM1SIZE);

#endif /* HAVE_EXTSRAM0_REGION */

#ifdef HAVE_EXTSRAM2_REGION
  /* Allow user access to the heap memory */

  sam_mpu_uheap(SAM_EXTCS2_BASE, CONFIG_SAMV7_EXTSRAM2SIZE);

  /* Add the region */

  kumm_addregion((FAR void*)SAM_EXTCS2_BASE, CONFIG_SAMV7_EXTSRAM2SIZE);

#endif /* HAVE_EXTSRAM0_REGION */

#ifdef HAVE_EXTSRAM3_REGION
  /* Allow user access to the heap memory */

  sam_mpu_uheap(SAM_EXTCS3_BASE, CONFIG_SAMV7_EXTSRAM3SIZE);

  /* Add the region */

  kumm_addregion((FAR void*)SAM_EXTCS3_BASE, CONFIG_SAMV7_EXTSRAM3SIZE);

#endif /* HAVE_EXTSRAM0_REGION */
}
#endif /* CONFIG_MM_REGIONS > 1 */
