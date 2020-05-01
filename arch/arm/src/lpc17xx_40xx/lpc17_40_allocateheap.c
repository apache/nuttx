/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc17_40_allocateheap.c
 *
 *   Copyright (C) 2010-2011, 2013, 2015 Gregory Nutt. All rights reserved.
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

#include "chip.h"
#include "mpu.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include "hardware/lpc17_40_memorymap.h"
#include "lpc17_40_emacram.h"
#include "lpc17_40_ohciram.h"
#include "lpc17_40_mpuinit.h"
#include "lpc17_40_start.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* The configured RAM start address must be the beginning of CPU SRAM */

#if CONFIG_RAM_START != LPC17_40_SRAM_BASE
#  warning "CONFIG_RAM_START is not at LPC17_40_SRAM_BASE"
#  undef CONFIG_RAM_START
#  undef CONFIG_RAM_END
#  define CONFIG_RAM_START LPC17_40_SRAM_BASE
#  define CONFIG_RAM_END  (LPC17_40_SRAM_BASE+LPC17_40_CPUSRAM_SIZE)
#endif

/* The configured RAM size must be less then or equal to the CPU SRAM size */

#if CONFIG_RAM_SIZE > LPC17_40_CPUSRAM_SIZE
#  warning "CONFIG_RAM_SIZE is larger than the size of CPU SRAM"
#  undef CONFIG_RAM_SIZE
#  undef CONFIG_RAM_END
#  define CONFIG_RAM_SIZE LPC17_40_CPUSRAM_SIZE
#  define CONFIG_RAM_END (LPC17_40_SRAM_BASE+LPC17_40_CPUSRAM_SIZE)
#elif CONFIG_RAM_SIZE < LPC17_40_CPUSRAM_SIZE
#  warning "CONFIG_RAM_END is before end of CPU SRAM... not all of CPU SRAM used"
#endif

/* Figure out how much heap be have in AHB SRAM (if any).  Complications:
 * 1) AHB SRAM Bank 0 or 1 may on may not be supported in the hardware.
 * 2) Some or all of Bank 0 may be used for Ethernet Packet buffering.
 *    Ethernet packet buffering is used from the beginning of Bank 0 and
 *    any free memory at the end of Bank 0 will be contiguous with any
 *    free memory at the beginning of Bank 1.
 * 3) Some or all of Bank 1 may be used for OHCI descriptor memory.  OCHI
 *    memory is used from the end of Bank 1 and any free memory at the
 *    beginning of Bank 1 will be contiguous with any free memory at the
 *    end of Bank 0.
 */

#undef LPC17_40_AHB_HEAPBASE /* Assume that nothing is available */
#undef LPC17_40_AHB_HEAPSIZE

/* If we have Bank 0, then we may possibly also have Bank 1 */

#ifdef LPC17_40_HAVE_BANK0

  /* We have BANK0 (and, hence, possibly Bank1).  Is Bank0 all used for
   * Ethernet packet buffering?  Or is there any part of Bank0 available for
   * the heap.
   */

#  ifdef LPC17_40_BANK0_HEAPSIZE

     /* Some or all of Bank0 is available for the heap.  The heap will begin
      * in bank 1.
      */

#     define LPC17_40_AHB_HEAPBASE LPC17_40_BANK0_HEAPBASE

     /* Is Bank1 present? Has there available heap memory in Bank 1? */

#    if defined(LPC17_40_HAVE_BANK1) && defined(LPC17_40_BANK1_HEAPSIZE)

       /* Yes... the heap space available is the unused memory at the end
        * of Bank0 plus the unused memory at the beginning of Bank 1.
        */

#       define LPC17_40_AHB_HEAPSIZE (LPC17_40_BANK0_HEAPSIZE + LPC17_40_BANK1_HEAPSIZE)
#    else

        /* No... the heap space available is only the unused memory at the
         * end of Bank 0.
         */

#       define LPC17_40_AHB_HEAPSIZE LPC17_40_BANK0_HEAPSIZE

#    endif /* LPC17_40_HAVE_BANK1 && LPC17_40_BANK1_HEAPSIZE */
#  else /* !LPC17_40_BANK0_HEAPSIZE */

     /* We have Bank 0, but no memory is available for the heap there.
      * Do we have Bank 1? Is any heap memory available in Bank 1?
      */

#    if defined(LPC17_40_HAVE_BANK1) && defined(LPC17_40_BANK1_HEAPSIZE)

       /* Yes... the heap space available is the unused memory at the
        * beginning of Bank1.
        */

#      define LPC17_40_AHB_HEAPBASE LPC17_40_BANK1_HEAPBASE
#      define LPC17_40_AHB_HEAPSIZE LPC17_40_BANK1_HEAPSIZE

#    endif /* LPC17_40_HAVE_BANK1 && LPC17_40_BANK1_HEAPSIZE */
#  endif /* LPC17_40_BANK0_HEAPSIZE */
#endif /* LPC17_40_HAVE_BANK0 */

/* Sanity checking */

#if !defined(CONFIG_LPC17_40_EXTDRAMHEAP) && !defined(CONFIG_LPC17_40_EXTSRAM0HEAP)
#  define LPC17_40_EXT_MM_REGIONS 0
#elif defined(CONFIG_LPC17_40_EXTDRAMHEAP) && defined(CONFIG_LPC17_40_EXTSRAM0HEAP)
#  define LPC17_40_EXT_MM_REGIONS 2
#else
#  define LPC17_40_EXT_MM_REGIONS 1
#endif

#ifdef LPC17_40_AHB_HEAPBASE
#  if CONFIG_MM_REGIONS < 2 + LPC17_40_EXT_MM_REGIONS
#    warning "CONFIG_MM_REGIONS < 2: Available AHB SRAM Bank(s) not included in HEAP"
#  endif
#  if (CONFIG_MM_REGIONS > 2 + LPC17_40_EXT_MM_REGIONS)
#    warning "CONFIG_MM_REGIONS > 2: Are additional regions handled by application?"
#  endif
#else
#  if CONFIG_MM_REGIONS > 1 + LPC17_40_EXT_MM_REGIONS
#    warning "CONFIG_MM_REGIONS > 1: This configuration has no available AHB SRAM Bank0/1"
#    warning "CONFIG_MM_REGIONS > 1: Are additional regions handled by application?"
#  endif
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

   lpc17_40_mpu_uheap((uintptr_t)ubase, usize);

#elif CONFIG_MM_REGIONS >= 3 && defined(CONFIG_LPC17_40_EXTDRAM) && \
    defined(CONFIG_LPC17_40_EXTDRAMHEAP)
  /* We are going to allocate a DRAM heap.  In the case where a bootloader
   * is used (and has initialized SDRAM), it is possible that .data, .bss,
   * and the IDLE stack reside in SDRAM.
   */

    uintptr_t sram_start = CONFIG_RAM_START;

    /* Is SRAM the primary RAM? I.e., do .data, .bss, and the IDLE thread
     * stack lie in SRAM?
     */

    if (g_idle_topstack >= CONFIG_RAM_START &&
        g_idle_topstack < CONFIG_RAM_END)
      {
        /* Yes, then the SRAM heap starts in SRAM after the IDLE stack */

        sram_start = g_idle_topstack;
      }
    else
      {
        /* Use the entire SRAM for heap */

        sram_start = CONFIG_RAM_START;
      }

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void *)sram_start;
  *heap_size  = CONFIG_RAM_END - sram_start;

#else

  /* Return the heap settings (assuming that .data, .bss, and the IDLE
   * stack lie in SRAM).
   */

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
 *   REVISIT:  This does not account for the possibility that the kernel
 *   heap might lie in SDRAM.
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
  /* Banks 0 and 1 are each 16Kb.  If both are present, they occupy a
   * contiguous 32Kb memory region.
   *
   * If Ethernet is enabled, it will take some or all of bank 0 for packet
   * buffering and descriptor tables; If USB host is enabled, it will take
   * some or all of bank 1 for descriptor memory.  The complex conditional
   * compilation above should boil this all down to a very simple check
   * here:
   *
   * Is any memory available in AHB SRAM for the heap?
   */

#ifdef LPC17_40_AHB_HEAPBASE
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Yes.. allow user-mode access to the AHB SRAM user heap memory */

#if defined(LPC17_40_BANK0_HEAPBASE) && defined(LPC17_40_BANK0_HEAPSIZE)
  lpc17_40_mpu_uheap((uintptr_t)LPC17_40_BANK0_HEAPBASE, LPC17_40_BANK0_HEAPSIZE);
#endif

#if defined(LPC17_40_BANK1_HEAPBASE) && defined(LPC17_40_BANK1_HEAPSIZE)
  lpc17_40_mpu_uheap((uintptr_t)LPC17_40_BANK1_HEAPBASE, LPC17_40_BANK1_HEAPSIZE);
#endif

#endif /* CONFIG_BUILD_PROTECTED && CONFIG_MM_KERNEL_HEAP */

  /* Add the AHB SRAM user heap region. */

  kumm_addregion((FAR void *)LPC17_40_AHB_HEAPBASE, LPC17_40_AHB_HEAPSIZE);
#endif

#if CONFIG_MM_REGIONS >= 3
#if defined(CONFIG_LPC17_40_EXTDRAM) && defined(CONFIG_LPC17_40_EXTDRAMHEAP)
  {
    /* Memory may be reserved at the beginning of DRAM for other purposes
     * (for example for video framebuffers).  Memory can similar be
     * reserved at the end of DRAM using LPC17_40_EXTDRAMSIZE.  The amount to
     * be added to the heap will be from DRAM_BASE + LPC17_40_EXTDRAMHEAP_OFFSET
     * through DRAM_BASE + LPC17_40_EXTDRAMSIZE where (DRAM_BASE is the base
     * address of CS0).
     */

    uintptr_t dram_end = LPC17_40_EXTDRAM_CS0 + CONFIG_LPC17_40_EXTDRAMSIZE;
    uintptr_t dram_start;
    uintptr_t heap_size;

    /* Is SDRAM the primary RAM? I.e., do .data, .bss, and the IDLE thread
     * stack lie in SDRAM?
     */

    if (g_idle_topstack >= LPC17_40_EXTDRAM_CS0 &&
        g_idle_topstack < dram_end)
      {
        /* Yes, then the SDRAM heap starts in SDRAM after the IDLE stack */

        dram_start = g_idle_topstack;
      }
    else
      {
        /* Use the entire SDRAM for heap (possible reserving a portion at
         * the beginning of DRAM).
         */

        dram_start = LPC17_40_EXTDRAM_CS0 + CONFIG_LPC17_40_EXTDRAMHEAP_OFFSET;
      }

    heap_size = dram_end - dram_start;

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
    /* Allow user-mode access to the external DRAM heap memory.
     *
     * REVISIT:  In PROTECTED mode, it would be necessary to align
     * dram_start to an address that is easily spanned by an MPU
     * region if dram_start is not at the beginning of CS0.
     */

    lpc17_40_mpu_uheap(dram_start, heap_size);
#endif

    /* Add external DRAM heap memory to the user heap */

    kumm_addregion((FAR void *)dram_start, heap_size);
  }
#endif

#if !defined(CONFIG_LPC17_40_EXTDRAMHEAP) || (CONFIG_MM_REGIONS >= 4)
#if defined(CONFIG_LPC17_40_EXTSRAM0) && defined(CONFIG_LPC17_40_EXTSRAM0HEAP)
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to external SRAM heap memory */

  lpc17_40_mpu_uheap((uintptr_t)LPC17_40_EXTSRAM_CS0, CONFIG_LPC17_40_EXTSRAM0SIZE);

#endif
  /* Add external SRAM heap memory to the user heap */

  kumm_addregion((FAR void *)LPC17_40_EXTSRAM_CS0, CONFIG_LPC17_40_EXTSRAM0SIZE);
#endif
#endif
#endif /* CONFIG_MM_REGIONS >= 3 */
}
#endif
