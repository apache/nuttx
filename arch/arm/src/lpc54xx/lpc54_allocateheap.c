/****************************************************************************
 * arch/arm/src/lpc54xx/lpx54_allocateheap.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include "arm_arch.h"
#include "arm_internal.h"

#include "hardware/lpc54_memorymap.h"
#include "lpc54_mpuinit.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Terminology.  In the flat build (CONFIG_BUILD_FLAT=y), there is only a
 * single heap access with the standard allocations (malloc/free).  This
 * heap is referred to as the user heap.  In the protected build
 * (CONFIG_BUILD_PROTECTED=y) where an MPU is used to protect a region of
 * otherwise flat memory, there will be two allocators:  One that allocates
 * protected (kernel) memory and one that allocates unprotected (user)
 * memory.  These are referred to as the kernel and user heaps,
 * respectively.
 *
 * The ARMv7 has no MPU but does have an MMU.  With this MMU, it can support
 * the kernel build (CONFIG_BUILD_KERNEL=y).  In this configuration, there
 * is one kernel heap but multiple user heaps:  One per task group.  However,
 * in this case, we need only be concerned about initializing the single
 * kernel heap here.
 */

/* .bss and .data is always positioned in internal SRAM.  The remaining SRAM
 * after the static .bss, .data, and IDLE stack allocations are always added
 * to the heap.
 *
 * If the EMC is enabled, if there is SRAM or SDRAM configured into the
 * AND if the request heap size is non-zero, then that external RAM will
 * also be added to the system according to the following definitions:
 */

#undef HAVE_STATIC_CS0
#undef HAVE_STATIC_CS1
#undef HAVE_STATIC_CS2
#undef HAVE_STATIC_CS3

#undef HAVE_DYNAMIC_CS0
#undef HAVE_DYNAMIC_CS1
#undef HAVE_DYNAMIC_CS2
#undef HAVE_DYNAMIC_CS3

#ifdef CONFIG_LPC54_EMC
#  ifdef CONFIG_LPC54_EMC_STATIC
#    if defined(CONFIG_LPC54_EMC_STATIC_CS0) && CONFIG_LPC54_EMC_STATIC_CS0_SIZE > 0
#      define HAVE_STATIC_CS0  1
#    endif
#    if defined(CONFIG_LPC54_EMC_STATIC_CS1) && CONFIG_LPC54_EMC_STATIC_CS1_SIZE > 0
#      define HAVE_STATIC_CS1  1
#    endif
#    if defined(CONFIG_LPC54_EMC_STATIC_CS2) && CONFIG_LPC54_EMC_STATIC_CS2_SIZE > 0
#      define HAVE_STATIC_CS2  1
#    endif
#    if defined(CONFIG_LPC54_EMC_STATIC_CS3) && CONFIG_LPC54_EMC_STATIC_CS3_SIZE > 0
#      define HAVE_STATIC_CS3  1
#    endif
#  endif /* CONFIG_LPC54_EMC_STATIC */
#  ifdef CONFIG_LPC54_EMC_DYNAMIC
#    if defined(CONFIG_LPC54_EMC_DYNAMIC_CS0) && CONFIG_LPC54_EMC_DYNAMIC_CS0_SIZE > 0
#      define HAVE_DYNAMIC_CS0 1
#    endif
#    if defined(CONFIG_LPC54_EMC_DYNAMIC_CS1) && CONFIG_LPC54_EMC_DYNAMIC_CS1_SIZE > 0
#      define HAVE_DYNAMIC_CS1 1
#    endif
#    if defined(CONFIG_LPC54_EMC_DYNAMIC_CS2) && CONFIG_LPC54_EMC_DYNAMIC_CS2_SIZE > 0
#      define HAVE_DYNAMIC_CS2 1
#    endif
#    if defined(CONFIG_LPC54_EMC_DYNAMIC_CS3) && CONFIG_LPC54_EMC_DYNAMIC_CS3_SIZE > 0
#      define HAVE_DYNAMIC_CS3 1
#    endif
#  endif /* CONFIG_LPC54_EMC_DYNAMIC */
#endif /* CONFIG_LPC54_EMC */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* _sbss is the start of the BSS region (see the linker script) _ebss is the
 * end of the BSS region (see the linker script). The idle task stack starts
 * at the end of BSS and is of size CONFIG_IDLETHREAD_STACKSIZE.  The IDLE
 * thread is the thread that the system boots on and, eventually, becomes the
 * idle, do nothing task that runs only when there is nothing else to run.
 * The heap continues from there until the configured end of memory.
 * g_idle_topstack is the beginning of this heap region (not necessarily
 * aligned).
 */

const uint32_t g_idle_topstack = (uint32_t)&_ebss + CONFIG_IDLETHREAD_STACKSIZE;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap/up_allocate_kheap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   - For the normal "flat" build, this function returns the size of the
 *     single heap.
 *   - For the protected build (CONFIG_BUILD_PROTECTED=y) with both kernel-
 *     and user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function
 *     provides the size of the unprotected, user-space heap.
 *   - For the kernel build (CONFIG_BUILD_KERNEL=y), this function provides
 *     the size of the protected, kernel-space heap.
 *
 *   If a protected kernel-space heap is provided, the kernel heap must be
 *   allocated by an analogous up_allocate_kheap(). A custom version of this
 *   file is needed if memory protection of the kernel heap is required.
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

#ifdef CONFIG_BUILD_KERNEL
void up_allocate_kheap(FAR void **heap_start, size_t *heap_size)
#else
void up_allocate_heap(FAR void **heap_start, size_t *heap_size)
#endif
{
#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend + CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = CONFIG_RAM_END - ubase;

  DEBUGASSERT(ubase < (uintptr_t)CONFIG_RAM_END);

  /* Return the user-space heap settings */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void *)ubase;
  *heap_size  = usize;
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
 *   For the kernel build (CONFIG_BUILD_PROTECTED/KERNEL=y) with both kernel-
 *   and user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function allocates
 *   the kernel-space heap.  A custom version of this function is needed if
 *   memory protection of the kernel heap is required.
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
  DEBUGASSERT(ubase < (uintptr_t)CONFIG_RAM_END);

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
  int remaining = CONFIG_MM_REGIONS;
  FAR void *heapstart;
  size_t heapsize;

#ifdef HAVE_STATIC_CS0
  if (remaining > 0)
    {
      /* Add the SRAM to the user heap */

      heapstart = (FAR void *)(LPC54_SRAMCS0_BASE + CONFIG_LPC54_EMC_STATIC_CS0_OFFSET);
      heapsize  = CONFIG_LPC54_EMC_STATIC_CS0_SIZE;
      kumm_addregion(heapstart, heapsize);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the SDRAM heap */

      lpc54_mpu_uheap((uintptr_t)heapstart, heapsize);
#endif
      remaining--;
    }
#endif /* HAVE_STATIC_CS0 */

#ifdef HAVE_STATIC_CS1
  if (remaining > 0)
    {
      /* Add the SRAM to the user heap */

      heapstart = (FAR void *)(LPC54_SRAMCS1_BASE + CONFIG_LPC54_EMC_STATIC_CS1_OFFSET);
      heapsize  = CONFIG_LPC54_EMC_STATIC_CS1_SIZE;
      kumm_addregion(heapstart, heapsize);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the SDRAM heap */

      lpc54_mpu_uheap((uintptr_t)heapstart, heapsize);
#endif
      remaining--;
    }
#endif /* HAVE_STATIC_CS1 */

#ifdef HAVE_STATIC_CS2
  if (remaining > 0)
    {
      /* Add the SRAM to the user heap */

      heapstart = (FAR void *)(LPC54_SRAMCS2_BASE + CONFIG_LPC54_EMC_STATIC_CS2_OFFSET);
      heapsize  = CONFIG_LPC54_EMC_STATIC_CS2_SIZE;
      kumm_addregion(heapstart, heapsize);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the SDRAM heap */

      lpc54_mpu_uheap((uintptr_t)heapstart, heapsize);
#endif
      remaining--;
    }
#endif /* HAVE_STATIC_CS2 */

#ifdef HAVE_STATIC_CS3
  if (remaining > 0)
    {
      /* Add the SRAM to the user heap */

      heapstart = (FAR void *)(LPC54_SRAMCS3_BASE + CONFIG_LPC54_EMC_STATIC_CS3_OFFSET);
      heapsize  = CONFIG_LPC54_EMC_STATIC_CS3_SIZE;
      kumm_addregion(heapstart, heapsize);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the SDRAM heap */

      lpc54_mpu_uheap((uintptr_t)heapstart, heapsize);
#endif
      remaining--;
    }
#endif /* HAVE_STATIC_CS3 */

#ifdef HAVE_DYNAMIC_CS0
  if (remaining > 0)
    {
      /* Add the SDRAM to the user heap */

      heapstart = (FAR void *)(LPC54_DRAMCS0_BASE + CONFIG_LPC54_EMC_DYNAMIC_CS0_OFFSET);
      heapsize  = CONFIG_LPC54_EMC_DYNAMIC_CS0_SIZE;
      kumm_addregion(heapstart, heapsize);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the SDRAM heap */

      lpc54_mpu_uheap((uintptr_t)heapstart, heapsize);
#endif
      remaining--;
    }
#endif /* HAVE_DYNAMIC_CS0 */

#ifdef HAVE_DYNAMIC_CS1
  if (remaining > 0)
    {
      /* Add the SDRAM to the user heap */

      heapstart = (FAR void *)(LPC54_DRAMCS1_BASE + CONFIG_LPC54_EMC_DYNAMIC_CS1_OFFSET);
      heapsize  = CONFIG_LPC54_EMC_DYNAMIC_CS1_SIZE;
      kumm_addregion(heapstart, heapsize);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the SDRAM heap */

      lpc54_mpu_uheap((uintptr_t)heapstart, heapsize);
#endif
      remaining--;
    }
#endif /* HAVE_DYNAMIC_CS1 */

#ifdef HAVE_DYNAMIC_CS2
  if (remaining > 0)
    {
      /* Add the SDRAM to the user heap */

      heapstart = (FAR void *)(LPC54_DRAMCS2_BASE + CONFIG_LPC54_EMC_DYNAMIC_CS2_OFFSET);
      heapsize  = CONFIG_LPC54_EMC_DYNAMIC_CS2_SIZE;
      kumm_addregion(heapstart, heapsize);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the SDRAM heap */

      lpc54_mpu_uheap((uintptr_t)heapstart, heapsize);
#endif
      remaining--;
    }
#endif /* HAVE_DYNAMIC_CS2 */

#ifdef HAVE_DYNAMIC_CS3
  if (remaining > 0)
    {
      /* Add the SDRAM to the user heap */

      heapstart = (FAR void *)(LPC54_DRAMCS3_BASE + CONFIG_LPC54_EMC_DYNAMIC_CS3_OFFSET);
      heapsize  = CONFIG_LPC54_EMC_DYNAMIC_CS3_SIZE;
      kumm_addregion(heapstart, heapsize);

#if defined(CONFIG_BUILD_PROTECTED) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the SDRAM heap */

      lpc54_mpu_uheap((uintptr_t)heapstart, heapsize);
#endif
      remaining--;
    }
#endif /* HAVE_DYNAMIC_CS3 */
}
#endif /* CONFIG_MM_REGIONS > 1 */
