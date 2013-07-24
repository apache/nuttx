/****************************************************************************
 * arch/arm/src/sama5/sam_allocateheap.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
#include <nuttx/userspace.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/
/* We cannot use the memory for heap if it is not enabled.  Or, if it is
 * enabled, but does not hold SDRAM, SRAM, or PSRAM.
 *
 * We cannot add the region if it is if we are executing from it!  In that
 * case, the remainder of the memory will automatically be added to the heap
 * based on g_idle_topstack and CONFIG_DRAM_END
 */

#if defined(CONFIG_SAMA5_BOOT_ISRAM)
#  undef CONFIG_SAMA5_ISRAM_HEAP
#endif

#if !defined(CONFIG_SAMA5_DDRCS) || defined(CONFIG_SAMA5_BOOT_SDRAM)
#  undef CONFIG_SAMA5_DDRCS_HEAP
#endif

#if !defined(CONFIG_SAMA5_EBICS0) || defined(CONFIG_SAMA5_BOOT_CS0SRAM) || \
   (!defined(CONFIG_SAMA5_EBICS0_SRAM) && !defined(CONFIG_SAMA5_EBICS0_PSRAM))
   
#  undef SAMA5_EBICS0_HEAP
#endif

#if !defined(CONFIG_SAMA5_EBICS1) || defined(CONFIG_SAMA5_BOOT_CS1SRAM) || \
   (!defined(CONFIG_SAMA5_EBICS1_SRAM) && !defined(CONFIG_SAMA5_EBICS1_PSRAM))
   
#  undef SAMA5_EBICS1_HEAP
#endif

#if !defined(CONFIG_SAMA5_EBICS2) || defined(CONFIG_SAMA5_BOOT_CS2SRAM) || \
   (!defined(CONFIG_SAMA5_EBICS2_SRAM) && !defined(CONFIG_SAMA5_EBICS2_PSRAM))
   
#  undef SAMA5_EBICS2_HEAP
#endif

#if !defined(SAMA5_CONFIG_EBICS3) || defined(CONFIG_SAMA5_BOOT_CS3SRAM) || \
   (!defined(SAMA5_CONFIG_EBICS3_SRAM) && !defined(CONFIG_SAMA5_EBICS3_PSRAM))
   
#  undef SAMA5_EBICS3_HEAP
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
 *   For the kernel build (CONFIG_NUTTX_KERNEL=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function provides the
 *   size of the unprotected, user-space heap.
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

void up_allocate_heap(FAR void **heap_start, size_t *heap_size)
{
#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend + CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = CONFIG_DRAM_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)CONFIG_DRAM_END);

  /* Return the user-space heap settings */

  up_ledon(LED_HEAPALLOCATE);
  *heap_start = (FAR void*)ubase;
  *heap_size  = usize;
#else

  /* Return the heap settings */

  up_ledon(LED_HEAPALLOCATE);
  *heap_start = (FAR void*)g_idle_topstack;
  *heap_size  = CONFIG_DRAM_END - g_idle_topstack;
#endif
}

/****************************************************************************
 * Name: up_allocate_kheap
 *
 * Description:
 *   For the kernel build (CONFIG_NUTTX_KERNEL=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function allocates
 *   the kernel-space heap.  A custom version of this function is need if
 *   memory protection of the kernel heap is required.
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
  size_t    usize = CONFIG_DRAM_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)CONFIG_DRAM_END);

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
  int nregions = CONFIG_MM_REGIONS;
  size_t size;

#ifdef CONFIG_SAMA5_ISRAM_HEAP
#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)

  /* Allow user-mode access to the ISRAM heap */

  size = SAM_ISRAM0_SIZE + SAM_ISRAM1_SIZE;
  sam_uheap((uintptr_t)SAM_ISRAM0_VADDR, size);

#endif

  /* Add the ISRAM user heap region. */

  kumm_addregion((FAR void*)SAM_ISRAM0_VADDR, size);

  nregions--;
#endif

#ifdef CONFIG_SAMA5_DDRCS_HEAP
  if (nregions > 0)
    {
#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the ISRAM heap */

      size = CONFIG_SAMA5_DDRCS_SIZE;
      sam_uheap((uintptr_t)SAM_DDRCS_VSECTION, size);
#endif

      /* Add the ISRAM user heap region. */

      kumm_addregion((FAR void*)SAM_DDRCS_VSECTION, size);

      nregions--;
    }
  else
    {
      lldbg("ERROR: SDRAM memory not added to heap.  CONFIG_MM_NREGIONS=%d\n",
            CONFIG_MM_REGIONS);
      lldbg("       Increase the size of CONFIG_MM_NREGIONS\n");
    }
#endif

#ifdef SAMA5_EBICS0_HEAP
  if (nregions > 0)
    {
#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the ISRAM heap */

      size = CONFIG_SAMA5_EBICS0_SIZE;
      sam_uheap((uintptr_t)SAM_EBICS0_VSECTION, size);
#endif

      /* Add the ISRAM user heap region. */

      kumm_addregion((FAR void*)SAM_EBICS0_VSECTION, size);

      nregions--;
    }
  else
    {
      lldbg("ERROR: CS0 memory not added to heap.  CONFIG_MM_NREGIONS=%d\n",
            CONFIG_MM_REGIONS);
      lldbg("       Increase the size of CONFIG_MM_NREGIONS\n");
    }
#endif

#ifdef SAMA5_EBICS1_HEAP
  if (nregions > 0)
    {
#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the ISRAM heap */

      size = CONFIG_SAMA5_EBICS1_SIZE;
      sam_uheap((uintptr_t)SAM_EBICS1_VSECTION, size);
#endif

      /* Add the ISRAM user heap region. */

      kumm_addregion((FAR void*)SAM_EBICS1_VSECTION, size);

      nregions--;
    }
  else
    {
      lldbg("ERROR: CS1 memory not added to heap.  CONFIG_MM_NREGIONS=%d\n",
            CONFIG_MM_REGIONS);
      lldbg("       Increase the size of CONFIG_MM_NREGIONS\n");
    }
#endif

#ifdef SAMA5_EBICS2_HEAP
  if (nregions > 0)
    {
#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the ISRAM heap */

      size = CONFIG_SAMA5_EBICS2_SIZE;
      sam_uheap((uintptr_t)SAM_EBICS2_VSECTION, size);
#endif

      /* Add the ISRAM user heap region. */

      kumm_addregion((FAR void*)SAM_EBICS2_VSECTION, size);

      nregions--;
    }
  else
    {
      lldbg("ERROR: CS2 memory not added to heap.  CONFIG_MM_NREGIONS=%d\n",
            CONFIG_MM_REGIONS);
      lldbg("       Increase the size of CONFIG_MM_NREGIONS\n");
    }
#endif

#ifdef SAMA5_EBICS3_HEAP
  if (nregions > 0)
    {
#if defined(CONFIG_NUTTX_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the ISRAM heap */

      size = CONFIG_SAMA5_EBICS3_SIZE;
      sam_uheap((uintptr_t)SAM_EBICS3_VSECTION, size);
#endif

      /* Add the ISRAM user heap region. */

      kumm_addregion((FAR void*)SAM_EBICS3_VSECTION, size);

      nregions--;
    }
  else
    {
      lldbg("ERROR: CS3 memory not added to heap.  CONFIG_MM_NREGIONS=%d\n",
            CONFIG_MM_REGIONS);
      lldbg("       Increase the size of CONFIG_MM_NREGIONS\n");
    }
#endif

  /* Did we add all of the requestion regions */

  if (nregions > 0)
    {
      lldbg("ERROR: Not all regions added to heap: %d added, but CONFIG_MM_NREGIONS=%d\n",
            CONFIG_MM_REGIONS - nregions, CONFIG_MM_REGIONS);
      lldbg("       Decrease the size of CONFIG_MM_NREGIONS\n");
    }
}
#endif
