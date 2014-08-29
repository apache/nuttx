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
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "mmu.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/
/* The Primary Heap *********************************************************/
/* The physical address of the primary heap is defined by CONFIG_RAM_START,
 * CONFIG_RAM_SIZE, and CONFIG_RAM_END where:
 *
 *   CONFIG_RAM_END = CONFIG_RAM_START + CONFIG_RAM_SIZE
 *
 * and the corresponding virtual address are give by:
 *
 *   CONFIG_RAM_VEND = CONFIG_RAM_VSTART + CONFIG_RAM_SIZE
 *
 * CONFIG_RAM_VSTART is the usable beginning of the RAM region.  The "usable"
 * start would exclude, for example, any memory at the bottom of the RAM
 * region used for the 16KB page table.  If we are also executing from this
 * same RAM region then CONFIG_RAM_START is not used.  Instead, the value of
 * g_idle_stack is the used; this variable holds the first available byte of
 * memory after the .text, .data, .bss, and IDLE stack allocations.
 *
 * CONFIG_RAM_VEND is defined in the configuration it is the usable top of
 * the RAM region beginning at CONFIG_RAM_START.  The "usable" top would
 * exclude, for example, any memory reserved at the top of the for the 16KB
 * page table.
 *
 * A special may occur when we execute from DRAM.  In that case,
 * CONFIG_RAM_VSTART must be set to the (virtual) start of DRAM and
 * CONFIG_RAM_SIZE must be set to the size of the DRAM.  These settings are
 * necessary to provide the DRAM MMU mappings when the system boots and, by
 * default, the DRAM memory will be added to the heap all the way up to
 * CONFIG_RAM_VEND
 *
 * However, there are certain cases where you may want to reserve a block of
 * DRAM for other purposes such a large DMA buffer or an LCD framebuffer.
 * In those cases, the CONFIG_SAMA5_DDRCS_RESERVE can select a different end
 * of the DRAM memory to add to the heap.  If CONFIG_SAMA5_DDRCS_RESERVE is
 * selected, then the setting CONFIG_SAMA5_DDRCS_HEAP_END provides the end
 * (plus one) (virtual) address of memory to be added to the heap; DRAM after
 * this address will not be part of the heap and so will be available for
 * other purposes
 *
 *   NOTE: There is way to reserve memory before the start of the program
 *   in DRAM using this mechanism.  That configuration is possible, but
 *   not using this configuration setting.
 */

/* Memory Regions ***********************************************************/
/* Check if we have been asked to reserve memory at the end of the primary
 * memory region.  This option is only available if we are executing from
 * DRAM and only if CONFIG_SAMA5_DDRCS_RESERVE is selected.
 */

#ifdef CONFIG_SAMA5_DDRCS_RESERVE

  /* CONFIG_SAMA5_DDRCS_HEAP_END specifies the end (plus one) of the DRAM
   * to add to the heap.  Memory starting at CONFIG_SAMA5_DDRCS_HEAP_END
   * and extending to CONFIG_RAM_VEND is then available for other purposes.
   */

#  if !defined(CONFIG_SAMA5_DDRCS_HEAP_END)
#    error CONFIG_SAMA5_DDRCS_HEAP_END must be defined in this configuration
#  elif CONFIG_SAMA5_DDRCS_HEAP_END > CONFIG_RAM_VEND
#    error CONFIG_SAMA5_DDRCS_HEAP_END is beyond CONFIG_RAM_VEND
#  elif CONFIG_SAMA5_DDRCS_HEAP_END < CONFIG_RAM_VSTART
#    error CONFIG_SAMA5_DDRCS_HEAP_END is before CONFIG_RAM_VSTART
# endif

#  define SAMA5_PRIMARY_HEAP_END CONFIG_SAMA5_DDRCS_HEAP_END
#else
  /* Otherwise, add the RAM all the way to the the end of the primary memory
   * region to the heap.
   */

#  define SAMA5_PRIMARY_HEAP_END CONFIG_RAM_VEND
#endif

/* We cannot use the memory for heap if it is not enabled.  Or, if it is
 * enabled, but does not hold SDRAM, SRAM, or PSRAM.
 *
 * We cannot add the region if it is if we are executing from it!  In that
 * case, the remainder of the memory will automatically be added to the heap
 * based on g_idle_topstack and SAMA5_PRIMARY_HEAP_END.
 */

#if defined(CONFIG_SAMA5_BOOT_ISRAM)
#  undef CONFIG_SAMA5_ISRAM_HEAP
#endif

#if !defined(CONFIG_SAMA5_DDRCS) || defined(CONFIG_SAMA5_BOOT_SDRAM) || \
    defined(CONFIG_BOOT_SDRAM_DATA)
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

/* The heap space in the primary memory region is added automatically when
 * up_allocate heap is called.  So if the memory region is the primary region,
 * it should not be added to the heap (again).
 */

#if (CONFIG_RAM_VSTART & 0xfff00000) == SAM_ISRAM0_VADDR
#  undef CONFIG_SAMA5_ISRAM_HEAP
#endif

#if (CONFIG_RAM_VSTART & 0xfff00000) == SAM_DDRCS_VSECTION
#  undef CONFIG_SAMA5_DDRCS_HEAP
#endif

#if (CONFIG_RAM_VSTART & 0xfff00000) == SAM_EBICS0_VSECTION
#  undef SAMA5_EBICS0_HEAP
#endif

#if (CONFIG_RAM_VSTART & 0xfff00000) == SAM_EBICS1_VSECTION
#  undef SAMA5_EBICS1_HEAP
#endif

#if (CONFIG_RAM_VSTART & 0xfff00000) == SAM_EBICS2_VSECTION
#  undef SAMA5_EBICS2_HEAP
#endif

#if (CONFIG_RAM_VSTART & 0xfff00000) == SAM_EBICS3_VSECTION
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
 *   For the kernel build (CONFIG_BUILD_KERNEL=y) with both kernel- and
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
#if defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend + CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = SAMA5_PRIMARY_HEAP_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)SAMA5_PRIMARY_HEAP_END);

  /* Return the user-space heap settings */

  board_led_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void*)ubase;
  *heap_size  = usize;

#elif defined(CONFIG_BOOT_SDRAM_DATA)
  /* In this case, the IDLE stack is in ISRAM, but data is in SDRAM.  The
   * heap is at the end of BSS through the configured end of SDRAM.
   */

  board_led_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void*)&_ebss;
  *heap_size  = SAMA5_PRIMARY_HEAP_END - (size_t)&_ebss;

#else
  /* Both data and the heap are in ISRAM.  The heap is then from the end of
   * IDLE stack through the configured end of ISRAM.
   */

  board_led_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void*)g_idle_topstack;
  *heap_size  = SAMA5_PRIMARY_HEAP_END - g_idle_topstack;
#endif
}

/****************************************************************************
 * Name: up_allocate_kheap
 *
 * Description:
 *   For the kernel build (CONFIG_BUILD_KERNEL=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function allocates
 *   the kernel-space heap.  A custom version of this function is need if
 *   memory protection of the kernel heap is required.
 *
 ****************************************************************************/

#if defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
void up_allocate_kheap(FAR void **heap_start, size_t *heap_size)
{
  /* Get the unaligned size and position of the user-space heap.
   * This heap begins after the user-space .bss section at an offset
   * of CONFIG_MM_KERNEL_HEAPSIZE (subject to alignment).
   */

  uintptr_t ubase = (uintptr_t)USERSPACE->us_bssend + CONFIG_MM_KERNEL_HEAPSIZE;
  size_t    usize = SAMA5_PRIMARY_HEAP_END - ubase;
  int       log2;

  DEBUGASSERT(ubase < (uintptr_t)SAMA5_PRIMARY_HEAP_END);

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
  int nregions = CONFIG_MM_REGIONS - 1;
  uintptr_t vaddr;
  size_t size;

#ifdef CONFIG_SAMA5_ISRAM_HEAP
  vaddr = (uintptr_t)SAM_ISRAM0_VADDR
  size  = SAM_ISRAM0_SIZE + SAM_ISRAM1_SIZE;

#if defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
  /* Allow user-mode access to the ISRAM heap */

  sam_uheap(vaddr, size);

#endif

  /* Add the ISRAM user heap region. */

  kumm_addregion((void *)vaddr, size);
  nregions--;
#endif

#ifdef CONFIG_SAMA5_DDRCS_HEAP
  if (nregions > 0)
    {
      vaddr = (uintptr_t)SAM_DDRCS_VSECTION + SAMA5_DDRCS_HEAP_OFFSET;
      size  = SAMA5_DDRCS_HEAP_SIZE;

#if defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the DDR-SDRAM heap */

      sam_uheap(vaddr, size);
#endif

      /* Add the DDR-SDRAM user heap region. */

      kumm_addregion((void *)vaddr, size);
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
      vaddr = (uintptr_t)SAM_EBICS0_VSECTION + SAMA5_EBICS0_HEAP_OFFSET;
      size  = SAMA5_EBICS0_HEAP_SIZE;

#if defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the EBICS0 heap */

      sam_uheap(vaddr, size);
#endif

      /* Add the EBICS0 user heap region. */

      kumm_addregion((void *)vaddr, size);
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
      vaddr = (uintptr_t)SAM_EBICS1_VSECTION + SAMA5_EBICS1_HEAP_OFFSET;
      size  = SAMA5_EBICS1_HEAP_SIZE;

#if defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the EBICS1 heap */

      sam_uheap(vaddr, size);
#endif

      /* Add the EBICS1 user heap region. */

      kumm_addregion((void *)vaddr, size);
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
      vaddr = (uintptr_t)SAM_EBICS2_VSECTION + SAMA5_EBICS2_HEAP_OFFSET;
      size  = SAMA5_EBICS2_HEAP_SIZE;

#if defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the EBICS2 heap */

      sam_uheap(vaddr, size);
#endif

      /* Add the EBICS2 user heap region. */

      kumm_addregion((void *)vaddr, size);
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
      vaddr = (uintptr_t)SAM_EBICS3_VSECTION + SAMA5_EBICS3_HEAP_OFFSET;
      size  = SAMA5_EBICS3_HEAP_SIZE;

#if defined(CONFIG_BUILD_KERNEL) && defined(CONFIG_MM_KERNEL_HEAP)
      /* Allow user-mode access to the EBICS3 heap */

      sam_uheap(vaddr, size);
#endif

      /* Add the EBICS3 user heap region. */

      kumm_addregion(vaddr, size);
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
