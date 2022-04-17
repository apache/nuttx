/****************************************************************************
 * arch/arm/src/sama5/sam_allocateheap.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "mmu.h"

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

#if defined(CONFIG_BUILD_KERNEL)
#  define MM_ADDREGION kmm_addregion
#else
#  define MM_ADDREGION umm_addregion
#endif

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
  /* Otherwise, add the RAM all the way to the end of the primary memory
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
 * up_allocate_heap is called.  So if the memory region is the primary
 * region, it should not be added to the heap (again).
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
 * Name: up_allocate_heap/up_allocate_kheap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *   For the flat build, this heap is referred to as the user heap (for
 *   compatibility with other platforms).  For the kernel build
 *   (CONFIG_BUILD_KERNEL=y) this is referred to a the kernel heap.
 *
 ****************************************************************************/

#if defined(CONFIG_BUILD_KERNEL)
void up_allocate_kheap(void **heap_start, size_t *heap_size)
#else
void up_allocate_heap(void **heap_start, size_t *heap_size)
#endif
{
#if defined(CONFIG_BOOT_SDRAM_DATA)
  /* In this case, the IDLE stack is in ISRAM, but data is in SDRAM.  The
   * heap is at the end of BSS through the configured end of SDRAM.
   */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (void *)&_ebss;
  *heap_size  = SAMA5_PRIMARY_HEAP_END - (size_t)&_ebss;

#else
  /* Both data and the heap are in ISRAM.  The heap is then from the end of
   * IDLE stack through the configured end of ISRAM.
   */

  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (void *)g_idle_topstack;
  *heap_size  = SAMA5_PRIMARY_HEAP_END - g_idle_topstack;
#endif
}

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
  int nregions = CONFIG_MM_REGIONS - 1;
  uintptr_t vaddr;
  size_t size;

#ifdef CONFIG_SAMA5_ISRAM_HEAP
  vaddr = (uintptr_t)SAM_ISRAM0_VADDR
  size  = SAM_ISRAM0_SIZE + SAM_ISRAM1_SIZE;

  /* Add the ISRAM user heap region. */

  MM_ADDREGION((void *)vaddr, size);
  nregions--;
#endif

#ifdef CONFIG_SAMA5_DDRCS_HEAP
  if (nregions > 0)
    {
      vaddr = (uintptr_t)SAM_DDRCS_VSECTION + SAMA5_DDRCS_HEAP_OFFSET;
      size  = SAMA5_DDRCS_HEAP_SIZE;

      /* Add the DDR-SDRAM user heap region. */

      MM_ADDREGION((void *)vaddr, size);
      nregions--;
    }
  else
    {
      serr("ERROR: SDRAM memory not added to heap.  CONFIG_MM_NREGIONS=%d\n",
           CONFIG_MM_REGIONS);
      serr("       Increase the size of CONFIG_MM_NREGIONS\n");
    }
#endif

#ifdef SAMA5_EBICS0_HEAP
  if (nregions > 0)
    {
      vaddr = (uintptr_t)SAM_EBICS0_VSECTION + SAMA5_EBICS0_HEAP_OFFSET;
      size  = SAMA5_EBICS0_HEAP_SIZE;

      /* Add the EBICS0 user heap region. */

      MM_ADDREGION((void *)vaddr, size);
      nregions--;
    }
  else
    {
      serr("ERROR: CS0 memory not added to heap.  CONFIG_MM_NREGIONS=%d\n",
           CONFIG_MM_REGIONS);
      serr("       Increase the size of CONFIG_MM_NREGIONS\n");
    }
#endif

#ifdef SAMA5_EBICS1_HEAP
  if (nregions > 0)
    {
      vaddr = (uintptr_t)SAM_EBICS1_VSECTION + SAMA5_EBICS1_HEAP_OFFSET;
      size  = SAMA5_EBICS1_HEAP_SIZE;

      /* Add the EBICS1 user heap region. */

      MM_ADDREGION((void *)vaddr, size);
      nregions--;
    }
  else
    {
      serr("ERROR: CS1 memory not added to heap.  CONFIG_MM_NREGIONS=%d\n",
           CONFIG_MM_REGIONS);
      serr("       Increase the size of CONFIG_MM_NREGIONS\n");
    }
#endif

#ifdef SAMA5_EBICS2_HEAP
  if (nregions > 0)
    {
      vaddr = (uintptr_t)SAM_EBICS2_VSECTION + SAMA5_EBICS2_HEAP_OFFSET;
      size  = SAMA5_EBICS2_HEAP_SIZE;

      /* Add the EBICS2 user heap region. */

      MM_ADDREGION((void *)vaddr, size);
      nregions--;
    }
  else
    {
      serr("ERROR: CS2 memory not added to heap.  CONFIG_MM_NREGIONS=%d\n",
           CONFIG_MM_REGIONS);
      serr("       Increase the size of CONFIG_MM_NREGIONS\n");
    }
#endif

#ifdef SAMA5_EBICS3_HEAP
  if (nregions > 0)
    {
      vaddr = (uintptr_t)SAM_EBICS3_VSECTION + SAMA5_EBICS3_HEAP_OFFSET;
      size  = SAMA5_EBICS3_HEAP_SIZE;

      /* Add the EBICS3 user heap region. */

      MM_ADDREGION(vaddr, size);
      nregions--;
    }
  else
    {
      serr("ERROR: CS3 memory not added to heap.  CONFIG_MM_NREGIONS=%d\n",
           CONFIG_MM_REGIONS);
      serr("       Increase the size of CONFIG_MM_NREGIONS\n");
    }
#endif

  /* Did we add all of the requestion regions */

  if (nregions > 0)
    {
      serr("ERROR: Not all regions added to heap: %d added,",
           CONFIG_MM_REGIONS - nregions);
      serr(" but CONFIG_MM_NREGIONS=%d\n", CONFIG_MM_REGIONS);
      serr("       Decrease the size of CONFIG_MM_NREGIONS\n");
    }
}
#endif
