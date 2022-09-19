/****************************************************************************
 * arch/arm/src/am335x/am335x_boot.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/cache.h>
#ifdef CONFIG_PAGING
#  include <nuttx/page.h>
#endif

#include <arch/board/board.h>

#include "chip.h"
#include "arm.h"
#include "mmu.h"
#include "arm_internal.h"
#include "am335x_clockconfig.h"
#include "am335x_wdog.h"
#include "am335x_lowputc.h"
#include "am335x_boot.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* If the LCDC is enabled, then this will provide the number of sections
 * to map for the framebuffer.
 */

#define AM335X_LCDC_FBNSECTIONS \
  ((CONFIG_AM335X_LCDC_FB_SIZE + 0x000fffff) >> 20)

/****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the UART to show boot status.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) arm_lowputc(c)
#else
#  define showprogress(c)
#endif

/* The vectors are, by default, positioned at the beginning of the text
 * section.  They will always have to be copied to the correct location.
 *
 * If we are using high vectors (CONFIG_ARCH_LOWVECTORS=n).  In this case,
 * the vectors will lie at virtual address 0xffff:0000 and we will need
 * to a) copy the vectors to another location, and b) map the vectors
 * to that address, and
 *
 * For the case of CONFIG_ARCH_LOWVECTORS=y, defined.  Vectors will be
 * copied to OCMC0 RAM at address 0x0000:0000
 */

#if !defined(CONFIG_ARCH_LOWVECTORS) && defined(CONFIG_ARCH_ROMPGTABLE)
#  error High vector remap cannot be performed if we are using a ROM page table
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Symbols defined via the linker script */

extern uint8_t _vector_start[]; /* Beginning of vector block */
extern uint8_t _vector_end[];   /* End+1 of vector block */

#define SAMA5_LCDC_FBNSECTIONS \
  ((CONFIG_SAMA5_LCDC_FB_SIZE + 0x000fffff) >> 20)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This table describes how to map a set of 1Mb pages to space the physical
 * address space of the A1X.
 */

#ifndef CONFIG_ARCH_ROMPGTABLE
static const struct section_mapping_s g_section_mapping[] =
{
  {
    AM335X_GPMC_PSECTION,       AM335X_GPMC_VSECTION,  /* Includes vectors and page table */
    AM335X_GPMC_MMUFLAGS,       AM335X_GPMC_NSECTIONS
  },
  {
    AM335X_BROM_PSECTION,       AM335X_BROM_VSECTION,
    AM335X_BROM_MMUFLAGS,       AM335X_BROM_NSECTIONS
  },
  {
    AM335X_ISRAM_PSECTION,      AM335X_ISRAM_VSECTION,
    AM335X_ISRAM_MMUFLAGS,      AM335X_ISRAM_NSECTIONS
  },
  {
    AM335X_OCMC0_PSECTION,      AM335X_OCMC0_VSECTION,
    AM335X_OCMC0_MMUFLAGS,      AM335X_OCMC0_NSECTIONS
  },
  {
    AM335X_PERIPH_PSECTION,     AM335X_PERIPH_VSECTION,
    AM335X_PERIPH_MMUFLAGS,     AM335X_PERIPH_NSECTIONS
  },
  {
    AM335X_DDR_PSECTION,        AM335X_DDR_VSECTION,
    AM335X_DDR_MMUFLAGS,        AM335X_DDR_NSECTIONS
  }

#ifdef CONFIG_AM335X_LCDC
  ,

  /* LCDC Framebuffer.  This entry reprograms a part of one of the above
   * regions, making it non-cache-able and non-buffer-able.
   */

  {
    CONFIG_AM335X_LCDC_FB_PBASE, CONFIG_AM335X_LCDC_FB_VBASE,
    MMU_IOFLAGS, AM335X_LCDC_FBNSECTIONS
  }
#endif
};

#define NMAPPINGS \
  (sizeof(g_section_mapping) / sizeof(struct section_mapping_s))

static const size_t g_num_mappings = NMAPPINGS;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_setupmappings
 *
 * Description
 *   Map all of the initial memory regions defined in g_section_mapping[]
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
static inline void am335x_setupmappings(void)
{
  mmu_l1_map_regions(g_section_mapping, g_num_mappings);
}
#else
#  define am335x_setupmappings()
#endif

/****************************************************************************
 * Name: am335x_remap
 *
 * Description
 *   Map all of the final memory regions defined in g_operational_mapping[]
 *
 ****************************************************************************/

#ifdef NEED_SDRAM_REMAPPING
static inline void am335x_remap(void)
{
  mmu_l1_map_regions(g_operational_mapping, g_num_opmappings);
}
#endif

/****************************************************************************
 * Name: am335x_vectorpermissions
 *
 * Description:
 *   Set permissions on the vector mapping.
 *
 ****************************************************************************/

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_LOWVECTORS) && \
     defined(CONFIG_PAGING)
static void am335x_vectorpermissions(uint32_t mmuflags)
{
  /* The PTE for the beginning of OCMC0 RAM is at the base of the L2 page
   * table
   */

  uint32_t pte = mmu_l2_getentry(PG_L2_VECT_VADDR, 0);

  /* Mask out the old MMU flags from the page table entry.
   *
   * The pte might be zero the first time this function is called.
   */

  if (pte == 0)
    {
      pte = PG_VECT_PBASE;
    }
  else
    {
      pte &= PG_L1_PADDRMASK;
    }

  /* Update the page table entry with the MMU flags and save */

  mmu_l2_setentry(PG_L2_VECT_VADDR, pte, 0, mmuflags);
}
#endif

/****************************************************************************
 * Name: am335x_vectorsize
 *
 * Description:
 *   Return the size of the vector data
 *
 ****************************************************************************/

static inline size_t am335x_vectorsize(void)
{
  return _vector_end - _vector_start;
}

/****************************************************************************
 * Name: am335x_vectormapping
 *
 * Description:
 *   Setup a special mapping for the interrupt vectors when (1) the
 *   interrupt vectors are not positioned in ROM, and when (2) the interrupt
 *   vectors are located at the high address, 0xffff0000.  When the
 *   interrupt vectors are located in ROM, we just have to assume that they
 *   were set up correctly;  When vectors  are located in low memory,
 *   0x00000000, the mapping for the ROM memory region will be suppressed.
 *
 ****************************************************************************/

#if !defined(CONFIG_ARCH_ROMPGTABLE) && !defined(CONFIG_ARCH_LOWVECTORS)
static void am335x_vectormapping(void)
{
  uint32_t vector_paddr = AM335X_VECTOR_PADDR & PTE_SMALL_PADDR_MASK;
  uint32_t vector_vaddr = AM335X_VECTOR_VADDR & PTE_SMALL_PADDR_MASK;
  uint32_t vector_size  = _vector_end - _vector_start;
  uint32_t end_paddr    = AM335X_VECTOR_PADDR + vector_size;

  /* REVISIT:  Cannot really assert in this context */

  DEBUGASSERT (vector_size <= VECTOR_TABLE_SIZE);

  /* We want to keep our interrupt vectors and interrupt-related logic in
   * on-chip RAM (OCMC0).  The AM335X has 64Kb of OCMC0 RAM positioned at
   * physical address 0x4030:0000; we need to map this to 0xffff:0000.
   */

  while (vector_paddr < end_paddr)
    {
      mmu_l2_setentry(VECTOR_L2_VBASE,  vector_paddr, vector_vaddr,
                      MMU_L2_VECTORFLAGS);
      vector_paddr += 4096;
      vector_vaddr += 4096;
    }

  /* Now set the level 1 descriptor to refer to the level 2 page table. */

  mmu_l1_setentry(VECTOR_L2_PBASE & PMD_PTE_PADDR_MASK,
                  AM335X_VECTOR_VADDR & PMD_PTE_PADDR_MASK,
                  MMU_L1_VECTORFLAGS);
}
#else
  /* No vector remap */

#  define am335x_vectormapping()
#endif

/****************************************************************************
 * Name: am335x_copyvectorblock
 *
 * Description:
 *   Copy the interrupt block to its final destination.  Vectors are already
 *   positioned at the beginning of the text region and only need to be
 *   copied in the case where we are using high vectors or where the
 *   beginning of the text region cannot be remapped to address zero.
 *
 ****************************************************************************/

static void am335x_copyvectorblock(void)
{
  uint32_t *src;
  uint32_t *end;
  uint32_t *dest;

#ifdef CONFIG_PAGING
  /* If we are using re-mapped vectors in an area that has been marked
   * read only, then temporarily mark the mapping write-able (non-buffered).
   */

  am335x_vectorpermissions(MMU_L2_VECTRWFLAGS);
#endif

  /* Copy the vectors into OCMC0 RAM at the address that will be mapped to
   * the vector  address:
   *
   *   AM335X_VECTOR_PADDR - Unmapped, physical address of vector table in
   *                         OCMC0 RAM
   *   AM335X_VECTOR_VSRAM - Virtual address of vector table in OCMC0 RAM
   *   AM335X_VECTOR_VADDR - Virtual address of vector table (0x00000000 or
   *                         0xffff0000)
   */

  src  = (uint32_t *)_vector_start;
  end  = (uint32_t *)_vector_end;
  dest = (uint32_t *)AM335X_VECTOR_VSRAM;

  while (src < end)
    {
      *dest++ = *src++;
    }

#if !defined(CONFIG_ARCH_LOWVECTORS) && defined(CONFIG_PAGING)
  /* Make the vectors read-only, cache-able again */

  am335x_vectorpermissions(MMU_L2_VECTORFLAGS);

#else
  /* Flush the DCache to assure that the vector data is in physical RAM */

  up_clean_dcache((uintptr_t)AM335X_VECTOR_VSRAM,
                  (uintptr_t)AM335X_VECTOR_VSRAM + am335x_vectorsize());
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_boot
 *
 * Description:
 *   Complete boot operations started in arm_head.S
 *
 *   This logic will be executing in SDRAM.  This boot logic was started by
 *   the AM335x boot logic.  At this point in time, clocking and SDRAM have
 *   already be initialized (they must be because we are executing out of
 *   SDRAM).  So all that must be done here is to:
 *
 *     1) Refine the memory mapping,
 *     2) Configure the serial console, and
 *     3) Perform board-specific initializations.
 *
 ****************************************************************************/

void arm_boot(void)
{
#ifndef CONFIG_ARCH_ROMPGTABLE
  /* __start provided the basic MMU mappings for OCMC0 RAM.  Now provide
   * mappings for all IO regions (Including the vector region).
   */

  am335x_setupmappings();

  /* Provide a special mapping for the OCMC0 RAM interrupt vector positioned
   * in high memory.
   */

  am335x_vectormapping();

#endif /* CONFIG_ARCH_ROMPGTABLE */

  /* Setup up vector block.  _vector_start and _vector_end are exported from
   * arm_vector.S
   */

  am335x_copyvectorblock();

  /* Initialize clocking to settings provided by board-specific logic */

  am335x_clockconfig();

  /* Initialize the FPU */

  arm_fpuconfig();

  /* Disable CPU Watchdog */

  am335x_wdog_disable_all();

  /* Perform board-specific memory initialization,  This must include
   * initialization of board-specific memory resources (e.g., SDRAM)
   *
   * NOTE: We must use caution prior to this point to make sure that
   * the logic does not access any global variables that might lie
   * in SDRAM.
   */

  am335x_memory_initialize();

#ifdef NEED_SDRAM_REMAPPING
  /* SDRAM was configured in a temporary state to support low-level
   * initialization.  Now that the SDRAM has been fully initialized,
   * we can reconfigure the SDRAM in its final, fully cache-able state.
   */

  am335x_remap();
#endif

#ifdef CONFIG_BOOT_SDRAM_DATA
  /* This setting is inappropriate for the AM335x because the code is
   * *always* executing from SDRAM.  If CONFIG_BOOT_SDRAM_DATA happens
   * to be set, let's try to do the right thing and initialize the
   * .data and .bss sections.
   */

  arm_data_initialize();
#endif

  /* Perform common, low-level chip initialization (might do nothing) */

  am335x_lowsetup();

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization if we are going to use the serial
   * driver.
   */

  arm_earlyserialinit();
#endif

  /* Perform board-specific initialization,  This must include:
   *
   * - Initialization of board-specific memory resources (e.g., SDRAM)
   * - Configuration of board specific resources (PIOs, LEDs, etc).
   */

  am335x_board_initialize();
}
