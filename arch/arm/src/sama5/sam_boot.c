/****************************************************************************
 * arch/arm/src/sama5/sam_boot.c
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
#include "hardware/sam_wdt.h"
#include "hardware/sam_aximx.h"
#include "hardware/sam_sfr.h"

#include "sam_clockconfig.h"
#include "sam_memorymap.h"
#include "sam_lowputc.h"
#include "sam_serial.h"
#include "sam_lcd.h"
#include "sam_boot.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Symbols defined via the linker script */

extern uint32_t _vector_start; /* Beginning of vector block */
extern uint32_t _vector_end;   /* End+1 of vector block */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_setupmappings
 *
 * Description:
 *   Map all of the initial memory regions defined in g_section_mapping[]
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
static inline void sam_setupmappings(void)
{
  mmu_l1_map_regions(g_section_mapping, g_num_mappings);
}
#endif

/****************************************************************************
 * Name: sam_remap
 *
 * Description:
 *   Map all of the final memory regions defined in g_operational_mapping[]
 *
 ****************************************************************************/

#ifdef NEED_SDRAM_REMAPPING
static inline void sam_remap(void)
{
  mmu_l1_map_regions(g_operational_mapping, g_num_opmappings);
}
#endif

/****************************************************************************
 * Name: sam_vectorpermissions
 *
 * Description:
 *   Set permissions on the vector mapping.
 *
 ****************************************************************************/

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_LOWVECTORS) && \
     defined(CONFIG_PAGING)
static void sam_vectorpermissions(uint32_t mmuflags)
{
  /* The PTE for the beginning of ISRAM is at the base of the L2 page table */

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
 * Name: sam_vectorsize
 *
 * Description:
 *   Return the size of the vector data
 *
 ****************************************************************************/

static inline size_t sam_vectorsize(void)
{
  uintptr_t src;
  uintptr_t end;

  src  = (uintptr_t)&_vector_start;
  end  = (uintptr_t)&_vector_end;

  return (size_t)(end - src);
}

/****************************************************************************
 * Name: sam_vectormapping
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
static void sam_vectormapping(void)
{
  uint32_t vector_paddr = SAM_VECTOR_PADDR & PTE_SMALL_PADDR_MASK;
  uint32_t vector_vaddr = SAM_VECTOR_VADDR & PTE_SMALL_PADDR_MASK;
  uint32_t vector_size  = (uint32_t)&_vector_end - (uint32_t)&_vector_start;
  uint32_t end_paddr    = SAM_VECTOR_PADDR + vector_size;

  /* REVISIT:  Cannot really assert in this context */

  DEBUGASSERT (vector_size <= VECTOR_TABLE_SIZE);

  /* We want to keep our interrupt vectors and interrupt-related logic in
   * zero-wait state internal SRAM (ISRAM).  The SAMA5 has 128Kb of ISRAM
   * positioned at physical address 0x0300:0000; we need to map this to
   * 0xffff:0000.
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
                  SAM_VECTOR_VADDR & PMD_PTE_PADDR_MASK,
                  MMU_L1_VECTORFLAGS);
}
#else
  /* No vector remap */

#  define sam_vectormapping()
#endif

/****************************************************************************
 * Name: sam_copyvectorblock
 *
 * Description:
 *   Copy the interrupt block to its final destination.  Vectors are already
 *   positioned at the beginning of the text region and only need to be
 *   copied in the case where we are using high vectors or where the
 *   beginning of the text region cannot be remapped to address zero.
 *
 ****************************************************************************/

#if !defined(CONFIG_ARCH_LOWVECTORS)
static void sam_copyvectorblock(void)
{
  uint32_t *src;
  uint32_t *end;
  uint32_t *dest;

#ifdef CONFIG_PAGING
  /* If we are using re-mapped vectors in an area that has been marked
   * read only, then temporarily mark the mapping write-able (non-buffered).
   */

  sam_vectorpermissions(MMU_L2_VECTRWFLAGS);
#endif

  /* Copy the vectors into ISRAM at the address that will be mapped to the
   * vector address:
   *
   *   SAM_VECTOR_PADDR - Unmapped, physical address of vector table in SRAM
   *   SAM_VECTOR_VSRAM - Virtual address of vector table in SRAM
   *   SAM_VECTOR_VADDR - Virtual address of vector table (0x00000000 or
   *                      0xffff0000)
   */

  src  = (uint32_t *)&_vector_start;
  end  = (uint32_t *)&_vector_end;
  dest = (uint32_t *)SAM_VECTOR_VSRAM;

  while (src < end)
    {
      *dest++ = *src++;
    }

#if !defined(CONFIG_ARCH_LOWVECTORS) && defined(CONFIG_PAGING)
  /* Make the vectors read-only, cacheable again */

  sam_vectorpermissions(MMU_L2_VECTORFLAGS);

#else
  /* Flush the DCache to assure that the vector data is in physical in
   * ISRAM
   */

  up_clean_dcache((uintptr_t)SAM_VECTOR_VSRAM,
                  (uintptr_t)SAM_VECTOR_VSRAM + sam_vectorsize());
#endif
}

#else
/* Don't copy the vectors */

#  define sam_copyvectorblock()
#endif

/****************************************************************************
 * Name: sam_wdtdisable
 *
 * Description:
 *   Disable the watchdog timer.  The SAMA5 always boots with the watchdog
 *   timer enabled at its maximum timeout (16 seconds).  The watchdog timer
 *   can disabled by writing to the Watchdog Mode Register (WDT_MR).  The
 *   WDT_MR, however, can be written only one time after the CPU has been
 *   reset.
 *
 *   So if no watchdog timer driver has been configured, the watchdog timer
 *   must be disabled as part of the start up logic.  But, on the other
 *   hand, we must not write to the WDT_MR register if the watchdog timer
 *   driver is configured.  In that case, some later application will
 *   configure the WDT and begin periodic pinging (within 16 seconds,
 *   hopefully).
 *
 ****************************************************************************/

#ifndef CONFIG_SAMA5_WDT
static inline void sam_wdtdisable(void)
{
  putreg32(WDT_MR_WDDIS, SAM_WDT_MR);
}
#else
#  define sam_wdtdisable()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_boot
 *
 * Description:
 *   Complete boot operations started in arm_head.S
 *
 * Boot Sequence
 *
 *   This logic may be executing in ISRAM or in external memory: CS0, DDR,
 *   CS1, CS2, or CS3.  It may be executing in CS0 or ISRAM through the
 *   action of the SAMA5 "first level bootloader;"  it might be executing in
 *   CS1-3 through the action of some second level bootloader that provides
 *   configuration for those memories.
 *
 *   The system always boots from the ROM memory at address 0x0000:0000,
 *   starting the internal first level bootloader.  That bootloader can be
 *   configured to work in different ways using the BMS pin and the contents
 *   of the Boot Sequence Configuration Register (BSC_CR).
 *
 *   If the BMS_BIT is read "1", then the first level bootloader will
 *   support execution of code in the memory connected to CS0 on the EBI
 *   interface (presumably NOR flash).  The following sequence is performed
 *   by the first level bootloader if BMS_BIT is "1":
 *
 *     - The main clock is the on-chip 12 MHz RC oscillator,
 *     - The Static Memory Controller is configured with timing allowing
 *       code execution in CS0 external memory at 12 MHz
 *     - AXI matrix is configured to remap EBI CS0 address at 0x0
 *     - 0x0000:0000 is loaded in the Program Counter register
 *
 *   The user software in the external memory must perform the next
 *   operation in order to complete the clocks and SMC timings configuration
 *   to run at a higher clock frequency:
 *
 *     - Enable the 32768 Hz oscillator if best accuracy is needed
 *     - Reprogram the SMC setup, cycle, hold, mode timing registers for EBI
 *       CS0, to adapt them to the new clock.
 *     - Program the PMC (Main Oscillator Enable or Bypass mode)
 *     - Program and Start the PLL
 *     - Switch the system clock to the new value
 *
 *  If the BMS_BIT is read "0", then the first level bootloader will
 *  perform:
 *
 *     - Basic chip initialization: XTal or external clock frequency
 *       detection:
 *
 *       a. Stack Setup for ARM supervisor mode
 *       b. Main Oscillator Detection:  The bootloader attempts to use an
 *          external crystal.  If this is not successful, then  the 12 MHz
 *          Fast RC internal oscillator is used as the main oscillator.
 *       c. Main Clock Selection: The Master Clock source is switched from
 *          to the main oscillator without prescaler. PCK and MCK are now
 *          the Main Clock.
 *       d. PLLA Initialization: PLLA is configured to get a PCK at 96 MHz
 *          and an MCK at 48 MHz. If an external clock or crystal frequency
 *          running at 12 MHz is found, then the PLLA is configured to allow
 *          USB communication.
 *
 *     - Attempt to retrieve a valid code from external non-volatile
 *       memories (NVM): SPI0 CS0 Flash Boot, SD Card Boot, NAND Flash Boot,
 *       SPI0 CS1 Flash Boot, or TWI EEPROM Boot.  Different heuristics are
 *       used with each media type.  If a valid image is found, it is copied
 *       to internal SRAM and started.
 *
 *     - In case no valid application has been found on any NVM, the SAM-BA
 *       Monitor is started.
 *
 ****************************************************************************/

void arm_boot(void)
{
#ifdef CONFIG_ARCH_RAMFUNCS
  const uint32_t *src;
  uint32_t *dest;
#endif
#ifdef ATSAMA5D2
  uint32_t regval;
#endif

#ifndef CONFIG_ARCH_ROMPGTABLE
  /* __start provided the basic MMU mappings for SRAM.  Now provide mappings
   * for all IO regions (Including the vector region).
   */

  sam_setupmappings();

  /* Provide a special mapping for the IRAM interrupt vector positioned in
   * high memory.
   */

  sam_vectormapping();

#endif /* CONFIG_ARCH_ROMPGTABLE */

#ifdef ATSAMA5D2
  /* The SAMA5D2 features a second 128-Kbyte SRAM that can be allocated
   * either to the L2 cache controller or used as an internal SRAM. After
   * reset, this block is connected to the L2 cache controller. The
   * SRAM_SEL bit, located in the SFR_L2CC_HRAMC register, is used to
   * reassign this memory as system SRAM, making the two 128-Kbyte
   * RAMs contiguous.
   */

  regval = getreg32(SAM_SFR_L2CCHRAMC);

#ifdef CONFIG_ARCH_L2CACHE
  regval |= SFR_L2CCHRAMC_SRAMSEL;  /* 1=Selects L2CC */
#else
  regval &= ~SFR_L2CCHRAMC_SRAMSEL; /* 0=Selects SRAM */
#endif

  putreg32(regval, SAM_SFR_L2CCHRAMC);
#endif

#ifdef CONFIG_ARCH_RAMFUNCS
  /* Copy any necessary code sections from FLASH to RAM.  The correct
   * destination in SRAM is given by _sramfuncs and _eramfuncs.  The
   * temporary location is in flash after the data initialization code
   * at _framfuncs
   */

  for (src = &_framfuncs, dest = &_sramfuncs; dest < &_eramfuncs; )
    {
      *dest++ = *src++;
    }

  /* Flush the copied RAM functions into physical RAM so that will
   * be available when fetched into the I-Cache.
   */

  up_clean_dcache((uintptr_t)&_sramfuncs, (uintptr_t)&_eramfuncs)
#endif

  /* Setup up vector block.  _vector_start and _vector_end are exported from
   * arm_vector.S
   */

  sam_copyvectorblock();

  /* Disable the watchdog timer */

  sam_wdtdisable();

  /* Initialize clocking to settings provided by board-specific logic */

  sam_clockconfig();

  /* Initialize the FPU */

  arm_fpuconfig();

  /* Perform board-specific initialization,  This must include:
   *
   * - Initialization of board-specific memory resources (e.g., SDRAM)
   * - Configuration of board specific resources (PIOs, LEDs, etc).
   *
   * NOTE: We must use caution prior to this point to make sure that
   * the logic does not access any global variables that might lie
   * in SDRAM.
   */

  sam_boardinitialize();

#ifdef NEED_SDRAM_REMAPPING
  /* SDRAM was configured in a temporary state to support low-level
   * initialization.  Now that the SDRAM has been fully initialized,
   * we can reconfigure the SDRAM in its final, fully cache-able state.
   */

  sam_remap();
#endif

#ifdef CONFIG_BOOT_SDRAM_DATA
  /* If .data and .bss reside in SDRAM, then initialize the data sections
   * now after SDRAM has been initialized.
   */

  arm_data_initialize();
#endif

  /* Perform common, low-level chip initialization (might do nothing) */

  sam_lowsetup();

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization if we are going to use the serial
   * driver.
   */

  sam_earlyserialinit();
#endif
}
