/****************************************************************************
 * arch/arm/src/sama5/sam_boot.c
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <assert.h>

#ifdef CONFIG_PAGING
#  include <nuttx/page.h>
#endif

#include <arch/board/board.h>

#include "chip.h"
#include "arm.h"
#include "mmu.h"
#include "cache.h"
#include "fpu.h"
#include "up_internal.h"
#include "up_arch.h"

#include "chip/sam_wdt.h"
#include "chip/sam_aximx.h"
#include "sam_clockconfig.h"
#include "sam_lowputc.h"
#include "sam_serial.h"
#include "sam_lcd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* The vectors are, by default, positioned at the beginning of the text
 * section.  Under what conditions do we have to remap the these vectors?
 *
 * 1) If we are using high vectors (CONFIG_ARCH_LOWVECTORS=n).  In this case,
 *    the vectors will lie at virtual address 0xffff:000 and we will need
 *    to a) copy the vectors to another location, and b) map the vectors
 *    to that address, and
 *
 *    For the case of CONFIG_ARCH_LOWVECTORS=y, defined.  The SAMA5 boot-up
 *    logic will map the beginning of the boot memory to address 0x0000:0000
 *    using both the MMU and the AXI matrix REMAP register.  No vector copy
 *    is required because the vectors are position at the beginning of the
 *    boot memory at link time and no additional MMU mapping required.
 *
 * 2) We are not using a ROM page table.  We cannot set any custom mappings in
 *    the case and the build must conform to the ROM page table properties
 */

#if !defined(CONFIG_ARCH_LOWVECTORS) && defined(CONFIG_ARCH_ROMPGTABLE)
#  error High vector remap cannot be performed if we are using a ROM page table
#endif

/* If SDRAM needs to be configured, then it will be configured twice:  It
 * will first be configured to a temporary state to support low-level
 * initialization.  After the SDRAM has been fully initialized, SRAM be used
 * to set the SDRM in its final, fully cache-able state.
 */

#undef NEED_SDRAM_CONFIGURATION
#if defined(CONFIG_SAMA5_DDRCS) && !defined(CONFIG_SAMA5_BOOT_SDRAM)
#  define NEED_SDRAM_CONFIGURATION 1
#endif

#undef NEED_SDRAM_MAPPING
#undef NEED_SDRAM_REMAPPING
#if defined(NEED_SDRAM_CONFIGURATION) && !defined(CONFIG_ARCH_ROMPGTABLE)
#  define NEED_SDRAM_MAPPING 1
#  define NEED_SDRAM_REMAPPING 1
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* Symbols defined via the linker script */

extern uint32_t _vector_start; /* Beginning of vector block */
extern uint32_t _vector_end;   /* End+1 of vector block */

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/* This table describes how to map a set of 1Mb pages to space the physical
 * address space of the SAMA5.
 */

#ifndef CONFIG_ARCH_ROMPGTABLE
static const struct section_mapping_s section_mapping[] =
{
  /* SAMA5 Internal Memories */

  /* If CONFIG_ARCH_LOWVECTORS is defined, then the vectors located at the
   * beginning of the .text region must appear at address at the address
   * specified in the VBAR.  There are three ways to accomplish this:
   *
   *   1. By explicitly mapping the beginning of .text region with a page
   *      table entry so that the virtual address zero maps to the beginning
   *      of the .text region.  VBAR == 0x0000:0000.
   *
   *   2. A second way is to map the use the AXI MATRIX remap register to
   *      map physical address zero to the beginning of the text region,
   *      either internal SRAM or EBI CS 0.  Then we can set an identity
   *      mapping to map the boot region at 0x0000:0000 to virtual address
   *      0x0000:00000.   VBAR == 0x0000:0000.
   *
   *      This method is used when booting from ISRAM or NOR FLASH.  In
   &      that case, vectors must lie at the beginning of NOFR FLASH.
   *
   *   3. Set the Cortex-A5 VBAR register so that the vector table address
   *      is moved to a location other than 0x0000:0000.
   *
   *      This is the method used when booting from SDRAM.
   *
   * - When executing from NOR FLASH, the first level bootloader is supposed
   *   to provide the AXI MATRIX mapping for us at boot time base on the state
   *   of the BMS pin.  However, I have found that in the test environments
   *   that I use, I cannot always be assured of that physical address mapping.
   *
   *   So we do both here.  If we are executing from NOR FLASH, then we provide
   *   the MMU to map the physical address of FLASH to address 0x0000:0000;
   *
   * - If we are executing out of ISRAM, then the SAMA5 primary bootloader
   *   probably copied us into ISRAM and set the AXI REMAP bit for us.
   *
   * - If we are executing from external SDRAM, then a secondary bootloader must
   *   have loaded us into SDRAM.  In this case, simply set the VBAR register
   *   to the address of the vector table (not necessary at the beginning
   *   or SDRAM).
   */

#if defined(CONFIG_ARCH_LOWVECTORS) && !defined(CONFIG_SAMA5_BOOT_ISRAM) && \
    !defined(CONFIG_SAMA5_BOOT_SDRAM)
  { CONFIG_FLASH_START,    0x00000000,
    MMU_ROMFLAGS,          1
  },
#else
  { SAM_BOOTMEM_PSECTION,  SAM_BOOTMEM_VSECTION,
    SAM_BOOTMEM_MMUFLAGS,  SAM_BOOTMEM_NSECTIONS
  },
#endif

  { SAM_ROM_PSECTION,      SAM_ROM_VSECTION,
    SAM_ROM_MMUFLAGS,      SAM_ROM_NSECTIONS
  },
  { SAM_NFCSRAM_PSECTION,  SAM_NFCSRAM_VSECTION,
    SAM_NFCSRAM_MMUFLAGS,  SAM_NFCSRAM_NSECTIONS
  },
#ifndef CONFIG_PAGING /* Internal SRAM is already fully mapped */
  { SAM_ISRAM_PSECTION,    SAM_ISRAM_VSECTION,
    SAM_ISRAM_MMUFLAGS,    SAM_ISRAM_NSECTIONS
  },
#endif
  { SAM_SMD_PSECTION,      SAM_SMD_VSECTION,
    SAM_SMD_MMUFLAGS,      SAM_SMD_NSECTIONS
  },
  { SAM_UDPHSRAM_PSECTION, SAM_UDPHSRAM_VSECTION,
    SAM_UDPHSRAM_MMUFLAGS, SAM_UDPHSRAM_NSECTIONS
  },
  { SAM_UHPOHCI_PSECTION,  SAM_UHPOHCI_VSECTION,
    SAM_UHPOHCI_MMUFLAGS,  SAM_UHPOHCI_NSECTIONS
  },
  { SAM_UHPEHCI_PSECTION,  SAM_UHPEHCI_VSECTION,
    SAM_UHPEHCI_MMUFLAGS,  SAM_UHPEHCI_NSECTIONS
  },
  { SAM_AXIMX_PSECTION,    SAM_AXIMX_VSECTION,
    SAM_AXIMX_MMUFLAGS,    SAM_AXIMX_NSECTIONS
  },
  { SAM_DAP_PSECTION,      SAM_DAP_VSECTION,
    SAM_DAP_MMUFLAGS,      SAM_DAP_NSECTIONS
  },

/* SAMA5 CS0 External Memories */

#ifdef CONFIG_SAMA5_EBICS0
  { SAM_EBICS0_PSECTION, SAM_EBICS0_VSECTION,
    SAM_EBICS0_MMUFLAGS, SAM_EBICS0_NSECTIONS
  },
#endif

/* SAMA5 External SDRAM Memory.  The SDRAM is not usable until it has been
 * initialized.  If we are running out of SDRAM now, we can assume that some
 * second level boot loader has properly configured SRAM for us.  In that
 * case, we set the MMU flags for the final, fully cache-able state.
 *
 * Also, in this case, the mapping for the SDRAM was done in arm_head.S and
 * need not be repeated here.
 *
 * If we are running from ISRAM or NOR flash, then we will need to configure
 * the SDRAM ourselves.  In this case, we set the MMU flags to the strongly
 * ordered, non-cacheable state.  We need this direct access to SDRAM in
 * order to configure it.  Once SDRAM has been initialized, it will be re-
 * configured in its final state.
 */

#ifdef NEED_SDRAM_MAPPING
  { SAM_DDRCS_PSECTION,    SAM_DDRCS_VSECTION,
    MMU_STRONGLY_ORDERED,  SAM_DDRCS_NSECTIONS
  },
#endif

/* SAMA5 CS1-3 External Memories */

#ifdef CONFIG_SAMA5_EBICS1
  { SAM_EBICS1_PSECTION,   SAM_EBICS1_VSECTION,
    SAM_EBICS1_MMUFLAGS,   SAM_EBICS1_NSECTIONS
  },
#endif
#ifdef CONFIG_SAMA5_EBICS2
  { SAM_EBICS2_PSECTION,   SAM_EBICS2_VSECTION,
    SAM_EBICS2_MMUFLAGS,   SAM_EBICS2_NSECTIONS
  },
#endif
#ifdef CONFIG_SAMA5_EBICS3
  { SAM_EBICS3_PSECTION,   SAM_EBICS3_VSECTION,
    SAM_EBICS3_MMUFLAGS,   SAM_EBICS3_NSECTIONS
  },
#endif
#ifdef CONFIG_SAMA5_HAVE_NAND
  { SAM_NFCCR_PSECTION,   SAM_NFCCR_VSECTION,
    SAM_NFCCR_MMUFLAGS,   SAM_NFCCR_NSECTIONS
  },
#endif

/* SAMA5 Internal Peripherals */

  { SAM_PERIPHA_PSECTION, SAM_PERIPHA_VSECTION,
    SAM_PERIPHA_MMUFLAGS, SAM_PERIPHA_NSECTIONS
  },
  { SAM_PERIPHB_PSECTION, SAM_PERIPHB_VSECTION,
    SAM_PERIPHB_MMUFLAGS, SAM_PERIPHB_NSECTIONS
  },
  { SAM_SYSC_PSECTION,    SAM_SYSC_VSECTION,
    SAM_SYSC_MMUFLAGS,    SAM_SYSC_NSECTIONS
  },

/* LCDC Framebuffer.  This entry reprograms a part of one of the above
 * regions, making it non-cacheable and non-buffereable.
 *
 * If SDRAM will be reconfigured, then we will defer setup of the framebuffer
 * until after the SDRAM remapping (since the framebuffer problem resides) in
 * SDRAM.
 */

#if defined(CONFIG_SAMA5_LCDC) && !defined(NEED_SDRAM_REMAPPING)
  { CONFIG_SAMA5_LCDC_FB_PBASE, CONFIG_SAMA5_LCDC_FB_VBASE,
    MMU_IOFLAGS, SAMA5_LCDC_FBNSECTIONS
  },
#endif
};

#define NMAPPINGS \
  (sizeof(section_mapping) / sizeof(struct section_mapping_s))
#endif

/* SAMA5 External SDRAM Memory.  Final configuration.  The SDRAM was
 * configured in a temporary state to support low-level ininitialization.
 * After the SDRAM has been fully initialized, this structure is used to
 * set the SDRM in its final, fully cache-able state.
 */

#ifdef NEED_SDRAM_REMAPPING
static const struct section_mapping_s operational_mapping[] =
{
/* This entry reprograms the SDRAM entry, making it cacheable and
 * bufferable.
 */

  { SAM_DDRCS_PSECTION, SAM_DDRCS_VSECTION,
    SAM_DDRCS_MMUFLAGS, SAM_DDRCS_NSECTIONS
  },

/* LCDC Framebuffer.  This entry reprograms a part of one of the above
 * regions, making it non-cacheable and non-buffereable.
 */

#ifdef CONFIG_SAMA5_LCDC
  { CONFIG_SAMA5_LCDC_FB_PBASE, CONFIG_SAMA5_LCDC_FB_VBASE,
    MMU_IOFLAGS, SAMA5_LCDC_FBNSECTIONS
  },
#endif
};

#define NREMAPPINGS \
  (sizeof(operational_mapping) / sizeof(struct section_mapping_s))
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_setupmappings
 *
 * Description
 *   Map all of the initial memory regions defined in section_mapping[]
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
static inline void sam_setupmappings(void)
{
  int i;

  /* Set up each group of section mappings */

  for (i = 0; i < NMAPPINGS; i++)
    {
      mmu_l1_map_region(&section_mapping[i]);
    }
}
#endif

/****************************************************************************
 * Name: sam_remap
 *
 * Description
 *   Map all of the final memory regions defined in operation_mapping[]
 *
 ****************************************************************************/

#ifdef NEED_SDRAM_REMAPPING
static inline void sam_remap(void)
{
  int i;

  /* Re-map each group of section */

  for (i = 0; i < NREMAPPINGS; i++)
    {
      mmu_l1_map_region(&operational_mapping[i]);
    }
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
 *   copied in the case where we are using high vectors or where the beginning
 *   of the text region cannot be remapped to address zero.
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

  /* Copy the vectors into ISRAM at the address that will be mapped to the vector
   * address:
   *
   *   SAM_VECTOR_PADDR - Unmapped, physical address of vector table in SRAM
   *   SAM_VECTOR_VSRAM - Virtual address of vector table in SRAM
   *   SAM_VECTOR_VADDR - Virtual address of vector table (0x00000000 or
   *                      0xffff0000)
   */

  src  = (uint32_t*)&_vector_start;
  end  = (uint32_t*)&_vector_end;
  dest = (uint32_t*)SAM_VECTOR_VSRAM;

  while (src < end)
    {
      *dest++ = *src++;
    }

#if !defined(CONFIG_ARCH_LOWVECTORS) && defined(CONFIG_PAGING)
  /* Make the vectors read-only, cacheable again */

  sam_vectorpermissions(MMU_L2_VECTORFLAGS);

#else
  /* Flush the DCache to assure that the vector data is in physical in ISRAM */

  cp15_clean_dcache((uintptr_t)SAM_VECTOR_VSRAM,
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
 * Name: up_boot
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
 *          Fast RC internal oscillator is used as the main osciallator.
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

void up_boot(void)
{
#ifdef CONFIG_ARCH_RAMFUNCS
  const uint32_t *src;
  uint32_t *dest;
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

  cp15_clean_dcache((uintptr_t)&_sramfuncs, (uintptr_t)&_eramfuncs)
#endif

  /* Setup up vector block.  _vector_start and _vector_end are exported from
   * arm_vector.S
   */

  sam_copyvectorblock();

  /* Disable the watchdog timer */

  sam_wdtdisable();

  /* Initialize clocking to settings provided by board-specific logic */

  sam_clockconfig();

#ifdef CONFIG_ARCH_FPU
  /* Initialize the FPU */

  arm_fpuconfig();
#endif

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

#ifdef CONFIG_NUTTX_KERNEL
  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

  sam_userspace();
#endif
}
