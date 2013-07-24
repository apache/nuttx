/****************************************************************************
 * arch/arm/src/sama5/sam_boot.c
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
#include <stdint.h>

#ifdef CONFIG_PAGING
#  include <nuttx/page.h>
#endif

#include <arch/board/board.h>

#include "chip.h"
#include "arm.h"
#include "mmu.h"
#include "fpu.h"
#include "up_internal.h"
#include "up_arch.h"

#include "chip/sam_wdt.h"
#include "sam_clockconfig.h"
#include "sam_lowputc.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct section_mapping_s
{
  uint32_t physbase;   /* Physical address of the region to be mapped */
  uint32_t virtbase;   /* Virtual address of the region to be mapped */
  uint32_t mmuflags;   /* MMU settings for the region (e.g., cache-able) */
  uint32_t nsections;  /* Number of mappings in the region */
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

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

#ifndef CONFIG_ARCH_LOWVECTORS
  { SAM_BOOTMEM_PSECTION, SAM_BOOTMEM_VSECTION, 
    SAM_BOOTMEM_MMUFLAGS, SAM_BOOTMEM_NSECTIONS},
#endif
  { SAM_ROM_PSECTION, SAM_ROM_VSECTION, 
    SAM_ROM_MMUFLAGS, SAM_ROM_NSECTIONS},
  { SAM_NFCSRAM_PSECTION, SAM_NFCSRAM_VSECTION, 
    SAM_NFCSRAM_MMUFLAGS, SAM_NFCSRAM_NSECTIONS},
#ifndef CONFIG_PAGING /* Internal SRAM is already fully mapped */
  { SAM_ISRAM_PSECTION, SAM_ISRAM_VSECTION, 
    SAM_ISRAM_MMUFLAGS, SAM_ISRAM_NSECTIONS},
#endif
  { SAM_SMD_PSECTION, SAM_SMD_VSECTION, 
    SAM_SMD_MMUFLAGS, SAM_SMD_NSECTIONS},
  { SAM_UDPHSRAM_PSECTION, SAM_UDPHSRAM_VSECTION, 
    SAM_UDPHSRAM_MMUFLAGS, SAM_UDPHSRAM_NSECTIONS},
  { SAM_UHPOHCI_PSECTION, SAM_UHPOHCI_VSECTION, 
    SAM_UHPOHCI_MMUFLAGS, SAM_UHPOHCI_NSECTIONS},
  { SAM_UHPEHCI_PSECTION, SAM_UHPEHCI_VSECTION, 
    SAM_UHPEHCI_MMUFLAGS, SAM_UHPEHCI_NSECTIONS},
  { SAM_AXIMX_PSECTION, SAM_AXIMX_VSECTION, 
    SAM_AXIMX_MMUFLAGS, SAM_AXIMX_NSECTIONS},
  { SAM_DAP_PSECTION, SAM_DAP_VSECTION, 
    SAM_DAP_MMUFLAGS, SAM_DAP_NSECTIONS},

/* SAMA5 External Memories */

#ifdef CONFIG_SAMA5_EBICS0
  { SAM_EBICS0_PSECTION, SAM_EBICS0_VSECTION, 
    SAM_EBICS0_MMUFLAGS, SAM_EBICS0_NSECTIONS},
#endif
#ifdef CONFIG_SAMA5_DDRCS
  { SAM_DDRCS_PSECTION, SAM_DDRCS_VSECTION, 
    SAM_DDRCS_MMUFLAGS, SAM_DDRCS_NSECTIONS},
#endif
#ifdef CONFIG_SAMA5_EBICS1
  { SAM_EBICS1_PSECTION, SAM_EBICS1_VSECTION, 
    SAM_EBICS1_MMUFLAGS, SAM_EBICS1_NSECTIONS},
#endif
#ifdef CONFIG_SAMA5_EBICS2
  { SAM_EBICS2_PSECTION, SAM_EBICS2_VSECTION, 
    SAM_EBICS2_MMUFLAGS, SAM_EBICS2_NSECTIONS},
#endif
#ifdef CONFIG_SAMA5_EBICS3
  { SAM_EBICS3_PSECTION, SAM_EBICS3_VSECTION, 
    SAM_EBICS3_MMUFLAGS, SAM_EBICS3_NSECTIONS},
#endif
#ifdef CONFIG_SAMA5_NFCCR
  { SAM_NFCCR_PSECTION, SAM_NFCCR_VSECTION, 
    SAM_NFCCR_MMUFLAGS, SAM_NFCCR_NSECTIONS},
#endif

/* SAMA5 Internal Peripherals */

  { SAM_PERIPHA_PSECTION, SAM_PERIPHA_VSECTION, 
    SAM_PERIPHA_MMUFLAGS, SAM_PERIPHA_NSECTIONS},
  { SAM_PERIPHB_PSECTION, SAM_PERIPHB_VSECTION, 
    SAM_PERIPHB_MMUFLAGS, SAM_PERIPHB_NSECTIONS},
  { SAM_SYSC_PSECTION, SAM_SYSC_VSECTION, 
    SAM_SYSC_MMUFLAGS, SAM_SYSC_NSECTIONS},
};
#define NMAPPINGS (sizeof(section_mapping) / sizeof(struct section_mapping_s))
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_setlevel1entry
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
static inline void sam_setlevel1entry(uint32_t paddr, uint32_t vaddr,
                                      uint32_t mmuflags)
{
  uint32_t *pgtable = (uint32_t*)PGTABLE_BASE_VADDR;
  uint32_t  index   = vaddr >> 20;

  /* Save the page table entry */

  pgtable[index]  = (paddr | mmuflags);
}
#endif

/****************************************************************************
 * Name: sam_setlevel2coarseentry
 ****************************************************************************/

static inline void
sam_setlevel2coarseentry(uint32_t ctabvaddr, uint32_t paddr, uint32_t vaddr,
                         uint32_t mmuflags)
{
  uint32_t *ctable  = (uint32_t*)ctabvaddr;
  uint32_t  index;

  /* The table divides a 1Mb address space up into 256 entries, each
   * corresponding to 4Kb of address space.  The page table index is
   * related to the offset from the beginning of 1Mb region.
   */

  index = (vaddr & 0x000ff000) >> 12;

  /* Save the table entry */

  ctable[index] = (paddr | mmuflags);
}

/****************************************************************************
 * Name: sam_setupmappings
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
static void sam_setupmappings(void)
{
  int i, j;

  for (i = 0; i < NMAPPINGS; i++)
    {
      uint32_t sect_paddr = section_mapping[i].physbase;
      uint32_t sect_vaddr = section_mapping[i].virtbase;
      uint32_t mmuflags   = section_mapping[i].mmuflags;

      for (j = 0; j < section_mapping[i].nsections; j++)
        {
          sam_setlevel1entry(sect_paddr, sect_vaddr, mmuflags);
          sect_paddr += SECTION_SIZE;
          sect_vaddr += SECTION_SIZE;
        }
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
static void  sam_vectorpermissions(uint32_t mmuflags)
{
  /* The PTE for the beginning of ISRAM is at the base of the L2 page table */

  uint32_t *ptr = (uint32_t*)PG_L2_VECT_VADDR;
  uint32_t pte;

  /* The pte might be zero the first time this function is called. */

  pte = *ptr;
  if (pte == 0)
    {
      pte = PG_VECT_PBASE;   
    }
  else
    {
      pte &= PG_L1_PADDRMASK;
    }

  /* Update the MMU flags and save */

  *ptr = pte | mmuflags;

  /* Invalid the TLB for this address */

  tlb_invalidate_single(PG_L2_VECT_VADDR);
}
#endif

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
  uint32_t vector_paddr = SAM_VECTOR_PADDR;
  uint32_t vector_vaddr = SAM_VECTOR_VADDR;
  uint32_t end_paddr    = vector_paddr + VECTOR_TABLE_SIZE;

  /* We want to keep our interrupt vectors and interrupt-related logic in
   * zero-wait state internal SRAM (ISRAM).  The SAMA5 has 128Kb of ISRAM
   * positioned at physical address 0x0300:0000; we need to map this to
   * 0xffff:0000.
   */

  while (vector_paddr < end_paddr)
    {
      sam_setlevel2coarseentry(PGTABLE_L2_VBASE,  vector_paddr,
                               vector_vaddr, MMU_L2_VECTORFLAGS);
      vector_paddr += 4096;
      vector_vaddr += 4096;
    }

  /* Now set the level 1 descriptor to refer to the level 2 page table. */

  sam_setlevel1entry(PGTABLE_L2_PBASE, SAM_VECTOR_VCOARSE,
                     MMU_L1_VECTORFLAGS);
}
#endif

/****************************************************************************
 * Name: sam_copyvectorblock
 *
 * Description:
 *   Copy the interrupt block to its final destination.
 *
 ****************************************************************************/

static void sam_copyvectorblock(void)
{
  uint32_t *src;
  uint32_t *end;
  uint32_t *dest;

  /* If we are using vectors in low memory but RAM in that area has been marked
   * read only, then temparily mark the mapping write-able (non-buffered).
   */

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_LOWVECTORS) && \
     defined(CONFIG_PAGING)
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

  /* Make the vectors read-only, cacheable again */

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_LOWVECTORS) && \
     defined(CONFIG_PAGING)
  sam_vectorpermissions(MMU_L2_VECTROFLAGS);
#endif
}

/****************************************************************************
 * Name: sam_wdtdisable
 *
 * Description:
 *   Disable the watchdog timer
 *
 ****************************************************************************/

static inline void sam_wdtdisable(void)
{
  putreg32(WDT_MR_WDDIS, SAM_WDT_MR);
}

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
 *   This logic may be executing in ISRAM or in external mmemory: CS0, DDR,
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
  /* __start provided the basic MMU mappings for SRAM.  Now provide mappings
   * for all IO regions (Including the vector region).
   */

#ifndef CONFIG_ARCH_ROMPGTABLE
  sam_setupmappings();

  /* Provide a special mapping for the IRAM interrupt vector positioned in
   * high memory.
   */

#ifndef CONFIG_ARCH_LOWVECTORS
  sam_vectormapping();
#endif
#endif /* CONFIG_ARCH_ROMPGTABLE */

  /* Setup up vector block.  _vector_start and _vector_end are exported from
   * arm_vector.S
   */

  sam_copyvectorblock();

  /* Disable the watchdog timer */

  sam_wdtdisable();

  /* Initialize clocking to settings provided by board-specific logic */

  sam_clockconfig();   

  /* Initialize the FPU */

#ifdef CONFIG_ARCH_FPU
  arm_fpuconfig();
#endif

  /* Perform common, low-level chip initialization (might do nothing) */

  sam_lowsetup();

  /* Perform early serial initialization if we are going to use the serial
   * driver.
   */

#ifdef USE_EARLYSERIALINIT
  sam_earlyserialinit();
#endif

  /* For the case of the separate user-/kernel-space build, perform whatever
   * platform specific initialization of the user memory is required.
   * Normally this just means initializing the user space .data and .bss
   * segments.
   */

#ifdef CONFIG_NUTTX_KERNEL
  sam_userspace();
#endif

  /* Perform board-specific initialization */

  sam_boardinitialize();
}
