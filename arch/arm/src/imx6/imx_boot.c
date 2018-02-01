/****************************************************************************
 * arch/arm/src/imx6/imx_boot.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#ifdef CONFIG_PAGING
#  include <nuttx/page.h>
#endif

#include <arch/board/board.h>

#include "chip.h"
#include "arm.h"
#include "mmu.h"
#include "scu.h"
#include "cache.h"
#include "fpu.h"
#include "up_internal.h"
#include "up_arch.h"

#include "imx_config.h"
#include "imx_clockconfig.h"
#include "imx_memorymap.h"
#include "imx_lowputc.h"
#include "imx_serial.h"
#include "imx_boot.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define PROGRESS(c) imx_lowputc(c)
#else
#  define PROGRESS(c)
#endif

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
 * Name: imx_setupmappings
 *
 * Description:
 *   Map all of the initial memory regions defined in g_section_mapping[]
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
static inline void imx_setupmappings(void)
{
  int i;

  /* Set up each group of section mappings */

  for (i = 0; i < g_num_mappings; i++)
    {
      mmu_l1_map_region(&g_section_mapping[i]);
    }
}
#else
#  define imx_setupmappings()
#endif

/****************************************************************************
 * Name: imx_remap
 *
 * Description:
 *   Map all of the final memory regions defined in g_operational_mapping[]
 *
 ****************************************************************************/

#ifdef NEED_SDRAM_REMAPPING
static inline void imx_remap(void)
{
  int i;

  /* Re-map each group of section */

  for (i = 0; i < g_num_opmappings; i++)
    {
      mmu_l1_map_region(&g_operational_mapping[i]);
    }
}
#endif

/****************************************************************************
 * Name: imx_vectorpermissions
 *
 * Description:
 *   Set permissions on the vector mapping.
 *
 ****************************************************************************/

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_LOWVECTORS) && \
     defined(CONFIG_PAGING)
static void imx_vectorpermissions(uint32_t mmuflags)
{
  /* The PTE for the beginning of OCRAM is at the base of the L2 page table */

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
 * Name: imx_vectorsize
 *
 * Description:
 *   Return the size of the vector data
 *
 ****************************************************************************/

static inline size_t imx_vectorsize(void)
{
  uintptr_t src;
  uintptr_t end;

  src  = (uintptr_t)&_vector_start;
  end  = (uintptr_t)&_vector_end;

  return (size_t)(end - src);
}

/****************************************************************************
 * Name: imx_vectormapping
 *
 * Description:
 *   Setup a special mapping for the interrupt vectors when the interrupt
 *   vectors are located at the high address, 0xffff0000.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_LOWVECTORS
static void imx_vectormapping(void)
{
  uint32_t vector_paddr = IMX_VECTOR_PADDR & PTE_SMALL_PADDR_MASK;
  uint32_t vector_vaddr = IMX_VECTOR_VADDR & PTE_SMALL_PADDR_MASK;
  uint32_t vector_size  = (uint32_t)&_vector_end - (uint32_t)&_vector_start;
  uint32_t end_paddr    = IMX_VECTOR_PADDR + vector_size;

  /* REVISIT:  Cannot really assert in this context */

  DEBUGASSERT (vector_size <= VECTOR_TABLE_SIZE);

  /* We want to keep our interrupt vectors and interrupt-related logic in
   * on-chip RAM (OCRAM).  The i.MX6 has 256Kb of OCRAM positioned at
   * physical address 0x0090:0000; we need to map this to 0xffff:0000.
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
                  IMX_VECTOR_VADDR & PMD_PTE_PADDR_MASK,
                  MMU_L1_VECTORFLAGS);
}
#else
  /* No vector remap */

#  define imx_vectormapping()
#endif

/****************************************************************************
 * Name: imx_copyvectorblock
 *
 * Description:
 *   Copy the interrupt block to its final destination.  Vectors are already
 *   positioned at the beginning of the text region and only need to be
 *   copied in the case where we are using high vectors or where the beginning
 *   of the text region cannot be remapped to address zero.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_LOWVECTORS
static void imx_copyvectorblock(void)
{
  uint32_t *src;
  uint32_t *end;
  uint32_t *dest;

#ifdef CONFIG_PAGING
  /* If we are using re-mapped vectors in an area that has been marked
   * read only, then temporarily mark the mapping write-able (non-buffered).
   */

  imx_vectorpermissions(MMU_L2_VECTRWFLAGS);
#endif

  /* Copy the vectors into OCRAM at the address that will be mapped to the vector
   * address:
   *
   *   IMX_VECTOR_PADDR - Unmapped, physical address of vector table in OCRAM
   *   IMX_VECTOR_VSRAM - Virtual address of vector table in OCRAM
   *   IMX_VECTOR_VADDR - Virtual address of vector table (0x00000000 or
   *                      0xffff0000)
   */

  src  = (uint32_t *)&_vector_start;
  end  = (uint32_t *)&_vector_end;
  dest = (uint32_t *)IMX_VECTOR_VSRAM;

  while (src < end)
    {
      *dest++ = *src++;
    }

#if !defined(CONFIG_ARCH_LOWVECTORS) && defined(CONFIG_PAGING)
  /* Make the vectors read-only, cacheable again */

  imx_vectorpermissions(MMU_L2_VECTORFLAGS);

#else
  /* Flush the DCache to assure that the vector data is in physical RAM */

  arch_clean_dcache((uintptr_t)IMX_VECTOR_VSRAM,
                    (uintptr_t)IMX_VECTOR_VSRAM + imx_vectorsize());
#endif
}

#else
/* Don't copy the vectors */

#  define imx_copyvectorblock()
#endif

/****************************************************************************
 * Name: imx_wdtdisable
 *
 * Description:
 *   Disable the watchdog timer.  The i.MX6 always boots with the watchdog
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

#ifndef CONFIG_IMX6_WDT
static inline void imx_wdtdisable(void)
{
  /* REVISIT: WDT initialization */
}
#else
#  define imx_wdtdisable()
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
 *   This logic may be executing in OCRAM or in external memory: CS0, DDR,
 *   CS1, CS2, or CS3.  It may be executing in CS0 or OCRAM through the
 *   action of the i.MX6 "first level bootloader;"  it might be executing in
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
 *       to internal OCRAM and started.
 *
 ****************************************************************************/

void arm_boot(void)
{
#if defined(CONFIG_ARCH_RAMFUNCS)
  const uint32_t *src;
#endif
#if defined(CONFIG_ARCH_RAMFUNCS) || defined(CONFIG_SMP) && defined(SMP_INTERCPU_NONCACHED)
  uint32_t *dest;
#endif

  /* __start provided the basic MMU mappings for OCRAM.  Now provide mappings
   * for all IO regions (Including the vector region).
   */

  imx_setupmappings();
  PROGRESS('A');

  /* Make sure that all other CPUs are in the disabled state.  This is a
   * formality because the other CPUs are actually running then we have
   * probably already crashed.
   */

  imx_cpu_disable();
  PROGRESS('B');

#ifdef CONFIG_SMP
  /* Enable SMP cache coherency for CPU0 */

  arm_enable_smp(0);
  PROGRESS('C');
#endif

  /* Provide a special mapping for the OCRAM interrupt vector positioned in
   * high memory.
   */

  imx_vectormapping();
  PROGRESS('D');

#ifdef CONFIG_ARCH_RAMFUNCS
  /* Copy any necessary code sections from FLASH to RAM.  The correct
   * destination in OCRAM is given by _sramfuncs and _eramfuncs.  The
   * temporary location is in flash after the data initialization code
   * at _framfuncs
   */

  for (src = &_framfuncs, dest = &_sramfuncs; dest < &_eramfuncs; )
    {
      *dest++ = *src++;
    }

  PROGRESS('E');

  /* Flush the copied RAM functions into physical RAM so that will
   * be available when fetched into the I-Cache.
   */

  arch_clean_dcache((uintptr_t)&_sramfuncs, (uintptr_t)&_eramfuncs)
  PROGRESS('F');
#endif

  /* Setup up vector block.  _vector_start and _vector_end are exported from
   * arm_vector.S
   */

  imx_copyvectorblock();
  PROGRESS('G');

  /* Disable the watchdog timer */

  imx_wdtdisable();
  PROGRESS('H');

  /* Initialize clocking to settings provided by board-specific logic */

  imx_clockconfig();
  PROGRESS('I');

#ifdef CONFIG_ARCH_FPU
  /* Initialize the FPU */

  arm_fpuconfig();
  PROGRESS('J');
#endif

  /* Perform board-specific memroy initialization,  This must include
   * initialization of board-specific memory resources (e.g., SDRAM)
   *
   * NOTE: We must use caution prior to this point to make sure that
   * the logic does not access any global variables that might lie
   * in SDRAM.
   */

  imx_memory_initialize();
  PROGRESS('K');

#ifdef NEED_SDRAM_REMAPPING
  /* SDRAM was configured in a temporary state to support low-level
   * initialization.  Now that the SDRAM has been fully initialized,
   * we can reconfigure the SDRAM in its final, fully cache-able state.
   */

  imx_remap();
  PROGRESS('L');
#endif

#ifdef CONFIG_BOOT_SDRAM_DATA
  /* If .data and .bss reside in SDRAM, then initialize the data sections
   * now after SDRAM has been initialized.
   */

  arm_data_initialize();
  PROGRESS('M');
#endif

  /* Perform board-specific device initialization. This would include
   * configuration of board specific resources such as GPIOs, LEDs, etc.
   */

  imx_board_initialize();
  PROGRESS('N');

  /* Perform common, low-level chip initialization (might do nothing) */

  imx_lowsetup();
  PROGRESS('O');

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization if we are going to use the serial
   * driver.
   */

  imx_earlyserialinit();
  PROGRESS('P');
#endif

  /* Now we can enable all other CPUs.  The enabled CPUs will start execution
   * at __cpuN_start and, after very low-level CPU initialzation has been
   * performed, will branch to arm_cpu_boot() (see arch/arm/src/armv7-a/smp.h)
   */

  imx_cpu_enable();
  PROGRESS('Q');
  PROGRESS('\n');
}
