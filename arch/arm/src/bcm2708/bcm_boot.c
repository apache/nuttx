/****************************************************************************
 * arch/arm/src/bcm2708/bcm_boot.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#include "arm.h"
#include "cache.h"
#include "up_internal.h"
#include "up_arch.h"

#include "chip.h"
#include "bcm_config.h"
#include "bcm_clockconfig.h"
#include "bcm_memorymap.h"
#include "bcm_lowputc.h"
#include "bcm_serial.h"
#include "bcm_boot.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define PROGRESS(c) bcm_lowputc(c)
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
 * Name: bcm_set_level1entry
 *****************************************************************************/

static inline void bcm_set_level1entry(uint32_t paddr, uint32_t vaddr,
                                       uint32_t mmuflags)
{
  uint32_t *pgtable = (uint32_t *)PGTABLE_BASE_VADDR;
  uint32_t  index   = vaddr >> 20;

  /* Save the page table entry */

  pgtable[index]    = (paddr | mmuflags);
}

/****************************************************************************
 * Name: bcm_setupmappings
 *
 * Description:
 *   Map all of the initial memory regions defined in g_section_mapping[]
 *
 ****************************************************************************/

static inline void bcm_setupmappings(void)
{
  int i, j;

  for (i = 0; i < g_num_mappings; i++)
    {
      uint32_t sect_paddr = g_section_mapping[i].physbase;
      uint32_t sect_vaddr = g_section_mapping[i].virtbase;
      uint32_t mmuflags   = g_section_mapping[i].mmuflags;

      for (j = 0; j < g_section_mapping[i].nsections; j++)
        {
          bcm_set_level1entry(sect_paddr, sect_vaddr, mmuflags);
          sect_paddr += SECTION_SIZE;
          sect_vaddr += SECTION_SIZE;
        }
    }
}

/****************************************************************************
 * Name: bcm_vectorsize
 *
 * Description:
 *   Return the size of the vector data
 *
 ****************************************************************************/

static inline size_t bcm_vectorsize(void)
{
  uintptr_t src;
  uintptr_t end;

  src  = (uintptr_t)&_vector_start;
  end  = (uintptr_t)&_vector_end;

  return (size_t)(end - src);
}

/****************************************************************************
 * Name: bcm_copyvectorblock
 *
 * Description:
 *   Copy the interrupt block to its final destination.  Vectors are already
 *   positioned at the beginning of the text region and only need to be
 *   copied in the case where we are using high vectors or where the beginning
 *   of the text region cannot be remapped to address zero.
 *
 ****************************************************************************/

static void bcm_copyvectorblock(void)
{
  uint32_t *src;
  uint32_t *end;
  uint32_t *dest;

#ifdef CONFIG_PAGING
  /* If we are using re-mapped vectors in an area that has been marked
   * read only, then temporarily mark the mapping write-able (non-buffered).
   */

  bcm_vectorpermissions(MMU_L2_VECTRWFLAGS);
#endif

  /* Copy the vectors into SDRAM at the address that will be mapped to the
   * vector address:
   *
   *   BCM_VECTOR_PADDR  - Unmapped, physical address of vector table in SDRAM
   *   BCM_VECTOR_VSDRAM - Virtual address of vector table in SDRAM
   *   BCM_VECTOR_VADDR  - Virtual address of vector table (0x00000000 or
   *                       0xffff0000)
   */

  src  = (uint32_t *)&_vector_start;
  end  = (uint32_t *)&_vector_end;
  dest = (uint32_t *)BCM_VECTOR_VSDRAM;

  while (src < end)
    {
      *dest++ = *src++;
    }

  /* Flush the DCache to assure that the vector data is in physical RAM */

  cp15_flush_idcache((uint32_t)BCM_VECTOR_VSDRAM,
                     (uint32_t)BCM_VECTOR_VSDRAM + bcm_vectorsize());
}

/****************************************************************************
 * Name: bcm_wdtdisable
 *
 * Description:
 *   Disable the watchdog timer.  The BCM2708 always boots with the watchdog
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

#ifndef CONFIG_BCM2708_WDT
static inline void bcm_wdtdisable(void)
{
  /* REVISIT: WDT initialization */
}
#else
#  define bcm_wdtdisable()
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
 *   This logic may be executing in SDRAM or in external memory: CS0, DDR,
 *   CS1, CS2, or CS3.  It may be executing in CS0 or SDRAM through the
 *   action of the BCM2708 "first level bootloader;"  it might be executing in
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
 *       to internal SDRAM and started.
 *
 ****************************************************************************/

void arm_boot(void)
{
#if defined(CONFIG_ARCH_RAMFUNCS)
  const uint32_t *src;
#endif
#if defined(CONFIG_ARCH_RAMFUNCS)
  uint32_t *dest;
#endif

  /* __start provided the basic MMU mappings for SDRAM.  Now provide mappings
   * for all IO regions (Including the vector region).
   */

  bcm_setupmappings();
  PROGRESS('A');

#ifdef CONFIG_ARCH_RAMFUNCS
  /* Copy any necessary code sections from FLASH to RAM.  The correct
   * destination in SDRAM is given by _sramfuncs and _eramfuncs.  The
   * temporary location is in flash after the data initialization code
   * at _framfuncs
   */

  for (src = &_framfuncs, dest = &_sramfuncs; dest < &_eramfuncs; )
    {
      *dest++ = *src++;
    }

  PROGRESS('B');

  /* Flush the copied RAM functions into physical RAM so that will
   * be available when fetched into the I-Cache.
   */

  cp15_flush_idcache((uint32_t)&_sramfuncs, (uint32_t)&_eramfuncs)
  PROGRESS('C');
#endif

  /* Setup up vector block.  _vector_start and _vector_end are exported from
   * arm_vector.S
   */

  bcm_copyvectorblock();
  PROGRESS('D');

  /* Disable the watchdog timer */

  bcm_wdtdisable();
  PROGRESS('E');

  /* Initialize clocking to settings provided by board-specific logic */

  bcm_clockconfig();
  PROGRESS('F');

#ifdef CONFIG_ARCH_FPU
  /* Initialize the FPU */

  arm_fpuconfig();
  PROGRESS('G');
#endif

  /* Perform board-specific memroy initialization,  This must include
   * initialization of board-specific memory resources (e.g., SDRAM)
   *
   * NOTE: We must use caution prior to this point to make sure that
   * the logic does not access any global variables that might lie
   * in SDRAM.
   */

  bcm_memory_initialize();
  PROGRESS('H');

#ifdef NEED_SDRAM_REMAPPING
  /* SDRAM was configured in a temporary state to support low-level
   * initialization.  Now that the SDRAM has been fully initialized,
   * we can reconfigure the SDRAM in its final, fully cache-able state.
   */

  bcm_remap();
  PROGRESS('I');
#endif

#ifdef CONFIG_BOOT_SDRAM_DATA
  /* If .data and .bss reside in SDRAM, then initialize the data sections
   * now after SDRAM has been initialized.
   */

  arm_data_initialize();
  PROGRESS('J');
#endif

  /* Perform board-specific device initialization. This would include
   * configuration of board specific resources such as GPIOs, LEDs, etc.
   */

  bcm_board_initialize();
  PROGRESS('K');

  /* Perform common, low-level chip initialization (might do nothing) */

  bcm_lowsetup();
  PROGRESS('L');

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization if we are going to use the serial
   * driver.
   */

  bcm_earlyserialinit();
  PROGRESS('M');
#endif
  PROGRESS('\n');
}
