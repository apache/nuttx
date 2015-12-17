/****************************************************************************
 * arch/arm/src/tms570/tms570_boot.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This is all original code.  However, some logic in this file was inspired
 * by logic from TI's Project0 which has a compatible BSD license and credit
 * should be given in any case:
 *
 *   Copyright (c) 2012, Texas Instruments Incorporated
 *   All rights reserved.
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

#include <arch/board/board.h>

#include "chip.h"
#include "arm.h"
#include "cache.h"
#include "fpu.h"
#include "up_internal.h"
#include "up_arch.h"

#include "tms570_boot.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ARMV7R_MEMINIT
#  error CONFIG_ARMV7R_MEMINIT is required by this architecture.
#endif

#define HIGH_VECTOR_ADDRESS   0xffff0000

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
 * Name: tms570_vectorsize
 *
 * Description:
 *   Return the size of the vector data
 *
 ****************************************************************************/

static inline size_t tms570_vectorsize(void)
{
  uintptr_t src;
  uintptr_t end;

  src  = (uintptr_t)&_vector_start;
  end  = (uintptr_t)&_vector_end;

  return (size_t)(end - src);
}

/****************************************************************************
 * Name: tms570_copyvectorblock
 *
 * Description:
 *   Copy the interrupt block to its final destination.  Vectors are already
 *   positioned at the beginning of the text region and only need to be
 *   copied in the case where we are using high vectors or where the beginning
 *   of the text region cannot be remapped to address zero.
 *
 ****************************************************************************/

#if !defined(CONFIG_ARCH_LOWVECTORS)
static void tms570_copyvectorblock(void)
{
  uint32_t *src;
  uint32_t *end;
  uint32_t *dest;

  /* Copy the vectors into ISRAM at the address that will be mapped to the vector
   * address:
   *
   *   _vector_start       - Start sourcea ddress of the vector table
   *   _vector_end         - End+1 source address of the vector table
   *   HIGH_VECTOR_ADDRESS - Destinatino ddress of vector table in RAM
   */

  src  = (uint32_t *)&_vector_start;
  end  = (uint32_t *)&_vector_end;
  dest = (uint32_t *)HIGH_VECTOR_ADDRESS;

  while (src < end)
    {
      *dest++ = *src++;
    }

  /* Flush the DCache to assure that the vector data is in physical in RAM */

  arch_clean_dcache((uintptr_t)HIGH_VECTOR_ADDRESS,
                    (uintptr_t)HIGH_VECTOR_ADDRESS + tms570_vectorsize());
}

#else
/* Don't copy the vectors */

#  define tms570_copyvectorblock()
#endif

/****************************************************************************
 * Name: tms570_wdtdisable
 *
 * Description:
 *   Disable the watchdog timer.
 *
 ****************************************************************************/

#ifndef CONFIG_TMS570_WDT
static inline void tms570_wdtdisable(void)
{
#warning Missing logic
}
#else
#  define tms570_wdtdisable()
#endif

/****************************************************************************
 * Name: tms570_event_export
 *
 * Description:
 *   Enable CPU Event Export by setting the X bit in the PMCR.  In general,
 *   this bit enables the exporting of events to another debug device, such
 *   as a trace macrocell, over an event bus.
 *
 *   For the TMS570, this allows the CPU to signal any single-bit or double
 *   -bit errors detected by its ECC logic for accesses to program flash or
 *   data RAM.
 *
 ****************************************************************************/

static inline void tms570_event_export(void)
{
  uint32_t pmcr = cp15_rdpmcr();
  pmcr |= PCMR_X;
  cp15_wrpmcr(pmcr)
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
 * Boot Sequence
 *
 *   1.  The __start entry point in armv7-r/arm_head.S is invoked upon power-
 *       on reset.
 *   2.  __start prepares CPU for code execution.
 *   3a. If CONFIG_ARMV7R_MEMINIT is not defined, then __start will prepare
 *       memory resources by calling arm_data_initialize() and will then
 *       call this function.
 *   3b. Otherwise, this function will be called without having initialized
 *       memory resources!  We need to be very careful in this case.  Here,
 *       this function will call tms570_boardinitialize() which, among other
 *       things, much initialize SDRAM memory.  Upon return, this function
 *       will call arm_data_initialize() to initialize the memory resources
 *   4. This function will initialize all TMS570-specific resources and
 *      return to __start.
 *   4. _start will then branch to os_start() to start the operating system.
 *
 ****************************************************************************/

void arm_boot(void)
{
#ifdef CONFIG_ARCH_RAMFUNCS
  const uint32_t *src;
  uint32_t *dest;
#endif

  /* Enable CPU Event Export.
   *
   * This allows the CPU to signal any single-bit or double-bit errors
   * detected by its ECC logic for accesses to program flash or data RAM.
   */

  tms570_event_export();

  /* Read from the system exception status register to identify the cause of
   * the CPU reset.
   *
   * REVISIT: This logic is not used in the current design.  But if you
   * need to know the cause of the reset, here is where you would want
   * to do that.
   */

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

  arch_clean_dcache((uintptr_t)&_sramfuncs, (uintptr_t)&_eramfuncs)
#endif

  /* Setup up vector block.  _vector_start and _vector_end are exported from
   * arm_vector.S
   */

  tms570_copyvectorblock();

  /* Disable the watchdog timer */

  tms570_wdtdisable();

  /* Initialize clocking to settings provided by board-specific logic */

  tms570_clockconfig();

#ifdef CONFIG_ARCH_FPU
  /* Initialize the FPU */

  arm_fpuconfig();
#endif

  /* Perform board-specific initialization,  This must include:
   *
   * - Initialization of board-specific memory resources (e.g., SDRAM)
   * - Configuration of board specific resources (GPIOs, LEDs, etc).
   *
   * NOTE: We must use caution prior to this point to make sure that
   * the logic does not access any global variables that might lie
   * in SDRAM.
   */

  tms570_board_initialize();

#ifdef CONFIG_ARMV7R_MEMINIT
  /* If .data and .bss reside in SDRAM, then initialize the data sections
   * now after RAM has been initialized.
   */

  arm_data_initialize();
#endif

  /* Perform common, low-level chip initialization (might do nothing) */

  tms570_lowsetup();

#ifdef USE_EARLYSERIALINIT
  /* Perform early serial initialization if we are going to use the serial
   * driver.
   */

  tms570_earlyserialinit();
#endif
}
