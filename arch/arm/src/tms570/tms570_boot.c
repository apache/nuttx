/****************************************************************************
 * arch/arm/src/tms570/tms570_boot.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This is primarily original code.  However, some logic in this file was
 * inspired/leveraged from TI's Project0 which has a compatible BSD license
 * and credit should be given in any case:
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
#include "fpu.h"
#include "sctlr.h"
#include "arm_internal.h"
#include "arm_arch.h"

#include <nuttx/init.h>

#include "hardware/tms570_sys.h"
#include "hardware/tms570_esm.h"
#include "hardware/tms570_pbist.h"
#include "tms570_clockconfig.h"
#include "tms570_selftest.h"
#include "tms570_gio.h"
#include "tms570_esm.h"
#include "tms570_boot.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ARMV7R_MEMINIT
#  error CONFIG_ARMV7R_MEMINIT is required by this architecture.
#endif

#ifndef CONFIG_ARCH_LOWVECTORS
#  error CONFIG_ARCH_LOWVECTORS is required by this architecture.
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
  cp15_wrpmcr(pmcr);
}

/****************************************************************************
 * Name: tms570_check_reset
 *
 * Description:
 *   Assert if we go here through any mechanism other than a power-on reset.
 *
 ****************************************************************************/

static inline void tms570_check_reset(void)
{
#ifdef CONFIG_DEBUG_FEATURES
  uint32_t regval;

  /* Read from the system exception status register to identify the cause of
   * the CPU reset.
   */

  regval = getreg32(TMS570_SYS_ESR);

  /* Clear all reset status flags on normal reset */

  regval = getreg32(TMS570_SYS_ESR);
  putreg32(SYS_ESR_RSTALL, TMS570_SYS_ESR);

  /* Check for abnormal reset causes:  Oscillator failures or watchdog
   * timers. Ignore normal reset causes: External reset, software reset, CPU
   * reset, power-on reset
   *
   * REVISIT: The reset cause is not used in the current design.  But if you
   * need to know the cause of the reset, here is where you would want to
   * do that.
   */

#if 0
  DEBUGASSERT((regval & SYS_ESR_FAILALL) == 0);
#else
  UNUSED(regval);
#endif

#else
  /* Clear all reset status flags */

  putreg32(SYS_ESR_RSTALL, TMS570_SYS_ESR);
#endif
}

/****************************************************************************
 * Name: tms570_enable_ramecc
 *
 * Description:
 *   This function enables the CPU's ECC logic for accesses to B0TCM and
 *   B1TCM.
 *
 ****************************************************************************/

static inline void tms570_enable_ramecc(void)
{
  uint32_t actlr = cp15_rdactlr();
  actlr |= 0x0c000000;
  cp15_wractlr(actlr);
}

/****************************************************************************
 * Name: tms570_memory_initialize
 *
 * Description:
 *   Perform memory initialization of selected RAMs
 *
 *   This function uses the system module's hardware for auto-initialization
 *   of memories and their associated protection schemes.
 *
 ****************************************************************************/

static void tms570_memory_initialize(uint32_t ramset)
{
  /* Enable Memory Hardware Initialization */

  putreg32(SYS_MINITGCR_ENABLE, TMS570_SYS_MINITGCR);

  /* Enable Memory Hardware Initialization for selected RAM's */

  putreg32(ramset, TMS570_SYS_MSIENA);

  /* Wait until Memory Hardware Initialization complete */

  while ((getreg32(TMS570_SYS_MSTCGSTAT) & SYS_MSTCGSTAT_MINIDONE) == 0);

  /* Disable Memory Hardware Initialization */

  putreg32(SYS_MINITGCR_DISABLE, TMS570_SYS_MINITGCR);
}

/****************************************************************************
 * Name: go_nx_start
 *
 * Description:
 *   Re-initialize the stack and frame pointers and branch to OS start.
 *
 ****************************************************************************/

#ifdef CONFIG_STACK_COLORATION
static void go_nx_start(void *pv, unsigned int nbytes)
  naked_function noreturn_function;

static void go_nx_start(void *pv, unsigned int nbytes)
{
  /* Set the IDLE stack to the stack coloration value then jump to
   * nx_start().  We take extreme care here because were currently
   * executing on this stack.
   *
   * We want to avoid sneak stack access generated by the compiler.
   */

  __asm__ __volatile__
  (
    "\tmovs r1, r1, lsr #2\n"       /* R1 = nwords = nbytes >> 2 */
    "\tbeq  2f\n"                   /* (should not happen) */

    "\tbic  r0, r0, #3\n"           /* R0 = Aligned stackptr */
    "\tmovw r2, #0xbeef\n"          /* R2 = STACK_COLOR = 0xdeadbeef */
    "\tmovt r2, #0xdead\n"

    "1:\n"                          /* Top of the loop */
    "\tsub  r1, r1, #1\n"           /* R1 nwords-- */
    "\tcmp  r1, #0\n"               /* Check (nwords == 0) */
    "\tstr  r2, [r0], #4\n"         /* Save stack color word, increment stackptr */
    "\tbne  1b\n"                   /* Bottom of the loop */

    "2:\n"
    "\tldr  ip, =g_idle_topstack\n" /* IP=address of g_idle_topstack */
    "\tldr  sp, [ip]\n"             /* Reset the stack pointer */
    "\tmov  fp, #0\n"               /* Reset the frame pointer */
    "\tmov  r14, #0\n"              /* LR = return address (none) */
    "\tb    nx_start\n"             /* Branch to nx_start */
  );
}

#else
static void go_nx_start(void) naked_function noreturn_function;

static void go_nx_start(void)
{
  /* Reset the stack/frame pointer and jump to nx_start(). */

  __asm__ __volatile__
  (
    "\tldr  ip, =g_idle_topstack\n" /* IP=address of g_idle_topstack */
    "\tldr  sp, [ip]\n"             /* Reset the stack pointer */
    "\tmov  fp, #0\n"               /* Reset the frame pointer */
    "\tmov  r14, #0\n"              /* LR = return address (none) */
    "\tb    nx_start\n"             /* Branch to nx_start */
  );
}
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
 *   1.  The __start entry point in armv7-r/arm_head.S is invoked upon power-
 *       on reset.
 *   2.  __start prepares CPU for code execution.
 *   3a. If CONFIG_ARMV7R_MEMINIT is not defined, then __start will prepare
 *       memory resources by calling arm_data_initialize() and will then
 *       branch this function.
 *   3b. Otherwise, this function will be called without having initialized
 *       memory resources!  We need to be very careful in this case.  Here,
 *       this function will call tms570_boardinitialize() which, among other
 *       things, must initialize SDRAM memory.  After initializatino of the
 *       memories, this function will call arm_data_initialize() to
 *       initialize the memory resources
 *   4.  This function will then branch to nx_start() to start the operating
 *       system.
 *
 ****************************************************************************/

void arm_boot(void)
{
#ifdef CONFIG_TMS570_SELFTEST
  int check;
#endif /* CONFIG_TMS570_SELFTEST */

  /* Enable CPU Event Export.
   *
   * This allows the CPU to signal any single-bit or double-bit errors
   * detected by its ECC logic for accesses to program flash or data RAM.
   */

  tms570_event_export();

  /* Verify that we got here via a power-up reset */

  tms570_check_reset();

  /* Check if there were ESM group3 errors during power-up.
   *
   * These could occur during eFuse auto-load or during reads from flash OTP
   * during power-up. Device operation is not reliable and not recommended
   * in this case.
   *
   * An ESM group3 error only drives the nERROR pin low. An external circuit
   * that monitors the nERROR pin must take the appropriate action to ensure
   * that the system is placed in a safe state, as determined by the
   * application.
   */

  DEBUGASSERT(getreg32(TMS570_ESM_SR3) == 0);

  /* Initialize clocking to settings provided by board-specific logic */

  tms570_clockconfig();

#ifdef CONFIG_TMS570_SELFTEST
  /* Run a diagnostic check on the memory self-test controller.
   *
   * REVISIT: This is a destructive test.  It will most likely clobber the
   * current stack content and result in a failure if this function were to
   * attempt to return.
   */

  tms570_memtest_selftest();

  /* Run the memory selftest on CPU RAM. */

  tms570_memtest_start(PBIST_RINFOL_ESRAM1_RAM);
  check = tms570_memtest_complete();
  DEBUGASSERT(check == OK);
#endif /* CONFIG_TMS570_SELFTEST */

  /* Initialize CPU RAM. */

  tms570_memory_initialize(SYS_MSIENA_RAM);

  /* Enable ECC checking for TCRAM accesses. */

  tms570_enable_ramecc();

#ifdef CONFIG_TMS570_SELFTEST
  /* Perform PBIST on all dual-port memories */

  tms570_memtest_start(PBIST_RINFOL_VIM_RAM
#ifdef CONFIG_TMS570_DCAN1
                       | PBIST_RINFOL_DCAN1_RAM
#endif
#ifdef CONFIG_TMS570_DCAN2
                       | PBIST_RINFOL_DCAN2_RAM
#endif
#ifdef CONFIG_TMS570_MIBASPI1
                       | PBIST_RINFOL_MIBSPI1_RAM
#endif
#ifdef CONFIG_TMS570_MIBASPI1
                       | PBIST_RINFOL_MIBADC_RAM
#endif
#ifdef CONFIG_TMS570_N2HET
                       | PBIST_RINFOL_N2HET_RAM
                       | PBIST_RINFOL_HET_TU_RAM
#endif
                      );

  /* Test the CPU ECC mechanism for RAM accesses. */

  tms570_cpuecc_selftest();

  /* Wait for the memory test to complete */

  check = tms570_memtest_complete();
  DEBUGASSERT(check == OK);
  UNUSED(check);
#endif /* CONFIG_TMS570_SELFTEST */

#ifdef CONFIG_TMS570_MIBASPI1
  /* Release the MibSPI1 modules from local reset.
   *
   * This will cause the MibSPI1 RAMs to be initialized along with the
   * parity memory.
   */

  putreg32(MIBSPI_GCR0_RESET, TMS570_MIBSPI_GCR0);
#endif

  /* Initialize all on-chip SRAMs except for MibSPIx RAMs.
   *
   * The MibSPIx modules have their own auto-initialization mechanism which
   * is triggered as soon as the modules are brought out of local reset.
   *
   * The system module auto-init will hang on the MibSPI RAM if the module
   * is still in local reset.
   */

  tms570_memory_initialize(SYS_MSIENA_VIM_RAM
#ifdef CONFIG_TMS570_N2HET
                           | SYS_MSIENA_N2HET_RAM | SYS_MSIENA_HTU_RAM
#endif
#ifdef CONFIG_TMS570_DCAN1
                           | SYS_MSIENA_DCAN1_RAM
#endif
#ifdef CONFIG_TMS570_DCAN2
                           | SYS_MSIENA_DCAN2_RAM
#endif
#ifdef CONFIG_TMS570_MIBADC
                           | SYS_MSIENA_MIBADC_RAM
#endif
    );

#ifdef CONFIG_TMS570_SELFTEST
  /* Test the parity protection mechanism for peripheral RAMs */

#warning Missing logic
#endif

#ifdef CONFIG_TMS570_MIBASPI1
  /* Wait for MibSPI1 RAM to complete initialization */

#warning Missing logic
#endif

  /* Configure system response to error conditions */

  tms570_esm_initialize();

#ifdef CONFIG_ARCH_FPU
  /* Initialize the FPU */

  arm_fpuconfig();
#endif

#ifdef CONFIG_ARMV7R_MEMINIT
  /* Initialize the .bss and .data sections as well as RAM functions
   * now after RAM has been initialized.
   *
   * NOTE that if SDRAM were supported, this call might have to be
   * performed after returning from tms570_board_initialize()
   */

  arm_data_initialize();
#endif

  /* Initialize GIO for use by board initialization logic */

  tms570_gio_initialize();

  /* Perform board-specific initialization,  This must include:
   *
   * - Initialization of board-specific memory resources (e.g., SDRAM)
   * - Configuration of board specific resources (GIOs, LEDs, etc).
   *
   * NOTE: We must use caution prior to this point to make sure that
   * the logic does not access any global variables that might lie
   * in SDRAM.
   */

  tms570_board_initialize();

  /* Perform common, low-level chip initialization (might do nothing) */

  tms570_lowsetup();

  /* Then start NuttX */

#ifdef CONFIG_STACK_COLORATION
  /* Set the IDLE stack to the coloration value and jump into nx_start() */

  go_nx_start((FAR void *)&_ebss, CONFIG_IDLETHREAD_STACKSIZE);
#else
  /* Branch to nx_start(), resetting the stack and frame pointers. */

  go_nx_start();
#endif
}
