/****************************************************************************
 * arch/arm/src/rm57/rm57_boot.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/init.h>
#include <stdint.h>
#include <stdbool.h>

#include <arch/board/board.h>

#include "arm.h"
#include "arm_internal.h"
#include "sctlr.h"

#include "hardware/rm57_sys.h"
#include "rm57_boot.h"
#include "rm57_clockconfig.h"
#include "rm57_esm.h"
#include "rm57_gio.h"
#include "rm57_iomm.h"
#include "rm57_lowputc.h"
#include "rm57_mpuinit.h"
#include "rm57_selftest.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ARMV7R_MEMINIT
#error CONFIG_ARMV7R_MEMINIT is required by this architecture.
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_event_export
 *
 * Description:
 *   Enable CPU Event Export by setting the X bit in the PMCR.  This
 *   allows the CPU to signal any single-bit or double-bit errors
 *   detected by its ECC logic for accesses to program flash or data RAM.
 *   via TMS570's event bus to the ESM (Error Signaling Module).
 *
 ****************************************************************************/

static inline void rm57_event_export(void)
{
  cp15_pmu_pmcr(PMCR_X);
}

/****************************************************************************
 * Name: rm57_memory_initialize
 *
 * Description:
 *   Hardware-initialize (ECC-clear) the on-chip SRAM before .bss/.data
 *   are written to it.  Ported from TI's HALCoGen _memInit_ assembly
 *   routine to C using NuttX register macros.
 *
 ****************************************************************************/

static void rm57_memory_initialize(void)
{
  /* Enable Memory Hardware Initialization */

  putreg32(SYS_MINITGCR_ENABLE, RM57_SYS_MINITGCR);

  /* Enable Memory Hardware Initialization for main SRAM (bit 0) */

  putreg32(1, RM57_SYS_MSINENA);

  /* Wait until Memory Hardware Initialization completes */

  while ((getreg32(RM57_SYS_MSTCGSTAT) & SYS_MSTCGSTAT_MINIDONE) == 0)
    {
    }

  /* Disable Memory Hardware Initialization */

  putreg32(SYS_MINITGCR_DISABLE, RM57_SYS_MINITGCR);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rm57_boot_c
 *
 * Description:
 *   Complete boot operations started in arm_head.S and arm_boot
 *
 * Boot Sequence
 *
 *   1.  The __start entry point in armv7-r/arm_head.S is invoked upon
 *       reset and prepares the CPU for code execution.
 *   2.  It branches to arm_boot (rm57_coreinit.S), which gives every CPU
 *       register a defined value so that the two lockstep cores cannot
 *       disagree, and then branches here.
 *   3.  Because CONFIG_ARMV7R_MEMINIT is set, this function runs *before*
 *       .bss/.data have been initialized - it must not touch any
 *       global/static data until arm_data_initialize() has run.
 *   4.  This function configures clocking, hardware-initializes SRAM
 *       (clearing ECC), then calls arm_data_initialize().
 *   5.  Only after that can the MPU and board-specific bring-up run.
 *   6.  This function then branches to nx_start() to start the OS.
 *
 * CPU Reset
 *
 *   The CPU self-test (STC) ends in a reset of the CPU alone, and software
 *   can cause one through CPURSTCR.  Everything outside the CPU keeps its
 *   state across such a reset: the PLLs stay locked and remain the clock
 *   source, the peripherals stay configured and SRAM keeps its contents.
 *   The clock and memory set-up must then be skipped - rm57_clockconfig()
 *   would try to switch off the very PLL the device is running from.
 *
 *   With CONFIG_RM57_SELFTEST the CPU self-test is part of every boot, so
 *   a normal boot passes through here three times: once to start the STC
 *   self-check, once to start the real run, and once to carry on.  See
 *   rm57_selftest.c.
 *
 ****************************************************************************/

void rm57_boot_c(void)
{
#ifdef CONFIG_RM57_SELFTEST
  uint32_t sysesr;
  uint32_t results;
  bool cpureset;
#endif

  /* Enable CPU Event Export */

  rm57_event_export();

  /* Find out why we are here.  The CPU reset flag is sticky, so it alone
   * is not proof that the last reset was CPU-only: a system reset in the
   * meantime would have put the clock tree back on the oscillator.  Only
   * trust it while the PLL is still selected.
   */

  if ((getreg32(RM57_SYS_SYSESR) & SYS_SYSESR_CPURST_ALL) != 0 &&
      (getreg32(RM57_SYS_GHVSRC) & SYS_GHVSRC_GHVSRC_MASK) ==
      SYS_GHVSRC_GHVSRC_SRC(SYS_CLKSRC_PLL1))
    {
      putreg32(SYS_SYSESR_CPURST_ALL, RM57_SYS_SYSESR);

#ifdef CONFIG_RM57_SELFTEST
      sysesr   = getreg32(RM57_SYS_SYSESR) | SYS_SYSESR_CPURST;
      cpureset = true;
#endif
    }
  else
    {
      /* Initialize clocking (PLL, peripheral clocks, flash wait-states) */

      rm57_clockconfig();

      rm57_iomm_initialize();

      /* Put the Error Signaling Module into a known, quiet state (all
       * error-pin channels/interrupts disabled, nERROR pin released).
       */

      rm57_esm_initialize();

      /* Hardware-initialize (ECC-clear) SRAM before .bss/.data are
       * touched.  This wipes the stack as well, so nothing this function
       * needs later may be held in a local variable across the call -
       * hence the reset cause is only read afterwards.
       */

      rm57_memory_initialize();

#ifdef CONFIG_RM57_SELFTEST
      sysesr   = getreg32(RM57_SYS_SYSESR);
      cpureset = false;
#endif
    }

#ifdef CONFIG_RM57_SELFTEST
  /* ROM tests and CPU self-test.  Does not return on the passes that end
   * in a CPU reset.
   */

  results = rm57_selftest_early(sysesr, cpureset);
#endif

  /* Initialize the FPU (RM57L843 is Cortex-R5F) */

  arm_fpuconfig();

  /* Initialize .bss and .data now that SRAM has been initialized */

  arm_data_initialize();

#ifdef CONFIG_RM57_SELFTEST
  /* The remaining diagnostics, which need nothing from RAM beyond what
   * is now valid: CCM-R5F, eFuse controller, L2RAMW.
   */

  rm57_selftest_late(sysesr, results);
#endif

  /* Configure the MPU regions (flash, SRAM, peripherals) */

  rm57_mpu_init();

  /* Initialize GIO for use by board initialization logic */

  rm57_gio_initialize();

  /* Perform board-specific initialization */

  rm57_board_initialize();

#ifdef CONFIG_ARCH_PERF_EVENTS
  /* Start the Cortex-R5 PMU cycle counter, the only clock on this part
   * with a resolution below the 10 ms system tick.  It counts GCLK (the
   * CPU clock, BOARD_PLL_FREQUENCY), not HCLK.
   */

  up_perf_init((void *)BOARD_PLL_FREQUENCY);
#endif

  /* Bring up the console SCI (RX/TX pin function, baud/format/GCR1).
   * This must run before nx_start() - without it the SCI module is left
   * in its power-on-reset state (module held in reset, RX/TX pin
   * function bits unset) and produces no output even though register
   * writes to it "succeed" from software's point of view.
   */

  rm57_lowsetup();

#ifdef CONFIG_RM57_SELFTEST
  rm57_selftest_report();
#endif

  /* Then start NuttX */

  nx_start();
}
