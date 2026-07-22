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

#include <arch/board/board.h>

#include "arm.h"
#include "arm_internal.h"
#include "sctlr.h"

#include "hardware/rm57_sys.h"
#include "rm57_boot.h"
#include "rm57_clockconfig.h"
#include "rm57_esm.h"
#include "rm57_gio.h"
#include "rm57_lowputc.h"
#include "rm57_mpuinit.h"

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
 *   routine (HL_sys_core.s) to C using NuttX register macros.
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
 * Name: arm_boot
 *
 * Description:
 *   Complete boot operations started in arm_head.S
 *
 * Boot Sequence
 *
 *   1.  The __start entry point in armv7-r/arm_head.S is invoked upon
 *       power-on reset and prepares the CPU for code execution.
 *   2.  Because CONFIG_ARMV7R_MEMINIT is set, this function is called
 *       *before* .bss/.data have been initialized - it must not touch
 *       any global/static data until arm_data_initialize() has run.
 *   3.  This function configures clocking, hardware-initializes SRAM
 *       (clearing ECC), then calls arm_data_initialize().
 *   4.  Only after that can the MPU and board-specific bring-up run.
 *   5.  This function then branches to nx_start() to start the OS.
 *
 * NOTE: unlike the sibling tms570_boot.c, this function does not perform
 * PBIST/self-test - that is not yet implemented for RM57.
 *
 ****************************************************************************/

void arm_boot(void)
{
  /* Zero r0-r12/SPSR in every CPU mode first, before anything else, to
   * avoid a lockstep CPU Compare Module (CCM) mismatch between
   * RM57L843's two redundant cores (see rm57_coreinit.S).
   */

  rm57_core_init_registers();

  /* Enable CPU Event Export */

  rm57_event_export();

  /* Initialize clocking (PLL, peripheral clocks, flash wait-states) */

  rm57_clockconfig();

  /* Put the Error Signaling Module into a known, quiet state (all
   * error-pin channels/interrupts disabled, all latched flags
   * cleared, nERROR pin released).
   */

  rm57_esm_initialize();

  /* Hardware-initialize (ECC-clear) SRAM before .bss/.data are touched */

  rm57_memory_initialize();

  /* Initialize the FPU (RM57L843 is Cortex-R5F) */

  arm_fpuconfig();

  /* Initialize .bss and .data now that SRAM has been initialized */

  arm_data_initialize();

  /* Configure the MPU regions (flash, SRAM, peripherals) */

  rm57_mpu_init();

  /* Initialize GIO for use by board initialization logic */

  rm57_gio_initialize();

  /* Perform board-specific initialization */

  rm57_board_initialize();

  /* Bring up the console SCI (RX/TX pin function, baud/format/GCR1).
   * This must run before nx_start() - without it the SCI module is left
   * in its power-on-reset state (module held in reset, RX/TX pin
   * function bits unset) and produces no output even though register
   * writes to it "succeed" from software's point of view.
   */

  rm57_lowsetup();

  /* Then start NuttX */

  nx_start();
}
