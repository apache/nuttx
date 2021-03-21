/****************************************************************************
 * boards/arm/sama5/sama5d3x-ek/src/nor_main.c
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

#include <stdio.h>
#include <debug.h>

#include <nuttx/cache.h>
#include <arch/irq.h>

#include "arm_arch.h"
#include "mmu.h"
#include "cp15_cacheops.h"

#include "sam_periphclks.h"
#include "hardware/sam_hsmc.h"
#include "hardware/sam_matrix.h"
#include "hardware/sam_aximx.h"

#include "sama5d3x-ek.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NOR_ENTRY ((nor_entry_t)SAM_EBICS0_VSECTION)

#define NOR_WAIT        1
#define NOR_NO_WAIT     0

#ifdef CONFIG_SAMA5D3XEK_NOR_START
#  define NOR_BOOT_MODE NOR_NO_WAIT
#else
#  define NOR_BOOT_MODE NOR_WAIT
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef void (*nor_entry_t)(void);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nor_main
 *
 * Description:
 *   nor_main is a tiny program that runs in ISRAM.  nor_main will enable
 *   NOR flash then jump to the program in NOR flash
 *
 ****************************************************************************/

int nor_main(int argc, char *argv)
{
  uint32_t regval;

  /* Here we have a in memory value we can change in the debugger
   * to begin booting in NOR Flash
   */

  static volatile uint32_t wait = NOR_BOOT_MODE;

  printf("Configuring NOR FLASH on CS0 and %s\n",
         wait ? "waiting" : "booting");

  /* Make sure that the SMC peripheral is enabled (But of course it is... we
   * are executing from NOR FLASH now).
   */

  sam_hsmc_enableclk();

  /* The SAMA5D3x-EK has 118MB of 16-bit NOR FLASH at CS0.  The NOR FLASH
   * has already been configured by the first level ROM bootloader... we
   * simply need to modify the timing here.
   */

  regval = HSMC_SETUP_NWE_SETUP(1) |  HSMC_SETUP_NCS_WRSETUP(0) |
           HSMC_SETUP_NRD_SETUP(2) | HSMC_SETUP_NCS_RDSETUP(0);
  putreg32(regval, SAM_HSMC_SETUP(HSMC_CS0));

  regval = HSMC_PULSE_NWE_PULSE(10) | HSMC_PULSE_NCS_WRPULSE(10) |
           HSMC_PULSE_NRD_PULSE(11) | HSMC_PULSE_NCS_RDPULSE(11);
  putreg32(regval, SAM_HSMC_PULSE(HSMC_CS0));

  regval = HSMC_CYCLE_NWE_CYCLE(11) | HSMC_CYCLE_NRD_CYCLE(14);
  putreg32(regval, SAM_HSMC_CYCLE(HSMC_CS0));

  regval = HSMC_TIMINGS_TCLR(0) | HSMC_TIMINGS_TADL(0) |
           HSMC_TIMINGS_TAR(0) | HSMC_TIMINGS_TRR(0) |
           HSMC_TIMINGS_TWB(0) | HSMC_TIMINGS_RBNSEL(0);
  putreg32(regval, SAM_HSMC_TIMINGS(HSMC_CS0));

  regval = HSMC_MODE_READMODE | HSMC_MODE_WRITEMODE |
           HSMC_MODE_EXNWMODE_DISABLED | HSMC_MODE_BIT_16 |
           HSMC_MODE_TDFCYCLES(1);
  putreg32(regval, SAM_HSMC_MODE(HSMC_CS0));

  /* Interrupts must be disabled through the following.
   * In this configuration, there should only be timer interrupts.
   * Your NuttX configuration must use CONFIG_SERIAL_LOWCONSOLE=y or printf()
   * will hang when the interrupts are disabled!
   */

  up_irq_save();

  /* Disable MATRIX write protection */

#if 0 /* Disabled on reset */
  putreg32(MATRIX_WPMR_WPKEY, SAM_MATRIX_WPMR);
#endif

  /* Set remap state 1.
   *
   *   Boot state:    ROM is seen at address 0x00000000
   *   Remap State 0: SRAM is seen at address 0x00000000 (through AHB slave
   *                  interface) instead of ROM.
   *   Remap State 1: HEBI is seen at address 0x00000000 (through AHB slave
   *                  interface) instead of ROM for external boot.
   *
   * REVISIT:  This does not work.  No matter what I do, the internal
   * SRAM is always visible at address zero.  I am missing something.
   */

  putreg32(MATRIX_MRCR_RCB0, SAM_MATRIX_MRCR);   /* Enable remap */
  putreg32(AXIMX_REMAP_REMAP1, SAM_AXIMX_REMAP); /* Remap HEBI */

  /* Restore MATRIX write protection */

#if 0 /* Disabled on reset */
  putreg32(MATRIX_WPMR_WPKEY | MATRIX_WPMR_WPEN, SAM_MATRIX_WPMR);
#endif

  /* Disable the caches and the MMU.  Disabling the MMU should be safe here
   * because there is a 1-to-1 identity mapping between the physical and
   * virtual addressing.
   */

  /* NOTE:  This generates crashes and lots of error, but does leave the
   * system in the proper state to run from NOR:  very ugly but usable.
   * Better than the alternative.
   */

  cp15_disable_mmu();
  up_disable_icache();
  up_disable_dcache();

  /* Invalidate caches and TLBs */

  up_invalidate_icache_all();
  up_invalidate_dcache_all();
  cp15_invalidate_tlbs();

  /* Then jump into NOR flash */

  while (wait)
    {
    }

  NOR_ENTRY();

  return 0; /* We should not get here in either case */
}
