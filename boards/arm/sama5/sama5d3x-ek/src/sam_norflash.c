/****************************************************************************
 * boards/arm/sama5/sama5d3x-ek/src/sam_norflash.c
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

#include <debug.h>

#include "arm_internal.h"
#include "sam_periphclks.h"
#include "hardware/sam_hsmc.h"

#include "sama5d3x-ek.h"

#ifdef CONFIG_SAMA5_BOOT_CS0FLASH

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_norflash_config
 *
 * Description:
 *   If CONFIG_SAMA5_BOOT_CS0FLASH, then the system is boot directly off
 *   CS0 NOR FLASH.  In this case, we assume that we get here from the
 *   primary boot loader under these conditions:
 *
 *     "If BMS signal is tied to 0, BMS_BIT is read at 1.  The ROM Code
 *      allows execution of the code contained into the memory connected to
 *      Chip Select 0 of the External Bus Interface.
 *
 *     "To achieve that, the following sequence is performed by the ROM
 *      Code:
 *
 *        - The main clock is the on-chip 12 MHz RC oscillator,
 *        - The Static Memory Controller is configured with timing allowing
 *          code execution inCS0 external memory at 12 MHz
 *        - AXI matrix is configured to remap EBI CS0 address at 0x0
 *        - 0x0 is loaded in the Program Counter register
 *
 *     "The user software in the external memory must perform the next
 *      operation in order to complete the clocks and SMC timings
 *      configuration to run at a higher clock frequency:
 *
 *        - Enable the 32768 Hz oscillator if best accuracy is needed
 *        - Reprogram the SMC setup, cycle, hold, mode timing registers
 *          for EBI CS0, to adapt them to the new clock
 *        - Program the PMC (Main Oscillator Enable or Bypass mode)
 *        - Program and Start the PLL
 *        - Switch the system clock to the new value"
 *
 *   This function provides the board-specific implementation of the logic
 *   to reprogram the SMC.
 *
 ****************************************************************************/

void board_norflash_config(void)
{
  uint32_t regval;

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
}

#endif /* CONFIG_SAMA5_BOOT_CS0FLASH */
