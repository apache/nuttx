/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc176x_clockconfig.c
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

#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "lpc17_40_clockconfig.h"
#include "hardware/lpc17_40_syscon.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef LPC176x
#  error "The logic in this file applies only to the LPC176x family"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_clockconfig
 *
 * Description:
 *   Called to initialize the LPC176x.  This does whatever setup is needed to
 *   put the SoC in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 *   The LPC176x and LPC178x/40xx system control block is *nearly* identical
 *   but we have found that the LPC178x/40xx is more sensitive to the
 *   ordering of certain operations. So, although the hardware seems very
 *   similar, the safer thing to do is to separate the LPC176x and
 *   LPC178x/40xx into separate files.
 *
 ****************************************************************************/

void lpc17_40_clockconfig(void)
{
  /* Enable the main oscillator (or not) and
   * the frequency range of the main oscillator
   */

  putreg32(BOARD_SCS_VALUE, LPC17_40_SYSCON_SCS);

  /* Wait for the main oscillator to be ready. */

#ifdef CONFIG_LPC17_40_MAINOSC
  while ((getreg32(LPC17_40_SYSCON_SCS) & SYSCON_SCS_OSCSTAT) == 0);
#endif

  /* Setup up the divider value for the CPU clock.
   * The output of the divider is CCLK.
   * The input to the divider (PLLCLK) is equal to SYSCLK unless PLL0 is
   * enabled. CCLK will be further divided to produce peripheral clocks,
   * but that peripheral clock setup is performed in the peripheral device
   * drivers.  Here only CCLK is configured.
   */

  putreg32(BOARD_CCLKCFG_VALUE, LPC17_40_SYSCON_CCLKCFG);

  /* PLL0 is used to generate the CPU clock divider input (PLLCLK). */

#ifdef CONFIG_LPC17_40_PLL0
  /* Select the PLL0 source clock, multiplier, and pre-divider values.
   * NOTE that a special "feed" sequence must be written to the PLL0FEED
   * register in order for changes to the PLL0CFG register to take effect.
   */

  putreg32(BOARD_CLKSRCSEL_VALUE, LPC17_40_SYSCON_CLKSRCSEL);
  putreg32(BOARD_PLL0CFG_VALUE, LPC17_40_SYSCON_PLL0CFG);
  putreg32(0xaa, LPC17_40_SYSCON_PLL0FEED);
  putreg32(0x55, LPC17_40_SYSCON_PLL0FEED);

  /* Enable the PLL.
   * NOTE that a special "feed" sequence must be written to the PLL0FEED
   * register in order for changes to the PLL0CON register to take effect.
   */

  putreg32(SYSCON_PLLCON_PLLE, LPC17_40_SYSCON_PLL0CON);
  putreg32(0xaa, LPC17_40_SYSCON_PLL0FEED);
  putreg32(0x55, LPC17_40_SYSCON_PLL0FEED);

  /* Wait for PLL0 to lock */

  while ((getreg32(LPC17_40_SYSCON_PLL0STAT) & SYSCON_PLL0STAT_PLOCK) == 0);

  /* Enable and connect PLL0 */

  putreg32(SYSCON_PLLCON_PLLE | SYSCON_PLLCON_PLLC, LPC17_40_SYSCON_PLL0CON);
  putreg32(0xaa, LPC17_40_SYSCON_PLL0FEED);
  putreg32(0x55, LPC17_40_SYSCON_PLL0FEED);

  /* Wait for PLL to report that it is connected and enabled */

  while ((getreg32(LPC17_40_SYSCON_PLL0STAT) &
         (SYSCON_PLL0STAT_PLLE | SYSCON_PLL0STAT_PLLC))
          != (SYSCON_PLL0STAT_PLLE | SYSCON_PLL0STAT_PLLC));

#endif /* CONFIG_LPC17_40_PLL0 */

  /* PLL1 receives its clock input from the main oscillator only and can be
   * used to provide a fixed 48 MHz clock only to the USB subsystem
   * (if that clock cannot be obtained from PLL0).
   */

#ifdef CONFIG_LPC17_40_PLL1
  /* Select the PLL1 multiplier, and pre-divider values.
   * NOTE that a special "feed" sequence must be written to the PLL1FEED
   * register in order for changes to the PLL1CFG register to take effect.
   */

  putreg32(BOARD_PLL1CFG_VALUE, LPC17_40_SYSCON_PLL1CFG);
  putreg32(0xaa, LPC17_40_SYSCON_PLL1FEED);
  putreg32(0x55, LPC17_40_SYSCON_PLL1FEED);

  /* Enable the PLL.
   * NOTE that a special "feed" sequence must be written to the PLL1FEED
   * register in order for changes to the PLL1CON register to take effect.
   */

  putreg32(SYSCON_PLLCON_PLLE, LPC17_40_SYSCON_PLL1CON);
  putreg32(0xaa, LPC17_40_SYSCON_PLL1FEED);
  putreg32(0x55, LPC17_40_SYSCON_PLL1FEED);

  /* Wait for PLL1 to lock */

  while ((getreg32(LPC17_40_SYSCON_PLL1STAT) & SYSCON_PLL1STAT_PLOCK) == 0);

  /* Enable and connect PLL1 */

  putreg32(SYSCON_PLLCON_PLLE | SYSCON_PLLCON_PLLC, LPC17_40_SYSCON_PLL1CON);
  putreg32(0xaa, LPC17_40_SYSCON_PLL1FEED);
  putreg32(0x55, LPC17_40_SYSCON_PLL1FEED);

  /* Wait for PLL to report that it is connected and enabled */

  while ((getreg32(LPC17_40_SYSCON_PLL1STAT) &
         (SYSCON_PLL1STAT_PLLE | SYSCON_PLL1STAT_PLLC))
         != (SYSCON_PLL1STAT_PLLE | SYSCON_PLL1STAT_PLLC));

#else /* CONFIG_LPC17_40_PLL1 */

  /* Otherwise, setup up the USB clock divider to generate the USB clock
   * from PLL0
   */

  putreg32(BOARD_USBCLKCFG_VALUE, LPC17_40_SYSCON_USBCLKCFG);

#endif /* CONFIG_LPC17_40_PLL1 */

  /* Disable all peripheral clocks.  They must be configured by each device
   * driver when the device driver is initialized.
   */

  putreg32(0, LPC17_40_SYSCON_PCLKSEL0);
  putreg32(0, LPC17_40_SYSCON_PCLKSEL1);

  /* Disable power to all peripherals (execpt GPIO).  Peripherals must be
   * re-powered one at a time by each device driver when the driver is
   * initialized.
   */

  putreg32(SYSCON_PCONP_PCGPIO, LPC17_40_SYSCON_PCONP);

  /* Disable CLKOUT */

  putreg32(0, LPC17_40_SYSCON_CLKOUTCFG);

  /* Configure FLASH */

#ifdef CONFIG_LPC17_40_FLASH
    {
      uint32_t regval;

      if (BOARD_FLASHCFG_VALUE & ~SYSCON_FLASHCFG_TIM_MASK)
        {
          regval = BOARD_FLASHCFG_VALUE;
        }
      else
        {
          regval = getreg32(LPC17_40_SYSCON_FLASHCFG);
          regval &= ~SYSCON_FLASHCFG_TIM_MASK;
          regval |= BOARD_FLASHCFG_VALUE;
        }

      putreg32(BOARD_FLASHCFG_VALUE, LPC17_40_SYSCON_FLASHCFG);
    }
#endif
}
