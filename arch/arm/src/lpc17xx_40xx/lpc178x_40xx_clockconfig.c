/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/lpc178x_40xx_clockconfig.c
 * arch/arm/src/chip/lpc17_40_clockconfig.c
 *
 *   Copyright (C) 2010, 2013 Gregory Nutt. All rights reserved.
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
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "arm_internal.h"
#include "lpc17_40_clockconfig.h"
#include "hardware/lpc17_40_syscon.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef LPC178x_40xx
#  error "The logic in this file applies only to the LPC178x/40xx family"
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

/************************************************************************************
 * Name: lpc17_40_clockconfig
 *
 * Description:
 *   Called to initialize the LPC17xx/LPC40xx.  This does whatever setup is needed to put the
 *   SoC in a usable state.  This includes the initialization of clocking using the
 *   settings in board.h.
 *
 *   The LPC176x and LPC178x/40xx system control block is *nearly* identical but we have
 *   found that the LPC178x/40xx is more sensitive to the ordering of certain operations.
 *   So, although the hardware seems very similar, the safer thing to do is to
 *   separate the LPC176x and LPC178x/40xx into separate files.
 *
 ************************************************************************************/

void lpc17_40_clockconfig(void)
{
  uint32_t regval;

  /* TODO:
   *
   * (1) "Make sure that the PLL output is not already being used.  The CCLKSEL,
   *      USBCLKSEL, and SPIFICLKSEL registers must not select the PLL being set up.
   *      Clock dividers included in these registers may also be set up at this time
   *      if writing to any of the noted registers."
   *
   * (2) "If the main PLL is being set up, and the main clock source is being changed
   *      (IRC versus main oscillator), change this first by writing the correct
   *      value to the CLKSRCSEL register."
   *
   * This is not an issue now because we only setup the clocks on power up, so the
   * PLL cannot be the select source.  However, this could be an issue at some point
   * later when, for example, we may want to implement reduced power mode with other
   * clocking.
   */

  /* Enable the main oscillator (or not) and the frequency range of the main oscillator */

  putreg32(BOARD_SCS_VALUE, LPC17_40_SYSCON_SCS);

  /* Wait for the main oscillator to be ready. */

#ifdef CONFIG_LPC17_40_MAINOSC
  while ((getreg32(LPC17_40_SYSCON_SCS) & SYSCON_SCS_OSCSTAT) == 0);
#endif

  /* PLL0 is used to generate the CPU clock divider input (PLLCLK). */

#ifdef CONFIG_LPC17_40_PLL0
  /* (3) "Write PLL new setup values to the PLLCFG register. Write a 1 to the
   *      PLLE bit in the PLLCON register. Perform a PLL feed sequence by writing
   *      first the value 0xAA, then the value 0x55 to the PLLFEED register"
   *
   * Select the PLL0 source clock, multiplier, and pre-divider values.  NOTE that
   * a special "feed" sequence must be written to the PLL0FEED register in order
   * for changes to the PLL0CFG register to take effect.
   */

  putreg32(BOARD_CLKSRCSEL_VALUE, LPC17_40_SYSCON_CLKSRCSEL);
  putreg32(BOARD_PLL0CFG_VALUE, LPC17_40_SYSCON_PLL0CFG);
  putreg32(SYSCON_PLLCON_PLLE, LPC17_40_SYSCON_PLL0CON);

  /* Enable the PLL. NOTE that a special "feed" sequence must be written to the
   * PLL0FEED register in order for changes to the PLL0CON register to take effect.
   */

  putreg32(0xaa, LPC17_40_SYSCON_PLL0FEED);
  putreg32(0x55, LPC17_40_SYSCON_PLL0FEED);

  /* (4) "Set up the necessary clock dividers. These may include the CCLKSEL,
   *      PCLKSEL, EMCCLKSEL, USBCLKSEL, and the SPIFICLKSEL registers.
   */

  putreg32(BOARD_CCLKSEL_VALUE, LPC17_40_SYSCON_CCLKSEL);
  putreg32(BOARD_PCLKDIV, LPC17_40_SYSCON_PCLKSEL);

#ifdef CONFIG_LPC17_40_EMC
  putreg32(BOARD_EMCCLKSEL_VALUE, LPC17_40_SYSCON_EMCCLKSEL);
#endif
#if defined(CONFIG_LPC17_40_USBDEV) || defined(CONFIG_LPC17_40_USBHOST)
  putreg32(BOARD_USBCLKSEL_VALUE, LPC17_40_SYSCON_USBCLKSEL);
#endif
#ifdef CONFIG_LPC17_40_SPIFI
  putreg32(BOARD_SPIFICLKSEL_VALUE, LPC17_40_SPIFICLKSEL_CCLKSEL);
#endif

  /* (5) "Wait for the PLL to lock. This may be accomplished by polling the
   *      PLLSTAT register and testing for PLOCK = 1, or by using the PLL lock
   *      interrupt.Wait for PLL0 to lock.
   */

  while ((getreg32(LPC17_40_SYSCON_PLL0STAT) & SYSCON_PLL0STAT_PLOCK) == 0);

  /* (6) "Connect the PLL by selecting its output in the appropriate places. This
   *      may include the CCLKSEL, USBCLKSEL, and SPIFICLKSEL registers.
   */

#endif /* CONFIG_LPC17_40_PLL0 */

  /* PLL1 receives its clock input from the main oscillator only and can be used to
   * provide a fixed 48 MHz clock only to the USB subsystem (if that clock cannot be
   * obtained from PLL0).
   */

#ifdef CONFIG_LPC17_40_PLL1
  /* (3) "Write PLL new setup values to the PLLCFG register. Write a 1 to the
   *      PLLE bit in the PLLCON register. Perform a PLL feed sequence by writing
   *      first the value 0xAA, then the value 0x55 to the PLLFEED register"
   *
   * Select the PLL1 multiplier, and pre-divider values.  NOTE that a special "feed"
   * sequence must be written to the PLL1FEED register in order for changes to the
   * PLL1CFG register to take effect.
   */

  putreg32(BOARD_PLL1CFG_VALUE, LPC17_40_SYSCON_PLL1CFG);
  putreg32(SYSCON_PLLCON_PLLE, LPC17_40_SYSCON_PLL1CON);

  /* Enable the PLL. NOTE that a special "feed" sequence must be written to the
   * PLL1FEED register in order for changes to the PLL1CON register to take effect.
   */

  putreg32(0xaa, LPC17_40_SYSCON_PLL1FEED);
  putreg32(0x55, LPC17_40_SYSCON_PLL1FEED);

  /* (4) "Set up the necessary clock dividers. These may include the CCLKSEL,
   *      PCLKSEL, EMCCLKSEL, USBCLKSEL, and the SPIFICLKSEL registers.
   */

  /* (5) "Wait for the PLL to lock. This may be accomplished by polling the
   *      PLLSTAT register and testing for PLOCK = 1, or by using the PLL lock
   *      interrupt.Wait for PLL0 to lock.
   */

  while ((getreg32(LPC17_40_SYSCON_PLL1STAT) & SYSCON_PLL1STAT_PLOCK) == 0);

  /* (6) "Connect the PLL by selecting its output in the appropriate places. This
   *      may include the CCLKSEL, USBCLKSEL, and SPIFICLKSEL registers.
   */

#endif /* CONFIG_LPC17_40_PLL1 */

  /* Disable power to all peripherals (except GPIO and left EMC intact).
   * EMC is switched off after reset but if there is boot-loader,
   * it can left it on and if SDRAM is used for NuttX execution then
   * disabling blocks system. Other peripherals must be re-powered
   * one at a time by each device driver when the driver is initialized.
   */

  regval  = getreg32(LPC17_40_SYSCON_PCONP);
  regval &= SYSCON_PCONP_PCEMC;
  regval |= SYSCON_PCONP_PCGPIO;
  putreg32(regval, LPC17_40_SYSCON_PCONP);

  /* Disable CLKOUT */

  putreg32(0, LPC17_40_SYSCON_CLKOUTCFG);

  /* Configure FLASH */

#ifdef CONFIG_LPC17_40_FLASH
  putreg32(BOARD_FLASHCFG_VALUE, LPC17_40_SYSCON_FLASHCFG);
#endif
}
