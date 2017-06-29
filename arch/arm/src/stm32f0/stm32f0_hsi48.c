/****************************************************************************
 *  arch/arm/src/stm32f0/stm32f0_hsi48.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include "up_arch.h"
#include "chip.h"
#include "chip/stm32f0_rcc.h"
#include "chip/stm32f0_crs.h"

#include "stm32f0_hsi48.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32f0_enable_hsi48
 *
 * Description:
 *   On STM32F04x, STM32F07x and STM32F09x devices only, the HSI48 clock
 *   signal is generated from an internal 48 MHz RC oscillator and can be
 *   used directly as a system clock or divided and be used as PLL input.
 *
 *   The internal 48MHz RC oscillator is mainly dedicated to provide a high
 *   precision clock to the USB peripheral by means of a special Clock
 *   Recovery System (CRS) circuitry, which could use the USB SOF signal or
 *   the LSE or an external signal to automatically adjust the oscillator
 *   frequency on-fly, in a very small steps. This oscillator can also be
 *   used as a system clock source when the system is in run mode; it will
 *   be disabled as soon as the system enters in Stop or Standby mode. When
 *   the CRS is not used, the HSI48 RC oscillator runs on its default
 *   frequency which is subject to manufacturing process variations.
 *
 * Input Parameters:
 *   Identifies the syncrhonization source for the HSI48.  When used as the
 *   USB source clock, this must be set to SYNCSRC_USB.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32f0_enable_hsi48(enum syncsrc_e syncsrc)
{
  uint32_t regval;

  /* Enable the HSI48 clock.
   *
   * The HSI48 RC can be switched on and off using the HSI48ON bit in the
   * Clock control register (RCC_CR).
   *
   * The USB clock may be derived from either the PLL clock or from the
   * HSI48 clock.  This oscillator will be also automatically enabled (by
   * hardware forcing HSI48ON bit to one) as soon as it is chosen as a clock
   * source for the USB and the peripheral is
   * enabled.
   */

  regval  = getreg32(STM32F0_RCC_CR2);
  regval |= RCC_CR2_HSI48ON;
  putreg32(regval, STM32F0_RCC_CR2);

  if (syncsrc == SYNCSRC_USB)
    {
      /* Select the HSI48 as the USB clock source */

      regval  = getreg32(STM32F0_RCC_CFGR3);
      regval &= ~RCC_CFGR3_USBSW;
      putreg32(regval, STM32F0_RCC_CFGR3);
    }

  /* Wait for the HSI48 clock to stabilize */

  while ((getreg32(STM32F0_RCC_CR2) & RCC_CR2_HSI48RDY) == 0);


  /* The CRS synchronization (SYNC) source, selectable through the CRS_CFGR
   * register, can be the signal from the external CRS_SYNC pin, the LSE
   * clock or the USB SOF signal.
   */

  regval = getreg32(STM32F0_CRS_CFGR);
  regval &= ~CRS_CFGR_SYNCSRC_MASK;

  switch (syncsrc)
    {
      default:
      case SYNCSRC_GPIO: /* GPIO selected as SYNC signal source */
        regval |= CRS_CFGR_SYNCSRC_GPIO;
        break;

      case SYNCSRC_LSE:  /* LSE selected as SYNC signal source */
        regval |= CRS_CFGR_SYNCSRC_LSE;
        break;

      case SYNCSRC_USB:  /* USB SOF selected as SYNC signal source */
        regval |= CRS_CFGR_SYNCSRC_USBSOF;
        break;
    }

  putreg32(regval, STM32F0_CRS_CFGR);

  /* Set the AUTOTRIMEN bit the CRS_CR register to enables the automatic
   * hardware adjustment of TRIM bits according to the measured frequency
   * error between the selected SYNC event.
   */

  regval  = getreg32(STM32F0_CRS_CR);
  regval |= CRS_CR_AUTOTRIMEN;
  putreg32(regval, STM32F0_CRS_CR);
}

/****************************************************************************
 * Name: stm32f0_disable_hsi48
 *
 * Description:
 *   Disable the HSI48 clock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32f0_disable_hsi48(void)
{
  uint32_t regval;

  /* Disable the HSI48 clock */

  regval  = getreg32(STM32F0_RCC_CR2);
  regval &= ~RCC_CR2_HSI48ON;
  putreg32(regval, STM32F0_RCC_CR2);

  /* Set other registers to the default settings. */

  regval = getreg32(STM32F0_CRS_CFGR);
  regval &= ~CRS_CFGR_SYNCSRC_MASK;
  putreg32(regval, STM32F0_CRS_CFGR);

  regval  = getreg32(STM32F0_CRS_CR);
  regval &= ~CRS_CR_AUTOTRIMEN;
  putreg32(regval, STM32F0_CRS_CR);
}
