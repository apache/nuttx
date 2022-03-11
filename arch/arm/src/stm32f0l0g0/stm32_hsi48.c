/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32_hsi48.c
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

#include "arm_internal.h"
#include "chip.h"
#include "hardware/stm32_rcc.h"
#include "hardware/stm32_crs.h"

#include "stm32_hsi48.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ARCH_CHIP_STM32F0)
#  define STM32_HSI48_REG STM32_RCC_CR2
#  define STM32_HSI48ON   RCC_CR2_HSI48ON
#  define STM32_HSI48RDY  RCC_CR2_HSI48RDY
#elif defined(CONFIG_ARCH_CHIP_STM32L0)
#  define STM32_HSI48_REG STM32_RCC_CRRCR
#  define STM32_HSI48ON   RCC_CRRCR_HSI48ON
#  define STM32_HSI48RDY  RCC_CRRCR_HSI48RDY
#else
#  error "Unsupported STM32F0/L0 HSI48"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_enable_hsi48
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

void stm32_enable_hsi48(enum syncsrc_e syncsrc)
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

  regval  = getreg32(STM32_HSI48_REG);
  regval |= STM32_HSI48ON;
  putreg32(regval, STM32_HSI48_REG);

  /* Wait for the HSI48 clock to stabilize */

  while ((getreg32(STM32_HSI48_REG) & STM32_HSI48RDY) == 0);

  /* Return if no synchronization */

  if (syncsrc == SYNCSRC_NONE)
    {
      return;
    }

  /* The CRS synchronization (SYNC) source, selectable through the CRS_CFGR
   * register, can be the signal from the external CRS_SYNC pin, the LSE
   * clock or the USB SOF signal.
   */

  regval = getreg32(STM32_CRS_CFGR);
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

  putreg32(regval, STM32_CRS_CFGR);

  /* Set the AUTOTRIMEN bit the CRS_CR register to enables the automatic
   * hardware adjustment of TRIM bits according to the measured frequency
   * error between the selected SYNC event.
   */

  regval  = getreg32(STM32_CRS_CR);
  regval |= CRS_CR_AUTOTRIMEN;
  putreg32(regval, STM32_CRS_CR);
}

/****************************************************************************
 * Name: stm32_disable_hsi48
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

void stm32_disable_hsi48(void)
{
  uint32_t regval;

  /* Disable the HSI48 clock */

  regval  = getreg32(STM32_HSI48_REG);
  regval &= ~STM32_HSI48ON;
  putreg32(regval, STM32_HSI48_REG);

  /* Set other registers to the default settings. */

  regval = getreg32(STM32_CRS_CFGR);
  regval &= ~CRS_CFGR_SYNCSRC_MASK;
  putreg32(regval, STM32_CRS_CFGR);

  regval  = getreg32(STM32_CRS_CR);
  regval &= ~CRS_CR_AUTOTRIMEN;
  putreg32(regval, STM32_CRS_CR);
}
