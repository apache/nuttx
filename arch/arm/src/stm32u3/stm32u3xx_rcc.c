/****************************************************************************
 * arch/arm/src/stm32u3/stm32u3xx_rcc.c
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

#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32_rcc.h"

#include "hardware/stm32_flash.h"
#include "hardware/stm32u3xx_pwr.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Configure the STM32U3 system clock from MSIRC0 at 96 MHz.
 *
 ****************************************************************************/

void stm32_stdclockconfig(void)
{
  uint32_t regval;

  /* The EPOD booster and voltage scaling control require the PWR clock. */

  modifyreg32(STM32_RCC_AHB1ENR2, 0, RCC_AHB1ENR2_PWREN);

  /* Supply the EPOD booster from undivided MSIS. */

  regval  = getreg32(STM32_RCC_CFGR4);
  regval &= ~(RCC_CFGR4_BOOSTSEL_MASK | RCC_CFGR4_BOOSTDIV_MASK);
  regval |= RCC_CFGR4_BOOSTSEL_MSIS;
  putreg32(regval, STM32_RCC_CFGR4);

  modifyreg32(STM32_PWR_VOSR, 0, PWR_VOSR_BOOSTEN);
  while ((getreg32(STM32_PWR_VOSR) & PWR_VOSR_BOOSTRDY) == 0)
    {
    }

  /* Select voltage scale 1 and wait until it is active. */

  modifyreg32(STM32_PWR_VOSR, PWR_VOSR_R2EN, PWR_VOSR_R1EN);
  while ((getreg32(STM32_PWR_VOSR) &
          (PWR_VOSR_R1RDY | PWR_VOSR_R2RDY)) != PWR_VOSR_R1RDY)
    {
    }

  /* Three wait states are required before raising HCLK to 96 MHz. */

  modifyreg32(STM32_FLASH_ACR, FLASH_ACR_LATENCY_MASK,
              FLASH_ACR_LATENCY(3));

  /* Select MSIRC0 divided by one and use it as the 96 MHz system clock. */

  regval  = getreg32(STM32_RCC_ICSCR1);
  regval &= ~(RCC_ICSCR1_MSISSEL | RCC_ICSCR1_MSISDIV_MASK);
  regval |= RCC_ICSCR1_MSIRGSEL;
  putreg32(regval, STM32_RCC_ICSCR1);

  modifyreg32(STM32_RCC_CR, 0, RCC_CR_MSISON);
  while ((getreg32(STM32_RCC_CR) & RCC_CR_MSISRDY) == 0)
    {
    }

  modifyreg32(STM32_RCC_CFGR1, RCC_CFGR1_SW_MASK, RCC_CFGR1_SW_MSIS);
  while ((getreg32(STM32_RCC_CFGR1) & RCC_CFGR1_SWS_MASK) !=
         RCC_CFGR1_SWS_MSIS)
    {
    }

  /* Keep all bus clocks undivided. */

  modifyreg32(STM32_RCC_CFGR2,
              RCC_CFGR2_HPRE_MASK | RCC_CFGR2_PPRE1_MASK |
              RCC_CFGR2_PPRE2_MASK, 0);
  modifyreg32(STM32_RCC_CFGR3, RCC_CFGR3_PPRE3_MASK, 0);
}

/****************************************************************************
 * Name: stm32_rcc_enableperipherals
 *
 * Description:
 *   Enable clocks for configured peripherals.
 *
 ****************************************************************************/

void stm32_rcc_enableperipherals(void)
{
  uint32_t regval;

  /* Enable all GPIO ports implemented by STM32U3C5. */

  regval = RCC_AHB2ENR1_GPIOAEN | RCC_AHB2ENR1_GPIOBEN |
           RCC_AHB2ENR1_GPIOCEN | RCC_AHB2ENR1_GPIODEN |
           RCC_AHB2ENR1_GPIOEEN | RCC_AHB2ENR1_GPIOFEN |
           RCC_AHB2ENR1_GPIOGEN | RCC_AHB2ENR1_GPIOHEN;
  modifyreg32(STM32_RCC_AHB2ENR1, 0, regval);

#ifdef CONFIG_STM32_USART1
  modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_USART1EN);
#endif

  regval = 0;
#ifdef CONFIG_STM32_USART2
  regval |= RCC_APB1ENR1_USART2EN;
#endif
#ifdef CONFIG_STM32_USART3
  regval |= RCC_APB1ENR1_USART3EN;
#endif
#ifdef CONFIG_STM32_UART4
  regval |= RCC_APB1ENR1_UART4EN;
#endif
#ifdef CONFIG_STM32_UART5
  regval |= RCC_APB1ENR1_UART5EN;
#endif
  modifyreg32(STM32_RCC_APB1ENR1, 0, regval);

  regval = 0;
#ifdef CONFIG_STM32_SYSCFG
  regval |= RCC_APB3ENR_SYSCFGEN;
#endif
#ifdef CONFIG_STM32_LPUART1
  regval |= RCC_APB3ENR_LPUART1EN;
#endif
  modifyreg32(STM32_RCC_APB3ENR, 0, regval);
}
