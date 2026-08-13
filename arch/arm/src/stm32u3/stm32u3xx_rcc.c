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

#include <assert.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32_rcc.h"

#include "hardware/stm32_flash.h"
#include "hardware/stm32u3xx_pwr.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG

static_assert(CONFIG_BOARD_LOOPSPERMSEC != -1,
              "Configure BOARD_LOOPSPERMSEC to a non-default value");

/* Allow up to 100 milliseconds for each clock or regulator transition. */

#define CLOCK_READY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

#ifndef STM32_BOARD_USEMSIS
#  error stm32_stdclockconfig() requires MSIS board configuration
#endif

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Configure the STM32U3 system clock from the settings in board.h.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG
void stm32_stdclockconfig(void)
{
  uint32_t regval;
  volatile int32_t timeout;

  /* The EPOD booster and voltage scaling control require the PWR clock. */

  modifyreg32(STM32_RCC_AHB1ENR2, 0, RCC_AHB1ENR2_PWREN);

  /* Select the board regulator and wait until the transition completes. */

  modifyreg32(STM32_PWR_CR3, PWR_CR3_REGSEL, STM32_PWR_CR3_REGSEL);
  for (timeout = CLOCK_READY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_PWR_SVMSR) & PWR_SVMSR_REGS) ==
          STM32_PWR_CR3_REGSEL)
        {
          break;
        }
    }

  if (timeout == 0)
    {
      return;
    }

  /* Configure and enable the EPOD booster. */

  regval  = getreg32(STM32_RCC_CFGR4);
  regval &= ~(RCC_CFGR4_BOOSTSEL_MASK | RCC_CFGR4_BOOSTDIV_MASK);
  regval |= STM32_RCC_CFGR4_BOOSTSEL | STM32_RCC_CFGR4_BOOSTDIV;
  putreg32(regval, STM32_RCC_CFGR4);

  modifyreg32(STM32_PWR_VOSR, 0, STM32_PWR_VOSR_BOOST);
  for (timeout = CLOCK_READY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_PWR_VOSR) & PWR_VOSR_BOOSTRDY) != 0)
        {
          break;
        }
    }

  if (timeout == 0)
    {
      return;
    }

  /* Select the board voltage scale and wait until it is active. */

  modifyreg32(STM32_PWR_VOSR, PWR_VOSR_R1EN | PWR_VOSR_R2EN,
              STM32_PWR_VOSR_RANGE);
  for (timeout = CLOCK_READY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_PWR_VOSR) &
           (PWR_VOSR_R1RDY | PWR_VOSR_R2RDY)) ==
          STM32_PWR_VOSR_RANGERDY)
        {
          break;
        }
    }

  if (timeout == 0)
    {
      return;
    }

  /* Configure FLASH before raising HCLK. */

  modifyreg32(STM32_FLASH_ACR, FLASH_ACR_LATENCY_MASK,
              STM32_FLASH_ACR_LATENCY);

  /* Configure and start MSIS. */

  regval  = getreg32(STM32_RCC_ICSCR1);
  regval &= ~(RCC_ICSCR1_MSISSEL | RCC_ICSCR1_MSISDIV_MASK);
  regval |= RCC_ICSCR1_MSIRGSEL | STM32_RCC_ICSCR1_MSISSEL |
            STM32_RCC_ICSCR1_MSISDIV;
  putreg32(regval, STM32_RCC_ICSCR1);

  modifyreg32(STM32_RCC_CR, 0, RCC_CR_MSISON);
  for (timeout = CLOCK_READY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_RCC_CR) & RCC_CR_MSISRDY) != 0)
        {
          break;
        }
    }

  if (timeout == 0)
    {
      return;
    }

  /* Configure the AHB and APB prescalers. */

  modifyreg32(STM32_RCC_CFGR2,
              RCC_CFGR2_HPRE_MASK | RCC_CFGR2_PPRE1_MASK |
              RCC_CFGR2_PPRE2_MASK,
              STM32_RCC_CFGR2_HPRE | STM32_RCC_CFGR2_PPRE1 |
              STM32_RCC_CFGR2_PPRE2);
  modifyreg32(STM32_RCC_CFGR3, RCC_CFGR3_PPRE3_MASK,
              STM32_RCC_CFGR3_PPRE3);

  /* Use MSIS as the system clock and wait for the switch to complete. */

  modifyreg32(STM32_RCC_CFGR1, RCC_CFGR1_SW_MASK, RCC_CFGR1_SW_MSIS);
  for (timeout = CLOCK_READY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_RCC_CFGR1) & RCC_CFGR1_SWS_MASK) ==
          RCC_CFGR1_SWS_MSIS)
        {
          break;
        }
    }
}
#endif

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
