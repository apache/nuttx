/****************************************************************************
 * arch/arm/src/stm32c5/stm32c5xx_rcc.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG

static_assert(CONFIG_BOARD_LOOPSPERMSEC != -1,
              "Configure BOARD_LOOPSPERMSEC to a non-default value");

/* Allow up to 100 milliseconds for each clock source to become ready. */

#define CLOCK_READY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

#if !defined(STM32_BOARD_USEHSE) || !defined(STM32_BOARD_USEPSI)
#  error stm32_stdclockconfig() requires HSE and PSI board configuration
#endif

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Configure the STM32C5 system clock from the settings in board.h.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG
void stm32_stdclockconfig(void)
{
  uint32_t regval;
  volatile int32_t timeout;

  /* Start the external oscillator used as the PSI reference. */

  regval  = getreg32(STM32_RCC_CR1);
  regval &= ~(RCC_CR1_HSEBYP | RCC_CR1_HSEEXT);

#ifdef STM32_HSEBYP_ENABLE
  regval |= RCC_CR1_HSEBYP;
#  ifdef STM32_HSEEXT_DIGITAL
  regval |= RCC_CR1_HSEEXT;
#  endif
#endif

  regval |= RCC_CR1_HSEON;
  putreg32(regval, STM32_RCC_CR1);

  for (timeout = CLOCK_READY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_RCC_CR1) & RCC_CR1_HSERDY) != 0)
        {
          break;
        }
    }

  if (timeout == 0)
    {
      return;
    }

  /* Configure the PSI reference and output frequency. */

  regval  = getreg32(STM32_RCC_CR2);
  regval &= ~(RCC_CR2_PSIREFSRC_MASK | RCC_CR2_PSIREF_MASK |
              RCC_CR2_PSIFREQ_MASK);
  regval |= STM32_RCC_CR2_PSIREFSRC | STM32_RCC_CR2_PSIREF |
            STM32_RCC_CR2_PSIFREQ;
  putreg32(regval, STM32_RCC_CR2);

  modifyreg32(STM32_RCC_CR1, 0, RCC_CR1_PSISON);
  for (timeout = CLOCK_READY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_RCC_CR1) & RCC_CR1_PSISRDY) != 0)
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
              RCC_CFGR2_PPRE2_MASK | RCC_CFGR2_PPRE3_MASK,
              STM32_RCC_CFGR2_HPRE | STM32_RCC_CFGR2_PPRE1 |
              STM32_RCC_CFGR2_PPRE2 | STM32_RCC_CFGR2_PPRE3);

  /* Configure FLASH before raising HCLK. */

  modifyreg32(STM32_FLASH_ACR, FLASH_ACR_LATENCY_MASK,
              STM32_FLASH_ACR_LATENCY);

  /* Use PSIS as the system clock and wait for the switch to complete. */

  modifyreg32(STM32_RCC_CFGR1, RCC_CFGR1_SW_MASK, RCC_CFGR1_SW_PSIS);
  for (timeout = CLOCK_READY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((getreg32(STM32_RCC_CFGR1) & RCC_CFGR1_SWS_MASK) ==
          RCC_CFGR1_SWS_PSIS)
        {
          break;
        }
    }

  if (timeout == 0)
    {
      return;
    }

  /* Configure the FLASH programming delay for the resulting HCLK. */

  modifyreg32(STM32_FLASH_ACR, FLASH_ACR_WRHIGHFREQ_MASK,
              STM32_FLASH_ACR_WRHIGHFREQ);
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

  /* Enable the GPIO ports implemented by STM32C5. */

  regval = RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN |
           RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIODEN |
           RCC_AHB2ENR_GPIOEEN | RCC_AHB2ENR_GPIOHEN;
  modifyreg32(STM32_RCC_AHB2ENR, 0, regval);

#ifdef CONFIG_STM32_USART1
  modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_USART1EN);
#endif

  regval = 0;
#ifdef CONFIG_STM32_USART2
  regval |= RCC_APB1LENR_USART2EN;
#endif
#ifdef CONFIG_STM32_USART3
  regval |= RCC_APB1LENR_USART3EN;
#endif
#ifdef CONFIG_STM32_UART4
  regval |= RCC_APB1LENR_UART4EN;
#endif
#ifdef CONFIG_STM32_UART5
  regval |= RCC_APB1LENR_UART5EN;
#endif
  modifyreg32(STM32_RCC_APB1LENR, 0, regval);

#ifdef CONFIG_STM32_LPUART1
  modifyreg32(STM32_RCC_APB3ENR, 0, RCC_APB3ENR_LPUART1EN);
#endif
}
