/****************************************************************************
 * boards/arm/stm32/mikroe-stm32f4/src/stm32_clockconfig.c
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

#include <arch/board/board.h>

#include "arm_internal.h"
#include "mikroe-stm32f4.h"

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
 * Name: stm32_board_clockconfig
 *
 * Description:
 *   The Mikroe-STM32F4 board does not have an external crystal, so it must
 *   rely on the internal 16Mhz RC oscillator.  The default clock
 *   configuration in the OS for the STM32 architecture assumes an external
 *   crystal, so we must provide a board specific clock configuration
 *   routine.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG)
void stm32_board_clockconfig(void)
{
  uint32_t regval;

  /* Configure chip clocking to use the internal 16Mhz RC oscillator.
   *
   * NOTE: We will assume the HSIRDY (High Speed Internal RC Ready) bit is
   * set, otherwise we wouldn't be here executing code.
   */

  regval  = getreg32(STM32_RCC_APB1ENR);
  regval |= RCC_APB1ENR_PWREN;
  putreg32(regval, STM32_RCC_APB1ENR);

  regval  = getreg32(STM32_PWR_CR);
  regval |= PWR_CR_VOS;
  putreg32(regval, STM32_PWR_CR);

  /* Set the HCLK source/divider */

  regval = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_HPRE_MASK;
  regval |= STM32_RCC_CFGR_HPRE;
  putreg32(regval, STM32_RCC_CFGR);

  /* Set the PCLK2 divider */

  regval = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_PPRE2_MASK;
  regval |= STM32_RCC_CFGR_PPRE2;
  putreg32(regval, STM32_RCC_CFGR);

  /* Set the PCLK1 divider */

  regval = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_PPRE1_MASK;
  regval |= STM32_RCC_CFGR_PPRE1;
  putreg32(regval, STM32_RCC_CFGR);

  /* Set the PLL dividers and multipliers to configure the main PLL */

  regval = (STM32_PLLCFG_PLLM | STM32_PLLCFG_PLLN | STM32_PLLCFG_PLLP |
            RCC_PLLCFG_PLLSRC_HSI | STM32_PLLCFG_PLLQ);
  putreg32(regval, STM32_RCC_PLLCFG);

  /* Enable the main PLL */

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_PLLON;
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the PLL is ready */

  while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLRDY) == 0)
    ;

  /* Enable FLASH prefetch, instruction cache, data cache, and 5 wait
   * states
   */

#ifdef CONFIG_STM32_FLASH_PREFETCH
  regval = (FLASH_ACR_LATENCY_5 | FLASH_ACR_ICEN |
            FLASH_ACR_DCEN | FLASH_ACR_PRFTEN);
#else
  regval = (FLASH_ACR_LATENCY_5 | FLASH_ACR_ICEN | FLASH_ACR_DCEN);
#endif
  putreg32(regval, STM32_FLASH_ACR);

  /* Select the main PLL as system clock source */

  regval  = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_SW_MASK;
  regval |= RCC_CFGR_SW_PLL;
  putreg32(regval, STM32_RCC_CFGR);

  /* Wait until the PLL source is used as the system clock source */

  while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL)
    ;
}
#endif
