/************************************************************************************
 * configs/stm32l476vg-disco/src/stm32_clockconfig.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: dev@ziggurat29.com
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <arch/board/board.h>
#include <arch/board/stm32l476vg-disco-clocking.h>

#include "up_arch.h"
#include "stm32l476vg-disco.h"

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_board_clockconfig
 *
 * Description:
 *   I provided this module when I was doing some debugging of a problem I had with
 *  clocking (it was helpful to do A/B tests).  I'm leaving it here in the config
 *  partially because I expect to have similar problems again as I develop more of
 *  the various peripheral support, but also because it may become necessary in the
 *  end for certain project configurations which have specialized clock configurations
 *  that aren't appropriate to expose in the 'arch' default code.
 *
 ************************************************************************************/

#if defined(CONFIG_ARCH_BOARD_STM32L4_CUSTOM_CLOCKCONFIG)
void stm32l4_board_clockconfig(void)
{
  uint32_t regval;

  /* Enable Internal High-Speed Clock (HSI) */

  regval  = getreg32(STM32L4_RCC_CR);
  regval |= RCC_CR_HSION;           /* Enable HSI */
  putreg32(regval, STM32L4_RCC_CR);

  /* Wait until the HSI is ready */

  while ((getreg32(STM32L4_RCC_CR) & RCC_CR_HSIRDY) == 0)
    {
    }

  /* Set the HCLK source/divider */

  regval  = getreg32(STM32L4_RCC_CFGR);
  regval &= ~RCC_CFGR_HPRE_MASK;
  regval |= STM32L4_RCC_CFGR_HPRE;
  putreg32(regval, STM32L4_RCC_CFGR);

  /* Set the PCLK2 divider */

  regval  = getreg32(STM32L4_RCC_CFGR);
  regval &= ~RCC_CFGR_PPRE2_MASK;
  regval |= STM32L4_RCC_CFGR_PPRE2;
  putreg32(regval, STM32L4_RCC_CFGR);

  /* Set the PCLK1 divider */

  regval  = getreg32(STM32L4_RCC_CFGR);
  regval &= ~RCC_CFGR_PPRE1_MASK;
  regval |= STM32L4_RCC_CFGR_PPRE1;
  putreg32(regval, STM32L4_RCC_CFGR);

  /* Set the PLL source and main divider */

  regval  = getreg32(STM32L4_RCC_PLLCFG);

  /* Configure Main PLL */
  /* Set the PLL dividers and multipliers to configure the main PLL */

  regval = (STM32L4_PLLCFG_PLLM | STM32L4_PLLCFG_PLLN | STM32L4_PLLCFG_PLLP
             | STM32L4_PLLCFG_PLLQ | STM32L4_PLLCFG_PLLR);
  regval |= RCC_PLLCFG_PLLQEN;
  regval |= RCC_PLLCFG_PLLREN;

  /* XXX The choice of clock source to PLL (all three) is independent
   * of the sys clock source choice, review the STM32L4_BOARD_USEHSI
   * name; probably split it into two, one for PLL source and one
   * for sys clock source.
   */

  regval |= RCC_PLLCFG_PLLSRC_HSI;
  putreg32(regval, STM32L4_RCC_PLLCFG);

  /* Enable the main PLL */

  regval  = getreg32(STM32L4_RCC_CR);
  regval |= RCC_CR_PLLON;
  putreg32(regval, STM32L4_RCC_CR);

  /* Wait until the PLL is ready */

  while ((getreg32(STM32L4_RCC_CR) & RCC_CR_PLLRDY) == 0)
    {
    }

  /* Configure SAI1 PLL */

  regval  = getreg32(STM32L4_RCC_PLLSAI1CFG);

  /* Set the PLL dividers and multipliers to configure the SAI1 PLL */

  regval  = (STM32L4_PLLSAI1CFG_PLLN | STM32L4_PLLSAI1CFG_PLLP |
             STM32L4_PLLSAI1CFG_PLLQ | STM32L4_PLLSAI1CFG_PLLR);
  regval |= RCC_PLLSAI1CFG_PLLQEN;
  putreg32(regval, STM32L4_RCC_PLLSAI1CFG);

  /* Enable the SAI1 PLL */

  regval  = getreg32(STM32L4_RCC_CR);
  regval |= RCC_CR_PLLSAI1ON;
  putreg32(regval, STM32L4_RCC_CR);

  /* Wait until the PLL is ready */

  while ((getreg32(STM32L4_RCC_CR) & RCC_CR_PLLSAI1RDY) == 0)
    {
    }

  /* Configure SAI2 PLL */

  regval  = getreg32(STM32L4_RCC_PLLSAI2CFG);

  /* Enable the SAI2 PLL */
  /* Set the PLL dividers and multipliers to configure the SAI2 PLL */

  regval = (STM32L4_PLLSAI2CFG_PLLN | STM32L4_PLLSAI2CFG_PLLP |
            STM32L4_PLLSAI2CFG_PLLR);
  putreg32(regval, STM32L4_RCC_PLLSAI2CFG);

  /* Enable the SAI1 PLL */

  regval  = getreg32(STM32L4_RCC_CR);
  regval |= RCC_CR_PLLSAI2ON;
  putreg32(regval, STM32L4_RCC_CR);

  /* Wait until the PLL is ready */

  while ((getreg32(STM32L4_RCC_CR) & RCC_CR_PLLSAI2RDY) == 0)
    {
    }

  /* Enable FLASH prefetch, instruction cache, data cache, and 5 wait states */

#ifdef CONFIG_STM32L4_FLASH_PREFETCH
  regval = (FLASH_ACR_LATENCY_4 | FLASH_ACR_ICEN | FLASH_ACR_DCEN |
            FLASH_ACR_PRFTEN);
#else
  regval = (FLASH_ACR_LATENCY_4 | FLASH_ACR_ICEN | FLASH_ACR_DCEN);
#endif
  putreg32(regval, STM32L4_FLASH_ACR);

  /* Select the main PLL as system clock source */

  regval  = getreg32(STM32L4_RCC_CFGR);
  regval &= ~RCC_CFGR_SW_MASK;
  regval |= RCC_CFGR_SW_PLL;
  putreg32(regval, STM32L4_RCC_CFGR);

  /* Wait until the PLL source is used as the system clock source */

  while ((getreg32(STM32L4_RCC_CFGR) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL)
    {
    }

#if defined(CONFIG_STM32L4_IWDG) || defined(CONFIG_STM32L4_RTC_LSICLOCK)

  /* Low speed internal clock source LSI */

   stm32l4_rcc_enablelsi();
#endif

#if defined(STM32L4_USE_LSE)

  /* Low speed external clock source LSE
   *
   * TODO: There is another case where the LSE needs to
   * be enabled: if the MCO1 pin selects LSE as source.
   */

  stm32l4_pwr_enableclk(true);
  stm32l4_rcc_enablelse();
#endif
}
#endif
