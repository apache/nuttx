/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32f0_rcc.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Alan Carvalho de Assis <acassis@gmail.com>
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
#include "stm32_rcc.h"
#include "hardware/stm32_syscfg.h"
#include "hardware/stm32_flash.h"
#include "hardware/stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Determine if board wants to use HSI48 as 48 MHz oscillator. */

#if defined(CONFIG_STM32F0L0G0_HAVE_HSI48) && defined(STM32_USE_CLK48)
#  if STM32_CLK48_SEL == RCC_CFGR3_CLK48_HSI48
#    define STM32_USE_HSI48
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rcc_reset
 *
 * Description:
 *   Put all RCC registers in reset state
 *
 ****************************************************************************/

static inline void rcc_reset(void)
{
  /* Nothing here for now */
}

/****************************************************************************
 * Name: rcc_enableio
 *
 * Description:
 *   Enable selected GPIO
 *
 ****************************************************************************/

static inline void rcc_enableio(void)
{
  uint32_t regval = 0;

  /* Enable basic peripheral support */
  /* Enable all GPIO modules */

  regval  = getreg32(STM32_RCC_AHBENR);
  regval |= RCC_AHBENR_IOPAEN | RCC_AHBENR_IOPBEN | RCC_AHBENR_IOPCEN |\
            RCC_AHBENR_IOPDEN | RCC_AHBENR_IOPEEN | RCC_AHBENR_IOPFEN;
  putreg32(regval, STM32_RCC_AHBENR);
}

/****************************************************************************
 * Name: rcc_enableahb
 *
 * Description:
 *   Enable selected AHB peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb(void)
{
  uint32_t regval = 0;

  /* Set the appropriate bits in the AHBENR register to enabled the
   * selected AHBENR peripherals.
   */

  regval  = getreg32(STM32_RCC_AHBENR);

#ifdef CONFIG_STM32F0L0G0_DMA1
  /* DMA 1 clock enable */

  regval |= RCC_AHBENR_DMA1EN;
#endif

#ifdef CONFIG_STM32F0L0G0_DMA2
  /* DMA 2 clock enable */

  regval |= RCC_AHBENR_DMA2EN;
#endif

#ifdef CONFIG_STM32F0L0G0_CRC
  /* CRC clock enable */

  regval |= RCC_AHBENR_CRCEN;
#endif

#ifdef CONFIG_STM32F0L0G0_TSC
  /* TSC clock enable */

  regval |= RCC_AHBENR_TSCEN;
#endif

  putreg32(regval, STM32_RCC_AHBENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb1
 *
 * Description:
 *   Enable selected APB1 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb1(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB1ENR register to enabled the
   * selected APB1 peripherals.
   */

  regval  = getreg32(STM32_RCC_APB1ENR);

#ifdef CONFIG_STM32F0L0G0_TIM2
  /* Timer 2 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB1ENR_TIM2EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM3
  /* Timer 3 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB1ENR_TIM3EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM4
  /* Timer 4 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB1ENR_TIM4EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM6
  /* Timer 6 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB1ENR_TIM6EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM7
  /* Timer 7 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB1ENR_TIM7EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM14
  /* Timer 14 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB1ENR_TIM14EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_WWDG
  /* Window Watchdog clock enable */

  regval |= RCC_APB1ENR_WWDGEN;
#endif

#ifdef CONFIG_STM32F0L0G0_SPI2
  /* SPI 2 clock enable */

  regval |= RCC_APB1ENR_SPI2EN;
#endif

#ifdef CONFIG_STM32F0L0G0_USART2
  /* USART 2 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB1ENR_USART2EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_USART3
  /* USART 3 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB1ENR_USART3EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_USART4
  /* USART 4 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB1ENR_USART4EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_USART5
  /* USART 5 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB1ENR_USART5EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_I2C1
  /* I2C 1 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB1ENR_I2C1EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_I2C2
  /* I2C 2 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB1ENR_I2C2EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_USB
  /* USB clock enable */

  regval |= RCC_APB1ENR_USBEN;
#endif

#ifdef CONFIG_STM32F0L0G0_CAN1
  /* CAN1 clock enable */

  regval |= RCC_APB1ENR_CAN1EN;
#endif

#ifdef CONFIG_STM32F0L0G0_CRS
  /* Clock recovery system clock enable */

  regval |= RCC_APB1ENR_CRSEN;
#endif

#ifdef CONFIG_STM32F0L0G0_PWR
  /* Power interface clock enable */

  regval |= RCC_APB1ENR_PWREN;
#endif

#ifdef CONFIG_STM32F0L0G0_DAC1
  /* DAC 1 interface clock enable */

  regval |= RCC_APB1ENR_DAC1EN;
#endif

#ifdef CONFIG_STM32F0L0G0_CEC
  /* CEC interface clock enable */

  regval |= RCC_APB1ENR_CECEN;
#endif

  putreg32(regval, STM32_RCC_APB1ENR);
}

/****************************************************************************
 * Name: rcc_enableapb2
 *
 * Description:
 *   Enable selected APB2 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb2(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB2ENR register to enabled the
   * selected APB2 peripherals.
   */

  regval = getreg32(STM32_RCC_APB2ENR);

#ifdef CONFIG_STM32F0L0G0_SYSCFG
  /* SYSCFG clock */

  regval |= RCC_APB2ENR_SYSCFGCOMPEN;
#endif

#ifdef CONFIG_STM32F0L0G0_USART6
  /* USART 6 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB2ENR_USART6EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_USART7
  /* USART 7 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB2ENR_USART7EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_USART8
  /* USART 8 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB2ENR_USART8EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_ADC1
  /* ADC 1 clock enable */

  regval |= RCC_APB2ENR_ADC1EN;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM1
  /* Timer 1 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB2ENR_TIM1EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_SPI1
  /* SPI 1 clock enable */

  regval |= RCC_APB2ENR_SPI1EN;
#endif

#ifdef CONFIG_STM32F0L0G0_USART1
  /* USART1 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB2ENR_USART1EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM15
  /* Timer 15 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB2ENR_TIM15EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM16
  /* Timer 16 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB2ENR_TIM16EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM17
  /* Timer 17 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB2ENR_TIM17EN;
#endif
#endif

#if 0
  /* DBG clock enable */

  regval |= RCC_APB2ENR_DBGMCUEN;
#endif

  putreg32(regval, STM32_RCC_APB2ENR);
}

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h.
 *
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes or any clocking other than PLL driven by the HSE.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG
static void stm32_stdclockconfig(void)
{
  uint32_t regval;

  /*  Verify if PLL is already setup.  If so configure to use HSI mode */

  if ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) == RCC_CFGR_SWS_PLL)
    {
      /* Select HSI mode */

      regval  = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_SW_MASK;
      putreg32(regval, STM32_RCC_CFGR);

      while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_HSI);
    }

  /* Disable the PLL */

  regval  = getreg32(STM32_RCC_CR);
  regval &= ~RCC_CR_PLLON;
  putreg32(regval, STM32_RCC_CR);
  while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLRDY) != 0);

  /* Enable FLASH prefetch buffer and set flash latency */

  regval = getreg32(STM32_FLASH_ACR);
  regval &= ~FLASH_ACR_LATENCY_MASK;
  regval |= (FLASH_ACR_LATENCY_1 | FLASH_ACR_PRTFBE);
  putreg32(regval, STM32_FLASH_ACR);

  /* Set HCLK = SYSCLK */

  regval  = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_HPRE_MASK;
  regval |= RCC_CFGR_HPRE_SYSCLK;
  putreg32(regval, STM32_RCC_CFGR);

  /* Set PCLK = HCLK */

  regval &= ~RCC_CFGR_PPRE1_MASK;
  regval |= RCC_CFGR_PPRE1_HCLK;
  putreg32(regval, STM32_RCC_CFGR);

  /* Configure the PLL to generate the system clock
   *
   * 1. Use source = HSI/2
   * 2. Use PREDIV = 1
   * 3. Use multiplier from board.h
   */

  regval &= ~(RCC_CFGR_PLLSRC_MASK | RCC_CFGR_PLLXTPRE_MASK | RCC_CFGR_PLLMUL_MASK);
  regval |= (RCC_CFGR_PLLSRC_HSId2 | RCC_CFGR_PLLXTPRE_DIV1 | STM32_CFGR_PLLMUL);
  putreg32(regval, STM32_RCC_CFGR);

  /* Enable the PLL */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_PLLON;
  putreg32(regval, STM32_RCC_CR);
  while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLRDY) == 0);

  /* Configure to use the PLL */

  regval  = getreg32(STM32_RCC_CFGR);
  regval |= RCC_CFGR_SW_PLL;
  putreg32(regval, STM32_RCC_CFGR);
  while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SW_MASK) != RCC_CFGR_SW_PLL);

#ifdef STM32_USE_CLK48
  /* Select the HSI48 clock source (USB clock source) */

  regval  = getreg32(STM32_RCC_CFGR3);
  regval |= STM32_CLK48_SEL;
  putreg32(regval, STM32_RCC_CFGR3);
#endif
}
#endif

/****************************************************************************
 * Name: rcc_enableperiphals
 ****************************************************************************/

static inline void rcc_enableperipherals(void)
{
  rcc_enableio();
  rcc_enableahb();
  rcc_enableapb2();
  rcc_enableapb1();

#ifdef STM32_USE_HSI48
  /* Enable HSI48 clocking to support USB transfers or RNG */

  stm32_enable_hsi48(STM32_HSI48_SYNCSRC);
#endif
}
