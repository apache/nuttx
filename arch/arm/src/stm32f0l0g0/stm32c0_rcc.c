/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32c0_rcc.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow up to 100 milliseconds for the high speed clock to become
 * ready. that is a very long delay, but if the clock does not become
 * ready we are hosed anyway.  Normally this is very fast, but I have
 * seen at least one board that required this long, long timeout for
 * the HSE to be ready.
 */

#define HSERDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
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
  uint32_t regval;

  /* Reset CFGR register */

  putreg32(RCC_CFGR_RESET, STM32_RCC_CFGR);

  /* Reset CR register */

  putreg32(RCC_CR_RESET, STM32_RCC_CR);

  /* DBG clock enable */

  regval = RCC_APB1ENR_DBGEN;
  putreg32(regval, STM32_RCC_APB1ENR);
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

  /* REVISIT: */

  regval |= (RCC_IOPENR_IOPAEN | RCC_IOPENR_IOPBEN | RCC_IOPENR_IOPCEN | \
             RCC_IOPENR_IOPDEN | RCC_IOPENR_IOPFEN);

  putreg32(regval, STM32_RCC_IOPENR);   /* Enable GPIO */
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

#ifdef CONFIG_STM32F0L0G0_MIF
  /* Memory interface clock enable */

  regval |= RCC_AHBENR_MIFEN;
#endif

#ifdef CONFIG_STM32F0L0G0_CRC
  /* CRC clock enable */

  regval |= RCC_AHBENR_CRCEN;
#endif

#ifdef CONFIG_STM32F0L0G0_RNG
  /* Random number generator clock enable */

  regval |= RCC_AHBENR_RNGEN;
#endif

#ifdef CONFIG_STM32F0L0G0_AES
  /* AES modules clock enable */

  regval |= RCC_AHBENR_AESEN;
#endif

  putreg32(regval, STM32_RCC_AHBENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb1
 *
 * Description:
 *   Enable selected APB peripherals from the first APB register
 *
 ****************************************************************************/

static inline void rcc_enableapb1(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB1ENR register to enabled the
   * selected APB peripherals.
   */

  regval  = getreg32(STM32_RCC_APB1ENR);

#ifdef CONFIG_STM32F0L0G0_TIM2
  /* Timer 2 clock enable */

  regval |= RCC_APB1ENR_TIM2EN;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM3
  /* Timer 3 clock enable */

  regval |= RCC_APB1ENR_TIM3EN;
#endif

#ifdef CONFIG_STM32F0L0G0_FDCAN1
  /* FDCAN1 clock enable */

  regval |= RCC_APB1ENR_FDCANEN;
#endif

#ifdef CONFIG_STM32F0L0G0_SPI2
  /* SPI 2 clock enable */

  regval |= RCC_APB1ENR_SPI2EN;
#endif

#ifdef CONFIG_STM32F0L0G0_USB
  /* USB clock enable */

  regval |= RCC_APB1ENR_USBEN;
#endif

#ifdef CONFIG_STM32F0L0G0_CRC
  /* CRC clock enable */

  regval |= RCC_APB1ENR_CRCEN;
#endif

#ifdef CONFIG_STM32F0L0G0_USART2
  /* USART 2 clock enable */

  regval |= RCC_APB1ENR_USART2EN;
#endif

#ifdef CONFIG_STM32F0L0G0_USART3
  /* USART 3 clock enable */

  regval |= RCC_APB1ENR_USART3EN;
#endif

#ifdef CONFIG_STM32F0L0G0_USART4
  /* USART 4 clock enable */

  regval |= RCC_APB1ENR_USART4EN;
#endif

#ifdef CONFIG_STM32F0L0G0_I2C1
  /* I2C 1 clock enable */

  regval |= RCC_APB1ENR_I2C1EN;
#endif

#ifdef CONFIG_STM32F0L0G0_PWR
  /* Power interface clock enable */

  regval |= RCC_APB1ENR_PWREN;
#endif

  putreg32(regval, STM32_RCC_APB1ENR);
}

/****************************************************************************
 * Name: rcc_enableapb2
 *
 * Description:
 *   Enable selected APB peripherals from the second APB register.
 *
 ****************************************************************************/

static inline void rcc_enableapb2(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB2ENR register to enabled the
   * selected APB peripherals.
   */

  regval = getreg32(STM32_RCC_APB2ENR);

#ifdef CONFIG_STM32F0L0G0_SYSCFG
  /* SYSCFG clock */

  regval |= RCC_APB2ENR_SYSCFGEN;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM1
  /* TIM1 Timer clock enable */

  regval |= RCC_APB2ENR_TIM1EN;
#endif

#ifdef CONFIG_STM32F0L0G0_SPI1
  /* SPI 1 clock enable */

  regval |= RCC_APB2ENR_SPI1EN;
#endif

#ifdef CONFIG_STM32F0L0G0_USART1
  /* USART1 clock enable */

  regval |= RCC_APB2ENR_USART1EN;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM14
  /* TIM14 Timer clock enable */

  regval |= RCC_APB2ENR_TIM14EN;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM15
  /* TIM5 Timer clock enable */

  regval |= RCC_APB2ENR_TIM15EN;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM16
  /* TIM16 Timer clock enable */

  regval |= RCC_APB2ENR_TIM16EN;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM17
  /* TIM17 Timer clock enable */

  regval |= RCC_APB2ENR_TIM17EN;
#endif

#ifdef CONFIG_STM32F0L0G0_ADC1
  /* ADC 1 clock enable */

  regval |= RCC_APB2ENR_ADC1EN;
#endif

  putreg32(regval, STM32_RCC_APB2ENR);
}

/****************************************************************************
 * Name: stm32_rcc_enablehse
 *
 * Description:
 *   Enable the External High-Speed (HSE) Oscillator.
 *
 ****************************************************************************/

#if (STM32_SYSCLK_SW == RCC_CFGR_SW_HSE)
static inline bool stm32_rcc_enablehse(void)
{
  uint32_t regval;
  volatile int32_t timeout;

  /* Enable External High-Speed Clock (HSE) */

  regval  = getreg32(STM32_RCC_CR);
#ifdef STM32_HSEBYP_ENABLE          /* May be defined in board.h header file */
  regval |= RCC_CR_HSEBYP;          /* Enable HSE clock bypass */
#else
  regval &= ~RCC_CR_HSEBYP;         /* Disable HSE clock bypass */
#endif
  regval |= RCC_CR_HSEON;           /* Enable HSE */
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the HSE is ready (or until a timeout elapsed) */

  for (timeout = HSERDY_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if the HSERDY flag is set in the CR */

      if ((getreg32(STM32_RCC_CR) & RCC_CR_HSERDY) != 0)
        {
          /* If so, then return TRUE */

          return true;
        }
    }

  /* In the case of a timeout starting the HSE, we really don't have a
   * strategy.  This is almost always a hardware failure or misconfiguration.
   */

  return false;
}
#endif

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_STM32F0G0L0_CUSTOM_CLOCKCONFIG
static void stm32_stdclockconfig(void)
{
  uint32_t regval;
  uint32_t flash_ws;

  /* Flash wait states (latency) according to range and HCLK:
   *
   * - Flash 0WS if HCLK <= 24
   * - Flash 1WS if HCLK <= 48
   *
   * Where HCLK = (SYSCLK / HPRE div)
   */

  if (STM32_HCLK_FREQUENCY <= 24000000)
    {
      flash_ws = FLASH_ACR_LATENCY_0;
    }
  else
    {
      flash_ws = FLASH_ACR_LATENCY_1;
    }

  /* Enable the main source clock */

#if (STM32_SYSCLK_SW == RCC_CFGR_SW_HSE)

  /* System clock uses HSE */

  if (!stm32_rcc_enablehse())
    {
      /* In the case of a timeout starting the HSE, we really don't have a
       * strategy.  This is almost always a hardware failure or
       * misconfiguration (for example, if no crystal is fitted on the board.
       */

      return;
    }

#elif (STM32_SYSCLK_SW == RCC_CFGR_SW_HSI)

  /* System clock uses HSI */

  regval  = getreg32(STM32_RCC_CR);   /* Enable the HSI */
  regval |= RCC_CR_HSION;

  /* Configure HSI divider */

  regval &= ~RCC_CR_HSIDIV_MASK;
  regval |= STM32_RCC_HSIDIV;

  putreg32(regval, STM32_RCC_CR);

  /* Wait until the HSI clock is ready.  Since this is an internal clock, no
   * timeout is expected
   */

  while ((getreg32(STM32_RCC_CR) & RCC_CR_HSIRDY) == 0);

#endif

  /* Configure FLASH wait states and enable prefetch */

  regval  = getreg32(STM32_FLASH_ACR);
  regval &= ~FLASH_ACR_LATENCY_MASK;
  regval |= (flash_ws & FLASH_ACR_LATENCY_MASK) | FLASH_ACR_PRFTEN;
  putreg32(regval, STM32_FLASH_ACR);

  /* Set the HCLK source/divider */

  regval  = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_HPRE_MASK;
  regval |= STM32_RCC_CFGR_HPRE;
  putreg32(regval, STM32_RCC_CFGR);

  /* Set the PCLK divider */

  regval  = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_PPRE_MASK;
  regval |= STM32_RCC_CFGR_PPRE;
  putreg32(regval, STM32_RCC_CFGR);

  /* Select the system clock source */

  regval  = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_SW_MASK;
  regval |= STM32_SYSCLK_SW;
  putreg32(regval, STM32_RCC_CFGR);

  /* Wait until the selected source is used as the system clock source */

  while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) != STM32_SYSCLK_SWS);

#ifdef CONFIG_STM32F0L0G0_FDCAN1
  /* Configure FDCAN1 clock source */

  regval  = getreg32(STM32_RCC_CCIPR1);
  regval |= STM32_FDCAN1_SEL;
  putreg32(regval, STM32_RCC_CCIPR1);
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
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
