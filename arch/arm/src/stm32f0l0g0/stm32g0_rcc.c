/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32g0_rcc.c
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

#include "stm32_pwr.h"

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

/* HSE divisor to yield ~1MHz RTC clock (valid for HSE = 8MHz) */

#define HSE_DIVISOR RCC_CR_RTCPRE_HSEd8

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

  /* Reset PLLCFGR register */

  putreg32(RCC_PLLCFGR_RESET, STM32_RCC_PLLCFG);

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

#ifdef CONFIG_STM32F0L0G0_LPUSART1
  /* USART 5 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB1ENR_LPUSART1EN;
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
#ifdef CONFIG_STM32F0L0G0_PWR
  /* Power interface clock enable */

  regval |= RCC_APB1ENR_PWREN;
#endif

#ifdef CONFIG_STM32F0L0G0_DAC1
  /* DAC 1 interface clock enable */

  regval |= RCC_APB1ENR_DAC1EN;
#endif

#ifdef CONFIG_STM32F0L0G0_LPTIM1
  /* LPTIM1 clock enable */

  regval |= RCC_APB1ENR_LPTIM1EN;
#endif

#ifdef CONFIG_STM32F0L0G0_LPTIM2
  /* LPTIM2 clock enable */

  regval |= RCC_APB1ENR_LPTIM2EN;
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

  regval |= RCC_APB2ENR_SYSCFGEN;
#endif

#ifdef CONFIG_STM32F0L0G0_TIM1
  /* TIM1 Timer clock enable */

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

#ifdef CONFIG_STM32F0L0G0_TIM14
  /* TIM14 Timer clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB2ENR_TIM14EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM15
  /* TIM5 Timer clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB2ENR_TIM15EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM16
  /* TIM16 Timer clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB2ENR_TIM16EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM17
  /* TIM17 Timer clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB2ENR_TIM17EN;
#endif
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

#if (STM32_PLLCFG_PLLSRC == RCC_PLLCFG_PLLSRC_HSE) || (STM32_SYSCLK_SW == RCC_CFGR_SW_HSE)
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
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes or any clocking other than PLL driven by the HSE.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_STM32F0G0L0_CUSTOM_CLOCKCONFIG
static void stm32_stdclockconfig(void)
{
  uint32_t regval;
#if defined(CONFIG_STM32F0L0G0_RTC_HSECLOCK) || defined(CONFIG_LCD_HSECLOCK)
  uint16_t pwrcr;
#endif
#ifdef CONFIG_STM32F0L0G0_PWR
  uint32_t pwr_vos;
#endif
  uint32_t flash_ws;

  /* Enable PWR clock from APB1 to give access to PWR_CR register */

  regval  = getreg32(STM32_RCC_APB1ENR);
  regval |= RCC_APB1ENR_PWREN;
  putreg32(regval, STM32_RCC_APB1ENR);

  /* Two voltage ranges are available:
   *
   * Range 1: High-performance range (default)
   *          Typical output voltage 1.2 V
   *          SYSLCK up to 64 MHz
   *
   * Range 2: Low-power range
   *          Typical output voltage 1.0V
   *          SYSLCK up to 16 MHz
   *
   * Flash wait states (latency) according to range and HCLK:
   *
   * Range 1:
   * - Flash 0WS if HCLK <= 24
   * - Flash 1WS if HCLK <= 48
   * - Flash 2WS if HCLK <= 64
   *
   * Range 2:
   * - Flash 0WS if HCLK <= 8
   * - Flash 1WS if HCLK <= 16
   *
   * Where HCLK = (SYSCLK / HPRE div)
   */

  if (STM32_SYSCLK_FREQUENCY > 16000000)
    {
#ifdef CONFIG_STM32F0L0G0_PWR
      pwr_vos = PWR_CR1_VOS_RANGE1;
#endif

      if (STM32_HCLK_FREQUENCY <= 24000000)
        {
          flash_ws = FLASH_ACR_LATENCY_0;
        }
      else if (STM32_HCLK_FREQUENCY <= 48000000)
        {
          flash_ws = FLASH_ACR_LATENCY_1;
        }
      else
        {
          flash_ws = FLASH_ACR_LATENCY_2;
        }
    }
  else
    {
#ifdef CONFIG_STM32F0L0G0_PWR
      pwr_vos = PWR_CR1_VOS_RANGE2;
#endif

      if (STM32_HCLK_FREQUENCY <= 8000000)
        {
          flash_ws = FLASH_ACR_LATENCY_0;
        }
      else
        {
          flash_ws = FLASH_ACR_LATENCY_1;
        }
    }

#ifdef CONFIG_STM32F0L0G0_PWR
  stm32_pwr_setvos(pwr_vos);
#endif

#if defined(CONFIG_STM32F0L0G0_RTC_HSECLOCK) || defined(CONFIG_LCD_HSECLOCK)
  /* If RTC / LCD selects HSE as clock source, the RTC prescaler
   * needs to be set before HSEON bit is set.
   */

  /* The RTC domain has write access denied after reset,
   * you have to enable write access using DBP bit in the PWR CR
   * register before to selecting the clock source ( and the PWR
   * peripheral must be enabled)
   */

  regval  = getreg32(STM32_RCC_APB1ENR);
  regval |= RCC_APB1ENR_PWREN;
  putreg32(regval, STM32_RCC_APB1ENR);

  pwrcr = getreg16(STM32_PWR_CR);
  putreg16(pwrcr | PWR_CR_DBP, STM32_PWR_CR);

  /* Set the RTC clock divisor */

  regval = getreg32(STM32_RCC_CSR);
  regval &= ~RCC_CSR_RTCSEL_MASK;
  regval |= RCC_CSR_RTCSEL_HSE;
  putreg32(regval, STM32_RCC_CSR);

  regval = getreg32(STM32_RCC_CR);
  regval &= ~RCC_CR_RTCPRE_MASK;
  regval |= HSE_DIVISOR;
  putreg32(regval, STM32_RCC_CR);

  /* Restore the previous state of the DBP bit */

  putreg32(regval, STM32_PWR_CR);

#endif

  /* Enable the source clock for the PLL (via HSE or HSI), HSE, and HSI. */

#if (STM32_SYSCLK_SW == RCC_CFGR_SW_HSE) || \
    ((STM32_SYSCLK_SW == RCC_CFGR_SW_PLL) && (STM32_PLLCFG_PLLSRC == RCC_PLLCFG_PLLSRC_HSE))

  /* The PLL is using the HSE, or the HSE is the system clock.  In either
   * case, we need to enable HSE clocking.
   */

  if (!stm32_rcc_enablehse())
    {
      /* In the case of a timeout starting the HSE, we really don't have a
       * strategy.  This is almost always a hardware failure or
       * misconfiguration (for example, if no crystal is fitted on the board.
       */

      return;
    }

#elif (STM32_SYSCLK_SW == RCC_CFGR_SW_HSI) || \
      ((STM32_SYSCLK_SW == RCC_CFGR_SW_PLL) && STM32_PLLCFG_PLLSRC == RCC_PLLCFG_PLLSRC_HSI)

  /* The PLL is using the HSI, or the HSI is the system clock.  In either
   * case, we need to enable HSI clocking.
   */

  regval  = getreg32(STM32_RCC_CR);   /* Enable the HSI */
  regval |= RCC_CR_HSION;
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

  /* Set the PCLK1 divider */

  regval  = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_PPRE1_MASK;
  regval |= STM32_RCC_CFGR_PPRE1;
  putreg32(regval, STM32_RCC_CFGR);

  /* If we are using the PLL, configure and start it */

#if STM32_SYSCLK_SW == RCC_CFGR_SW_PLL

  /* Configure PLLs */

  regval = STM32_PLLCFG_PLLSRC | STM32_PLLCFG_PLLCFG;

  /* Configure PLL clock input */

  regval |= STM32_PLLCFG_PLLM | STM32_PLLCFG_PLLN;

  /* Configure PLL clock outputs division */

#if defined(CONFIG_ARCH_CHIP_STM32G070KB) || defined(CONFIG_ARCH_CHIP_STM32G070CB) || \
    defined(CONFIG_ARCH_CHIP_STM32G070RB)
  regval |= STM32_PLLCFG_PLLP | STM32_PLLCFG_PLLR;
#else
  regval |= STM32_PLLCFG_PLLP | STM32_PLLCFG_PLLQ | STM32_PLLCFG_PLLR;
#endif

  /* Write PLLCFG register */

  putreg32(regval, STM32_RCC_PLLCFG);

  /* Enable PLL */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_PLLON;
  putreg32(regval, STM32_RCC_CR);
#endif

  /* Select the system clock source (probably the PLL) */

  regval  = getreg32(STM32_RCC_CFGR);
  regval &= ~RCC_CFGR_SW_MASK;
  regval |= STM32_SYSCLK_SW;
  putreg32(regval, STM32_RCC_CFGR);

  /* Wait until the selected source is used as the system clock source */

  while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) != STM32_SYSCLK_SWS);

#if defined(CONFIG_STM32F0L0G0_IWDG)   || \
    defined(CONFIG_STM32F0L0G0_RTC_LSICLOCK) || defined(CONFIG_LCD_LSICLOCK)
  /* Low speed internal clock source LSI
   *
   * TODO: There is another case where the LSI needs to
   * be enabled: if the MCO pin selects LSI as source.
   */

  stm32_rcc_enablelsi();

#endif

#if defined(CONFIG_STM32F0L0G0_RTC_LSECLOCK) || defined(CONFIG_LCD_LSECLOCK)
  /* Low speed external clock source LSE
   *
   * TODO: There is another case where the LSE needs to
   * be enabled: if the MCO pin selects LSE as source.
   *
   * TODO: There is another case where the LSE needs to
   * be enabled: if TIM9-10 Channel 1 selects LSE as input.
   *
   * TODO: There is another case where the LSE needs to
   * be enabled: if TIM10-11 selects LSE as ETR Input.
   *
   */

  stm32_rcc_enablelse();
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
