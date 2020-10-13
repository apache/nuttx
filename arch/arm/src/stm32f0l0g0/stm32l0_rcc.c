/****************************************************************************
 * arch/arm/src/stm32f0l0g0/stm32l0_rcc.c
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

#include "hardware/stm32_syscfg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * that is a very long delay, but if the clock does not become ready we are
 * hosed anyway.  Normally this is very fast, but I have seen at least one
 * board that required this long, long timeout for the HSE to be ready.
 */

#define HSERDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/* HSE divisor to yield ~1MHz RTC clock (valid for HSE = 8MHz) */

#define HSE_DIVISOR RCC_CR_RTCPRE_HSEd8

/* Determine if board wants to use HSI48 as 48 MHz oscillator. */

#if defined(CONFIG_STM32F0L0G0_HAVE_HSI48) && defined(STM32_USE_CLK48)
#  if STM32_CLK48_SEL == RCC_CCIPR_CLK48SEL_HSI48
#    define STM32_USE_HSI48
#    ifndef CONFIG_STM32F0L0G0_VREFINT
#      error VREFINT must be enabled if HSI48 used
#    endif
#  endif
#endif

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

  /* Set the appropriate bits in the APB2ENR register to enabled the
   * selected APB2 peripherals.
   */

  regval = getreg32(STM32_RCC_APB2ENR);

#if 1
  /* DBG clock enable */

  regval |= RCC_APB2ENR_DBGEN;
#endif

  putreg32(regval, STM32_RCC_APB2ENR);
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
             RCC_IOPENR_IOPDEN | RCC_IOPENR_IOPEEN | RCC_IOPENR_IOPHEN);

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

#ifdef CONFIG_STM32F0L0G0_TSC
  /* TSC clock enable */

  regval |= RCC_AHBENR_TSCEN;
#endif

#ifdef CONFIG_STM32F0L0G0_RNG
  /* Random number generator clock enable */

  regval |= RCC_AHBENR_RNGEN;
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

#ifdef CONFIG_STM32F0L0G0_LCD
  /* LCD clock enable */

  regval |= RCC_APB1ENR_LCDEN;
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

#ifdef CONFIG_STM32F0L0G0_I2C3
  /* I2C 3 clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB1ENR_I2C4EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_LPTIM1
  /* LPTIM1 clock enable */

  regval |= RCC_APB1ENR_LPTIM1EN;
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

#ifdef CONFIG_STM32F0L0G0_TIM21
  /* TIM21 Timer clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB2ENR_TIM21EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_TIM22
  /* TIM22 Timer clock enable */

#ifdef CONFIG_STM32F0L0G0_FORCEPOWER
  regval |= RCC_APB2ENR_TIM10EN;
#endif
#endif

#ifdef CONFIG_STM32F0L0G0_ADC1
  /* ADC 1 clock enable */

  regval |= RCC_APB2ENR_ADC1EN;
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

#if 0
  /* DBG clock enable */

  regval |= RCC_APB2ENR_DBGEN;
#endif

  putreg32(regval, STM32_RCC_APB2ENR);
}

/****************************************************************************
 * Name: rcc_enableccip
 *
 * Description:
 *   Set peripherals independent clock configuration.
 *
 ****************************************************************************/

static inline void rcc_enableccip(void)
{
  uint32_t regval;

  /* Certain peripherals have no clock selected even when their enable bit is
   * set. Set some defaults in the CCIPR register so those peripherals
   * will at least have a clock.
   */

  regval = getreg32(STM32_RCC_CCIPR);

#if defined(STM32_USE_CLK48)
  regval |= STM32_CLK48_SEL;
#endif

  putreg32(regval, STM32_RCC_CCIPR);
}

/****************************************************************************
 * Name: stm32_rcc_enablehse
 *
 * Description:
 *   Enable the External High-Speed (HSE) Oscillator.
 *
 ****************************************************************************/

#if (STM32_CFGR_PLLSRC == RCC_CFGR_PLLSRC) || (STM32_SYSCLK_SW == RCC_CFGR_SW_HSE)
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

#ifndef CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG
static void stm32_stdclockconfig(void)
{
  uint32_t regval;
#if defined(CONFIG_STM32F0L0G0_RTC_HSECLOCK) || defined(CONFIG_LCD_HSECLOCK)
  uint16_t pwrcr;
#endif
  uint32_t pwr_vos;
  bool flash_1ws;

  /* Enable PWR clock from APB1 to give access to PWR_CR register */

  regval  = getreg32(STM32_RCC_APB1ENR);
  regval |= RCC_APB1ENR_PWREN;
  putreg32(regval, STM32_RCC_APB1ENR);

  /* Go to the high performance voltage range 1 if necessary.  In this mode,
   * the PLL VCO frequency can be up to 96MHz.
   * USB and SDIO can be supported.
   *
   * Range 1: PLLVCO up to 96MHz in range 1 (1.8V)
   * Range 2: PLLVCO up to 48MHz in range 2 (1.5V) (default)
   * Range 3: PLLVCO up to 24MHz in range 3 (1.2V)
   *
   * Range 1: SYSCLK up to 32Mhz
   * Range 2: SYSCLK up to 16Mhz
   * Range 3: SYSCLK up to 4.2Mhz
   *
   * Range 1: Flash 1WS if SYSCLK > 16Mhz
   * Range 2: Flash 1WS if SYSCLK > 8Mhz
   * Range 3: Flash 1WS if SYSCLK > 2.1Mhz
   */

  pwr_vos   = PWR_CR_VOS_SCALE_2;
  flash_1ws = false;

#ifdef STM32_PLL_FREQUENCY
  if (STM32_PLL_FREQUENCY > 48000000)
    {
      pwr_vos = PWR_CR_VOS_SCALE_1;
    }
#endif

  if (STM32_SYSCLK_FREQUENCY > 16000000)
    {
      pwr_vos = PWR_CR_VOS_SCALE_1;
    }

  if ((pwr_vos == PWR_CR_VOS_SCALE_1 && STM32_SYSCLK_FREQUENCY > 16000000) ||
      (pwr_vos == PWR_CR_VOS_SCALE_2 && STM32_SYSCLK_FREQUENCY > 8000000))
    {
      flash_1ws = true;
    }

  stm32_pwr_setvos(pwr_vos);

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
    ((STM32_SYSCLK_SW == RCC_CFGR_SW_PLL) && (STM32_CFGR_PLLSRC == RCC_CFGR_PLLSRC))

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
      ((STM32_SYSCLK_SW == RCC_CFGR_SW_PLL) && STM32_CFGR_PLLSRC == 0)

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

#if (STM32_SYSCLK_SW != RCC_CFGR_SW_MSI)
  /* Increasing the CPU frequency (in the same voltage range):
   *
   * After reset, the used clock is the MSI (2 MHz) with 0 WS configured in
   * the FLASH_ACR register. 32-bit access is enabled and prefetch is
   * disabled. ST strongly recommends to use the following software sequences
   * to tune the number of wait states needed to access the Flash memory with
   * the CPU frequency.
   *
   *   - Program the 64-bit access by setting the ACC64 bit in Flash access
   *     control register (FLASH_ACR)
   *   - Check that 64-bit access is taken into account by reading FLASH_ACR
   *   - Program 1 WS to the LATENCY bit in FLASH_ACR
   *   - Check that the new number of WS is taken into account by reading
   *     FLASH_ACR
   *   - Modify the CPU clock source by writing to the SW bits in the Clock
   *     configuration register (RCC_CFGR)
   *   - If needed, modify the CPU clock prescaler by writing to the HPRE
   *     bits in RCC_CFGR
   *   - Check that the new CPU clock source or/and the new CPU clock
   *     prescaler value is/are taken into account by reading the clock
   *     source status (SWS bits) or/and the AHB prescaler value (HPRE bits),
   *     respectively, in the RCC_CFGR register
   */

  regval = getreg32(STM32_FLASH_ACR);
  regval |= FLASH_ACR_ACC64;          /* 64-bit access mode */
  putreg32(regval, STM32_FLASH_ACR);

  if (flash_1ws)
    {
      regval |= FLASH_ACR_LATENCY;    /* One wait state */
    }
  else
    {
      regval &= ~FLASH_ACR_LATENCY;   /* Zero wait state */
    }

  putreg32(regval, STM32_FLASH_ACR);

  /* Enable FLASH prefetch */

  regval |= FLASH_ACR_PRFTEN;
  putreg32(regval, STM32_FLASH_ACR);

#endif /* STM32_SYSCLK_SW != RCC_CFGR_SW_MSI */

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

  /* If we are using the PLL, configure and start it */

#if STM32_SYSCLK_SW == RCC_CFGR_SW_PLL

  /* Set the PLL divider and multiplier.  NOTE:  The PLL needs to be disabled
   * to do these operation.  We know this is the case here because
   * pll_reset() was previously called by stm32_clockconfig().
   */

  regval  = getreg32(STM32_RCC_CFGR);
  regval &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL_MASK | RCC_CFGR_PLLDIV_MASK);
  regval |= (STM32_CFGR_PLLSRC | STM32_CFGR_PLLMUL | STM32_CFGR_PLLDIV);
  putreg32(regval, STM32_RCC_CFGR);

  /* Enable the PLL */

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_PLLON;
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the PLL is ready */

  while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLRDY) == 0);

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
 * Name: vrefint_enable
 *
 * Description:
 *   Enable and configure internal voltage reference (VREFINT)
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F0L0G0_VREFINT
static void vrefint_enable(void)
{
  uint32_t regval = 0;

  /* The HSI48 requires VREFINT and its reference to HSI48  */

  regval = getreg32(STM32_SYSCFG_CFGR3);

  /* Enable VREFINT */

  regval |= SYSCFG_CFGR3_ENVREFINT;
  putreg32(regval, STM32_SYSCFG_CFGR3);

#ifdef STM32_USE_HSI48
  /* Enable VREFINT reference to HSI48 */

  regval |= SYSCFG_CFGR3_ENBUFVREFINTHSI48;
#endif

  /* Wait for VREFINT ready */

  while ((getreg32(STM32_SYSCFG_CFGR3) & SYSCFG_CFGR3_VREFINTRDYF) == 0);
}
#endif

/****************************************************************************
 * Name: rcc_enableperiphals
 ****************************************************************************/

static inline void rcc_enableperipherals(void)
{
  rcc_enableccip();
  rcc_enableio();
  rcc_enableahb();
  rcc_enableapb2();
  rcc_enableapb1();
#ifdef CONFIG_STM32F0L0G0_VREFINT
  vrefint_enable();
#endif

#ifdef STM32_USE_HSI48
  /* Enable HSI48 clocking to support USB transfers or RNG */

  stm32_enable_hsi48(STM32_HSI48_SYNCSRC);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
