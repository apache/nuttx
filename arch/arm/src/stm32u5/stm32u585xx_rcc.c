/****************************************************************************
 * arch/arm/src/stm32u5/stm32u585xx_rcc.c
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
#include <arch/stm32u5/chip.h>
#include <arch/board/board.h>

#include "stm32_pwr.h"
#include "stm32_flash.h"
#include "stm32_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * that is a very long delay, but if the clock does not become ready we are
 * hosed anyway.  Normally this is very fast, but I have seen at least one
 * board that required this long, long timeout for the HSE to be ready.
 */

#define HSERDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/* Same for HSI and MSI */

#define HSIRDY_TIMEOUT HSERDY_TIMEOUT
#define MSIRDY_TIMEOUT HSERDY_TIMEOUT

/* HSE divisor to yield ~1MHz RTC clock */

#define HSE_DIVISOR (STM32_HSE_FREQUENCY + 500000) / 1000000

/* Determine if board wants to use HSI48 as 48 MHz oscillator. */

#if defined(CONFIG_STM32U5_HAVE_HSI48) && defined(STM32_USE_CLK48)
#  if STM32_CLK48_SEL == RCC_CCIPR_CLK48SEL_HSI48
#    define STM32_USE_HSI48
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rcc_enableahb1
 *
 * Description:
 *   Enable selected AHB1 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb1(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB1ENR register to enabled the clocks
   * of selected AHB1 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB1ENR);

#ifdef CONFIG_STM32U5_GPDMA1
  regval |= RCC_AHB1ENR_GPDMA1EN;
#endif

#ifdef CONFIG_STM32U5_CORDIC
  regval |= RCC_AHB1ENR_CORDIC;
#endif

#ifdef CONFIG_STM32U5_FMAC
  regval |= RCC_AHB1ENR_FMACEN;
#endif

#ifdef CONFIG_STM32U5_MDF1
  regval |= RCC_AHB1ENR_MDF1EN;
#endif

#ifdef CONFIG_STM32U5_FLASH
  regval |= RCC_AHB1ENR_FLASHEN;
#endif

#ifdef CONFIG_STM32U5_CRC
  regval |= RCC_AHB1ENR_CRCEN;
#endif

#ifdef CONFIG_STM32U5_TSC
  regval |= RCC_AHB1ENR_TSCEN;
#endif

#ifdef CONFIG_STM32U5_RAMCFG
  regval |= RCC_AHB1ENR_RAMCFGEN;
#endif

#ifdef CONFIG_STM32U5_DMA2D
  regval |= RCC_AHB1ENR_DMA2DEN;
#endif

#ifdef CONFIG_STM32U5_GTZC1
  regval |= RCC_AHB1ENR_GTZC1EN;
#endif

#ifdef CONFIG_STM32U5_BKPSRAM
  regval |= RCC_AHB1ENR_BKPSRAMEN;
#endif

#ifdef CONFIG_STM32U5_DCACHE1
  regval |= RCC_AHB1ENR_DCACHE1EN;
#endif

#ifdef CONFIG_STM32U5_SRAM1
  regval |= RCC_AHB1ENR_SRAM1EN;
#endif

  putreg32(regval, STM32_RCC_AHB1ENR);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableahb2
 *
 * Description:
 *   Enable selected AHB2 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb2(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB2ENR1 and AHB2ENR2 registers to
   * enable the clocks of selected AHB2 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB2ENR1);

#if STM32_NPORTS > 0
  regval |= (RCC_AHB2ENR1_GPIOAEN
#if STM32_NPORTS > 1
             | RCC_AHB2ENR1_GPIOBEN
#endif
#if STM32_NPORTS > 2
             | RCC_AHB2ENR1_GPIOCEN
#endif
#if STM32_NPORTS > 3
             | RCC_AHB2ENR1_GPIODEN
#endif
#if STM32_NPORTS > 4
             | RCC_AHB2ENR1_GPIOEEN
#endif
#if STM32_NPORTS > 5
             | RCC_AHB2ENR1_GPIOFEN
#endif
#if STM32_NPORTS > 6
             | RCC_AHB2ENR1_GPIOGEN
#endif
#if STM32_NPORTS > 7
             | RCC_AHB2ENR1_GPIOHEN
#endif
#if STM32_NPORTS > 8
             | RCC_AHB2ENR1_GPIOIEN
#endif
       );
#endif

#if defined(CONFIG_STM32U5_ADC1)
  regval |= RCC_AHB2ENR1_ADC1EN;
#endif

#if defined(CONFIG_STM32U5_DCMI_PSSI)
  regval |= RCC_AHB2ENR1_DCMI_PSSIEN;
#endif

#ifdef CONFIG_STM32U5_OTG
  regval |= RCC_AHB2ENR1_OTGEN;
#endif

#ifdef CONFIG_STM32U5_AES
  regval |= RCC_AHB2ENR1_AESEN;
#endif

#ifdef CONFIG_STM32U5_HASH
  regval |= RCC_AHB2ENR1_HASHEN
#endif

#ifdef CONFIG_STM32U5_RNG
  regval |= RCC_AHB2ENR1_RNGEN;
#endif

#ifdef CONFIG_STM32U5_PKA
  regval |= RCC_AHB2ENR_PKAEN;
#endif

#ifdef CONFIG_STM32U5_SAES
  regval |= RCC_AHB2ENR1_SAES;
#endif

#ifdef CONFIG_STM32U5_OCTOSPIM
  regval |= RCC_AHB2ENR1_OCTOSPIM;
#endif

#ifdef CONFIG_STM32U5_OTFDEC1
  regval |= RCC_AHB2ENR1_OTFDEC1;
#endif

#ifdef CONFIG_STM32U5_OTFDEC2
  regval |= RCC_AHB2ENR1_OTFDEC2;
#endif

#ifdef CONFIG_STM32U5_SDMMC1EN
  regval |= RCC_AHB2ENR1_SDMMC1EN;
#endif

#ifdef CONFIG_STM32U5_SDMMC2EN
  regval |= RCC_AHB2ENR1_SDMMC2EN;
#endif

#ifdef CONFIG_STM32U5_SRAM2
  regval |= RCC_AHB2ENR1_SRAM2EN;
#endif

#ifdef CONFIG_STM32U5_SRAM3
  regval |= RCC_AHB2ENR1_SRAM3EN;
#endif

  putreg32(regval, STM32_RCC_AHB2ENR1);

  regval = getreg32(STM32_RCC_AHB2ENR2);

#ifdef CONFIG_STM32U5_FSMC
  regval |= RCC_AHB2ENR2_FSMCEN;
#endif

#ifdef CONFIG_STM32U5_OCTOSPI1
  regval |= RCC_AHB2ENR2_OCTOSPI1EN;
#endif

#ifdef CONFIG_STM32U5_OCTOSPI2
  regval |= RCC_AHB2ENR2_OCTOSPI2EN;
#endif

  putreg32(regval, STM32_RCC_AHB2ENR2);
}

/****************************************************************************
 * Name: rcc_enableahb3
 *
 * Description:
 *   Enable selected AHB3 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb3(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the AHB3ENR register to enabled the clocks
   * of selected AHB3 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB3ENR);

#ifdef CONFIG_STM32U5_LPGPIO1
  regval |= RCC_AHB3ENR_LPGPIO1EN;
#endif

#ifdef CONFIG_STM32U5_PWR
  regval |= RCC_AHB3ENR_PWREN;
#endif

#ifdef CONFIG_STM32U5_ADC4
  regval |= RCC_AHB3ENR_ADC4EN;
#endif

#ifdef CONFIG_STM32U5_DAC1
  regval |= RCC_AHB3ENR_DAC1EN;
#endif

#ifdef CONFIG_STM32U5_LPDMA1
  regval |= RCC_AHB3ENR_LPDMA1EN;
#endif

#ifdef CONFIG_STM32U5_ADF1
  regval |= RCC_AHB3ENR_ADF1EN;
#endif

#ifdef CONFIG_STM32U5_GTZC2
  regval |= RCC_AHB3ENR_GTZC2EN;
#endif

#ifdef CONFIG_STM32U5_SRAM4
  regval |= RCC_AHB3ENR_SRAM4EN;
#endif

  putreg32(regval, STM32_RCC_AHB3ENR);   /* Enable peripherals */
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

  /* Set the appropriate bits in the APB1ENR register to enabled the clocks
   * of selected APB1 peripherals.
   */

  regval = getreg32(STM32_RCC_APB1ENR1);

#ifdef CONFIG_STM32U5_TIM2
  regval |= RCC_APB1ENR1_TIM2EN;
#endif

#ifdef CONFIG_STM32U5_TIM3
  regval |= RCC_APB1ENR1_TIM3EN;
#endif

#ifdef CONFIG_STM32U5_TIM4
  regval |= RCC_APB1ENR1_TIM4EN;
#endif

#ifdef CONFIG_STM32U5_TIM5
  regval |= RCC_APB1ENR1_TIM5EN;
#endif

#ifdef CONFIG_STM32U5_TIM6
  regval |= RCC_APB1ENR1_TIM6EN;
#endif

#ifdef CONFIG_STM32U5_TIM7
  regval |= RCC_APB1ENR1_TIM7EN;
#endif

#ifdef CONFIG_STM32U5_WWDG
  regval |= RCC_APB1ENR1_WWDGEN;
#endif

#ifdef CONFIG_STM32U5_SPI2
  regval |= RCC_APB1ENR1_SPI2EN;
#endif

#ifdef CONFIG_STM32U5_USART2
  regval |= RCC_APB1ENR1_USART2EN;
#endif

#ifdef CONFIG_STM32U5_USART3
  regval |= RCC_APB1ENR1_USART3EN;
#endif

#ifdef CONFIG_STM32U5_UART4
  regval |= RCC_APB1ENR1_UART4EN;
#endif

#ifdef CONFIG_STM32U5_UART5
  regval |= RCC_APB1ENR1_UART5EN;
#endif

#ifdef CONFIG_STM32U5_I2C1
  regval |= RCC_APB1ENR1_I2C1EN;
#endif

#ifdef CONFIG_STM32U5_I2C2
  regval |= RCC_APB1ENR1_I2C2EN;
#endif

#ifdef CONFIG_STM32U5_CRS
  regval |= RCC_APB1ENR1_CRSEN;
#endif

  putreg32(regval, STM32_RCC_APB1ENR1);   /* Enable peripherals */

  /* Second APB1 register */

  regval = getreg32(STM32_RCC_APB1ENR2);

#ifdef CONFIG_STM32U5_I2C4
  regval |= RCC_APB1ENR2_I2C4EN;
#endif

#ifdef CONFIG_STM32U5_LPTIM2
  regval |= RCC_APB1ENR2_LPTIM2EN;
#endif

#ifdef CONFIG_STM32U5_FDCAN1
  regval |= RCC_APB1ENR2_FDCAN1EN;
#endif

#ifdef CONFIG_STM32U5_UCPD1
  regval |= RCC_APB1ENR2_UCPD1EN;
#endif

  putreg32(regval, STM32_RCC_APB1ENR2);
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

  /* Set the appropriate bits in the APB2ENR register to enabled the clocks
   * of selected APB2 peripherals.
   */

  regval = getreg32(STM32_RCC_APB2ENR);

#ifdef CONFIG_STM32U5_TIM1
  regval |= RCC_APB2ENR_TIM1EN;
#endif

#ifdef CONFIG_STM32U5_SPI1
  regval |= RCC_APB2ENR_SPI1EN;
#endif

#ifdef CONFIG_STM32U5_TIM8
  regval |= RCC_APB2ENR_TIM8EN;
#endif

#ifdef CONFIG_STM32U5_USART1
  regval |= RCC_APB2ENR_USART1EN;
#endif

#ifdef CONFIG_STM32U5_TIM15
  regval |= RCC_APB2ENR_TIM15EN;
#endif

#ifdef CONFIG_STM32U5_TIM16
  regval |= RCC_APB2ENR_TIM16EN;
#endif

#ifdef CONFIG_STM32U5_TIM17
  regval |= RCC_APB2ENR_TIM17EN;
#endif

#ifdef CONFIG_STM32U5_SAI1
  regval |= RCC_APB2ENR_SAI1EN;
#endif

#ifdef CONFIG_STM32U5_SAI2
  regval |= RCC_APB2ENR_SAI2EN;
#endif

  putreg32(regval, STM32_RCC_APB2ENR);
}

/****************************************************************************
 * Name: rcc_enableapb3
 *
 * Description:
 *   Enable selected APB3 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb3(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the APB3ENR register to enabled the clocks
   * of selected APB3 peripherals.
   */

  regval = getreg32(STM32_RCC_APB3ENR);

#ifdef CONFIG_STM32U5_SYSCFG
  regval |= RCC_APB3ENR_SYSCFGEN;
#endif

#ifdef CONFIG_STM32U5_SPI3
  regval |= RCC_APB3ENR_SPI3EN;
#endif

#ifdef CONFIG_STM32U5_LPUART1
  regval |= RCC_APB3ENR_LPUART1EN;
#endif

#ifdef CONFIG_STM32U5_I2C3EN
  regval |= RCC_APB3ENR_I2C3EN;
#endif

#ifdef CONFIG_STM32U5_LPTIM1
  regval |= RCC_APB3ENR_LPTIM1EN;
#endif

#ifdef CONFIG_STM32U5_LPTIM3
  regval |= RCC_APB3ENR_LPTIM3EN;
#endif

#ifdef CONFIG_STM32U5_LPTIM4
  regval |= RCC_APB3ENR_LPTIM4EN;
#endif

#ifdef CONFIG_STM32U5_OPAMP
  regval |= RCC_APB3ENR_OPAMPEN;
#endif

#ifdef CONFIG_STM32U5_COMP
  regval |= RCC_APB3ENR_COMPEN;
#endif

#ifdef CONFIG_STM32U5_VREF
  regval |= RCC_APB3ENR_VREFEN;
#endif

#ifdef CONFIG_STM32U5_RTCAPB
  regval |= RCC_APB3ENR_RTCAPBEN;
#endif

  putreg32(regval, STM32_RCC_APB3ENR);
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
  /* Certain peripherals have no clock selected even when their enable bit is
   * set. Set some defaults in the CCIPR register so those peripherals
   * will at least have a clock.
   */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_rcc_enableperipherals
 ****************************************************************************/

void stm32_rcc_enableperipherals(void)
{
  rcc_enableccip();
  rcc_enableahb1();
  rcc_enableahb2();
  rcc_enableahb3();
  rcc_enableapb1();
  rcc_enableapb2();
  rcc_enableapb3();
}

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h
 *
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes!
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_STM32U5_CUSTOM_CLOCKCONFIG
void stm32_stdclockconfig(void)
{
  uint32_t regval;
  volatile int32_t timeout;

  /* Enable Internal Multi-Speed System (MSIS) and Kernel (MSIK) Clock */

#if defined(STM32_BOARD_USEHSI) || defined(STM32_I2C_USE_HSI16)
  /* Enable Internal High-Speed Clock (HSI) */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSION;           /* Enable HSI */
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the HSI is ready (or until a timeout elapsed) */

  for (timeout = HSIRDY_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if the HSIRDY flag is the set in the CR */

      if ((getreg32(STM32_RCC_CR) & RCC_CR_HSIRDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }
#endif

#if defined(STM32_BOARD_USEHSI)
  /* Already set above */

#elif defined(STM32_BOARD_USEMSI)
  /* Enable Internal Multi-Speed Clock (MSI) */

  /* Wait until the MSI is either off or ready (or until a timeout elapsed) */

  for (timeout = MSIRDY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((regval = getreg32(STM32_RCC_CR)),
          (regval & RCC_CR_MSIRDY) || ~(regval & RCC_CR_MSION))
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }

  /* setting MSIRANGE */

  regval  = getreg32(STM32_RCC_CR);
  regval |= (STM32_BOARD_MSIRANGE | RCC_CR_MSION);    /* Enable MSI and frequency */
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the MSI is ready (or until a timeout elapsed) */

  for (timeout = MSIRDY_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if the MSIRDY flag is the set in the CR */

      if ((getreg32(STM32_RCC_CR) & RCC_CR_MSIRDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }

#elif defined(STM32_BOARD_USEHSE)
  /* Enable External High-Speed Clock (HSE) */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSEON;           /* Enable HSE */
  putreg32(regval, STM32_RCC_CR);

  /* Wait until the HSE is ready (or until a timeout elapsed) */

  for (timeout = HSERDY_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if the HSERDY flag is the set in the CR */

      if ((getreg32(STM32_RCC_CR) & RCC_CR_HSERDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }
#else

#  error stm32_stdclockconfig(), must have one of STM32_BOARD_USEHSI, STM32_BOARD_USEMSI, STM32_BOARD_USEHSE defined

#endif

  /* Check for a timeout.  If this timeout occurs, then we are hosed.  We
   * have no real back-up plan, although the following logic makes it look
   * as though we do.
   */

  if (timeout > 0)
    {
      /* Select main regulator voltage range according to system clock
       * frequency.
       */

      /* Ensure Power control is enabled before modifying it. */

      stm32_pwr_enableclk(true);

      /* Select correct main regulator range */

      regval = getreg32(STM32_PWR_CR1);
      regval &= ~PWR_CR1_VOS_MASK;

      if (STM32_SYSCLK_FREQUENCY > 80000000)
        {
          regval |= PWR_CR1_VOS_RANGE0;
        }
      else if (STM32_SYSCLK_FREQUENCY > 26000000)
        {
          regval |= PWR_CR1_VOS_RANGE1;
        }
      else
        {
          regval |= PWR_CR1_VOS_RANGE0;
        }

      putreg32(regval, STM32_PWR_CR1);

      /* Wait for voltage regulator to stabilize */

      while (getreg32(STM32_PWR_SR2) & PWR_SR2_VOSF)
        {
        }

      /* Set the HCLK source/divider */

      regval  = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_HPRE_MASK;
      regval |= STM32_RCC_CFGR_HPRE;
      putreg32(regval, STM32_RCC_CFGR);

      /* Set the PCLK2 divider */

      regval  = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_PPRE2_MASK;
      regval |= STM32_RCC_CFGR_PPRE2;
      putreg32(regval, STM32_RCC_CFGR);

      /* Set the PCLK1 divider */

      regval  = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_PPRE1_MASK;
      regval |= STM32_RCC_CFGR_PPRE1;
      putreg32(regval, STM32_RCC_CFGR);

#ifdef CONFIG_STM32U5_RTC_HSECLOCK
      /* Set the RTC clock divisor */

      regval  = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_RTCPRE_MASK;
      regval |= RCC_CFGR_RTCPRE(HSE_DIVISOR);
      putreg32(regval, STM32_RCC_CFGR);
#endif

      /* Set the PLL source and main divider */

      regval  = getreg32(STM32_RCC_PLLCFG);

      /* Configure Main PLL */

      /* Set the PLL dividers and multipliers to configure the main PLL */

      regval = (STM32_PLLCFG_PLLM | STM32_PLLCFG_PLLN |
                STM32_PLLCFG_PLLP | STM32_PLLCFG_PLLQ |
                STM32_PLLCFG_PLLR);

#ifdef STM32_PLLCFG_PLLP_ENABLED
      regval |= RCC_PLLCFG_PLLPEN;
#endif
#ifdef STM32_PLLCFG_PLLQ_ENABLED
      regval |= RCC_PLLCFG_PLLQEN;
#endif
#ifdef STM32_PLLCFG_PLLR_ENABLED
      regval |= RCC_PLLCFG_PLLREN;
#endif

      /* XXX The choice of clock source to PLL (all three) is independent
       * of the sys clock source choice, review the STM32_BOARD_USEHSI
       * name; probably split it into two, one for PLL source and one
       * for sys clock source.
       */

#ifdef STM32_BOARD_USEHSI
      regval |= RCC_PLLCFG_PLLSRC_HSI16;
#elif defined(STM32_BOARD_USEMSI)
      regval |= RCC_PLLCFG_PLLSRC_MSI;
#else /* if STM32_BOARD_USEHSE */
      regval |= RCC_PLLCFG_PLLSRC_HSE;
#endif

      putreg32(regval, STM32_RCC_PLLCFG);

      /* Enable the main PLL */

      regval  = getreg32(STM32_RCC_CR);
      regval |= RCC_CR_PLLON;
      putreg32(regval, STM32_RCC_CR);

      /* Wait until the PLL is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLRDY) == 0)
        {
        }

#ifdef CONFIG_STM32U5_SAI1PLL
      /* Configure SAI1 PLL */

      regval  = getreg32(STM32_RCC_PLLSAI1CFG);

      /* Set the PLL dividers and multipliers to configure the SAI1 PLL */

      regval = (STM32_PLLSAI1CFG_PLLN | STM32_PLLSAI1CFG_PLLP
                 | STM32_PLLSAI1CFG_PLLQ | STM32_PLLSAI1CFG_PLLR);

#ifdef STM32_PLLSAI1CFG_PLLP_ENABLED
      regval |= RCC_PLLSAI1CFG_PLLPEN;
#endif
#ifdef STM32_PLLSAI1CFG_PLLQ_ENABLED
      regval |= RCC_PLLSAI1CFG_PLLQEN;
#endif
#ifdef STM32_PLLSAI1CFG_PLLR_ENABLED
      regval |= RCC_PLLSAI1CFG_PLLREN;
#endif

      putreg32(regval, STM32_RCC_PLLSAI1CFG);

      /* Enable the SAI1 PLL */

      regval  = getreg32(STM32_RCC_CR);
      regval |= RCC_CR_PLLSAI1ON;
      putreg32(regval, STM32_RCC_CR);

      /* Wait until the PLL is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLSAI1RDY) == 0)
        {
        }
#endif

#ifdef CONFIG_STM32U5_SAI2PLL
      /* Configure SAI2 PLL */

      regval  = getreg32(STM32_RCC_PLLSAI2CFG);

      /* Set the PLL dividers and multipliers to configure the SAI2 PLL */

      regval = (STM32_PLLSAI2CFG_PLLN | STM32_PLLSAI2CFG_PLLP |
                STM32_PLLSAI2CFG_PLLR);

#ifdef STM32_PLLSAI2CFG_PLLP_ENABLED
      regval |= RCC_PLLSAI2CFG_PLLPEN;
#endif
#ifdef STM32_PLLSAI2CFG_PLLR_ENABLED
      regval |= RCC_PLLSAI2CFG_PLLREN;
#endif

      putreg32(regval, STM32_RCC_PLLSAI2CFG);

      /* Enable the SAI2 PLL */

      regval  = getreg32(STM32_RCC_CR);
      regval |= RCC_CR_PLLSAI2ON;
      putreg32(regval, STM32_RCC_CR);

      /* Wait until the PLL is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLSAI2RDY) == 0)
        {
        }
#endif

      /* Enable FLASH 5 wait states */

      regval = FLASH_ACR_LATENCY_5;
      putreg32(regval, STM32_FLASH_ACR);

      /* Select the main PLL as system clock source */

      regval  = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_SW_MASK;
      regval |= RCC_CFGR_SW_PLL;
      putreg32(regval, STM32_RCC_CFGR);

      /* Wait until the PLL source is used as the system clock source */

      while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) !=
             RCC_CFGR_SWS_PLL)
        {
        }

#if defined(CONFIG_STM32U5_IWDG) || defined(CONFIG_STM32U5_RTC_LSICLOCK)
      /* Low speed internal clock source LSI */

      stm32_rcc_enablelsi();
#endif

#if defined(STM32_USE_LSE)
      /* Low speed external clock source LSE
       *
       * TODO: There is another case where the LSE needs to
       * be enabled: if the MCO1 pin selects LSE as source.
       * XXX and other cases, like automatic trimming of MSI for USB use
       */

      /* ensure Power control is enabled since it is indirectly required
       * to alter the LSE parameters.
       */

      stm32_pwr_enableclk(true);

      /* XXX other LSE settings must be made before turning on the oscillator
       * and we need to ensure it is first off before doing so.
       */

      /* Turn on the LSE oscillator
       * XXX this will almost surely get moved since we also want to use
       * this for automatically trimming MSI, etc.
       */

      stm32_rcc_enablelse();

#  if defined(STM32_BOARD_USEMSI)
      /* Now that LSE is up, auto trim the MSI */

      regval  = getreg32(STM32_RCC_CR);
      regval |= RCC_CR_MSIPLLEN;
      putreg32(regval, STM32_RCC_CR);
#  endif
#endif /* STM32_USE_LSE */
    }
}
#endif
