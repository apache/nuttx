/****************************************************************************
 * arch/arm/src/stm32wb/stm32wb_rcc.c
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

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "stm32wb_rcc.h"
#include "stm32wb_rtc.h"
#include "stm32wb_pwr.h"
#include "hardware/stm32wb_flash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * that is a very long delay, but if the clock does not become ready we are
 * hosed anyway.
 */

#define HSERDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/* Same for HSI and MSI */

#define HSIRDY_TIMEOUT HSERDY_TIMEOUT
#define MSIRDY_TIMEOUT HSERDY_TIMEOUT

/* Determine if board wants to use HSI48 as 48 MHz oscillator. */

#if defined(CONFIG_STM32WB_HAVE_HSI48) && defined(STM32WB_USE_CLK48)
#  if STM32WB_CLK48_SEL == RCC_CCIPR_CLK48SEL_HSI48
#    define STM32WB_USE_HSI48
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
 *   Reset the RCC clock configuration to the default reset state
 *
 ****************************************************************************/

static inline void rcc_reset(void)
{
  uint32_t regval;

  /* Enable the Multi-Speed Internal clock (MSI) @ 4MHz */

  regval  = getreg32(STM32WB_RCC_CR);
  regval &= ~RCC_CR_MSIRANGE_MASK;
  regval |= RCC_CR_MSIRANGE_4M | RCC_CR_MSION;
  putreg32(regval, STM32WB_RCC_CR);

  /* Reset CFGR register */

  regval  = getreg32(STM32WB_RCC_CFGR);
  regval &= RCC_CFGR_RESET_MASK;
  putreg32(regval, STM32WB_RCC_CFGR);

  /* Reset PLLSAI1ON, PLLON, HSECSSON, HSEON, HSION, and MSIPLLON bits */

  regval  = getreg32(STM32WB_RCC_CR);
  regval &= ~(RCC_CR_PLLON | RCC_CR_PLLSAI1ON | RCC_CR_CSSON |
              RCC_CR_HSEON | RCC_CR_HSION | RCC_CR_MSIPLLEN);
  putreg32(regval, STM32WB_RCC_CR);

  /* Reset LSI1 and LSI2 bits */

  regval  = getreg32(STM32WB_RCC_CSR);
  regval &= ~(RCC_CSR_LSI1ON | RCC_CSR_LSI2ON);
  putreg32(regval, STM32WB_RCC_CSR);

  /* Reset HSI48ON bit */

  regval  = getreg32(STM32WB_RCC_CRRCR);
  regval &= ~(RCC_CRRCR_HSI48ON);
  putreg32(regval, STM32WB_RCC_CRRCR);

  /* Reset PLLCFGR register */

  putreg32(RCC_PLLCFG_RESET, STM32WB_RCC_PLLCFG);

  /* Reset PLLSAI1CFG register */

  putreg32(RCC_PLLSAI1CFG_RESET, STM32WB_RCC_PLLSAI1CFG);

  /* Disable all interrupts */

  putreg32(0x00000000, STM32WB_RCC_CIER);
}

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

  /* Set the appropriate bits in the AHB1ENR register to enable the
   * selected AHB1 peripherals.
   */

  regval = getreg32(STM32WB_RCC_AHB1ENR);

#ifdef CONFIG_STM32WB_DMA1
  /* DMA 1 clock enable */

  regval |= RCC_AHB1ENR_DMA1EN;
#endif

#ifdef CONFIG_STM32WB_DMA2
  /* DMA 2 clock enable */

  regval |= RCC_AHB1ENR_DMA2EN;
#endif

#ifdef CONFIG_STM32WB_DMAMUX
  /* DMAMUX 1 clock enable */

  regval |= RCC_AHB1ENR_DMAMUX1EN;
#endif

#ifdef CONFIG_STM32WB_CRC
  /* CRC clock enable */

  regval |= RCC_AHB1ENR_CRCEN;
#endif

#ifdef CONFIG_STM32WB_TSC
  /* TSC clock enable */

  regval |= RCC_AHB1ENR_TSCEN;
#endif

  putreg32(regval, STM32WB_RCC_AHB1ENR);   /* Enable peripherals */
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

  /* Set the appropriate bits in the AHB2ENR register to enabled the
   * selected AHB2 peripherals.
   */

  regval = getreg32(STM32WB_RCC_AHB2ENR);

  /* Enable GPIO ports A-E, H */

  regval |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN
#if defined(CONFIG_STM32WB_GPIO_HAVE_PORTD)
             | RCC_AHB2ENR_GPIODEN
#endif
#if defined(CONFIG_STM32WB_GPIO_HAVE_PORTE)
             | RCC_AHB2ENR_GPIOEEN
#endif
             | RCC_AHB2ENR_GPIOHEN);

#if defined(CONFIG_STM32WB_ADC1)
  /* ADC clock enable */

  regval |= RCC_AHB2ENR_ADCEN;
#endif

#ifdef CONFIG_STM32WB_AES1
  /* AES1 cryptographic accelerator clock enable */

  regval |= RCC_AHB2ENR_AES1EN;
#endif

  putreg32(regval, STM32WB_RCC_AHB2ENR);   /* Enable peripherals */
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

  /* Set the appropriate bits in the AHB3ENR register to enabled the
   * selected AHB3 peripherals.
   */

  regval = getreg32(STM32WB_RCC_AHB3ENR);

#ifdef CONFIG_STM32WB_QSPI
  /* QuadSPI module clock enable */

  regval |= RCC_AHB3ENR_QSPIEN;
#endif

#ifdef CONFIG_STM32WB_PKA
  /* Public key accelerator clock enable */

  regval |= RCC_AHB3ENR_PKAEN;
#endif

#ifdef CONFIG_STM32WB_AES2
  /* AES2 cryptographic accelerator clock enable */

  regval |= RCC_AHB3ENR_AES2EN;
#endif

#ifdef CONFIG_STM32WB_RNG
  /* Random number generator clock enable */

  regval |= RCC_AHB3ENR_RNGEN;
#endif

#ifdef CONFIG_STM32WB_HSEM
  /* Hardware semaphore clock enable */

  regval |= RCC_AHB3ENR_HSEMEN;
#endif

#ifdef CONFIG_STM32WB_IPCC
  /* Inter-processor communication controller clock enable */

  regval |= RCC_AHB3ENR_IPCCEN;
#endif

#ifdef CONFIG_STM32WB_FLASH
  /* Flash memory interface clock enable */

  regval |= RCC_AHB3ENR_FLASHEN;
#endif

  putreg32(regval, STM32WB_RCC_AHB3ENR);   /* Enable peripherals */
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

  regval = getreg32(STM32WB_RCC_APB1ENR1);

#ifdef CONFIG_STM32WB_TIM2
  /* TIM2 clock enable */

  regval |= RCC_APB1ENR1_TIM2EN;
#endif

#ifdef CONFIG_STM32WB_LCD
  /* LCD clock enable */

  regval |= RCC_APB1ENR1_LCDEN;
#endif

#if defined(CONFIG_STM32WB_RTC)
  /* RTC APB clock enable */

  regval |= RCC_APB1ENR1_RTCAPBEN;
#endif

#if defined(CONFIG_STM32WB_WWDG)
  /* Window watchdog clock enable */

  regval |= RCC_APB1ENR1_WWDGEN;
#endif

#ifdef CONFIG_STM32WB_SPI2
  /* SPI2 clock enable */

  regval |= RCC_APB1ENR1_SPI2EN;
#endif

#ifdef CONFIG_STM32WB_I2C1
  /* I2C1 clock enable */

  regval |= RCC_APB1ENR1_I2C1EN;
#endif

#ifdef CONFIG_STM32WB_I2C3
  /* I2C3 clock enable */

  regval |= RCC_APB1ENR1_I2C3EN;
#endif

#ifdef STM32WB_USE_HSI48
  if (STM32WB_HSI48_SYNCSRC != SYNCSRC_NONE)
    {
      /* Clock Recovery System clock enable */

      regval |= RCC_APB1ENR1_CRSEN;
    }
#endif

#if defined(CONFIG_STM32WB_USB)
  /* USB clock enable */

  regval |= RCC_APB1ENR1_USBEN;
#endif

#ifdef CONFIG_STM32WB_LPTIM1
  /* Low power timer 1 clock enable */

  regval |= RCC_APB1ENR1_LPTIM1EN;
#endif

  putreg32(regval, STM32WB_RCC_APB1ENR1);   /* Enable peripherals */

  /* Second APB1 register */

  regval = getreg32(STM32WB_RCC_APB1ENR2);

#ifdef CONFIG_STM32WB_LPUART1
  /* Low power uart clock enable */

  regval |= RCC_APB1ENR2_LPUART1EN;
#endif

#ifdef CONFIG_STM32WB_LPTIM2
  /* Low power timer 2 clock enable */

  regval |= RCC_APB1ENR2_LPTIM2EN;
#endif

  putreg32(regval, STM32WB_RCC_APB1ENR2);   /* Enable peripherals */
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

  regval = getreg32(STM32WB_RCC_APB2ENR);

#ifdef CONFIG_STM32WB_TIM1
  /* TIM1 clock enable */

  regval |= RCC_APB2ENR_TIM1EN;
#endif

#ifdef CONFIG_STM32WB_SPI1
  /* SPI1 clock enable */

  regval |= RCC_APB2ENR_SPI1EN;
#endif

#ifdef CONFIG_STM32WB_USART1
  /* USART1 clock enable */

  regval |= RCC_APB2ENR_USART1EN;
#endif

#ifdef CONFIG_STM32WB_TIM16
  /* TIM16 clock enable */

  regval |= RCC_APB2ENR_TIM16EN;
#endif

#ifdef CONFIG_STM32WB_TIM17
  /* TIM17 clock enable */

  regval |= RCC_APB2ENR_TIM17EN;
#endif

#ifdef CONFIG_STM32WB_SAI1
  /* SAI1 clock enable */

  regval |= RCC_APB2ENR_SAI1EN;
#endif

  putreg32(regval, STM32WB_RCC_APB2ENR);   /* Enable peripherals */
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

  regval = getreg32(STM32WB_RCC_CCIPR);

#if defined(STM32WB_I2C_USE_HSI16)
#ifdef CONFIG_STM32WB_I2C1
  /* Select HSI16 as I2C1 clock source. */

  regval &= ~RCC_CCIPR_I2C1SEL_MASK;
  regval |= RCC_CCIPR_I2C1SEL_HSI16;
#endif
#ifdef CONFIG_STM32WB_I2C3
  /* Select HSI16 as I2C3 clock source. */

  regval &= ~RCC_CCIPR_I2C3SEL_MASK;
  regval |= RCC_CCIPR_I2C3SEL_HSI16;
#endif
#endif /* STM32WB_I2C_USE_HSI16 */

#if defined(STM32WB_USE_CLK48)
  /* XXX sanity if usb or rng, then we need to set the clk48 source
   * and then we can also do away with STM32WB_USE_CLK48, and give better
   * warning messages.
   */

  regval &= ~RCC_CCIPR_CLK48SEL_MASK;
  regval |= RCC_CCIPR_CLK48SEL_HSI48;
#endif

#if defined(CONFIG_STM32WB_ADC1)
  /* Select SYSCLK as ADC clock source */

  regval &= ~RCC_CCIPR_ADCSEL_MASK;
  regval |= RCC_CCIPR_ADCSEL_SYSCLK;
#endif

  putreg32(regval, STM32WB_RCC_CCIPR);
}

/****************************************************************************
 * Name: stm32wb_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h
 *
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes!
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_STM32WB_CUSTOM_CLOCKCONFIG
static void stm32wb_stdclockconfig(void)
{
  uint32_t regval;
  volatile int32_t timeout;

#if defined(STM32WB_BOARD_USEHSI) || defined(STM32WB_I2C_USE_HSI16)
  /* Enable Internal High-Speed Clock (HSI) */

  regval  = getreg32(STM32WB_RCC_CR);
  regval |= RCC_CR_HSION;           /* Enable HSI */
  putreg32(regval, STM32WB_RCC_CR);

  /* Wait until the HSI is ready (or until a timeout elapsed) */

  for (timeout = HSIRDY_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if the HSIRDY flag is the set in the CR */

      if ((getreg32(STM32WB_RCC_CR) & RCC_CR_HSIRDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }
#endif

#if defined(STM32WB_BOARD_USEHSI)
  /* Already set above */

#elif defined(STM32WB_BOARD_USEMSI)
  /* Enable Internal Multi-Speed Clock (MSI) */

  /* Wait until the MSI is either off or ready (or until a timeout elapsed) */

  for (timeout = MSIRDY_TIMEOUT; timeout > 0; timeout--)
    {
      regval = getreg32(STM32WB_RCC_CR);

      if ((regval & RCC_CR_MSIRDY) || ~(regval & RCC_CR_MSION))
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }

  /* Setting MSIRANGE */

  regval  = getreg32(STM32WB_RCC_CR);
  regval &= ~RCC_CR_MSIRANGE_MASK;
  regval |= (STM32WB_BOARD_MSIRANGE | RCC_CR_MSION);    /* Enable MSI and frequency */
  putreg32(regval, STM32WB_RCC_CR);

  /* Wait until the MSI is ready (or until a timeout elapsed) */

  for (timeout = MSIRDY_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if the MSIRDY flag is the set in the CR */

      if ((getreg32(STM32WB_RCC_CR) & RCC_CR_MSIRDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }

#elif defined(STM32WB_BOARD_USEHSE)
  /* Enable External High-Speed Clock (HSE) */

  regval  = getreg32(STM32WB_RCC_CR);
  regval |= RCC_CR_HSEON;           /* Enable HSE */
  putreg32(regval, STM32WB_RCC_CR);

  /* Wait until the HSE is ready (or until a timeout elapsed) */

  for (timeout = HSERDY_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if the HSERDY flag is the set in the CR */

      if ((getreg32(STM32WB_RCC_CR) & RCC_CR_HSERDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }
#else

#  error stm32wb_stdclockconfig(), must have one of STM32WB_BOARD_USEHSI, STM32WB_BOARD_USEMSI, STM32WB_BOARD_USEHSE defined

#endif

  /* Check for a timeout.  If this timeout occurs, then we are hosed.  We
   * have no real back-up plan, although the following logic makes it look
   * as though we do.
   */

  if (timeout > 0)
    {
      /* Setup regulator voltage according to clock frequency */

      regval  = getreg32(STM32WB_PWR_CR1);
      regval &= ~PWR_CR1_VOS_MASK;
#if STM32WB_SYSCLK_FREQUENCY > 16000000 || \
    (defined(BOARD_MAX_PLL_FREQUENCY) && BOARD_MAX_PLL_FREQUENCY > 16000000)
      regval |= PWR_CR1_VOS_RANGE1;
#else
      regval |= PWR_CR1_VOS_RANGE2;
#endif
      putreg32(regval, STM32WB_PWR_CR1);

      /* Set the HCLK source/divider */

      regval  = getreg32(STM32WB_RCC_CFGR);
      regval &= ~RCC_CFGR_HPRE_MASK;
      regval |= STM32WB_RCC_CFGR_HPRE;
      putreg32(regval, STM32WB_RCC_CFGR);

      /* Set the CPU2 HCLK2 source/divider */

      regval  = getreg32(STM32WB_RCC_EXTCFGR);
      regval &= ~RCC_EXTCFGR_C2HPRE_MASK;
      regval |= STM32WB_RCC_EXTCFGR_C2HPRE;
      putreg32(regval, STM32WB_RCC_EXTCFGR);

      /* Set the HCLK4 source/divider */

      regval  = getreg32(STM32WB_RCC_EXTCFGR);
      regval &= ~RCC_EXTCFGR_SHDHPRE_MASK;
      regval |= STM32WB_RCC_EXTCFGR_SHDHPRE;
      putreg32(regval, STM32WB_RCC_EXTCFGR);

      /* Set the PCLK1 divider */

      regval  = getreg32(STM32WB_RCC_CFGR);
      regval &= ~RCC_CFGR_PPRE1_MASK;
      regval |= STM32WB_RCC_CFGR_PPRE1;
      putreg32(regval, STM32WB_RCC_CFGR);

      /* Set the PCLK2 divider */

      regval  = getreg32(STM32WB_RCC_CFGR);
      regval &= ~RCC_CFGR_PPRE2_MASK;
      regval |= STM32WB_RCC_CFGR_PPRE2;
      putreg32(regval, STM32WB_RCC_CFGR);

      /* Configure Main PLL */

      regval  = getreg32(STM32WB_RCC_PLLCFG);
      regval &= ~(RCC_PLLCFG_PLLM_MASK | RCC_PLLCFG_PLLN_MASK);
      regval |= (STM32WB_PLLCFG_PLLM | STM32WB_PLLCFG_PLLN);

      /* Set the PLL dividers and multipliers to configure the main PLL */

      regval &= ~(RCC_PLLCFG_PLLPEN | RCC_PLLCFG_PLLQEN | RCC_PLLCFG_PLLREN);
#ifdef STM32WB_PLLCFG_PLLP_ENABLED
      regval &= ~RCC_PLLCFG_PLLP_MASK;
      regval |= (RCC_PLLCFG_PLLPEN | STM32WB_PLLCFG_PLLP);
#endif
#ifdef STM32WB_PLLCFG_PLLQ_ENABLED
      regval &= ~RCC_PLLCFG_PLLQ_MASK;
      regval |= (RCC_PLLCFG_PLLQEN | STM32WB_PLLCFG_PLLQ);
#endif
#ifdef STM32WB_PLLCFG_PLLR_ENABLED
      regval &= ~RCC_PLLCFG_PLLR_MASK;
      regval |= (RCC_PLLCFG_PLLREN | STM32WB_PLLCFG_PLLR);
#endif

      /* XXX The choice of clock source to PLL (all three) is independent
       * of the sys clock source choice, review the STM32WB_BOARD_USEHSI
       * name; probably split it into two, one for PLL source and one
       * for sys clock source.
       */

      regval &= ~RCC_PLLCFG_PLLSRC_MASK;
#ifdef STM32WB_BOARD_USEHSI
      regval |= RCC_PLLCFG_PLLSRC_HSI16;
#elif defined(STM32WB_BOARD_USEMSI)
      regval |= RCC_PLLCFG_PLLSRC_MSI;
#else /* if STM32WB_BOARD_USEHSE */
      regval |= RCC_PLLCFG_PLLSRC_HSE;
#endif

      putreg32(regval, STM32WB_RCC_PLLCFG);

      /* Enable the main PLL */

      regval  = getreg32(STM32WB_RCC_CR);
      regval |= RCC_CR_PLLON;
      putreg32(regval, STM32WB_RCC_CR);

      /* Wait until the PLL is ready */

      while ((getreg32(STM32WB_RCC_CR) & RCC_CR_PLLRDY) == 0)
        {
        }

#ifdef CONFIG_STM32WB_SAI1PLL
      /* Configure SAI1 PLL */

      regval  = getreg32(STM32WB_RCC_PLLSAI1CFG);
      regval &= ~RCC_PLLSAI1CFG_PLLN_MASK;
      regval |= STM32WB_PLLSAI1CFG_PLLN;

      /* Set the PLL dividers and multipliers to configure the SAI1 PLL */

      regval &= ~(RCC_PLLSAI1CFG_PLLPEN | RCC_PLLSAI1CFG_PLLQEN |
                  RCC_PLLSAI1CFG_PLLREN);
#ifdef STM32WB_PLLSAI1CFG_PLLP_ENABLED
      regval &= ~RCC_PLLSAI1CFG_PLLP_MASK;
      regval |= (RCC_PLLSAI1CFG_PLLPEN | STM32WB_PLLSAI1CFG_PLLP);
#endif
#ifdef STM32WB_PLLSAI1CFG_PLLQ_ENABLED
      regval &= ~RCC_PLLSAI1CFG_PLLQ_MASK;
      regval |= (RCC_PLLSAI1CFG_PLLQEN | STM32WB_PLLSAI1CFG_PLLQ);
#endif
#ifdef STM32WB_PLLSAI1CFG_PLLR_ENABLED
      regval &= ~RCC_PLLSAI1CFG_PLLR_MASK;
      regval |= (RCC_PLLSAI1CFG_PLLREN | STM32WB_PLLSAI1CFG_PLLR);
#endif

      putreg32(regval, STM32WB_RCC_PLLSAI1CFG);

      /* Enable the SAI1 PLL */

      regval  = getreg32(STM32WB_RCC_CR);
      regval |= RCC_CR_PLLSAI1ON;
      putreg32(regval, STM32WB_RCC_CR);

      /* Wait until the PLL is ready */

      while ((getreg32(STM32WB_RCC_CR) & RCC_CR_PLLSAI1RDY) == 0)
        {
        }
#endif

      /* Configure FLASH wait states */

      regval  = getreg32(STM32WB_FLASH_ACR);
      regval &= ~FLASH_ACR_LATENCY_MASK;
#ifdef BOARD_FLASH_WAITSTATES
      regval |= FLASH_ACR_LATENCY(BOARD_FLASH_WAITSTATES);
#else
      regval |= FLASH_ACR_LATENCY_3;
#endif

      /* Enable FLASH prefetch, instruction cache and data cache */

#ifdef CONFIG_STM32WB_FLASH_PREFETCH
      regval |= FLASH_ACR_PRFTEN;
#else
      regval &= ~FLASH_ACR_PRFTEN;
#endif
      regval |= (FLASH_ACR_ICEN | FLASH_ACR_DCEN);
      putreg32(regval, STM32WB_FLASH_ACR);

      /* Select the main PLL as system clock source */

      regval  = getreg32(STM32WB_RCC_CFGR);
      regval &= ~RCC_CFGR_SW_MASK;
      regval |= RCC_CFGR_SW_PLL;
      putreg32(regval, STM32WB_RCC_CFGR);

      /* Wait until the PLL source is used as the system clock source */

      while ((getreg32(STM32WB_RCC_CFGR) & RCC_CFGR_SWS_MASK) !=
                       RCC_CFGR_SWS_PLL)
        {
        }

#if defined(CONFIG_STM32WB_IWDG) || defined(CONFIG_STM32WB_RTC_LSICLOCK)
      /* Low speed internal clock source LSI */

      stm32wb_rcc_enable_lsi();
#endif

#if defined(STM32WB_USE_LSE)
      /* Low speed external clock source LSE
       *
       * TODO: There is another case where the LSE needs to
       * be enabled: if the MCO1 pin selects LSE as source.
       * XXX and other cases, like automatic trimming of MSI for USB use
       */

      /* Turn on the LSE oscillator
       * XXX this will almost surely get moved since we also want to use
       * this for automatically trimming MSI, etc.
       */

      stm32wb_rcc_enable_lse();

#  if defined(STM32WB_BOARD_USEMSI)
      /* Now that LSE is up, auto trim the MSI */

      regval  = getreg32(STM32WB_RCC_CR);
      regval |= RCC_CR_MSIPLLEN;
      putreg32(regval, STM32WB_RCC_CR);
#  endif
#endif /* STM32WB_USE_LSE */

      /* Select CPU2 RF wakeup clock source, no clock if not set */

      regval  = getreg32(STM32WB_RCC_CSR);
      regval &= ~RCC_CSR_RFWKPSEL_MASK;
#if defined(STM32WB_BOARD_RFWKP_USELSE)
      regval |= RCC_CSR_RFWKPSEL_LSE;
#elif defined(STM32WB_BOARD_RFWKP_USEHSE)
      regval |= RCC_CSR_RFWKPSEL_HSE;
#endif
      putreg32(regval, STM32WB_RCC_CSR);
    }
}
#endif

/****************************************************************************
 * Name: rcc_enableperipherals
 ****************************************************************************/

static inline void rcc_enableperipherals(void)
{
  rcc_enableccip();
  rcc_enableahb1();
  rcc_enableahb2();
  rcc_enableahb3();
  rcc_enableapb1();
  rcc_enableapb2();

#ifdef STM32WB_USE_HSI48
  /* Enable HSI48 clocking to support USB transfers or RNG */

  stm32wb_enable_hsi48(STM32WB_HSI48_SYNCSRC);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rcc_resetbkp
 *
 * Description:
 *   The RTC needs to reset the Backup Domain to change RTCSEL and resetting
 *   the Backup Domain renders to disabling the LSE as consequence.   In
 *   order to avoid resetting the Backup Domain when we already configured
 *   LSE we will reset the Backup Domain early (here).
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_STM32WB_PWR) && defined(CONFIG_STM32WB_RTC)
static inline void rcc_resetbkp(void)
{
  bool init_stat;

  /* Check if the RTC is already configured */

  init_stat = stm32wb_rtc_is_initialized();
  if (!init_stat)
    {
      uint32_t bkregs[STM32WB_RTC_BKCOUNT];
      int i;

      /* Backup backup-registers before RTC reset. */

      for (i = 0; i < STM32WB_RTC_BKCOUNT; i++)
        {
          bkregs[i] = getreg32(STM32WB_RTC_BKPR(i));
        }

      /* Enable write access to the backup domain (RTC registers, RTC
       * backup data registers and backup SRAM).
       */

      stm32wb_pwr_enablebkp(true);

      /* We might be changing RTCSEL - to ensure such changes work, we must
       * reset the backup domain (having backed up the RTC_MAGIC token)
       */

      modifyreg32(STM32WB_RCC_BDCR, 0, RCC_BDCR_BDRST);
      modifyreg32(STM32WB_RCC_BDCR, RCC_BDCR_BDRST, 0);

      /* Restore backup-registers, except RTC related. */

      for (i = 0; i < STM32WB_RTC_BKCOUNT; i++)
        {
          if (RTC_MAGIC_REG != STM32WB_RTC_BKPR(i))
            {
              putreg32(bkregs[i], STM32WB_RTC_BKPR(i));
            }
        }

      stm32wb_pwr_enablebkp(false);
    }
}
#else
#  define rcc_resetbkp()
#endif

/****************************************************************************
 * Name: stm32wb_clockconfig
 *
 * Description:
 *   Called to establish the clock settings based on the values in board.h.
 *   This function (by default) will reset most everything, enable the PLL,
 *   and enable peripheral clocking for all peripherals enabled in the NuttX
 *   configuration file.
 *
 *   If CONFIG_ARCH_BOARD_STM32WB_CUSTOM_CLOCKCONFIG is defined, then
 *   clocking will be enabled by an externally provided, board-specific
 *   function called stm32wb_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32wb_clockconfig(void)
{
  /* Make sure that we are starting in the reset state */

  rcc_reset();

  /* Reset backup domain if appropriate */

  rcc_resetbkp();

#if defined(CONFIG_ARCH_BOARD_STM32WB_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  stm32wb_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions in
   * board.h
   */

  stm32wb_stdclockconfig();

#endif

  /* Enable peripheral clocking */

  rcc_enableperipherals();
}

/****************************************************************************
 * Name: stm32wb_clockenable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings in
 *   board.h.
 *   This function is only available to support low-power modes of operation:
 *   When re-awakening from deep-sleep modes, it is necessary to
 *   re-enable/re-start the PLL
 *
 *   This functional performs a subset of the operations performed by
 *   stm32wb_clockconfig():  It does not reset any devices, and it does not
 *   reset the currently enabled peripheral clocks.
 *
 *   If CONFIG_ARCH_BOARD_STM32WB_CUSTOM_CLOCKCONFIG is defined, then
 *   clocking will be enabled by an externally provided, board-specific
 *   function called stm32wb_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void stm32wb_clockenable(void)
{
#if defined(CONFIG_ARCH_BOARD_STM32WB_CUSTOM_CLOCKCONFIG)

  /* Invoke Board Custom Clock Configuration */

  stm32wb_board_clockconfig();

#else

  /* Invoke standard, fixed clock configuration based on definitions in
   * board.h
   */

  stm32wb_stdclockconfig();

#endif
}
#endif
