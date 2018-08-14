/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4xrxx_rcc.c
 *
 *   Copyright (C) 2011-2012, 2014-2015 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2016 Sebastien Lorquet. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Sebastien Lorquet <sebastien@lorquet.fr>
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
#include <arch/stm32l4/chip.h>

#include "stm32l4_pwr.h"
#include "stm32l4_flash.h"

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

#define HSE_DIVISOR (STM32L4_HSE_FREQUENCY + 500000) / 1000000

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

  /* Enable the Internal High Speed clock (HSI) */

  regval = getreg32(STM32L4_RCC_CR);
  regval |= RCC_CR_HSION;
  putreg32(regval, STM32L4_RCC_CR);

  /* Reset CFGR register */

  putreg32(0x00000000, STM32L4_RCC_CFGR);

  /* Reset HSION, HSEON, CSSON and PLLON bits */

  regval  = getreg32(STM32L4_RCC_CR);
  regval &= ~(RCC_CR_HSION | RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);
  putreg32(regval, STM32L4_RCC_CR);

  /* Reset PLLCFGR register to reset default */

  putreg32(RCC_PLLCFG_RESET, STM32L4_RCC_PLLCFG);

  /* Reset HSEBYP bit */

  regval  = getreg32(STM32L4_RCC_CR);
  regval &= ~RCC_CR_HSEBYP;
  putreg32(regval, STM32L4_RCC_CR);

  /* Disable all interrupts */

  putreg32(0x00000000, STM32L4_RCC_CIER);
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

  regval = getreg32(STM32L4_RCC_AHB1ENR);

#ifdef CONFIG_STM32L4_DMA1
  /* DMA 1 clock enable */

  regval |= RCC_AHB1ENR_DMA1EN;
#endif

#ifdef CONFIG_STM32L4_DMA2
  /* DMA 2 clock enable */

  regval |= RCC_AHB1ENR_DMA2EN;
#endif

#ifdef CONFIG_STM32L4_CRC
  /* CRC clock enable */

  regval |= RCC_AHB1ENR_CRCEN;
#endif

#ifdef CONFIG_STM32L4_TSC
  /* TSC clock enable */

  regval |= RCC_AHB1ENR_TSCEN;
#endif

#ifdef CONFIG_STM32L4_DMA2D
  /* DMA2D clock enable */

  regval |= RCC_AHB1ENR_DMA2DEN;
#endif

  putreg32(regval, STM32L4_RCC_AHB1ENR);   /* Enable peripherals */
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

  regval = getreg32(STM32L4_RCC_AHB2ENR);

  /* Enable GPIOA, GPIOB, .... GPIOI */

#if STM32L4_NPORTS > 0
  regval |= (RCC_AHB2ENR_GPIOAEN
#if STM32L4_NPORTS > 1
             | RCC_AHB2ENR_GPIOBEN
#endif
#if STM32L4_NPORTS > 2
             | RCC_AHB2ENR_GPIOCEN
#endif
#if STM32L4_NPORTS > 3
             | RCC_AHB2ENR_GPIODEN
#endif
#if STM32L4_NPORTS > 4
             | RCC_AHB2ENR_GPIOEEN
#endif
#if STM32L4_NPORTS > 5
             | RCC_AHB2ENR_GPIOFEN
#endif
#if STM32L4_NPORTS > 6
             | RCC_AHB2ENR_GPIOGEN
#endif
#if STM32L4_NPORTS > 7
             | RCC_AHB2ENR_GPIOHEN
#endif
#if STM32L4_NPORTS > 8
             | RCC_AHB2ENR_GPIOIEN
#endif
             );
#endif

#ifdef CONFIG_STM32L4_OTGFS
  /* USB OTG FS clock enable */

  regval |= RCC_AHB2ENR_OTGFSEN;
#endif

#if defined(CONFIG_STM32L4_ADC1)
  /* ADC clock enable */

  regval |= RCC_AHB2ENR_ADCEN;
#endif

#ifdef CONFIG_STM32L4_DCMI
  /* Digital Camera interfaces clock enable */

  regval |= RCC_AHB2ENR_DCMIEN;
#endif

#ifdef CONFIG_STM32L4_AES
  /* Cryptographic modules clock enable */

  regval |= RCC_AHB2ENR_AESEN;
#endif

#ifdef CONFIG_STM32L4_HASH
  /* HASH module clock enable */

  regval |= RCC_AHB2ENR_HASHEN;
#endif

#ifdef CONFIG_STM32L4_RNG
  /* Random number generator clock enable */

  regval |= RCC_AHB2ENR_RNGEN;
#endif

#ifdef CONFIG_STM32L4_SDMMC
  /* SDMMC clock enable */

  regval |= RCC_AHB2ENR_SDMMC1EN;
#endif

  putreg32(regval, STM32L4_RCC_AHB2ENR);   /* Enable peripherals */
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

  regval = getreg32(STM32L4_RCC_AHB3ENR);

#ifdef CONFIG_STM32L4_FSMC
  /* Flexible static memory controller module clock enable */

  regval |= RCC_AHB3ENR_FSMCEN;
#endif

  putreg32(regval, STM32L4_RCC_AHB3ENR);   /* Enable peripherals */
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

  regval = getreg32(STM32L4_RCC_APB1ENR1);

#ifdef CONFIG_STM32L4_TIM2
  /* TIM2 clock enable */

  regval |= RCC_APB1ENR1_TIM2EN;
#endif

#ifdef CONFIG_STM32L4_TIM3
  /* TIM3 clock enable */

  regval |= RCC_APB1ENR1_TIM3EN;
#endif

#ifdef CONFIG_STM32L4_TIM4
  /* TIM4 clock enable */

  regval |= RCC_APB1ENR1_TIM4EN;
#endif

#ifdef CONFIG_STM32L4_TIM5
  /* TIM5 clock enable */

  regval |= RCC_APB1ENR1_TIM5EN;
#endif

#ifdef CONFIG_STM32L4_TIM6
  /* TIM6 clock enable */

  regval |= RCC_APB1ENR1_TIM6EN;
#endif

#ifdef CONFIG_STM32L4_TIM7
  /* TIM7 clock enable */

  regval |= RCC_APB1ENR1_TIM7EN;
#endif

#ifdef CONFIG_STM32L4_SPI2
  /* SPI2 clock enable */

  regval |= RCC_APB1ENR1_SPI2EN;
#endif

#ifdef CONFIG_STM32L4_SPI3
  /* SPI3 clock enable */

  regval |= RCC_APB1ENR1_SPI3EN;
#endif

#ifdef CONFIG_STM32L4_USART2
  /* USART 2 clock enable */

  regval |= RCC_APB1ENR1_USART2EN;
#endif

#ifdef CONFIG_STM32L4_USART3
  /* USART3 clock enable */

  regval |= RCC_APB1ENR1_USART3EN;
#endif

#ifdef CONFIG_STM32L4_UART4
  /* UART4 clock enable */

  regval |= RCC_APB1ENR1_UART4EN;
#endif

#ifdef CONFIG_STM32L4_UART5
  /* UART5 clock enable */

  regval |= RCC_APB1ENR1_UART5EN;
#endif

#ifdef CONFIG_STM32L4_I2C1
  /* I2C1 clock enable */

  regval |= RCC_APB1ENR1_I2C1EN;
#endif

#ifdef CONFIG_STM32L4_I2C2
  /* I2C2 clock enable */

  regval |= RCC_APB1ENR1_I2C2EN;
#endif

#ifdef CONFIG_STM32L4_I2C3
  /* I2C3 clock enable */

  regval |= RCC_APB1ENR1_I2C3EN;
#endif

#ifdef CONFIG_STM32L4_CAN1
  /* CAN 1 clock enable */

  regval |= RCC_APB1ENR1_CAN1EN;
#endif

  /* Power interface clock enable.  The PWR block is always enabled so that
   * we can set the internal voltage regulator as required.
   */

  regval |= RCC_APB1ENR1_PWREN;

#if defined (CONFIG_STM32L4_DAC1) || defined(CONFIG_STM32L4_DAC2)
  /* DAC interface clock enable */

  regval |= RCC_APB1ENR1_DAC1EN;
#endif

#ifdef CONFIG_STM32L4_OPAMP
  /* OPAMP clock enable */

  regval |= RCC_APB1ENR1_OPAMPEN;
#endif

#ifdef CONFIG_STM32L4_LPTIM1
  /* Low power timer 1 clock enable */

  regval |= RCC_APB1ENR1_LPTIM1EN;
#endif

  putreg32(regval, STM32L4_RCC_APB1ENR1);   /* Enable peripherals */

 /* Second APB1 register */

  regval = getreg32(STM32L4_RCC_APB1ENR2);

#ifdef CONFIG_STM32L4_LPUART1
  /* Low power uart clock enable */

  regval |= RCC_APB1ENR2_LPUART1EN;
#endif

#ifdef CONFIG_STM32L4_I2C4
  /* I2C4 clock enable */

  regval |= RCC_APB1ENR2_I2C4EN;
#endif

#ifdef CONFIG_STM32L4_LPTIM2
  /* Low power timer 2 clock enable */

  regval |= RCC_APB1ENR2_LPTIM2EN;
#endif

  putreg32(regval, STM32L4_RCC_APB1ENR2);   /* Enable peripherals */
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

  regval = getreg32(STM32L4_RCC_APB2ENR);

#if defined(CONFIG_STM32L4_SYSCFG) || defined(CONFIG_STM32L4_COMP)
  /* System configuration controller, comparators, and voltage reference buffer
   * clock enable
   */

  regval |= RCC_APB2ENR_SYSCFGEN;
#endif

#ifdef CONFIG_STM32L4_FIREWALL
  /* Firewall clock enable */

  regval |= RCC_APB2ENR_FWEN;
#endif

#ifdef CONFIG_STM32L4_TIM1
  /* TIM1 clock enable */

  regval |= RCC_APB2ENR_TIM1EN;
#endif

#ifdef CONFIG_STM32L4_SPI1
  /* SPI1 clock enable */

  regval |= RCC_APB2ENR_SPI1EN;
#endif

#ifdef CONFIG_STM32L4_TIM8
  /* TIM8 clock enable */

  regval |= RCC_APB2ENR_TIM8EN;
#endif

#ifdef CONFIG_STM32L4_USART1
  /* USART1 clock enable */

  regval |= RCC_APB2ENR_USART1EN;
#endif

#ifdef CONFIG_STM32L4_TIM15
  /* TIM15 clock enable */

  regval |= RCC_APB2ENR_TIM15EN;
#endif

#ifdef CONFIG_STM32L4_TIM16
  /* TIM16 clock enable */

  regval |= RCC_APB2ENR_TIM16EN;
#endif

#ifdef CONFIG_STM32L4_TIM17
  /* TIM17 clock enable */

  regval |= RCC_APB2ENR_TIM17EN;
#endif

#ifdef CONFIG_STM32L4_SAI1
  /* SAI1 clock enable */

  regval |= RCC_APB2ENR_SAI1EN;
#endif

#ifdef CONFIG_STM32L4_SAI2
  /* SAI2 clock enable */

  regval |= RCC_APB2ENR_SAI2EN;
#endif

#ifdef CONFIG_STM32L4_DFSDM1
  /* DFSDM clock enable */

  regval |= RCC_APB2ENR_DFSDMEN;
#endif

  putreg32(regval, STM32L4_RCC_APB2ENR);   /* Enable peripherals */
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

  regval = getreg32(STM32L4_RCC_CCIPR);

#if defined(STM32L4_USE_CLK48)
  /* XXX sanity if sdmmc1 or usb or rng, then we need to set the clk48 source
   * and then we can also do away with STM32L4_USE_CLK48, and give better
   * warning messages.
   */

  regval |= STM32L4_CLK48_SEL;
#endif

#if defined(CONFIG_STM32L4_ADC1)
  /* Select SYSCLK as ADC clock source */

  regval |= RCC_CCIPR_ADCSEL_SYSCLK;
#endif

  putreg32(regval, STM32L4_RCC_CCIPR);

  /* Some peripherals have their clock selection in CCIPR2 register. */

  regval = getreg32(STM32L4_RCC_CCIPR2);

#ifdef CONFIG_STM32L4_DFSDM1
  /* Select SAI1 as DFSDM audio clock source. */

  regval |= RCC_CCIPR2_ADFSDMSEL_SAI1;

  /* Select SYSCLK as DFSDM kernel clock source. */

  regval |= RCC_CCIPR2_DFSDMSEL_PCLK;
#endif

  putreg32(regval, STM32L4_RCC_CCIPR2);
}

/****************************************************************************
 * Name: stm32l4_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h
 *
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes!
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_STM32L4_CUSTOM_CLOCKCONFIG
static void stm32l4_stdclockconfig(void)
{
  uint32_t regval;
  volatile int32_t timeout;

#ifdef STM32L4_BOARD_USEHSI
  /* Enable Internal High-Speed Clock (HSI) */

  regval  = getreg32(STM32L4_RCC_CR);
  regval |= RCC_CR_HSION;           /* Enable HSI */
  putreg32(regval, STM32L4_RCC_CR);

  /* Wait until the HSI is ready (or until a timeout elapsed) */

  for (timeout = HSIRDY_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if the HSIRDY flag is the set in the CR */

      if ((getreg32(STM32L4_RCC_CR) & RCC_CR_HSIRDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }

#elif defined(STM32L4_BOARD_USEMSI)
  /* Enable Internal Multi-Speed Clock (MSI) */

  /* Wait until the MSI is either off or ready (or until a timeout elapsed) */

  for (timeout = MSIRDY_TIMEOUT; timeout > 0; timeout--)
    {
      if ((regval = getreg32(STM32L4_RCC_CR)), (regval & RCC_CR_MSIRDY) || ~(regval & RCC_CR_MSION))
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }

  /* setting MSIRANGE */

  regval  = getreg32(STM32L4_RCC_CR);
  regval |= (STM32L4_BOARD_MSIRANGE | RCC_CR_MSION);    /* Enable MSI and frequency */
  putreg32(regval, STM32L4_RCC_CR);

  /* Wait until the MSI is ready (or until a timeout elapsed) */

  for (timeout = MSIRDY_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if the MSIRDY flag is the set in the CR */

      if ((getreg32(STM32L4_RCC_CR) & RCC_CR_MSIRDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }

#elif defined(STM32L4_BOARD_USEHSE)
  /* Enable External High-Speed Clock (HSE) */

  regval  = getreg32(STM32L4_RCC_CR);
  regval |= RCC_CR_HSEON;           /* Enable HSE */
  putreg32(regval, STM32L4_RCC_CR);

  /* Wait until the HSE is ready (or until a timeout elapsed) */

  for (timeout = HSERDY_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if the HSERDY flag is the set in the CR */

      if ((getreg32(STM32L4_RCC_CR) & RCC_CR_HSERDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }
#else

#  error stm32l4_stdclockconfig(), must have one of STM32L4_BOARD_USEHSI, STM32L4_BOARD_USEMSI, STM32L4_BOARD_USEHSE defined

#endif

  /* Check for a timeout.  If this timeout occurs, then we are hosed.  We
   * have no real back-up plan, although the following logic makes it look
   * as though we do.
   */

  if (timeout > 0)
    {
#warning todo: regulator voltage according to clock freq
#if 0
      /* Ensure Power control is enabled before modifying it. */

      regval  = getreg32(STM32L4_RCC_APB1ENR);
      regval |= RCC_APB1ENR_PWREN;
      putreg32(regval, STM32L4_RCC_APB1ENR);

      /* Select regulator voltage output Scale 1 mode to support system
       * frequencies up to 168 MHz.
       */

      regval  = getreg32(STM32L4_PWR_CR);
      regval &= ~PWR_CR_VOS_MASK;
      regval |= PWR_CR_VOS_SCALE_1;
      putreg32(regval, STM32L4_PWR_CR);
#endif

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

#ifdef CONFIG_STM32L4_RTC_HSECLOCK
      /* Set the RTC clock divisor */

      regval  = getreg32(STM32L4_RCC_CFGR);
      regval &= ~RCC_CFGR_RTCPRE_MASK;
      regval |= RCC_CFGR_RTCPRE(HSE_DIVISOR);
      putreg32(regval, STM32L4_RCC_CFGR);
#endif

      /* Set the PLL source and main divider */

      regval  = getreg32(STM32L4_RCC_PLLCFG);

      /* Configure Main PLL */

      /* Set the PLL dividers and multipliers to configure the main PLL */

      regval = (STM32L4_PLLCFG_PLLM | STM32L4_PLLCFG_PLLN | STM32L4_PLLCFG_PLLP
                 | STM32L4_PLLCFG_PLLQ | STM32L4_PLLCFG_PLLR);

#ifdef STM32L4_PLLCFG_PLLP_ENABLED
      regval |= RCC_PLLCFG_PLLPEN;
#endif
#ifdef STM32L4_PLLCFG_PLLQ_ENABLED
      regval |= RCC_PLLCFG_PLLQEN;
#endif
#ifdef STM32L4_PLLCFG_PLLR_ENABLED
      regval |= RCC_PLLCFG_PLLREN;
#endif

      /* XXX The choice of clock source to PLL (all three) is independent
       * of the sys clock source choice, review the STM32L4_BOARD_USEHSI
       * name; probably split it into two, one for PLL source and one
       * for sys clock source.
       */

#ifdef STM32L4_BOARD_USEHSI
      regval |= RCC_PLLCFG_PLLSRC_HSI;
#elif defined(STM32L4_BOARD_USEMSI)
      regval |= RCC_PLLCFG_PLLSRC_MSI;
#else /* if STM32L4_BOARD_USEHSE */
      regval |= RCC_PLLCFG_PLLSRC_HSE;
#endif

      putreg32(regval, STM32L4_RCC_PLLCFG);

      /* Enable the main PLL */

      regval  = getreg32(STM32L4_RCC_CR);
      regval |= RCC_CR_PLLON;
      putreg32(regval, STM32L4_RCC_CR);

      /* Wait until the PLL is ready */

      while ((getreg32(STM32L4_RCC_CR) & RCC_CR_PLLRDY) == 0)
        {
        }

#ifdef CONFIG_STM32L4_SAI1PLL
      /* Configure SAI1 PLL */

      regval  = getreg32(STM32L4_RCC_PLLSAI1CFG);

      /* Set the PLL dividers and multipliers to configure the SAI1 PLL */

      regval = (STM32L4_PLLSAI1CFG_PLLN | STM32L4_PLLSAI1CFG_PLLP
                 | STM32L4_PLLSAI1CFG_PLLQ | STM32L4_PLLSAI1CFG_PLLR);

#ifdef STM32L4_PLLSAI1CFG_PLLP_ENABLED
      regval |= RCC_PLLSAI1CFG_PLLPEN;
#endif
#ifdef STM32L4_PLLSAI1CFG_PLLQ_ENABLED
      regval |= RCC_PLLSAI1CFG_PLLQEN;
#endif
#ifdef STM32L4_PLLSAI1CFG_PLLR_ENABLED
      regval |= RCC_PLLSAI1CFG_PLLREN;
#endif

      putreg32(regval, STM32L4_RCC_PLLSAI1CFG);

      /* Enable the SAI1 PLL */

      regval  = getreg32(STM32L4_RCC_CR);
      regval |= RCC_CR_PLLSAI1ON;
      putreg32(regval, STM32L4_RCC_CR);

       /* Wait until the PLL is ready */

      while ((getreg32(STM32L4_RCC_CR) & RCC_CR_PLLSAI1RDY) == 0)
        {
        }
#endif

#ifdef CONFIG_STM32L4_SAI2PLL
      /* Configure SAI2 PLL */

      regval  = getreg32(STM32L4_RCC_PLLSAI2CFG);

      /* Set the PLL dividers and multipliers to configure the SAI2 PLL */

      regval = (STM32L4_PLLSAI2CFG_PLLN | STM32L4_PLLSAI2CFG_PLLP |
                STM32L4_PLLSAI2CFG_PLLR);

#ifdef STM32L4_PLLSAI2CFG_PLLP_ENABLED
      regval |= RCC_PLLSAI2CFG_PLLPEN;
#endif
#ifdef STM32L4_PLLSAI2CFG_PLLR_ENABLED
      regval |= RCC_PLLSAI2CFG_PLLREN;
#endif

      putreg32(regval, STM32L4_RCC_PLLSAI2CFG);

      /* Enable the SAI2 PLL */

      regval  = getreg32(STM32L4_RCC_CR);
      regval |= RCC_CR_PLLSAI2ON;
      putreg32(regval, STM32L4_RCC_CR);

      /* Wait until the PLL is ready */

      while ((getreg32(STM32L4_RCC_CR) & RCC_CR_PLLSAI2RDY) == 0)
        {
        }
#endif

      /* Configure FLASH wait states */

#ifdef BOARD_FLASH_WAITSTATES
      regval = FLASH_ACR_LATENCY(BOARD_FLASH_WAITSTATES);
#else
      regval = FLASH_ACR_LATENCY_3; /* For Vcore range 1 ≤ 80 MHz (older STM32L4 require 4 WS) */
#endif

      /* Enable FLASH prefetch, instruction cache and data cache */

#ifdef CONFIG_STM32L4_FLASH_PREFETCH
      regval |= (FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN);
#else
      regval |= (FLASH_ACR_ICEN | FLASH_ACR_DCEN);
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
       * XXX and other cases, like automatic trimming of MSI for USB use
       */

      /* ensure Power control is enabled since it is indirectly required
       * to alter the LSE parameters.
       */
      stm32l4_pwr_enableclk(true);

      /* XXX other LSE settings must be made before turning on the oscillator
       * and we need to ensure it is first off before doing so.
       */

      /* Turn on the LSE oscillator
       * XXX this will almost surely get moved since we also want to use
       * this for automatically trimming MSI, etc.
       */

      stm32l4_rcc_enablelse();

#  if defined(STM32L4_BOARD_USEMSI)
      /* Now that LSE is up, auto trim the MSI */

      regval  = getreg32(STM32L4_RCC_CR);
      regval |= RCC_CR_MSIPLLEN;
      putreg32(regval, STM32L4_RCC_CR);
#  endif
#endif
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
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
