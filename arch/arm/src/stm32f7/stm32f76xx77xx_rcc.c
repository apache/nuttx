/****************************************************************************
 * arch/arm/src/stm32f7/stm32f76xxx77xx_rcc.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            David Sidrane <david_s5@nscdg.com>
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

#include "stm32_pwr.h"
#include "stm32_dbgmcu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allow up to 100 milliseconds for the high speed clock to become ready.
 * that is a very long delay, but if the clock does not become ready we are
 * hosed anyway.  Normally this is very fast, but I have seen at least one
 * board that required this long, long timeout for the HSE to be ready.
 */

#define HSERDY_TIMEOUT (100 * CONFIG_BOARD_LOOPSPERMSEC)

/* Same for HSI */

#define HSIRDY_TIMEOUT HSERDY_TIMEOUT

/* HSE divisor to yield ~1MHz RTC clock */

#define HSE_DIVISOR (STM32_HSE_FREQUENCY + 500000) / 1000000

/* If CONFIG_STM32F7_DSIHOST is defined in the board configuration, then
 * STM32_RCC_DCKCFGR2_DSISRC must also be defined to select the clock
 * source.
 */

#ifndef STM32_RCC_DCKCFGR2_DSISRC
#  define STM32_RCC_DCKCFGR2_DSISRC RCC_DCKCFGR2_DSISEL_PHY
#endif

/* FLASH wait states */

#if !defined(BOARD_FLASH_WAITSTATES)
#  error BOARD_FLASH_WAITSTATES not defined
#elif BOARD_FLASH_WAITSTATES < 0 || BOARD_FLASH_WAITSTATES > 15
#  error BOARD_FLASH_WAITSTATES is out of range
#endif

/* Voltage output scale (default to Scale 1 mode) */

#ifndef STM32_PWR_VOS_SCALE
#  define STM32_PWR_VOS_SCALE PWR_CR1_VOS_SCALE_1
#endif

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

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSION;
  putreg32(regval, STM32_RCC_CR);

  /* Reset CFGR register */

  putreg32(0x00000000, STM32_RCC_CFGR);

  /* Reset HSION, HSEON, CSSON and PLLON bits */

  regval  = getreg32(STM32_RCC_CR);
  regval &= ~(RCC_CR_HSION | RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);
  putreg32(regval, STM32_RCC_CR);

  /* Reset PLLCFGR register to reset default */

  putreg32(RCC_PLLCFG_RESET, STM32_RCC_PLLCFG);

  /* Reset HSEBYP bit */

  regval  = getreg32(STM32_RCC_CR);
  regval &= ~RCC_CR_HSEBYP;
  putreg32(regval, STM32_RCC_CR);

  /* Disable all interrupts */

  putreg32(0x00000000, STM32_RCC_CIR);
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

  /* Set the appropriate bits in the AHB1ENR register to enabled the
   * selected AHB1 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB1ENR);

  /* Enable GPIOA, GPIOB, .... GPIOI */

#if STM32F7_NGPIO > 0
  regval |= (RCC_AHB1ENR_GPIOAEN
#if STM32F7_NGPIO > 1
             | RCC_AHB1ENR_GPIOBEN
#endif
#if STM32F7_NGPIO > 2
             | RCC_AHB1ENR_GPIOCEN
#endif
#if STM32F7_NGPIO > 3
             | RCC_AHB1ENR_GPIODEN
#endif
#if STM32F7_NGPIO > 4
             | RCC_AHB1ENR_GPIOEEN
#endif
#if STM32F7_NGPIO > 5
             | RCC_AHB1ENR_GPIOFEN
#endif
#if STM32F7_NGPIO > 6
             | RCC_AHB1ENR_GPIOGEN
#endif
#if STM32F7_NGPIO > 7
             | RCC_AHB1ENR_GPIOHEN
#endif
#if STM32F7_NGPIO > 8
             | RCC_AHB1ENR_GPIOIEN
#endif
#if STM32F7_NGPIO > 9
             | RCC_AHB1ENR_GPIOJEN
#endif
#if STM32F7_NGPIO > 10
             | RCC_AHB1ENR_GPIOKEN
#endif
             );
#endif

#ifdef CONFIG_STM32F7_CRC
  /* CRC clock enable */

  regval |= RCC_AHB1ENR_CRCEN;
#endif

#ifdef CONFIG_STM32F7_BKPSRAM
  /* Backup SRAM clock enable */

  regval |= RCC_AHB1ENR_BKPSRAMEN;
#endif

#ifdef CONFIG_ARMV7M_DTCM
  /* DTCM data RAM clock enable */

  regval |= RCC_AHB1ENR_DTCMRAMEN;
#endif

#ifdef CONFIG_STM32F7_DMA1
  /* DMA 1 clock enable */

  regval |= RCC_AHB1ENR_DMA1EN;
#endif

#ifdef CONFIG_STM32F7_DMA2
  /* DMA 2 clock enable */

  regval |= RCC_AHB1ENR_DMA2EN;
#endif

#ifdef CONFIG_STM32F7_DMA2D
  /* DMA2D clock */

  regval |= RCC_AHB1ENR_DMA2DEN;
#endif

#ifdef CONFIG_STM32F7_ETHMAC
  /* Ethernet MAC clocking */

  regval |= (RCC_AHB1ENR_ETHMACEN | RCC_AHB1ENR_ETHMACTXEN | \
             RCC_AHB1ENR_ETHMACRXEN);

#ifdef CONFIG_STM32F7_ETH_PTP
  /* Precision Time Protocol (PTP) */

  regval |= RCC_AHB1ENR_ETHMACPTPEN;

#endif
#endif

#ifdef CONFIG_STM32F7_OTGFSHS
#ifdef BOARD_ENABLE_USBOTG_HSULPI
  /* Enable clocking for  USB OTG HS and external PHY */

  regval |= (RCC_AHB1ENR_OTGHSEN | RCC_AHB1ENR_OTGHSULPIEN);
#else
  /* Enable only clocking for USB OTG HS */

  regval |= RCC_AHB1ENR_OTGHSEN;
#endif
#endif /* CONFIG_STM32F7_OTGFSHS */

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

  /* Set the appropriate bits in the AHB2ENR register to enabled the
   * selected AHB2 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB2ENR);

#ifdef CONFIG_STM32F7_DCMI
  /* Camera interface enable */

  regval |= RCC_AHB2ENR_DCMIEN;
#endif

#ifdef CONFIG_STM32F7_CRYP
  /* Cryptographic modules clock enable */

  regval |= RCC_AHB2ENR_CRYPEN;
#endif

#ifdef CONFIG_STM32F7_HASH
  /* Hash modules clock enable */

  regval |= RCC_AHB2ENR_HASHEN;
#endif

#ifdef CONFIG_STM32F7_RNG
  /* Random number generator clock enable */

  regval |= RCC_AHB2ENR_RNGEN;
#endif

#ifdef CONFIG_STM32F7_OTGFS
  /* USB OTG FS clock enable */

  regval |= RCC_AHB2ENR_OTGFSEN;
#endif

  putreg32(regval, STM32_RCC_AHB2ENR);   /* Enable peripherals */
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

  regval = getreg32(STM32_RCC_AHB3ENR);

#ifdef CONFIG_STM32F7_FMC
  /* Flexible static memory controller module clock enable */

  regval |= RCC_AHB3ENR_FMCEN;
#endif

#ifdef CONFIG_STM32F7_QUADSPI
  /* FQuad SPI memory controller clock enable */

  regval |= RCC_AHB3ENR_QSPIEN;
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

  /* Set the appropriate bits in the APB1ENR register to enabled the
   * selected APB1 peripherals.
   */

  regval = getreg32(STM32_RCC_APB1ENR);

#ifdef CONFIG_STM32F7_TIM2
  /* TIM2 clock enable */

  regval |= RCC_APB1ENR_TIM2EN;
#endif

#ifdef CONFIG_STM32F7_TIM3
  /* TIM3 clock enable */

  regval |= RCC_APB1ENR_TIM3EN;
#endif

#ifdef CONFIG_STM32F7_TIM4
  /* TIM4 clock enable */

  regval |= RCC_APB1ENR_TIM4EN;
#endif

#ifdef CONFIG_STM32F7_TIM5
  /* TIM5 clock enable */

  regval |= RCC_APB1ENR_TIM5EN;
#endif

#ifdef CONFIG_STM32F7_TIM6
  /* TIM6 clock enable */

  regval |= RCC_APB1ENR_TIM6EN;
#endif

#ifdef CONFIG_STM32F7_TIM7
  /* TIM7 clock enable */

  regval |= RCC_APB1ENR_TIM7EN;
#endif

#ifdef CONFIG_STM32F7_TIM12
  /* TIM12 clock enable */

  regval |= RCC_APB1ENR_TIM12EN;
#endif

#ifdef CONFIG_STM32F7_TIM13
  /* TIM13 clock enable */

  regval |= RCC_APB1ENR_TIM13EN;
#endif

#ifdef CONFIG_STM32F7_TIM14
  /* TIM14 clock enable */

  regval |= RCC_APB1ENR_TIM14EN;
#endif

#ifdef CONFIG_STM32F7_LPTIM1
  /* Low-power timer 1 clock enable */

  regval |= RCC_APB1ENR_LPTIM1EN;
#endif

#ifdef CONFIG_STM32F7_WWDG
  /* Window watchdog clock enable */

  regval |= RCC_APB1ENR_WWDGEN;
#endif

#ifdef CONFIG_STM32F7_SPI2
  /* SPI2 clock enable */

  regval |= RCC_APB1ENR_SPI2EN;
#endif

#ifdef CONFIG_STM32F7_SPI3
  /* SPI3 clock enable */

  regval |= RCC_APB1ENR_SPI3EN;
#endif

#ifdef CONFIG_STM32F7_SPDIFRX
  /* SPDIFRX clock enable */

  regval |= RCC_APB1ENR_SPDIFRXEN;
#endif

#ifdef CONFIG_STM32F7_USART2
  /* USART 2 clock enable */

  regval |= RCC_APB1ENR_USART2EN;
#endif

#ifdef CONFIG_STM32F7_USART3
  /* USART3 clock enable */

  regval |= RCC_APB1ENR_USART3EN;
#endif

#ifdef CONFIG_STM32F7_UART4
  /* UART4 clock enable */

  regval |= RCC_APB1ENR_UART4EN;
#endif

#ifdef CONFIG_STM32F7_UART5
  /* UART5 clock enable */

  regval |= RCC_APB1ENR_UART5EN;
#endif

#ifdef CONFIG_STM32F7_I2C1
  /* I2C1 clock enable */

  regval |= RCC_APB1ENR_I2C1EN;
#endif

#ifdef CONFIG_STM32F7_I2C2
  /* I2C2 clock enable */

  regval |= RCC_APB1ENR_I2C2EN;
#endif

#ifdef CONFIG_STM32F7_I2C3
  /* I2C3 clock enable */

  regval |= RCC_APB1ENR_I2C3EN;
#endif

#ifdef CONFIG_STM32F7_I2C4
  /* I2C4 clock enable */

  regval |= RCC_APB1ENR_I2C4EN;
#endif

#ifdef CONFIG_STM32F7_CAN1
  /* CAN 1 clock enable */

  regval |= RCC_APB1ENR_CAN1EN;
#endif

#ifdef CONFIG_STM32F7_CAN2
  /* CAN2 clock enable.  NOTE: CAN2 needs CAN1 clock as well. */

  regval |= (RCC_APB1ENR_CAN1EN | RCC_APB1ENR_CAN2EN);
#endif

#ifdef CONFIG_STM32F7_CAN3
  /* CAN3 clock enable. */

  regval |= (RCC_APB1ENR_CAN3EN);
#endif

#ifdef CONFIG_STM32F7_CEC
  /* CEC clock enable. */

  regval |= RCC_APB1ENR_CECEN;
#endif

  /* Power interface clock enable.  The PWR block is always enabled so that
   * we can set the internal voltage regulator for maximum performance.
   */

  regval |= RCC_APB1ENR_PWREN;

#if defined (CONFIG_STM32F7_DAC1) || defined(CONFIG_STM32F7_DAC2)
  /* DAC interface clock enable */

  regval |= RCC_APB1ENR_DACEN;
#endif

#ifdef CONFIG_STM32F7_UART7
  /* UART7 clock enable */

  regval |= RCC_APB1ENR_UART7EN;
#endif

#ifdef CONFIG_STM32F7_UART8
  /* UART8 clock enable */

  regval |= RCC_APB1ENR_UART8EN;
#endif

  putreg32(regval, STM32_RCC_APB1ENR);   /* Enable peripherals */
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

#ifdef CONFIG_STM32F7_TIM1
  /* TIM1 clock enable */

  regval |= RCC_APB2ENR_TIM1EN;
#endif

#ifdef CONFIG_STM32F7_TIM8
  /* TIM8 clock enable */

  regval |= RCC_APB2ENR_TIM8EN;
#endif

#ifdef CONFIG_STM32F7_USART1
  /* USART1 clock enable */

  regval |= RCC_APB2ENR_USART1EN;
#endif

#ifdef CONFIG_STM32F7_USART6
  /* USART6 clock enable */

  regval |= RCC_APB2ENR_USART6EN;
#endif

#ifdef CONFIG_STM32F7_ADC1
  /* ADC1 clock enable */

  regval |= RCC_APB2ENR_ADC1EN;
#endif

#ifdef CONFIG_STM32F7_ADC2
  /* ADC2 clock enable */

  regval |= RCC_APB2ENR_ADC2EN;
#endif

#ifdef CONFIG_STM32F7_ADC3
  /* ADC3 clock enable */

  regval |= RCC_APB2ENR_ADC3EN;
#endif

#ifdef CONFIG_STM32F7_SDMMC1
  /* SDIO_1 clock enable */

  regval |= RCC_APB2ENR_SDMMC1EN;
#endif

#ifdef CONFIG_STM32F7_SDMMC2
  /* SDIO_2 clock enable */

  regval |= RCC_APB2ENR_SDMMC2EN;
#endif

#ifdef CONFIG_STM32F7_SPI1
  /* SPI1 clock enable */

  regval |= RCC_APB2ENR_SPI1EN;
#endif

#ifdef CONFIG_STM32F7_SPI4
  /* SPI4 clock enable */

  regval |= RCC_APB2ENR_SPI4EN;
#endif

  /* System configuration controller clock enable */

  regval |= RCC_APB2ENR_SYSCFGEN;

#ifdef CONFIG_STM32F7_TIM9
  /* TIM9 clock enable */

  regval |= RCC_APB2ENR_TIM9EN;
#endif

#ifdef CONFIG_STM32F7_TIM10
  /* TIM10 clock enable */

  regval |= RCC_APB2ENR_TIM10EN;
#endif

#ifdef CONFIG_STM32F7_TIM11
  /* TIM11 clock enable */

  regval |= RCC_APB2ENR_TIM11EN;
#endif

#ifdef CONFIG_STM32F7_SPI5
  /* SPI5 clock enable */

  regval |= RCC_APB2ENR_SPI5EN;
#endif

#ifdef CONFIG_STM32F7_SPI6
  /* SPI6 clock enable */

  regval |= RCC_APB2ENR_SPI6EN;
#endif

#ifdef CONFIG_STM32F7_SAI1
  /* SPI6 clock enable */

  regval |= RCC_APB2ENR_SAI1EN;
#endif

#ifdef CONFIG_STM32F7_SAI2
  /* SPI6 clock enable */

  regval |= RCC_APB2ENR_SAI2EN;
#endif

#ifdef CONFIG_STM32F7_LTDC
  /* LTDC clock enable */

  regval |= RCC_APB2ENR_LTDCEN;
#endif

#ifdef CONFIG_STM32F7_DSIHOST
  /* LTDC clock enable */

  regval |= RCC_APB2ENR_DSIEN;
#endif

  putreg32(regval, STM32_RCC_APB2ENR);   /* Enable peripherals */
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

#ifndef CONFIG_STM32F7_CUSTOM_CLOCKCONFIG
static void stm32_stdclockconfig(void)
{
  uint32_t regval;
  volatile int32_t timeout;

#ifdef STM32_BOARD_USEHSI
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

#else /* if STM32_BOARD_USEHSE */
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
      /* Check if the HSERDY flag is the set in the CR */

      if ((getreg32(STM32_RCC_CR) & RCC_CR_HSERDY) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }
#endif

  /* Check for a timeout.  If this timeout occurs, then we are hosed.  We
   * have no real back-up plan, although the following logic makes it look
   * as though we do.
   */

  if (timeout > 0)
    {
      /* Select regulator voltage output Scale 1 mode to support system
       * frequencies up to 216 MHz.
       */

      regval  = getreg32(STM32_RCC_APB1ENR);
      regval |= RCC_APB1ENR_PWREN;
      putreg32(regval, STM32_RCC_APB1ENR);

      regval  = getreg32(STM32_PWR_CR1);
      regval &= ~PWR_CR1_VOS_MASK;
      regval |= STM32_PWR_VOS_SCALE;
      putreg32(regval, STM32_PWR_CR1);

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

#ifdef CONFIG_STM32F7_RTC_HSECLOCK
      /* Set the RTC clock divisor */

      regval = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_RTCPRE_MASK;
      regval |= RCC_CFGR_RTCPRE(HSE_DIVISOR);
      putreg32(regval, STM32_RCC_CFGR);
#endif

      /* Set the PLL dividers and multipliers to configure the main PLL */

#ifdef STM32_BOARD_USEHSI
      regval = (STM32_PLLCFG_PLLM | STM32_PLLCFG_PLLN | STM32_PLLCFG_PLLP |
                RCC_PLLCFG_PLLSRC_HSI | STM32_PLLCFG_PLLQ);
#else /* if STM32_BOARD_USEHSE */
      regval = (STM32_PLLCFG_PLLM | STM32_PLLCFG_PLLN | STM32_PLLCFG_PLLP |
                RCC_PLLCFG_PLLSRC_HSE | STM32_PLLCFG_PLLQ);
#endif
      putreg32(regval, STM32_RCC_PLLCFG);

      /* Enable the main PLL */

      regval = getreg32(STM32_RCC_CR);
      regval |= RCC_CR_PLLON;
      putreg32(regval, STM32_RCC_CR);

      /* Wait until the PLL is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLRDY) == 0)
        {
        }

      /* Over-drive is needed if
       *  - Voltage output scale 1 mode is selected and SYSCLK frequency is
       *    over 180 MHz.
       *  - Voltage output scale 2 mode is selected and SYSCLK frequence is
       *    over 168 MHz.
       */

      if ((STM32_PWR_VOS_SCALE == PWR_CR1_VOS_SCALE_1 &&
           STM32_SYSCLK_FREQUENCY > 180000000) ||
          (STM32_PWR_VOS_SCALE == PWR_CR1_VOS_SCALE_2 &&
           STM32_SYSCLK_FREQUENCY > 168000000))
        {
          /* Enable the Over-drive to extend the clock frequency up to
           * 216 MHz.
           */

          regval  = getreg32(STM32_PWR_CR1);
          regval |= PWR_CR1_ODEN;
          putreg32(regval, STM32_PWR_CR1);
          while ((getreg32(STM32_PWR_CSR1) & PWR_CSR1_ODRDY) == 0)
            {
            }

          regval = getreg32(STM32_PWR_CR1);
          regval |= PWR_CR1_ODSWEN;
          putreg32(regval, STM32_PWR_CR1);
          while ((getreg32(STM32_PWR_CSR1) & PWR_CSR1_ODSWRDY) == 0)
            {
            }
        }

      /* Configure FLASH wait states */

      regval = FLASH_ACR_LATENCY(BOARD_FLASH_WAITSTATES);

#ifdef CONFIG_STM32F7_FLASH_ART_ACCELERATOR
      /* The Flash memory interface accelerates code execution with a system
       * of instruction prefetch and cache lines on ITCM interface (ART
       * Accelerator™).
       */

      regval |= FLASH_ACR_ARTEN;
      regval |= FLASH_ACR_PRFTEN;
#endif

      putreg32(regval, STM32_FLASH_ACR);

      /* Select the main PLL as system clock source */

      regval  = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_SW_MASK;
      regval |= RCC_CFGR_SW_PLL;
      putreg32(regval, STM32_RCC_CFGR);

      /* Wait until the PLL source is used as the system clock source */

      while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK) != RCC_CFGR_SWS_PLL)
        {
        }

#if defined(CONFIG_STM32F7_LTDC) || defined(CONFIG_STM32F7_PLLSAI)

      /* Configure PLLSAI */

      regval = getreg32(STM32_RCC_PLLSAICFGR);
      regval &= ~(RCC_PLLSAICFGR_PLLSAIN_MASK
                  | RCC_PLLSAICFGR_PLLSAIP_MASK
                  | RCC_PLLSAICFGR_PLLSAIQ_MASK
                  | RCC_PLLSAICFGR_PLLSAIR_MASK);
      regval |= (STM32_RCC_PLLSAICFGR_PLLSAIN
                 | STM32_RCC_PLLSAICFGR_PLLSAIP
                 | STM32_RCC_PLLSAICFGR_PLLSAIQ
                 | STM32_RCC_PLLSAICFGR_PLLSAIR);
      putreg32(regval, STM32_RCC_PLLSAICFGR);

      regval  = getreg32(STM32_RCC_DCKCFGR1);
      regval &= ~(RCC_DCKCFGR1_PLLI2SDIVQ_MASK
                  | RCC_DCKCFGR1_PLLSAIDIVQ_MASK
                  | RCC_DCKCFGR1_PLLSAIDIVR_MASK
                  | RCC_DCKCFGR1_SAI1SEL_MASK
                  | RCC_DCKCFGR1_SAI2SEL_MASK
                  | RCC_DCKCFGR1_TIMPRESEL
                  | RCC_DCKCFGR1_DFSDM1SEL
                  | RCC_DCKCFGR1_ADFSDM1SEL);

      regval |= (STM32_RCC_DCKCFGR1_PLLI2SDIVQ
                 | STM32_RCC_DCKCFGR1_PLLSAIDIVQ
                 | STM32_RCC_DCKCFGR1_PLLSAIDIVR
                 | STM32_RCC_DCKCFGR1_SAI1SRC
                 | STM32_RCC_DCKCFGR1_SAI2SRC
                 | STM32_RCC_DCKCFGR1_TIMPRESRC
                 | STM32_RCC_DCKCFGR1_DFSDM1SRC
                 | STM32_RCC_DCKCFGR1_ADFSDM1SRC);

      putreg32(regval, STM32_RCC_DCKCFGR1);

      /* Enable PLLSAI */

      regval = getreg32(STM32_RCC_CR);
      regval |= RCC_CR_PLLSAION;
      putreg32(regval, STM32_RCC_CR);

      /* Wait until the PLLSAI is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLSAIRDY) == 0)
        {
        }
#endif
#if defined(CONFIG_STM32F7_PLLI2S) || (STM32_RCC_DCKCFGR1_SAI1SRC == RCC_DCKCFGR1_SAI1SEL(1)) || (STM32_RCC_DCKCFGR1_SAI2SRC == RCC_DCKCFGR1_SAI2SEL(1))

      /* Configure PLLI2S */

      regval = getreg32(STM32_RCC_PLLI2SCFGR);
      regval &= ~(RCC_PLLI2SCFGR_PLLI2SN_MASK
                  | RCC_PLLI2SCFGR_PLLI2SP_MASK
                  | RCC_PLLI2SCFGR_PLLI2SQ_MASK
                  | RCC_PLLI2SCFGR_PLLI2SR_MASK);
      regval |= (STM32_RCC_PLLSAICFGR_PLLSAIN
                 | STM32_RCC_PLLSAICFGR_PLLSAIP
                 | STM32_RCC_PLLSAICFGR_PLLSAIQ
                 | STM32_RCC_PLLSAICFGR_PLLSAIR);
      putreg32(regval, STM32_RCC_PLLI2SCFGR);

      /* Enable PLLI2S */

      regval = getreg32(STM32_RCC_CR);
      regval |= RCC_CR_PLLI2SON;
      putreg32(regval, STM32_RCC_CR);

      /* Wait until the PLLI2S is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLI2SRDY) == 0)
        {
        }
#endif

      regval  = getreg32(STM32_RCC_DCKCFGR2);
      regval &= ~(RCC_DCKCFGR2_USART1SEL_MASK
                  | RCC_DCKCFGR2_USART2SEL_MASK
                  | RCC_DCKCFGR2_UART4SEL_MASK
                  | RCC_DCKCFGR2_UART5SEL_MASK
                  | RCC_DCKCFGR2_USART6SEL_MASK
                  | RCC_DCKCFGR2_UART7SEL_MASK
                  | RCC_DCKCFGR2_UART8SEL_MASK
                  | RCC_DCKCFGR2_I2C1SEL_MASK
                  | RCC_DCKCFGR2_I2C2SEL_MASK
                  | RCC_DCKCFGR2_I2C3SEL_MASK
                  | RCC_DCKCFGR2_I2C4SEL_MASK
                  | RCC_DCKCFGR2_LPTIM1SEL_MASK
                  | RCC_DCKCFGR2_CECSEL_MASK
                  | RCC_DCKCFGR2_CK48MSEL_MASK
                  | RCC_DCKCFGR2_SDMMCSEL_MASK
                  | RCC_DCKCFGR2_SDMMC2SEL_MASK
                  | RCC_DCKCFGR2_DSISEL_MASK);

      regval |= (STM32_RCC_DCKCFGR2_USART1SRC
                 | STM32_RCC_DCKCFGR2_USART2SRC
                 | STM32_RCC_DCKCFGR2_UART4SRC
                 | STM32_RCC_DCKCFGR2_UART5SRC
                 | STM32_RCC_DCKCFGR2_USART6SRC
                 | STM32_RCC_DCKCFGR2_UART7SRC
                 | STM32_RCC_DCKCFGR2_UART8SRC
                 | STM32_RCC_DCKCFGR2_I2C1SRC
                 | STM32_RCC_DCKCFGR2_I2C2SRC
                 | STM32_RCC_DCKCFGR2_I2C3SRC
                 | STM32_RCC_DCKCFGR2_I2C4SRC
                 | STM32_RCC_DCKCFGR2_LPTIM1SRC
                 | STM32_RCC_DCKCFGR2_CECSRC
                 | STM32_RCC_DCKCFGR2_CK48MSRC
                 | STM32_RCC_DCKCFGR2_SDMMCSRC
                 | STM32_RCC_DCKCFGR2_SDMMC2SRC
                 | STM32_RCC_DCKCFGR2_DSISRC);

      putreg32(regval, STM32_RCC_DCKCFGR2);

#if defined(CONFIG_STM32F7_IWDG) || defined(CONFIG_STM32F7_RTC_LSICLOCK)
      /* Low speed internal clock source LSI */

      stm32_rcc_enablelsi();
#endif

#if defined(CONFIG_STM32F7_RTC_LSECLOCK)
      /* Low speed external clock source LSE
       *
       * TODO: There is another case where the LSE needs to
       * be enabled: if the MCO1 pin selects LSE as source.
       */

      stm32_rcc_enablelse();
#endif
    }
}
#endif

#ifdef CONFIG_ARMV7M_ITMSYSLOG
static inline void rcc_itm_syslog(void)
{
  /* Enable SWO output */

  modifyreg32(STM32_DBGMCU_CR, DBGMCU_CR_TRACEMODE_MASK,
              DBGMCU_CR_ASYNCH | DBGMCU_CR_TRACEIOEN);
}
#else
#  define rcc_itm_syslog()
#endif

/****************************************************************************
 * Name: rcc_enableperiphals
 ****************************************************************************/

static inline void rcc_enableperipherals(void)
{
  rcc_enableahb1();
  rcc_enableahb2();
  rcc_enableahb3();
  rcc_enableapb1();
  rcc_enableapb2();
  rcc_itm_syslog();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32f7x9_rcc_dsisrcphy
 *
 * Description:
 *   Set DSI clock source to DSI PHY
 *
 ****************************************************************************/

void stm32f7x9_rcc_dsisrcphy(void)
{
  uint32_t regval;
  regval  = getreg32(STM32_RCC_DCKCFGR2);
  regval &= ~(RCC_DCKCFGR2_DSISEL_MASK);

  regval |= (RCC_DCKCFGR2_DSISEL_PHY);
  putreg32(regval, STM32_RCC_DCKCFGR2);
}

/****************************************************************************
 * Name: stm32f7x9_rcc_dsisrcpllr
 *
 * Description:
 *   Set DSI clock source to PLLR
 *
 ****************************************************************************/

void stm32f7x9_rcc_dsisrcpllr(void)
{
  uint32_t regval;
  regval  = getreg32(STM32_RCC_DCKCFGR2);
  regval &= ~RCC_DCKCFGR2_DSISEL_MASK;

  regval |= RCC_DCKCFGR2_DSISEL_SYSCLK;
  putreg32(regval, STM32_RCC_DCKCFGR2);
}
