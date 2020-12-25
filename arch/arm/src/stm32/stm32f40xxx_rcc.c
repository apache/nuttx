/****************************************************************************
 * arch/arm/src/stm32/stm32f40xxx_rcc.c
 *
 *   Copyright (C) 2011-2012, 2014-2015 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2016 Omni Hoverboards Inc. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           David Sidrane <david_s5@nscdg.com>
 *           Paul Alexander Patience <paul-a.patience@polymtl.ca>
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

#include <arch/board/board.h>

#include "chip.h"
#include "stm32_pwr.h"
#include "itm_syslog.h"

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

/* The FLASH latency depends on the system clock, and voltage input
 * of the microcontroller. The following macros calculate the correct
 * wait cycles for every STM32_SYSCLK_FREQUENCY & BOARD_STM32F4_VDD
 * combination. BOARD_STM32F4_VDD is defined in mV.
 */

#ifndef BOARD_STM32F4_VDD
#  define BOARD_STM32F4_VDD 3300
#endif

#if (BOARD_STM32F4_VDD <= 2100)
#  if (STM32_SYSCLK_FREQUENCY <= 20000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_0
#  elif (STM32_SYSCLK_FREQUENCY <= 40000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_1
#  elif (STM32_SYSCLK_FREQUENCY <= 60000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_2
#  elif (STM32_SYSCLK_FREQUENCY <= 800000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_3
#  elif (STM32_SYSCLK_FREQUENCY <= 100000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_4
#  elif (STM32_SYSCLK_FREQUENCY <= 120000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_5
#  elif (STM32_SYSCLK_FREQUENCY <= 140000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_6
#  elif (STM32_SYSCLK_FREQUENCY <= 160000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_7
#  elif (STM32_SYSCLK_FREQUENCY <= 168000000) && \
        (defined(CONFIG_STM32_STM32F427) || defined(CONFIG_STM32_STM32F429) || \
        defined(CONFIG_STM32_STM32F446) || defined(CONFIG_STM32_STM32F469))
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_8
#  else
#    error "STM32_SYSCLK_FREQUENCY is out of range!"
#  endif
#elif (BOARD_STM32F4_VDD <= 2400)
#  if (STM32_SYSCLK_FREQUENCY <= 22000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_0
#  elif (STM32_SYSCLK_FREQUENCY <= 44000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_1
#  elif (STM32_SYSCLK_FREQUENCY <= 66000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_2
#  elif (STM32_SYSCLK_FREQUENCY <= 880000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_3
#  elif (STM32_SYSCLK_FREQUENCY <= 110000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_4
#  elif (STM32_SYSCLK_FREQUENCY <= 132000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_5
#  elif (STM32_SYSCLK_FREQUENCY <= 154000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_6
#  elif (STM32_SYSCLK_FREQUENCY <= 168000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_7
#  elif (STM32_SYSCLK_FREQUENCY <= 176000000) && \
        (defined(CONFIG_STM32_STM32F427) || defined(CONFIG_STM32_STM32F429) || \
        defined(CONFIG_STM32_STM32F446) || defined(CONFIG_STM32_STM32F469))
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_7
#  elif (STM32_SYSCLK_FREQUENCY <= 180000000) && \
        (defined(CONFIG_STM32_STM32F427) || defined(CONFIG_STM32_STM32F429) || \
        defined(CONFIG_STM32_STM32F446) || defined(CONFIG_STM32_STM32F469))
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_8
#  else
#    error "STM32_SYSCLK_FREQUENCY is out of range!"
#  endif
#elif (BOARD_STM32F4_VDD <= 2700)
#  if (STM32_SYSCLK_FREQUENCY <= 24000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_0
#  elif (STM32_SYSCLK_FREQUENCY <= 48000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_1
#  elif (STM32_SYSCLK_FREQUENCY <= 72000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_2
#  elif (STM32_SYSCLK_FREQUENCY <= 960000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_3
#  elif (STM32_SYSCLK_FREQUENCY <= 120000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_4
#  elif (STM32_SYSCLK_FREQUENCY <= 144000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_5
#  elif (STM32_SYSCLK_FREQUENCY <= 168000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_6
#  elif (STM32_SYSCLK_FREQUENCY <= 180000000) && \
        (defined(CONFIG_STM32_STM32F427) || defined(CONFIG_STM32_STM32F429) || \
        defined(CONFIG_STM32_STM32F446) || defined(CONFIG_STM32_STM32F469))
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_7
#  else
#    error "STM32_SYSCLK_FREQUENCY is out of range!"
#  endif
#elif (BOARD_STM32F4_VDD <= 3600)
#  if (STM32_SYSCLK_FREQUENCY <= 30000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_0
#  elif (STM32_SYSCLK_FREQUENCY <= 60000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_1
#  elif (STM32_SYSCLK_FREQUENCY <= 90000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_2
#  elif (STM32_SYSCLK_FREQUENCY <= 120000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_3
#  elif (STM32_SYSCLK_FREQUENCY <= 150000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_4
#  elif (STM32_SYSCLK_FREQUENCY <= 168000000)
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_5
#  elif (STM32_SYSCLK_FREQUENCY <= 180000000) && \
        (defined(CONFIG_STM32_STM32F427) || defined(CONFIG_STM32_STM32F429) || \
        defined(CONFIG_STM32_STM32F446) || defined(CONFIG_STM32_STM32F469))
#    define FLASH_ACR_LATENCY_SETTING  FLASH_ACR_LATENCY_5
#  else
#    error "STM32_SYSCLK_FREQUENCY is out of range!"
#  endif
#else
#  error "BOARD_STM32F4_VDD is out of range!"
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

  /* Enable the Internal High Speed clock (HSI) */

  regval = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSION;
  putreg32(regval, STM32_RCC_CR);

  /* Reset CFGR register */

  putreg32(0x00000000, STM32_RCC_CFGR);

  /* Reset HSEON, CSSON and PLLON bits */

  regval  = getreg32(STM32_RCC_CR);
  regval &= ~(RCC_CR_HSEON | RCC_CR_CSSON | RCC_CR_PLLON);
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

#if STM32_NGPIO_PORTS > 0
  regval |= (RCC_AHB1ENR_GPIOAEN
#if STM32_NGPIO_PORTS > 1
             | RCC_AHB1ENR_GPIOBEN
#endif
#if STM32_NGPIO_PORTS > 2
             | RCC_AHB1ENR_GPIOCEN
#endif
#if STM32_NGPIO_PORTS > 3
             | RCC_AHB1ENR_GPIODEN
#endif
#if STM32_NGPIO_PORTS > 4
             | RCC_AHB1ENR_GPIOEEN
#endif
#if STM32_NGPIO_PORTS > 5
             | RCC_AHB1ENR_GPIOFEN
#endif
#if STM32_NGPIO_PORTS > 6
             | RCC_AHB1ENR_GPIOGEN
#endif
#if STM32_NGPIO_PORTS > 7
             | RCC_AHB1ENR_GPIOHEN
#endif
#if STM32_NGPIO_PORTS > 8
             | RCC_AHB1ENR_GPIOIEN
#endif
#if STM32_NGPIO_PORTS > 9
             | RCC_AHB1ENR_GPIOJEN
#endif
#if STM32_NGPIO_PORTS > 10
             | RCC_AHB1ENR_GPIOKEN
#endif
             );
#endif

#ifdef CONFIG_STM32_CRC
  /* CRC clock enable */

  regval |= RCC_AHB1ENR_CRCEN;
#endif

#ifdef CONFIG_STM32_BKPSRAM
  /* Backup SRAM clock enable */

  regval |= RCC_AHB1ENR_BKPSRAMEN;
#endif

#ifdef CONFIG_STM32_CCMDATARAM
  /* CCM data RAM clock enable */

  regval |= RCC_AHB1ENR_CCMDATARAMEN;
#endif

#ifdef CONFIG_STM32_DMA1
  /* DMA 1 clock enable */

  regval |= RCC_AHB1ENR_DMA1EN;
#endif

#ifdef CONFIG_STM32_DMA2
  /* DMA 2 clock enable */

  regval |= RCC_AHB1ENR_DMA2EN;
#endif

#ifdef CONFIG_STM32_ETHMAC
  /* Ethernet MAC clocking */

  regval |= (RCC_AHB1ENR_ETHMACEN | RCC_AHB1ENR_ETHMACTXEN
            | RCC_AHB1ENR_ETHMACRXEN);

#ifdef CONFIG_STM32_ETH_PTP
  /* Precision Time Protocol (PTP) */

  regval |= RCC_AHB1ENR_ETHMACPTPEN;

#endif
#endif

#ifdef CONFIG_STM32_OTGHS
#ifdef BOARD_ENABLE_USBOTG_HSULPI
  /* Enable clocking for  USB OTG HS and external PHY */

  regval |= (RCC_AHB1ENR_OTGHSEN | RCC_AHB1ENR_OTGHSULPIEN);
#else
  /* Enable only clocking for USB OTG HS */

  regval |= RCC_AHB1ENR_OTGHSEN;
#endif
#endif /* CONFIG_STM32_OTGHS */

#ifdef CONFIG_STM32_DMA2D
  /* DMA2D clock */

  regval |= RCC_AHB1ENR_DMA2DEN;
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

  /* Set the appropriate bits in the AHB2ENR register to enabled the
   * selected AHB2 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB2ENR);

#ifdef CONFIG_STM32_DCMI
  /* Camera interface enable */

  regval |= RCC_AHB2ENR_DCMIEN;
#endif

#ifdef CONFIG_STM32_CRYP
  /* Cryptographic modules clock enable */

  regval |= RCC_AHB2ENR_CRYPEN;
#endif

#ifdef CONFIG_STM32_HASH
  /* Hash modules clock enable */

  regval |= RCC_AHB2ENR_HASHEN;
#endif

#ifdef CONFIG_STM32_RNG
  /* Random number generator clock enable */

  regval |= RCC_AHB2ENR_RNGEN;
#endif

#ifdef CONFIG_STM32_OTGFS
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
#ifdef CONFIG_STM32_FSMC
  uint32_t regval;

  /* Set the appropriate bits in the AHB3ENR register to enabled the
   * selected AHB3 peripherals.
   */

  regval = getreg32(STM32_RCC_AHB3ENR);

  /* Flexible static memory controller module clock enable */

  regval |= RCC_AHB3ENR_FSMCEN;

  putreg32(regval, STM32_RCC_AHB3ENR);   /* Enable peripherals */
#endif
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

#ifdef CONFIG_STM32_TIM2
  /* TIM2 clock enable */

  regval |= RCC_APB1ENR_TIM2EN;
#endif

#ifdef CONFIG_STM32_TIM3
  /* TIM3 clock enable */

  regval |= RCC_APB1ENR_TIM3EN;
#endif

#ifdef CONFIG_STM32_TIM4
  /* TIM4 clock enable */

  regval |= RCC_APB1ENR_TIM4EN;
#endif

#ifdef CONFIG_STM32_TIM5
  /* TIM5 clock enable */

  regval |= RCC_APB1ENR_TIM5EN;
#endif

#ifdef CONFIG_STM32_TIM6
  /* TIM6 clock enable */

  regval |= RCC_APB1ENR_TIM6EN;
#endif

#ifdef CONFIG_STM32_TIM7
  /* TIM7 clock enable */

  regval |= RCC_APB1ENR_TIM7EN;
#endif

#ifdef CONFIG_STM32_TIM12
  /* TIM12 clock enable */

  regval |= RCC_APB1ENR_TIM12EN;
#endif

#ifdef CONFIG_STM32_TIM13
  /* TIM13 clock enable */

  regval |= RCC_APB1ENR_TIM13EN;
#endif

#ifdef CONFIG_STM32_TIM14
  /* TIM14 clock enable */

  regval |= RCC_APB1ENR_TIM14EN;
#endif

#ifdef CONFIG_STM32_WWDG
  /* Window watchdog clock enable */

  regval |= RCC_APB1ENR_WWDGEN;
#endif

#ifdef CONFIG_STM32_SPI2
  /* SPI2 clock enable */

  regval |= RCC_APB1ENR_SPI2EN;
#endif

#ifdef CONFIG_STM32_SPI3
  /* SPI3 clock enable */

  regval |= RCC_APB1ENR_SPI3EN;
#endif

#ifdef CONFIG_STM32_USART2
  /* USART 2 clock enable */

  regval |= RCC_APB1ENR_USART2EN;
#endif

#ifdef CONFIG_STM32_USART3
  /* USART3 clock enable */

  regval |= RCC_APB1ENR_USART3EN;
#endif

#ifdef CONFIG_STM32_UART4
  /* UART4 clock enable */

  regval |= RCC_APB1ENR_UART4EN;
#endif

#ifdef CONFIG_STM32_UART5
  /* UART5 clock enable */

  regval |= RCC_APB1ENR_UART5EN;
#endif

#ifdef CONFIG_STM32_I2C1
  /* I2C1 clock enable */

  regval |= RCC_APB1ENR_I2C1EN;
#endif

#ifdef CONFIG_STM32_I2C2
  /* I2C2 clock enable */

  regval |= RCC_APB1ENR_I2C2EN;
#endif

#ifdef CONFIG_STM32_I2C3
  /* I2C3 clock enable */

  regval |= RCC_APB1ENR_I2C3EN;
#endif

#ifdef CONFIG_STM32_CAN1
  /* CAN 1 clock enable */

  regval |= RCC_APB1ENR_CAN1EN;
#endif

#ifdef CONFIG_STM32_CAN2
  /* CAN2 clock enable.  NOTE: CAN2 needs CAN1 clock as well. */

  regval |= (RCC_APB1ENR_CAN1EN | RCC_APB1ENR_CAN2EN);
#endif

  /* Power interface clock enable.  The PWR block is always enabled so that
   * we can set the internal voltage regulator for maximum performance.
   */

  regval |= RCC_APB1ENR_PWREN;

#if defined (CONFIG_STM32_DAC1)
  /* DAC1 interface clock enable */

  regval |= RCC_APB1ENR_DAC1EN;
#endif

#ifdef CONFIG_STM32_UART7
  /* UART7 clock enable */

  regval |= RCC_APB1ENR_UART7EN;
#endif

#ifdef CONFIG_STM32_UART8
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

#ifdef CONFIG_STM32_TIM1
  /* TIM1 clock enable */

  regval |= RCC_APB2ENR_TIM1EN;
#endif

#ifdef CONFIG_STM32_TIM8
  /* TIM8 clock enable */

  regval |= RCC_APB2ENR_TIM8EN;
#endif

#ifdef CONFIG_STM32_USART1
  /* USART1 clock enable */

  regval |= RCC_APB2ENR_USART1EN;
#endif

#ifdef CONFIG_STM32_USART6
  /* USART6 clock enable */

  regval |= RCC_APB2ENR_USART6EN;
#endif

#ifdef CONFIG_STM32_ADC1
  /* ADC1 clock enable */

  regval |= RCC_APB2ENR_ADC1EN;
#endif

#ifdef CONFIG_STM32_ADC2
  /* ADC2 clock enable */

  regval |= RCC_APB2ENR_ADC2EN;
#endif

#ifdef CONFIG_STM32_ADC3
  /* ADC3 clock enable */

  regval |= RCC_APB2ENR_ADC3EN;
#endif

#ifdef CONFIG_STM32_SDIO
  /* SDIO clock enable */

  regval |= RCC_APB2ENR_SDIOEN;
#endif

#ifdef CONFIG_STM32_SPI1
  /* SPI1 clock enable */

  regval |= RCC_APB2ENR_SPI1EN;
#endif

#ifdef CONFIG_STM32_SPI4
  /* SPI4 clock enable */

  regval |= RCC_APB2ENR_SPI4EN;
#endif

#ifdef CONFIG_STM32_SYSCFG
  /* System configuration controller clock enable */

  regval |= RCC_APB2ENR_SYSCFGEN;
#endif

#ifdef CONFIG_STM32_TIM9
  /* TIM9 clock enable */

  regval |= RCC_APB2ENR_TIM9EN;
#endif

#ifdef CONFIG_STM32_TIM10
  /* TIM10 clock enable */

  regval |= RCC_APB2ENR_TIM10EN;
#endif

#ifdef CONFIG_STM32_TIM11
  /* TIM11 clock enable */

  regval |= RCC_APB2ENR_TIM11EN;
#endif

#ifdef CONFIG_STM32_SPI5
  /* SPI5 clock enable */

  regval |= RCC_APB2ENR_SPI5EN;
#endif

#ifdef CONFIG_STM32_SPI6
  /* SPI6 clock enable */

  regval |= RCC_APB2ENR_SPI6EN;
#endif

#ifdef CONFIG_STM32_LTDC
  /* LTDC clock enable */

  regval |= RCC_APB2ENR_LTDCEN;
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

#ifndef CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG
static void stm32_stdclockconfig(void)
{
  uint32_t regval;
  volatile int32_t timeout;

#ifdef STM32_BOARD_USEHSI

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

#ifdef STM32_RCC_CR_HSEBYP
  /* Bypass HSE oscillator when using an external oscillator module
   * Do NOT define STM32_RCC_CR_HSEBYP when using a crystal with HSE
   */

  regval  = getreg32(STM32_RCC_CR);
  regval |= RCC_CR_HSEBYP;
  putreg32(regval, STM32_RCC_CR);
#endif

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
#endif

  /* Check for a timeout.  If this timeout occurs, then we are hosed.  We
   * have no real back-up plan, although the following logic makes it look
   * as though we do.
   */

  if (timeout > 0)
    {
      /* Select regulator voltage output Scale 1 mode to support system
       * frequencies up to 168 MHz.
       */

      regval  = getreg32(STM32_RCC_APB1ENR);
      regval |= RCC_APB1ENR_PWREN;
      putreg32(regval, STM32_RCC_APB1ENR);

      regval  = getreg32(STM32_PWR_CR);
#if defined(CONFIG_STM32_STM32F427) || defined(CONFIG_STM32_STM32F429) || \
    defined(CONFIG_STM32_STM32F446) || defined(CONFIG_STM32_STM32F469) || \
    defined(CONFIG_STM32_STM32F412)
      regval &= ~PWR_CR_VOS_MASK;
      regval |= PWR_CR_VOS_SCALE_1;
#else
      regval |= PWR_CR_VOS;
#endif
      putreg32(regval, STM32_PWR_CR);

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

#ifdef CONFIG_STM32_RTC_HSECLOCK
      /* Set the RTC clock divisor */

      regval  = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_RTCPRE_MASK;
      regval |= RCC_CFGR_RTCPRE(HSE_DIVISOR);
      putreg32(regval, STM32_RCC_CFGR);
#endif

      /* Set the PLL dividers and multipliers to configure the main PLL */

      regval = (STM32_PLLCFG_PLLM | STM32_PLLCFG_PLLN | STM32_PLLCFG_PLLP
                | STM32_PLLCFG_PLLQ
#ifdef STM32_BOARD_USEHSI
                | RCC_PLLCFG_PLLSRC_HSI
#else /* if STM32_BOARD_USEHSE */
                | RCC_PLLCFG_PLLSRC_HSE
#endif
#if defined(STM32_PLLCFG_PLLR)
                | STM32_PLLCFG_PLLR
#endif
                );
      putreg32(regval, STM32_RCC_PLLCFG);

      /* Enable the main PLL */

      regval  = getreg32(STM32_RCC_CR);
      regval |= RCC_CR_PLLON;
      putreg32(regval, STM32_RCC_CR);

      /* Wait until the PLL is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLRDY) == 0)
        {
        }

#if defined(CONFIG_STM32_HAVE_OVERDRIVE) && (STM32_SYSCLK_FREQUENCY > 168000000)

      /* Enable the Over-drive to extend the clock frequency to 180 MHz */

      regval  = getreg32(STM32_PWR_CR);
      regval |= PWR_CR_ODEN;
      putreg32(regval, STM32_PWR_CR);
      while ((getreg32(STM32_PWR_CSR) & PWR_CSR_ODRDY) == 0)
        {
        }

      regval  = getreg32(STM32_PWR_CR);
      regval |= PWR_CR_ODSWEN;
      putreg32(regval, STM32_PWR_CR);
      while ((getreg32(STM32_PWR_CSR) & PWR_CSR_ODSWRDY) == 0)
        {
        }
#endif

      /* Enable FLASH prefetch, instruction cache, data cache,
       * and set FLASH wait states.
       */

      regval = (FLASH_ACR_LATENCY_SETTING
#ifdef CONFIG_STM32_FLASH_ICACHE
                | FLASH_ACR_ICEN
#endif
#ifdef CONFIG_STM32_FLASH_DCACHE
                | FLASH_ACR_DCEN
#endif
#if defined(CONFIG_STM32_FLASH_PREFETCH) && (BOARD_STM32F4_VDD > 2100)
                | FLASH_ACR_PRFTEN
#endif
                );
      putreg32(regval, STM32_FLASH_ACR);

      /* Select the main PLL as system clock source */

      regval  = getreg32(STM32_RCC_CFGR);
      regval &= ~RCC_CFGR_SW_MASK;
      regval |= RCC_CFGR_SW_PLL;
      putreg32(regval, STM32_RCC_CFGR);

      /* Wait until the PLL source is used as the system clock source */

      while ((getreg32(STM32_RCC_CFGR) & RCC_CFGR_SWS_MASK)
              != RCC_CFGR_SWS_PLL)
        {
        }

#if defined(CONFIG_STM32_LTDC) || defined(CONFIG_STM32_SAIPLL)

      /* Configure PLLSAI */

      regval  = getreg32(STM32_RCC_PLLSAICFGR);
#  if defined(CONFIG_STM32_STM32F446)
      regval &= ~(RCC_PLLSAICFGR_PLLSAIM_MASK
                  | RCC_PLLSAICFGR_PLLSAIN_MASK
                  | RCC_PLLSAICFGR_PLLSAIP_MASK
                  | RCC_PLLSAICFGR_PLLSAIQ_MASK);
      regval |= (STM32_RCC_PLLSAICFGR_PLLSAIM
                 | STM32_RCC_PLLSAICFGR_PLLSAIN
                 | STM32_RCC_PLLSAICFGR_PLLSAIP
                 | STM32_RCC_PLLSAICFGR_PLLSAIQ);
#  elif defined(CONFIG_STM32_STM32F469)
      regval &= ~(RCC_PLLSAICFGR_PLLSAIN_MASK
                 | RCC_PLLSAICFGR_PLLSAIP_MASK
                 | RCC_PLLSAICFGR_PLLSAIQ_MASK
                 | RCC_PLLSAICFGR_PLLSAIR_MASK);
      regval |= (STM32_RCC_PLLSAICFGR_PLLSAIN
                | STM32_RCC_PLLSAICFGR_PLLSAIP
                | STM32_RCC_PLLSAICFGR_PLLSAIQ
                | STM32_RCC_PLLSAICFGR_PLLSAIR);
#  else
      regval &= ~(RCC_PLLSAICFGR_PLLSAIN_MASK
                 | RCC_PLLSAICFGR_PLLSAIQ_MASK
                 | RCC_PLLSAICFGR_PLLSAIR_MASK);
      regval |= (STM32_RCC_PLLSAICFGR_PLLSAIN
                | STM32_RCC_PLLSAICFGR_PLLSAIQ
                | STM32_RCC_PLLSAICFGR_PLLSAIR);
#  endif
      putreg32(regval, STM32_RCC_PLLSAICFGR);

      regval  = getreg32(STM32_RCC_DCKCFGR);
#  if defined(CONFIG_STM32_STM32F446)
      regval &= ~(RCC_DCKCFGR_PLLI2SDIVQ_MASK
                 | RCC_DCKCFGR_PLLSAIDIVQ_MASK
                 | RCC_DCKCFGR_SAI1SRC_MASK
                 | RCC_DCKCFGR_SAI2SRC_MASK
                 | RCC_DCKCFGR_TIMPRE
                 | RCC_DCKCFGR_I2S1SRC_MASK
                 | RCC_DCKCFGR_I2S2SRC_MASK);
      regval |= (STM32_RCC_DCKCFGR_PLLI2SDIVQ
                | STM32_RCC_DCKCFGR_PLLSAIDIVQ
                | STM32_RCC_DCKCFGR_SAI1SRC
                | STM32_RCC_DCKCFGR_SAI2SRC
                | STM32_RCC_DCKCFGR_TIMPRE
                | STM32_RCC_DCKCFGR_I2S1SRC
                | STM32_RCC_DCKCFGR_I2S2SRC);
#  elif defined(CONFIG_STM32_STM32F469)
      regval &= ~(RCC_DCKCFGR_PLLI2SDIVQ_MASK
                 | RCC_DCKCFGR_PLLSAIDIVQ_MASK
                 | RCC_DCKCFGR_PLLSAIDIVR_MASK
                 | RCC_DCKCFGR_SAI1ASRC_MASK
                 | RCC_DCKCFGR_SAI1BSRC_MASK
                 | RCC_DCKCFGR_TIMPRE
                 | RCC_DCKCFGR_48MSEL_MASK
                 | RCC_DCKCFGR_SDMMCSEL_MASK
                 | RCC_DCKCFGR_DSISEL_MASK);
      regval |= (STM32_RCC_DCKCFGR_PLLI2SDIVQ
                | STM32_RCC_DCKCFGR_PLLSAIDIVQ
                | STM32_RCC_DCKCFGR_PLLSAIDIVR
                | STM32_RCC_DCKCFGR_SAI1ASRC
                | STM32_RCC_DCKCFGR_SAI1BSRC
                | STM32_RCC_DCKCFGR_TIMPRE
                | STM32_RCC_DCKCFGR_48MSEL
                | STM32_RCC_DCKCFGR_SDMMCSEL
                | STM32_RCC_DCKCFGR_DSISEL);
#  else
      regval &= ~RCC_DCKCFGR_PLLSAIDIVR_MASK;
      regval |= STM32_RCC_DCKCFGR_PLLSAIDIVR;
#  endif
      putreg32(regval, STM32_RCC_DCKCFGR);

      /* Enable PLLSAI */

      regval  = getreg32(STM32_RCC_CR);
      regval |= RCC_CR_PLLSAION;
      putreg32(regval, STM32_RCC_CR);

      /* Wait until the PLLSAI is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLSAIRDY) == 0)
        {
        }
#endif

#if defined(CONFIG_STM32_I2SPLL)

      /* Configure PLLI2S */

      regval  = getreg32(STM32_RCC_PLLI2SCFGR);

#  if defined(CONFIG_STM32_STM32F446)

      regval &= ~(RCC_PLLI2SCFGR_PLLI2SM_MASK
                 | RCC_PLLI2SCFGR_PLLI2SN_MASK
                 | RCC_PLLI2SCFGR_PLLI2SP_MASK
                 | RCC_PLLI2SCFGR_PLLI2SQ_MASK
                 | RCC_PLLI2SCFGR_PLLI2SR_MASK);
      regval |= (STM32_RCC_PLLI2SCFGR_PLLI2SM
                | STM32_RCC_PLLI2SCFGR_PLLI2SN
                | STM32_RCC_PLLI2SCFGR_PLLI2SP
                | STM32_RCC_PLLI2SCFGR_PLLI2SQ
                | STM32_RCC_PLLI2SCFGR_PLLI2SR);

#  elif defined(CONFIG_STM32_STM32F469) || defined(CONFIG_STM32_STM32F427)

      regval &= ~(RCC_PLLI2SCFGR_PLLI2SN_MASK
                 | RCC_PLLI2SCFGR_PLLI2SQ_MASK
                 | RCC_PLLI2SCFGR_PLLI2SR_MASK);
      regval |= (STM32_RCC_PLLI2SCFGR_PLLI2SN
                | STM32_RCC_PLLI2SCFGR_PLLI2SQ
                | STM32_RCC_PLLI2SCFGR_PLLI2SR);

#  elif defined(CONFIG_STM32_STM32F412)

      regval &= ~(RCC_PLLI2SCFGR_PLLI2SM_MASK
                 | RCC_PLLI2SCFGR_PLLI2SN_MASK
                 | RCC_PLLI2SCFGR_PLLI2SQ_MASK
                 | RCC_PLLI2SCFGR_PLLI2SR_MASK);
      regval |= (STM32_RCC_PLLI2SCFGR_PLLI2SM
                | STM32_RCC_PLLI2SCFGR_PLLI2SN
                | STM32_RCC_PLLI2SCFGR_PLLI2SQ
                | STM32_RCC_PLLI2SCFGR_PLLI2SR
                | STM32_RCC_PLLI2SCFGR_PLLI2SSRC);
#  endif

      putreg32(regval, STM32_RCC_PLLI2SCFGR);

#  if defined(STM32_RCC_DCKCFGR2)

      regval  = getreg32(STM32_RCC_DCKCFGR2);

#    if defined(CONFIG_STM32_STM32F446)

      regval &= ~(RCC_DCKCFGR2_FMPI2C1SEL_MASK
                 | RCC_DCKCFGR2_CECSEL_MASK
                 | RCC_DCKCFGR2_CK48MSEL_MASK
                 | RCC_DCKCFGR2_SDIOSEL_MASK
                 | RCC_DCKCFGR2_SPDIFRXSEL_MASK);
      regval |= (STM32_RCC_DCKCFGR2_FMPI2C1SEL
                | STM32_RCC_DCKCFGR2_CECSEL
                | STM32_RCC_DCKCFGR2_CK48MSEL
                | STM32_RCC_DCKCFGR2_SDIOSEL
                | STM32_RCC_DCKCFGR2_SPDIFRXSEL);

#    elif defined(CONFIG_STM32_STM32F412)

      regval &= ~(RCC_DCKCFGR2_FMPI2C1SEL_MASK
                 | RCC_DCKCFGR2_CK48MSEL_MASK
                 | RCC_DCKCFGR2_SDIOSEL_MASK);
      regval |= (STM32_RCC_DCKCFGR2_FMPI2C1SEL
                | STM32_RCC_DCKCFGR2_CK48MSEL
                | STM32_RCC_DCKCFGR2_SDIOSEL);
#    endif

      putreg32(regval, STM32_RCC_DCKCFGR2);
#  endif

      /* Enable PLLI2S */

      regval  = getreg32(STM32_RCC_CR);
      regval |= RCC_CR_PLLI2SON;
      putreg32(regval, STM32_RCC_CR);

      /* Wait until the PLLI2S is ready */

      while ((getreg32(STM32_RCC_CR) & RCC_CR_PLLI2SRDY) == 0)
        {
        }
#endif

#if defined(CONFIG_STM32_IWDG) || defined(CONFIG_STM32_RTC_LSICLOCK)
      /* Low speed internal clock source LSI */

      stm32_rcc_enablelsi();
#endif

#if defined(CONFIG_STM32_RTC_LSECLOCK)
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

/****************************************************************************
 * Name: efm32_itm_syslog
 *
 * Description:
 *   Enable Serial wire output pin, configure debug clocking, and enable
 *   ITM syslog support.
 *
 ****************************************************************************/

#ifdef CONFIG_ARMV7M_ITMSYSLOG
static inline void rcc_itm_syslog(void)
{
  /* Enable SWO output */

  modifyreg32(STM32_DBGMCU_CR, DBGMCU_CR_TRACEMODE_MASK, DBGMCU_CR_ASYNCH |
              DBGMCU_CR_TRACEIOEN);
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
