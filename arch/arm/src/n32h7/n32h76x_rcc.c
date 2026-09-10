/****************************************************************************
 * arch/arm/src/n32h7/n32h76x_rcc.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>

#include <arch/board/board.h>
#include <arch/n32h7/chip.h>

#include "arm_internal.h"
#include "n32_gpio.h"
#include "n32_rcc.h"
#include "n32_pwr.h"

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

#define HSE_DIVISOR (N32_HSE_FREQUENCY + 500000) / 1000000

/* PLL are only enabled if the P,Q or R outputs are enabled. */

#undef USE_PLL1
#ifdef N32_PLL1_VCO_FREQUENCY
#  define USE_PLL1
#endif

#undef USE_PLL2
#ifdef N32_PLL2_VCO_FREQUENCY
#  define USE_PLL2
#endif

#undef USE_PLL3
#ifdef N32_PLL3_VCO_FREQUENCY
#  define USE_PLL3
#endif

#if defined(N32_BOARD_USEHSI) && !defined(N32_BOARD_HSIDIV)
#error When HSI is used, you have to define N32_BOARD_HSIDIV in board/include/board.h
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

  /* Config PWR */

  n32_pwr_setcorepwrsrc(PWR_VCORESRC_LDO);

  /* Set HSIEN bit */

  regval  = getreg32(N32_RCC_SRCCTRL1);
  regval |= RCC_SRCCTRL1_HSIEN;

  /* Reset SCLKSW, MSIEN ,HSEBP, HSEEN bits */

  regval &= ~(RCC_SRCCTRL1_SCLKSW_MASK | RCC_SRCCTRL1_MSIEN |
              RCC_SRCCTRL1_HSEBP       | RCC_SRCCTRL1_HSEEN);
  putreg32(regval, N32_RCC_SRCCTRL1);

  /* Reset M7HYPSEL, AXIHYPSEL */

  regval  = getreg32(N32_RCC_SRCCTRL2);
  regval &= ~(RCC_SRCCTRL2_M7HYPSEL_MASK | RCC_SRCCTRL2_AXIHYPSEL_MASK);
  putreg32(regval, N32_RCC_SRCCTRL2);

  /* Reset SYSBUSDIV1,  SYSBUSDIV2 */

  putreg32(0x00000000, N32_RCC_SYSBUSDIV1);
  putreg32(0x00000000, N32_RCC_SYSBUSDIV2);

  /* Reset PLL1CTRL1, PLL2CTRL1, PLL3CTRL1 */

  regval  = 0x09000000;
  regval |= RCC_PLL1CTRL1_PLL1SRC_NONE | RCC_PLL1CTRL1_PLL1RST |
            RCC_PLL1CTRL1_PLL1PD | RCC_PLL1CTRL1_PLL1BWAJ(4);
  putreg32(regval, N32_RCC_PLL1CTRL1);
  putreg32(regval, N32_RCC_PLL2CTRL1);
  putreg32(regval, N32_RCC_PLL3CTRL1);

  /* Reset CFG2 register */

  regval  = RCC_CFG2_M4CAHIEN    | RCC_CFG2_M4CAHIPCLKEN |
            RCC_CFG2_M4CAHDEN    | RCC_CFG2_M4CAHDPCLKEN |
            RCC_CFG2_M7MMUEN     | RCC_CFG2_M4MMUEN      |
            RCC_CFG2_M7SRAMBKPEN | RCC_CFG2_M4SRAMBKPEN  |
            RCC_CFG2_M7SRAM1EN   | RCC_CFG2_M4SRAM1EN    |
            RCC_CFG2_M7SRAM2EN   | RCC_CFG2_M4SRAM2EN    |
            RCC_CFG2_M7SRAM3EN   | RCC_CFG2_M4SRAM3EN    |
            RCC_CFG2_M7SRAM4EN   | RCC_CFG2_M4SRAM4EN;
  putreg32(regval, N32_RCC_CFG2);

  /* Reset CFG3 register */

  regval  = RCC_CFG3_M7STCLKDIV_DIV8 | RCC_CFG3_M4STCLKDIV_DIV8;
  putreg32(regval, N32_RCC_CFG3);

  /* Reset CFG4 register */

  regval  = ~(RCC_CFG4_RSVD | RCC_CFG4_DCMURST);
  putreg32(regval, N32_RCC_CFG4);

  /* Reset CFG5 register */

  regval  = RCC_CFG5_RTCHSEDIV(2) | RCC_CFG5_M7SRAM5EN |
            RCC_CFG5_M4SRAM5EN    | RCC_CFG5_DCDCLKEN;
  putreg32(regval, N32_RCC_CFG5);

  /* Disable all interrupts and clear pending bits  */

  putreg32(0x00000000, N32_RCC_CLKINT1);
  putreg32(0x00000000, N32_RCC_CLKINT2);
  putreg32(0x00000000, N32_RCC_CLKINT3);
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

  /* Set the appropriate bits in the RCC_AHB1ENx register to enabled the
   * selected AHB1 peripherals.
   */

  regval = getreg32(N32_RCC_AHB1EN1);

#if defined(CONFIG_N32H7_EMMC)
  /* eMMC clock enable */

  regval |= RCC_AHB1EN1_M7SDMMC2EN;
#endif

#if defined(CONFIG_N32H7_USB2)
  /* USB OTG FS clock enable */

  regval |= RCC_AHB1EN1_M7USB2EN;
#endif

#if defined(CONFIG_N32H7_DMA)
  /* DMA MUX1 clock enable */

  regval |= RCC_AHB1EN1_M7DMAMUX1EN;
#endif

#if defined(CONFIG_N32H7_ADC1)
  /* ADC1 clock enable */

  regval |= RCC_AHB1EN1_M7ADC1PLLEN;
  regval |= RCC_AHB1EN1_M7ADC1SYSEN;
  regval |= RCC_AHB1EN1_M7ADC1BUSEN;
#endif
  putreg32(regval, N32_RCC_AHB1EN1);   /* Enable peripherals */
  regval = getreg32(N32_RCC_AHB1EN2);

#if defined(CONFIG_N32H7_ETH2)
  /* ETH2 clock enable */

  regval |= RCC_AHB1EN2_M7ETH2TXEN;
  regval |= RCC_AHB1EN2_M7ETH2RXEN;
  regval |= RCC_AHB1EN2_M7ETH2MACEN;
#endif
  putreg32(regval, N32_RCC_AHB1EN2);   /* Enable peripherals */
  regval = getreg32(N32_RCC_AHB1EN3);

#if defined(CONFIG_N32H7_ECCMAC)
  /* ECCMAC clock enable */

  regval |= RCC_AHB1EN3_M7ECCMACEN;
#endif

#if defined(CONFIG_N32H7_DMA)
  /* DMA clock enable */

  regval |= RCC_AHB1EN3_M7DMA1EN;
  regval |= RCC_AHB1EN3_M7DMA2EN;
  regval |= RCC_AHB1EN3_M7DMA3EN;
#endif
  putreg32(regval, N32_RCC_AHB1EN3);   /* Enable peripherals */
  regval = getreg32(N32_RCC_AHB1EN4);

#if defined(CONFIG_N32H7_ADC2)
  /* ADC2 clock enable */

  regval |= RCC_AHB1EN4_M7ADC2PLLEN;
  regval |= RCC_AHB1EN4_M7ADC2SYSEN;
  regval |= RCC_AHB1EN4_M7ADC2BUSEN;
#endif
#if defined(CONFIG_N32H7_ADC3)
  /* ADC3 clock enable */

  regval |= RCC_AHB1EN4_M7ADC3PLLEN;
  regval |= RCC_AHB1EN4_M7ADC3SYSEN;
  regval |= RCC_AHB1EN4_M7ADC3BUSEN;
#endif
  putreg32(regval, N32_RCC_AHB1EN4);   /* Enable peripherals */
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

  /* Set the appropriate bits in the RCC_APB1ENx register to enable the
   * selected APB1 peripherals.
   */

  regval = getreg32(N32_RCC_APB1EN1);

#ifdef CONFIG_N32H7_SPI3
  /* SPI3 clock enable */

  regval |= RCC_APB1EN2_M7SPI3EN;
#endif

#ifdef CONFIG_N32H7_DAC12
  /* DAC12 clock enable */

  regval |= RCC_APB1EN2_M7DAC12EN;
#endif

#ifdef CONFIG_N32H7_WWDG2
  /* WWDG2 clock enable */

  regval |= RCC_APB1EN2_M7WWDG2EN;
#endif
  putreg32(regval, N32_RCC_APB1EN2);   /* Enable peripherals */
  regval = getreg32(N32_RCC_APB1EN3);

#ifdef CONFIG_N32H7_USART1
  /* USART1 clock enable */

  regval |= RCC_APB1EN3_M7USART1EN;
#endif

#ifdef CONFIG_N32H7_USART2
  /* USART2 clock enable */

  regval |= RCC_APB1EN3_M7USART2EN;
#endif

#ifdef CONFIG_N32H7_USART3
  /* USART3 clock enable */

  regval |= RCC_APB1EN3_M7USART3EN;
#endif

#ifdef CONFIG_N32H7_USART4
  /* USART4 clock enable */

  regval |= RCC_APB1EN3_M7USART4EN;
#endif

#ifdef CONFIG_N32H7_UART9
  /* UART9 clock enable */

  regval |= RCC_APB1EN3_M7UART9EN;
#endif

#ifdef CONFIG_N32H7_UART10
  /* UART10 clock enable */

  regval |= RCC_APB1EN3_M7UART10EN;
#endif

#ifdef CONFIG_N32H7_UART11
  /* UART11 clock enable */

  regval |= RCC_APB1EN3_M7UART11EN;
#endif

#ifdef CONFIG_N32H7_UART12
  /* UART12 clock enable */

  regval |= RCC_APB1EN3_M7UART12EN;
#endif
  putreg32(regval, N32_RCC_APB1EN3);   /* Enable peripherals */
  regval = getreg32(N32_RCC_APB1EN4);

#ifdef CONFIG_N32H7_I2S3
  /* I2S3 clock enable */

  regval |= RCC_APB1EN4_M7I2S3EN;
#endif

#ifdef CONFIG_N32H7_I2S4
  /* I2S4 clock enable */

  regval |= RCC_APB1EN4_M7I2S4EN;
#endif

#ifdef CONFIG_N32H7_I2C1
  /* I2C1 clock enable */

  regval |= RCC_APB1EN4_M7I2C1EN;
#endif

#ifdef CONFIG_N32H7_I2C2
  /* I2C2 clock enable */

  regval |= RCC_APB1EN4_M7I2C2EN;
#endif

#ifdef CONFIG_N32H7_I2C3
  /* I2C3 clock enable */

  regval |= RCC_APB1EN4_M7I2C3EN;
#endif
  putreg32(regval, N32_RCC_APB1EN4);   /* Enable peripherals */
  regval = getreg32(N32_RCC_APB1EN5);

#ifdef CONFIG_N32H7_FDCAN1
  /* FDCAN1 clock enable */

  regval |= RCC_APB1EN5_M7FDCAN1EN;
#endif

#ifdef CONFIG_N32H7_FDCAN2
  /* FDCAN2 clock enable */

  regval |= RCC_APB1EN5_M7FDCAN2EN;
#endif

#ifdef CONFIG_N32H7_FDCAN5
  /* FDCAN5 clock enable */

  regval |= RCC_APB1EN5_M7FDCAN5EN;
#endif

#ifdef CONFIG_N32H7_FDCAN6
  /* FDCAN6 clock enable */

  regval |= RCC_APB1EN5_M7FDCAN6EN;
#endif
  putreg32(regval, N32_RCC_APB1EN5);   /* Enable peripherals */
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

  /* Set the appropriate bits in the RCC_AHB2EN register to enable the
   * selected AHB2 peripherals.
   */

  regval = getreg32(N32_RCC_AHB2EN1);

#ifdef CONFIG_N32H7_USB1
  /* USB1 clock enable */

  regval |= RCC_AHB2EN1_M7USB1EN;
#endif

#ifdef CONFIG_N32H7_ECCM2
  /* ECCM2 clock enable */

  regval |= RCC_AHB2EN1_M7ECCM2EN;
#endif

#ifdef CONFIG_N32H7_CORDIC
  /* CORDIC clock enable */

  regval |= RCC_AHB2EN1_M7CORDICEN;
#endif

#ifdef CONFIG_N32H7_SDP
  /* SDP clock enable */

  regval |= RCC_AHB2EN1_M7SDPUEN;
#endif

#ifdef CONFIG_N32H7_FMAC
  /* FMAC clock enable */

  regval |= RCC_AHB2EN1_M7FMACEN;
#endif
  putreg32(regval, N32_RCC_AHB2EN1);   /* Enable peripherals */
  regval = getreg32(N32_RCC_AHB2EN2);

#ifdef CONFIG_N32H7_DAC56
  /* DAC56 clock enable */

  regval |= RCC_AHB2EN2_M7DAC56EN;
#endif

#ifdef CONFIG_N32H7_DAC34
  /* DAC34 clock enable */

  regval |= RCC_AHB2EN2_M7DAC34EN;
#endif

#ifdef CONFIG_N32H7_ETH1
  /* ETH1 clock enable */

  regval |= RCC_AHB2EN2_M7ETH1TEN;
  regval |= RCC_AHB2EN2_M7ETH1REN;
  regval |= RCC_AHB2EN2_M7ETH1MEN;
#endif
  putreg32(regval, N32_RCC_AHB2EN2);   /* Enable peripherals */
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

  /* Set the appropriate bits in the RCC_APB2ENx register to enable the
   * selected APB2 peripherals.
   */

  regval = getreg32(N32_RCC_APB2EN1);

#ifdef CONFIG_N32H7_SHRTIM1
  /* SHRTIM1 clock enable */

  regval |= RCC_APB2EN1_M7SHRTIM1EN;
#endif

#ifdef CONFIG_N32H7_SHRTIM2
  /* SHRTIM2 clock enable */

  regval |= RCC_APB2EN1_M7SHRTIM2EN;
#endif
  putreg32(regval, N32_RCC_APB2EN1);   /* Enable peripherals */
  regval = getreg32(N32_RCC_APB2EN2);

#ifdef CONFIG_N32H7_I2S1
  /* I2S1 clock enable */

  regval |= RCC_APB2EN2_M7I2S1EN;
#endif

#ifdef CONFIG_N32H7_I2S2
  /* I2S2 clock enable */

  regval |= RCC_APB2EN2_M7I2S2EN;
#endif

#ifdef CONFIG_N32H7_SPI1
  /* SPI1 clock enable */

  regval |= RCC_APB2EN2_M7SPI1EN;
#endif

#ifdef CONFIG_N32H7_SPI2
  /* SPI2 clock enable */

  regval |= RCC_APB2EN2_M7SPI2EN;
#endif

#ifdef CONFIG_N32H7_DSMU
  /* DSMU clock enable */

  regval |= RCC_APB2EN2_M7DSMUEN;
#endif

#ifdef CONFIG_N32H7_I2C4
  /* I2C4 clock enable */

  regval |= RCC_APB2EN2_M7I2C4EN;
#endif

#ifdef CONFIG_N32H7_I2C5
  /* I2C5 clock enable */

  regval |= RCC_APB2EN2_M7I2C5EN;
#endif

#ifdef CONFIG_N32H7_I2C6
  /* I2C6 clock enable */

  regval |= RCC_APB2EN2_M7I2C6EN;
#endif
  putreg32(regval, N32_RCC_APB2EN2);   /* Enable peripherals */
  regval = getreg32(N32_RCC_APB2EN3);

#ifdef CONFIG_N32H7_USART5
  /* USART5 clock enable */

  regval |= RCC_APB2EN3_M7USART5EN;
#endif

#ifdef CONFIG_N32H7_USART6
  /* USART6 clock enable */

  regval |= RCC_APB2EN3_M7USART6EN;
#endif

#ifdef CONFIG_N32H7_USART7
  /* USART7 clock enable */

  regval |= RCC_APB2EN3_M7USART7EN;
#endif

#ifdef CONFIG_N32H7_USART8
  /* USART8 clock enable */

  regval |= RCC_APB2EN3_M7USART8EN;
#endif

#ifdef CONFIG_N32H7_UART13
  /* UART13 clock enable */

  regval |= RCC_APB2EN3_M7UART13EN;
#endif

#ifdef CONFIG_N32H7_UART14
  /* UART14 clock enable */

  regval |= RCC_APB2EN3_M7UART14EN;
#endif

#ifdef CONFIG_N32H7_UART15
  /* UART15 clock enable */

  regval |= RCC_APB2EN3_M7UART15EN;
#endif
  putreg32(regval, N32_RCC_APB2EN3);   /* Enable peripherals */
  regval = getreg32(N32_RCC_APB2EN4);

#ifdef CONFIG_N32H7_FDCAN3
  /* FDCAN3 clock enable */

  regval |= RCC_APB2EN4_M7FDCAN3EN;
#endif

#ifdef CONFIG_N32H7_FDCAN4
  /* FDCAN4 clock enable */

  regval |= RCC_APB2EN4_M7FDCAN4EN;
#endif

#ifdef CONFIG_N32H7_FDCAN7
  /* FDCAN7 clock enable */

  regval |= RCC_APB2EN4_M7FDCAN7EN;
#endif

#ifdef CONFIG_N32H7_FDCAN8
  /* FDCAN8 clock enable */

  regval |= RCC_APB2EN4_M7FDCAN8EN;
#endif
  putreg32(regval, N32_RCC_APB2EN4);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableahb5
 *
 * Description:
 *   Enable selected AHB5 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb5(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the RCC_AHB5ENx register to enable the
   * selected AHB5 peripherals.
   */

  regval = getreg32(N32_RCC_AHB5EN1);

  /* GPIOA clock enable */

  regval |= RCC_AHB5EN1_M7GPIOAEN;

  /* GPIOB clock enable */

  regval |= RCC_AHB5EN1_M7GPIOBEN;

  /* GPIOC clock enable */

  regval |= RCC_AHB5EN1_M7GPIOCEN;

  /* GPIOD clock enable */

  regval |= RCC_AHB5EN1_M7GPIODEN;

#if N32H7_NGPIO > 4
  /* GPIOE clock enable */

  regval |= RCC_AHB5EN1_M7GPIOEEN;
#endif

#if N32H7_NGPIO > 5
  /* GPIOF clock enable */

  regval |= RCC_AHB5EN1_M7GPIOFEN;
#endif

#if N32H7_NGPIO > 6
  /* GPIOG clock enable */

  regval |= RCC_AHB5EN1_M7GPIOGEN;
#endif

#if N32H7_NGPIO > 7
  /* GPIOH clock enable */

  regval |= RCC_AHB5EN1_M7GPIOHEN;
#endif
  putreg32(regval, N32_RCC_AHB5EN1);   /* Enable peripherals */
  regval = getreg32(N32_RCC_AHB5EN2);

#if N32H7_NGPIO > 8
  /* GPIOI clock enable */

  regval |= RCC_AHB5EN2_M7GPIOIEN;
#endif

#if N32H7_NGPIO > 9
  /* GPIOJ clock enable */

  regval |= RCC_AHB5EN2_M7GPIOJEN;
#endif

#if N32H7_NGPIO > 10
  /* GPIOK clock enable */

  regval |= RCC_AHB5EN2_M7GPIOKEN;
#endif

#ifdef CONFIG_N32H7_ECCM3
  /* ECCM3 clock enable */

  regval |= RCC_AHB5EN2_M7ECCM3EN;
#endif

  /* PWR clock enable */

  regval |= RCC_AHB5EN2_PWREN;

#ifdef CONFIG_N32H7_CRC
  /* CRC clock enable */

  regval |= RCC_AHB5EN2_M7CRCEN;
#endif

#ifdef CONFIG_N32H7_SEMA4
  /* SEMA4 clock enable */

  regval |= RCC_AHB5EN2_M7SEMA4EN;
#endif

  /* AFIO clock enable */

  regval |= RCC_AHB5EN2_M7AFIOEN;

  putreg32(regval, N32_RCC_AHB5EN2);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableapb5
 *
 * Description:
 *   Enable selected APB5 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableapb5(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the RCC_APB5ENx register to enable the
   * selected APB5 peripherals.
   */

  regval = getreg32(N32_RCC_APB5EN1);

#ifdef CONFIG_N32H7_AFEC
  /* AFEC clock enable */

  regval |= RCC_APB5EN1_M7AFECEN;
#endif

#ifdef CONFIG_N32H7_SPI4
  /* SPI4 clock enable */

  regval |= RCC_APB5EN1_M7SPI4EN;
#endif

#ifdef CONFIG_N32H7_SPI5
  /* SPI5 clock enable */

  regval |= RCC_APB5EN1_M7SPI5EN;
#endif

#ifdef CONFIG_N32H7_SPI6
  /* SPI6 clock enable */

  regval |= RCC_APB5EN1_M7SPI6EN;
#endif

#ifdef CONFIG_N32H7_SPI7
  /* SPI7 clock enable */

  regval |= RCC_APB5EN1_M7SPI7EN;
#endif
  putreg32(regval, N32_RCC_APB5EN1);   /* Enable peripherals */
  regval = getreg32(N32_RCC_APB5EN2);

#ifdef CONFIG_N32H7_I2C7
  /* I2C7 clock enable */

  regval |= RCC_APB5EN2_M7I2C7EN;
#endif

#ifdef CONFIG_N32H7_I2C8
  /* I2C8 clock enable */

  regval |= RCC_APB5EN2_M7I2C8EN;
#endif

#ifdef CONFIG_N32H7_I2C9
  /* I2C9 clock enable */

  regval |= RCC_APB5EN2_M7I2C9EN;
#endif

#ifdef CONFIG_N32H7_I2C10
  /* I2C10 clock enable */

  regval |= RCC_APB5EN2_M7I2C10EN;
#endif

#ifdef CONFIG_N32H7_EXTI
  /* EXTI clock enable */

  regval |= RCC_APB5EN2_EXTIEN;
#endif

#ifdef CONFIG_N32H7_RTC
  /* RTC clock enable */

  regval |= RCC_APB5EN2_M7RTCPCLKEN;
#endif

#ifdef CONFIG_N32H7_IWDG1
  /* IWDG1 clock enable */

  regval |= RCC_APB5EN2_IWDG1PCLKEN;
#endif

#ifdef CONFIG_N32H7_IWDG2
  /* IWDG2 clock enable */

  regval |= RCC_APB5EN2_IWDG2PCLKEN;
#endif
  putreg32(regval, N32_RCC_APB5EN2);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableahb9
 *
 * Description:
 *   Enable selected APB3 peripherals
 *
 ****************************************************************************/

static inline void rcc_enableahb9(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the RCC_AHB9EN1 register to enable the
   * selected AHB9 peripherals.
   */

  regval = getreg32(N32_RCC_AHB9EN1);

#ifdef CONFIG_N32H7_ESC
  /* ESC clock enable */

  regval |= RCC_AHB9EN1_M7ESCEN;
#endif
  putreg32(regval, N32_RCC_AHB9EN1);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enablerd
 *
 * Description:
 *   Enable selected Retention Domain peripherals
 *
 ****************************************************************************/

static inline void rcc_enablerd(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the RCC_RDENx register to enable the
   * selected low-power peripherals.
   */

  regval = getreg32(N32_RCC_RDEN1);

#ifdef CONFIG_N32H7_LPUART1
  /* LPUART1 clock enable */

  regval |= RCC_RDEN1_M7LPUART1EN;
#endif

#ifdef CONFIG_N32H7_LPUART2
  /* LPUART2 clock enable */

  regval |= RCC_RDEN1_M7LPUART2EN;
#endif
  putreg32(regval, N32_RCC_RDEN1);   /* Enable peripherals */
  regval = getreg32(N32_RCC_RDEN2);

#ifdef CONFIG_N32H7_COMP
  /* COMP clock enable */

  regval |= RCC_RDEN2_M7COMPEN;
#endif
  putreg32(regval, N32_RCC_RDEN2);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enableaxi
 *
 * Description:
 *   Enable selected AXI peripherals
 *
 ****************************************************************************/

static inline void rcc_enableaxi(void)
{
  uint32_t regval;

  /* Set the appropriate bits in the RCC_AXIENx register to enable the
   * selected AXI peripherals.
   */

  regval = getreg32(N32_RCC_AXIEN1);

#ifdef CONFIG_N32H7_JPEGD
  /* JPEGD clock enable */

  regval |= RCC_AXIEN1_M7JPEGDEN;
#endif

#ifdef CONFIG_N32H7_JPEGE
  /* JPEGE clock enable */

  regval |= RCC_AXIEN1_M7JPEGEEN;
#endif

#ifdef CONFIG_N32H7_DMA
  /* DMA MUX2 clock enable */

  regval |= RCC_AXIEN1_M7DMAMUX2EN;
#endif

#ifdef CONFIG_N32H7_MDMA
  /* MDMA clock enable */

  regval |= RCC_AXIEN1_M7MDMAEN;
#endif

#ifdef CONFIG_N32H7_SDMMC1
  /* SDMMC1 clock enable */

  regval |= RCC_AXIEN1_M7SDMMC1EN;
#endif

#ifdef CONFIG_N32H7_ECCM1
  /* ECCM1 clock enable */

  regval |= RCC_AXIEN1_M7ECCM1EN;
#endif

#ifdef CONFIG_N32H7_OTPC
  /* OTPC clock enable */

  regval |= RCC_AXIEN1_M7OTPCEN;
#endif
  putreg32(regval, N32_RCC_AXIEN1);   /* Enable peripherals */
  regval = getreg32(N32_RCC_AXIEN2);

#ifdef CONFIG_N32H7_DSI
  /* DSI clock enable */

  regval |= RCC_AXIEN2_M7DSIEN;
#endif

#ifdef CONFIG_N32H7_LCD
  /* LCD clock enable */

  regval |= RCC_AXIEN2_M7LCDEN;
#endif

#ifdef CONFIG_N32H7_DVP1
  /* DVP1 clock enable */

  regval |= RCC_AXIEN2_M7DVP1EN;
#endif

#ifdef CONFIG_N32H7_DVP2
  /* DVP2 clock enable */

  regval |= RCC_AXIEN2_M7DVP2EN;
#endif

#ifdef CONFIG_N32H7_WWDG1
  /* WWDG1 clock enable */

  regval |= RCC_AXIEN2_M7WWDG1EN;
#endif
  putreg32(regval, N32_RCC_AXIEN2);   /* Enable peripherals */
  regval = getreg32(N32_RCC_AXIEN3);

#ifdef CONFIG_N32H7_TASRAM2
  /* TASRAM2 clock enable */

  regval |= RCC_AXIEN3_M7TASRAM2EN;
#endif

#ifdef CONFIG_N32H7_TASRAM3
  /* TASRAM3 clock enable */

  regval |= RCC_AXIEN3_M7TASRAM3EN;
#endif

#ifdef CONFIG_N32H7_TCM
  /* TCM clock enable */

  regval |= RCC_AXIEN3_M7TCMEN;
#endif

#ifdef CONFIG_N32H7_TCMAXI
  /* TCMAXI clock enable */

  regval |= RCC_AXIEN3_M7TCMAXIEN;
#endif

#ifdef CONFIG_N32H7_TCMAPB
  /* TCMAPB clock enable */

  regval |= RCC_AXIEN3_M7TCMAPBEN;
#endif

#ifdef CONFIG_N32H7_ASRAM1
  /* ASRAM1 clock enable */

  regval |= RCC_AXIEN3_M7ASRAM1EN;
#endif

#ifdef CONFIG_N32H7_AXIROM
  /* AXIROM clock enable */

  regval |= RCC_AXIEN3_M7AXIROMEN;
#endif

#ifdef CONFIG_N32H7_GPU
  /* GPU clock enable */

  regval |= RCC_AXIEN3_M7GPUEN;
#endif
  putreg32(regval, N32_RCC_AXIEN3);   /* Enable peripherals */
  regval = getreg32(N32_RCC_AXIEN4);

#ifdef CONFIG_N32H7_XSPI1
  /* XSPI1 clock enable */

  regval |= RCC_AXIEN4_M7XSPI1EN;
#endif

#ifdef CONFIG_N32H7_XSPI2
  /* XSPI2 clock enable */

  regval |= RCC_AXIEN4_M7XSPI2EN;
#endif

#ifdef CONFIG_N32H7_FEMC
  /* FEMC clock enable */

  regval |= RCC_AXIEN4_M7FEMCEN;
#endif

#ifdef CONFIG_N32H7_SDRAM
  /* SDRAM clock enable */

  regval |= RCC_AXIEN4_M7SDRAMEN;
#endif
  putreg32(regval, N32_RCC_AXIEN4);   /* Enable peripherals */
}

/****************************************************************************
 * Name: rcc_enable_rtc
 *
 * Description:
 *   Enable RTC
 *
 ****************************************************************************/

static inline void rcc_enable_rtc(void)
{
  uint32_t regval;

  regval = getreg32(N32_RCC_BDCTRL);

#ifdef CONFIG_N32H7_RTC
  /* RTC clock enable */

  regval |= RCC_BDCTRL_RTCEN;
#endif
  putreg32(regval, N32_RCC_BDCTRL);
}

/****************************************************************************
 * Name: rcc_enableperiphals
 ****************************************************************************/

static inline void rcc_enableperipherals(void)
{
  rcc_enableahb1();
  rcc_enableapb1();
  rcc_enableahb2();
  rcc_enableapb2();
  rcc_enableahb5();
  rcc_enableapb5();
  rcc_enableahb9();
  rcc_enablerd();
  rcc_enableaxi();
  rcc_enable_rtc();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n32_stdclockconfig
 *
 * Description:
 *   Called to change to new clock based on settings in board.h
 *
 *   NOTE:  This logic would need to be extended if you need to select low-
 *   power clocking modes!
 ****************************************************************************/

void n32_stdclockconfig(void)
{
  uint32_t regval;
  volatile int32_t timeout;

#ifdef N32_BOARD_USEHSI
  /* Enable Internal High-Speed Clock (HSI) */

  regval  = getreg32(N32_RCC_SRCCTRL1);
  regval |= RCC_SRCCTRL1_HSIEN;    /* Enable HSI */
  putreg32(regval, N32_RCC_SRCCTRL1);

  /* Set HSI predivider to board specific value */

  regval  = getreg32(N32_RCC_SYSBUSDIV1);
  regval &= ~RCC_SYSBUSDIV1_HSIDIV_MASK;
  regval |= N32_BOARD_HSIDIV;
  putreg32(regval, N32_RCC_SYSBUSDIV1);

  /* Wait until the HSI is ready (or until a timeout elapsed) */

  for (timeout = HSIRDY_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if the HSIRDF flag is the set in the SRCCTRL1 */

      if ((getreg32(N32_RCC_SRCCTRL1) & RCC_SRCCTRL1_HSIRDF) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }

#else /* if N32_BOARD_USEHSE */
  /* Enable External High-Speed Clock (HSE) */

  regval  = getreg32(N32_RCC_SRCCTRL1);
  regval |= RCC_SRCCTRL1_HSEEN;           /* Enable HSE */
  putreg32(regval, N32_RCC_SRCCTRL1);

  /* Wait until the HSE is ready (or until a timeout elapsed) */

  for (timeout = HSERDY_TIMEOUT; timeout > 0; timeout--)
    {
      /* Check if the HSERDY flag is the set in the SRCCTRL1 */

      if ((getreg32(N32_RCC_SRCCTRL1) & RCC_SRCCTRL1_HSERDF) != 0)
        {
          /* If so, then break-out with timeout > 0 */

          break;
        }
    }
#endif

#ifdef CONFIG_N32H7_MSI
  /* Enable MSI */

  regval  = getreg32(N32_RCC_SRCCTRL1);
  regval |= RCC_SRCCTRL1_MSIEN;
  putreg32(regval, N32_RCC_SRCCTRL1);

  /* Wait until the MSI is ready */

  while ((getreg32(N32_RCC_SRCCTRL1) & RCC_SRCCTRL1_MSIRDF) == 0);
#endif

  /* Check for a timeout.  If this timeout occurs, then we are hosed.  We
   * have no real back-up plan, although the following logic makes it look
   * as though we do.
   */

  if (timeout > 0)
    {
      /* Configure sys_div_clk is sys_clk(PLL1A) = 600M = M7 clock */

      regval = getreg32(N32_RCC_SYSBUSDIV1);
      regval &= ~RCC_SYSBUSDIV1_SYSCLKDIV_MASK;
      regval |= RCC_SYSBUSDIV1_SYSCLKDIV_DIV(1);

      /* Configure sys_bus_div_clk is sys_div_clk/2 = 300M = M4 clock
       *                                            = AHB1\2\5\9
       */

      regval &= ~RCC_SYSBUSDIV1_BUSDIV_MASK;
      regval |= RCC_SYSBUSDIV1_BUSDIV_DIV(2);

      /* Configure AXI clock is sys_div_clk/2 = 300M = AHB6 */

      regval &= ~RCC_SYSBUSDIV1_AXIDIV_MASK;
      regval |= RCC_SYSBUSDIV1_AXIDIV_DIV(2);
      regval &= ~RCC_SYSBUSDIV1_AXIHYPDIV_MASK;
      regval |= RCC_SYSBUSDIV1_AXIHYPDIV_DIV(2);

      /* Configure m7_hyp_div_clk is PLLxA = 600M */

      regval &= ~RCC_SYSBUSDIV1_M7HYPDIV_MASK;
      regval |= RCC_SYSBUSDIV1_M7HYPDIV_DIV(1);

      putreg32(regval, N32_RCC_SYSBUSDIV1);
      __RCC_DELAY_US(1);

      /* Configure M7 sysTick is sys_div_clk/2 = 300M */

      regval = getreg32(N32_RCC_CFG3);
      regval &= ~RCC_CFG3_M7STCLKDIV_MASK;
      regval |= RCC_CFG3_M7STCLKDIV_DIV1;
      putreg32(regval, N32_RCC_CFG3);

      regval = getreg32(N32_RCC_SYSBUSDIV2);

      /* Configure APB1 clock is AHB1/2 = 150M */

      regval &= ~RCC_SYSBUSDIV2_APB1DIV_MASK;
      regval |= RCC_SYSBUSDIV2_APB1DIV_DIV(2);

      /* Configure APB2 clock is AHB2/2 = 150M */

      regval &= ~RCC_SYSBUSDIV2_APB2DIV_MASK;
      regval |= RCC_SYSBUSDIV2_APB2DIV_DIV(2);

      /* Configure APB5 clock is AHB5/2 = 150M */

      regval &= ~RCC_SYSBUSDIV2_APB5DIV_MASK;
      regval |= RCC_SYSBUSDIV2_APB5DIV_DIV(2);

      /* Configure APB6 clock is AHB6/2 = 150M */

      regval &= ~RCC_SYSBUSDIV2_APB6DIV_MASK;
      regval |= RCC_SYSBUSDIV2_APB6DIV_DIV(2);

      putreg32(regval, N32_RCC_SYSBUSDIV2);

      /* configure PLL1_VCOx frequency 600M */

      regval  = getreg32(N32_RCC_PLL1CTRL1);
      regval &= ~RCC_PLL1CTRL1_PLL1BWAJ_MASK;
      regval |= RCC_PLL1CTRL1_PLL1BWAJ(PLL_BWAJ);
      putreg32(regval, N32_RCC_PLL1CTRL1);

      regval  = getreg32(N32_RCC_PLL1CTRL2);
      regval &= ~(RCC_PLL1CTRL2_PLL1CLKF_MASK | RCC_PLL1CTRL2_PLL1CLKR_MASK);
      regval |= (RCC_PLL1CTRL2_PLL1CLKF(PLL_PLLF) |
                 RCC_PLL1CTRL2_PLL1CLKR(PLL_PLLR));
      putreg32(regval, N32_RCC_PLL1CTRL2);

      /* Enable PLL module power */

      regval  = getreg32(N32_RCC_PLL1CTRL1);
      regval |= RCC_PLL1CTRL1_PLL1LDOEN;
      putreg32(regval, N32_RCC_PLL1CTRL1);
      __RCC_DELAY_US(10);

      /* Enable power to analog circuitry in PLL */

      regval  = getreg32(N32_RCC_PLL1CTRL1);
      regval &= ~RCC_PLL1CTRL1_PLL1PD;
      putreg32(regval, N32_RCC_PLL1CTRL1);

      /* Select Clock Source */

      regval  = getreg32(N32_RCC_PLL1CTRL1);
      regval &= ~RCC_PLL1CTRL1_PLL1SRC_MASK;
      regval |= PLL_SRC;
      putreg32(regval, N32_RCC_PLL1CTRL1);
      __RCC_DELAY_US(10);

      /* Clear PLL reset */

      regval  = getreg32(N32_RCC_PLL1CTRL1);
      regval &= ~RCC_PLL1CTRL1_PLL1RST;
      putreg32(regval, N32_RCC_PLL1CTRL1);

      /* Check if PLL1 is locked */

      while ((getreg32(N32_RCC_PLL1CTRL1)&RCC_PLL1CTRL1_PLL1PHLK) !=
             RCC_PLL1CTRL1_PLL1PHLK);

      /* Enable PLL */

      regval  = getreg32(N32_RCC_PLL1CTRL1);
      regval |= RCC_PLL1CTRL1_PLL1EN;
      putreg32(regval, N32_RCC_PLL1CTRL1);
      __RCC_DELAY_US(1);

      /* Configure PLL1A is 600M */

      regval  = getreg32(N32_RCC_PLL1DIV);
      regval &= ~RCC_PLL1DIV_PLL1ADIV_MASK;
      regval |= RCC_PLL1DIV_PLL1ADIV_DIV(1);
      putreg32(regval, N32_RCC_PLL1DIV);

      /* Configure PLL1B is 100M */

      regval  = getreg32(N32_RCC_PLL1DIV);
      regval &= ~RCC_PLL1DIV_PLL1BDIV_MASK;
      regval |= RCC_PLL1DIV_PLL1BDIV_DIV(6);
      putreg32(regval, N32_RCC_PLL1DIV);

      /* Configure sys_clk source is PLL1A */

      regval  = getreg32(N32_RCC_SRCCTRL1);
      regval &= ~RCC_SRCCTRL1_SCLKSW_MASK;
      regval |= RCC_SRCCTRL1_SCLKSW_PLL1A;
      putreg32(regval, N32_RCC_SRCCTRL1);

      /* Check if sys_clk source is PLL1A */

      while ((getreg32(N32_RCC_SRCCTRL1)&RCC_SRCCTRL1_SCLKSTS_MASK) !=
             RCC_SRCCTRL1_SCLKSTS_PLL1A);

      /* configure AXI clock source is PLL1A */

      regval  = getreg32(N32_RCC_SRCCTRL2);
      regval &= ~RCC_SRCCTRL2_AXIHYPSEL_MASK;
      regval |= RCC_SRCCTRL2_AXIHYPSEL_PLL1A;
      putreg32(regval, N32_RCC_SRCCTRL2);

      /* configure M7 clock source is PLL1A */

      regval &= ~RCC_SRCCTRL2_M7HYPSEL_MASK;
      regval |= RCC_SRCCTRL2_M7HYPSEL_PLL1A;
      putreg32(regval, N32_RCC_SRCCTRL2);
    }
}
