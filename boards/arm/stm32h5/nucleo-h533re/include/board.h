/****************************************************************************
 * boards/arm/stm32h5/nucleo-h533re/include/board.h
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

#ifndef __BOARDS_ARM_STM32H5_NUCLEO_H533RE_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32H5_NUCLEO_H533RE_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The Nucleo-H533RE supports using a HSE crystal (X3), but it is not
 * populated by default, so this configuration runs off the 32 MHz HSI
 * clock.
 *
 *    System Clock Source : PLL1
 *    SYSCLK Freq (MHz)   : 250
 *    HCLK Freq   (MHz)   : 250
 *    PLL1 Freq   (MHz)   : 250
 *    Flash Latency (WS)  : 5
 *
 * NOTE : The STM32H5 requires PLL1P to be configured, as this is used as the
 * system clock source. A custom clock config function must be supplied to
 * use a different system clock source.
 */

#define STM32_SYSCLK_FREQUENCY   250000000ul
#define STM32_LSI_FREQUENCY      32000
#define STM32_LSE_FREQUENCY      32768

#define STM32_BOARD_USEHSI       1
#define STM32_BOARD_HSIDIV       RCC_CR_HSIDIV(1)
#define STM32_HSI_FREQUENCY      32000000ul

/* PLL1 config: Used to generate system clock
 *  PLL1DIVR expects N, P, Q, and R should be defined.
 *  With HSI Freq = 32 MHz, this gives 250 MHz pll1_y_ck output
 */

#define STM32_PLLCFG_PLL1CFG      (RCC_PLL1CFGR_PLL1SRC_HSI  | \
                                   RCC_PLL1CFGR_PLL1RGE_4_8M | \
                                   RCC_PLL1CFGR_PLL1M(8) | \
                                   RCC_PLL1CFGR_PLL1PEN | \
                                   RCC_PLL1CFGR_PLL1QEN | \
                                   RCC_PLL1CFGR_PLL1REN)
#define STM32_PLLCFG_PLL1N         RCC_PLL1DIVR_PLL1N(125)
#define STM32_PLLCFG_PLL1P         RCC_PLL1DIVR_PLL1P(2)
#define STM32_PLLCFG_PLL1Q         RCC_PLL1DIVR_PLL1Q(4)
#define STM32_PLLCFG_PLL1R         RCC_PLL1DIVR_PLL1R(2)
#define STM32_PLLCFG_PLL1DIVR     (STM32_PLLCFG_PLL1N | \
                                   STM32_PLLCFG_PLL1P | \
                                   STM32_PLLCFG_PLL1Q | \
                                   STM32_PLLCFG_PLL1R)

#define STM32_VCO1_FRQ            ((STM32_HSI_FREQUENCY / 8) * 125)
#define STM32_PLL1P_FREQUENCY     (STM32_VCO1_FRQ / 2)
#define STM32_PLL1Q_FREQUENCY     (STM32_VCO1_FRQ / 4)
#define STM32_PLL1R_FREQUENCY     (STM32_VCO1_FRQ / 2)

/* PLL2 config: Needed to use 2 ADC at max speed. */

#define STM32_PLLCFG_PLL2CFG      (RCC_PLL2CFGR_PLL2SRC_HSI | \
                                   RCC_PLL2CFGR_PLL2RGE_4_8M | \
                                   RCC_PLL2CFGR_PLL2M(8) | \
                                   RCC_PLL2CFGR_PLL2REN)
#define STM32_PLLCFG_PLL2N         RCC_PLL2DIVR_PLL2N(75)
#define STM32_PLLCFG_PLL2R         RCC_PLL2DIVR_PLL2R(4)
#define STM32_PLLCFG_PLL2DIVR     (STM32_PLLCFG_PLL2N | \
                                   STM32_PLLCFG_PLL2R)

#define STM32_VCO2_FRQ            ((STM32_HSI_FREQUENCY / 8) * 75)
#define STM32_PLL2R_FREQUENCY     (STM32_VCO2_FRQ / 4)

/* Enable CLK48; get it from HSI48 */

#if defined(CONFIG_STM32_USBFS) || defined(CONFIG_STM32_RNG)
#  define STM32_USE_CLK48       1
#endif

#if defined(CONFIG_STM32_USBFS)
#  define STM32_CLKUSB_SEL      RCC_CCIPR4_USBSEL_HSI48KERCK
#  define STM32_HSI48_SYNCSRC   SYNCSRC_USB
#else
#  define STM32_HSI48_SYNCSRC   SYNCSRC_NONE
#endif

#if defined(CONFIG_STM32_RNG)
#  define STM32_CLKRNG_SEL      RCC_CCIPR5_RNGSEL_HSI48KERCK
#endif

/* Enable LSE (for the RTC) */

#define STM32_USE_LSE           1

/* Configure the HCLK divisor (for the AHB bus, core, memory, and DMA */

#define STM32_RCC_CFGR2_HPRE    RCC_CFGR2_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* Configure the APB1 prescaler */

#define STM32_RCC_CFGR2_PPRE1     RCC_CFGR2_PPRE1_HCLK1d2      /* PCLK1 = HCLK / 2 */
#define STM32_PCLK1_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)

#define STM32_TIM2_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_TIM3_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_TIM4_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_TIM5_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_TIM6_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_TIM7_CLKIN    (STM32_PCLK1_FREQUENCY)
#define STM32_TIM12_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_LPTIM2_CLKIN  (STM32_PCLK1_FREQUENCY)

/* Configure the APB2 prescaler */

#define STM32_RCC_CFGR2_PPRE2    RCC_CFGR2_PPRE2_HCLK1       /* PCLK2 = HCLK / 1 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY / 1)

#define STM32_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_TIM8_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_TIM15_CLKIN  (STM32_PCLK2_FREQUENCY)

/* Configure the APB3 prescaler */

#define STM32_RCC_CFGR2_PPRE3     RCC_CFGR2_PPRE3_HCLK1      /* PCLK3 = HCLK / 1 */
#define STM32_PCLK3_FREQUENCY    (STM32_HCLK_FREQUENCY / 1)

#define STM32_LPTIM1_CLKIN  (STM32_PCLK3_FREQUENCY)

/* LED definitions **********************************************************/

/* The Nucleo-H533RE has one user controllable LED, LD2 a Green LED
 * connected to PA5.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LED
 * in any way.
 * The following definition is used to access the LED.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_NLEDS       1

#define BOARD_LED_GREEN   BOARD_LED1

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)

/* If CONFIG_ARCH_LEDS is defined, the usage by the board port is defined in
 * include/board.h and src/stm32_autoleds.c. The LED is used to encode OS-
 * related events as follows:
 *
 *   SYMBOL                Meaning                   LED state
 *                                                   Green
 *   -------------------  -----------------------  -----------
 */

#define LED_STARTED        0 /* NuttX has been started   OFF      */
#define LED_HEAPALLOCATE   0 /* Heap has been allocated  OFF      */
#define LED_IRQSENABLED    0 /* Interrupts enabled       OFF      */
#define LED_STACKCREATED   1 /* Idle stack created       ON       */
#define LED_INIRQ          2 /* In an interrupt          N/C      */
#define LED_SIGNAL         2 /* In a signal handler      N/C      */
#define LED_ASSERTION      2 /* An assertion failed      N/C      */
#define LED_PANIC          3 /* The system has crashed   Blinking */

/* Thus if the Green LED is statically on, NuttX has successfully booted and
 * is, apparently, running normally.  If the Green LED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 */

/* Button definitions *******************************************************/

/* The Nucleo-H533RE supports one button:  Pushbutton B1, labeled "User", is
 * connected to GPIO PC13.
 * A high value will be sensed when the button is pressed.
 */

#define BUTTON_USER        0
#define NUM_BUTTONS        1
#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/* USART2 GPIOs *************************************************************/

/* USART2 (Nucleo Virtual Console) */

#define GPIO_USART2_RX   GPIO_USART2_RX_1    /* PA3 */
#define GPIO_USART2_TX   GPIO_USART2_TX_1    /* PA2 */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_board_initialize
 *
 * Description:
 *   All STM32H5 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices
 *   have been initialized.
 *
 ****************************************************************************/

void stm32_board_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __BOARDS_ARM_STM32H5_NUCLEO_H533RE_INCLUDE_BOARD_H */
