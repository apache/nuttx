/****************************************************************************
 * boards/arm/stm32n6/stm32n6570-dk/include/board.h
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

#ifndef __BOARDS_ARM_STM32N6_STM32N6570_DK_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32N6_STM32N6570_DK_INCLUDE_BOARD_H

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

/* Two clock trees are provided.  The default runs the CPU at 800 MHz from
 * the 48 MHz HSE crystal with VDDCORE overdrive.
 * CONFIG_STM32N6570_DK_CLOCK_HSI selects a conservative 200 MHz tree from
 * the internal HSI that needs neither the HSE nor overdrive.  See
 * arch/arm/src/stm32n6/stm32n6xx_rcc.c for the macros.
 */

#define STM32_HSI_FREQUENCY     64000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_LSE_FREQUENCY     32768

#ifndef CONFIG_STM32N6570_DK_CLOCK_HSI

/* HSE 48 MHz (X3)
 *   PLL1 = HSE / 3 * 150 = 2400 MHz   (CPU and system buses)
 *     IC1  /3 = 800 MHz  -> CPU clock (CPUSW)
 *     IC2  /6 = 400 MHz  \
 *     IC6  /8 = 300 MHz   > SYSCLK components (SYSSW IC2_IC6_IC11)
 *     IC11 /6 = 400 MHz  /
 *   PLL2 = HSI / 2 * 48  = 1536 MHz   (audio and SDMMC kernel clocks)
 *   PLL3 = HSE / 3 * 125 = 2000 MHz   (LTDC)
 *   PLL4 = HSI / 4 * 75  = 1200 MHz   (I2C, LTDC)
 *   HPRE /2   = 200 MHz -> HCLK
 *   PPREx /1  = 200 MHz -> PCLK1/2/4/5
 *   TIMPRE /2 = 200 MHz -> timer kernel clock
 *
 * 800 MHz needs VOS high and the board SMPS in overdrive: PF4 high selects
 * the higher VDDCORE voltage before the voltage range is switched.
 */

#define STM32_BOARD_USEHSE
#define STM32_HSE_FREQUENCY     48000000ul
#define STM32_BOARD_VOS_HIGH
#define BOARD_VDDCORE_OVERDRIVE_GPIO (GPIO_OUTPUT | GPIO_PUSHPULL | \
                                      GPIO_SPEED_2MHZ | GPIO_OUTPUT_SET | \
                                      GPIO_PORTF | GPIO_PIN4)

#define STM32_PLL1_SRC          RCC_PLL1CFGR1_SEL_HSE
#define STM32_PLL1_M            3
#define STM32_PLL1_N            150
#define STM32_PLL1_FREQUENCY    2400000000ul

#define STM32_PLL2_SRC          RCC_PLL1CFGR1_SEL_HSI
#define STM32_PLL2_M            2
#define STM32_PLL2_N            48
#define STM32_PLL2_FREQUENCY    1536000000ul

#define STM32_PLL3_SRC          RCC_PLL1CFGR1_SEL_HSE
#define STM32_PLL3_M            3
#define STM32_PLL3_N            125
#define STM32_PLL3_FREQUENCY    2000000000ul

#define STM32_PLL4_SRC          RCC_PLL1CFGR1_SEL_HSI
#define STM32_PLL4_M            4
#define STM32_PLL4_N            75
#define STM32_PLL4_FREQUENCY    1200000000ul

#define STM32_IC1_SRC           RCC_ICCFGR_SEL_PLL1
#define STM32_IC1_DIV           3
#define STM32_IC2_SRC           RCC_ICCFGR_SEL_PLL1
#define STM32_IC2_DIV           6
#define STM32_IC6_SRC           RCC_ICCFGR_SEL_PLL1
#define STM32_IC6_DIV           8
#define STM32_IC11_SRC          RCC_ICCFGR_SEL_PLL1
#define STM32_IC11_DIV          6

#define STM32_CPUCLK_FREQUENCY  (STM32_PLL1_FREQUENCY / STM32_IC1_DIV)
#define STM32_SYSCLK_FREQUENCY  (STM32_PLL1_FREQUENCY / STM32_IC2_DIV)
#define STM32_RCC_CFGR2_HPRE    RCC_CFGR2_HPRE_SYSCLKd2
#define STM32_RCC_CFGR2_TIMPRE  RCC_CFGR2_TIMPRE_SYSCLKd2
#define STM32_HCLK_FREQUENCY    (STM32_SYSCLK_FREQUENCY / 2)
#define STM32_TIMCLK_FREQUENCY  (STM32_SYSCLK_FREQUENCY / 2)

#else /* CONFIG_STM32N6570_DK_CLOCK_HSI */

/* HSI 64 MHz
 *   PLL1 = HSI / 4 * 50 = 800 MHz
 *     IC1  /4  = 200 MHz  -> CPU clock (CPUSW)
 *     IC2  /8  = 100 MHz  \
 *     IC6  /12 = 66.7 MHz  > SYSCLK components (SYSSW IC2_IC6_IC11)
 *     IC11 /8  = 100 MHz  /
 *   HPRE /2   = 50 MHz   -> HCLK
 *   PPREx /1  = 50 MHz   -> PCLK1/2/4/5
 *   TIMPRE /1 = 100 MHz  -> timer kernel clock
 *
 * Works with VOS low, no VDDCORE overdrive required.
 */

#define STM32_PLL1_SRC          RCC_PLL1CFGR1_SEL_HSI
#define STM32_PLL1_M            4
#define STM32_PLL1_N            50
#define STM32_PLL1_FREQUENCY    800000000ul

#define STM32_IC1_SRC           RCC_ICCFGR_SEL_PLL1
#define STM32_IC1_DIV           4
#define STM32_IC2_SRC           RCC_ICCFGR_SEL_PLL1
#define STM32_IC2_DIV           8
#define STM32_IC6_SRC           RCC_ICCFGR_SEL_PLL1
#define STM32_IC6_DIV           12
#define STM32_IC11_SRC          RCC_ICCFGR_SEL_PLL1
#define STM32_IC11_DIV          8

#define STM32_CPUCLK_FREQUENCY  (STM32_PLL1_FREQUENCY / STM32_IC1_DIV)
#define STM32_SYSCLK_FREQUENCY  (STM32_PLL1_FREQUENCY / STM32_IC2_DIV)
#define STM32_RCC_CFGR2_HPRE    RCC_CFGR2_HPRE_SYSCLKd2
#define STM32_RCC_CFGR2_TIMPRE  RCC_CFGR2_TIMPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    (STM32_SYSCLK_FREQUENCY / 2)
#define STM32_TIMCLK_FREQUENCY  STM32_SYSCLK_FREQUENCY

#endif /* CONFIG_STM32N6570_DK_CLOCK_HSI */

#define STM32_PCLK1_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_PCLK4_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_PCLK5_FREQUENCY   STM32_HCLK_FREQUENCY

/* Timer kernel clock (TIMG = SYSCLK / TIMPRE) */

#define STM32_APB1_TIM_FREQUENCY STM32_TIMCLK_FREQUENCY
#define STM32_APB2_TIM_FREQUENCY STM32_TIMCLK_FREQUENCY

#define STM32_TIM1_CLKIN         STM32_APB2_TIM_FREQUENCY
#define STM32_TIM8_CLKIN         STM32_APB2_TIM_FREQUENCY
#define STM32_TIM9_CLKIN         STM32_APB2_TIM_FREQUENCY
#define STM32_TIM15_CLKIN        STM32_APB2_TIM_FREQUENCY
#define STM32_TIM16_CLKIN        STM32_APB2_TIM_FREQUENCY
#define STM32_TIM17_CLKIN        STM32_APB2_TIM_FREQUENCY
#define STM32_TIM2_CLKIN         STM32_APB1_TIM_FREQUENCY
#define STM32_TIM3_CLKIN         STM32_APB1_TIM_FREQUENCY
#define STM32_TIM4_CLKIN         STM32_APB1_TIM_FREQUENCY
#define STM32_TIM5_CLKIN         STM32_APB1_TIM_FREQUENCY
#define STM32_TIM6_CLKIN         STM32_APB1_TIM_FREQUENCY
#define STM32_TIM7_CLKIN         STM32_APB1_TIM_FREQUENCY
#define STM32_TIM10_CLKIN        STM32_APB1_TIM_FREQUENCY
#define STM32_TIM11_CLKIN        STM32_APB1_TIM_FREQUENCY
#define STM32_TIM12_CLKIN        STM32_APB1_TIM_FREQUENCY
#define STM32_TIM13_CLKIN        STM32_APB1_TIM_FREQUENCY
#define STM32_TIM14_CLKIN        STM32_APB1_TIM_FREQUENCY

/* USART1 kernel clock: HSI, independent of the system clock tree */

#define STM32_RCC_CCIPR13_USART1SEL RCC_CCIPR13_USART1SEL_HSI
#define STM32_USART1_FREQUENCY      STM32_HSI_FREQUENCY

/* I/O voltage domains ******************************************************/

/* VDDIO2 and VDDIO3 are supplied at 1.8 V on the Discovery kit.
 * Mark both supplies valid and select their 1.8 V range before the
 * common STM32N6 startup configures I/O compensation.
 */

#define BOARD_PWR_VDDIO  (PWR_SVMCR3_VDDIO2SV    | PWR_SVMCR3_VDDIO3SV | \
                          PWR_SVMCR3_VDDIO2VRSEL | PWR_SVMCR3_VDDIO3VRSEL)

/* LED definitions **********************************************************/

/* LD1 (green, PO1) is active high; LD2 (red, PG10) is active low. */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_NLEDS       2

#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_RED     BOARD_LED2

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

/* OS event indications when CONFIG_ARCH_LEDS is enabled:
 *
 *   SYMBOL                 Meaning                  Green  Red
 *   ---------------------  -----------------------  -----  -----
 */

#define LED_STARTED        0 /* NuttX has started     OFF    OFF   */
#define LED_HEAPALLOCATE   0 /* Heap allocated        OFF    OFF   */
#define LED_IRQSENABLED    0 /* Interrupts enabled    OFF    OFF   */
#define LED_STACKCREATED   1 /* Idle stack created    ON     OFF   */
#define LED_INIRQ          2 /* In an interrupt       N/C    N/C   */
#define LED_SIGNAL         2 /* In a signal handler   N/C    N/C   */
#define LED_ASSERTION      2 /* An assertion failed   N/C    N/C   */
#define LED_PANIC          3 /* The system crashed    OFF    Blink */

/* Button definitions *******************************************************/

/* The STM32N6570-DK has two user buttons: USER1 (B2) on PC13 and TAMP (B4)
 * on PE0.  Both read high when pressed.  The RESET button (B3) drives NRST.
 */

#define BUTTON_USER        0
#define BUTTON_TAMP        1
#define NUM_BUTTONS        2
#define BUTTON_USER_BIT    (1 << BUTTON_USER)
#define BUTTON_TAMP_BIT    (1 << BUTTON_TAMP)

/* Alternate function pin selections ****************************************/

/* USART1 GPIOs *************************************************************/

/* USART1 (ST-LINK Virtual COM Port): PE5=TX (AF7), PE6=RX (AF7)
 * Connected to the on-board ST-Link to provide a Virtual COM Port.
 */

#define GPIO_USART1_TX   (GPIO_USART1_TX_1 | GPIO_SPEED_50MHZ)
#define GPIO_USART1_RX   (GPIO_USART1_RX_1 | GPIO_SPEED_50MHZ)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_board_initialize
 *
 * Description:
 *   All STM32N6 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices
 *   have been initialized.
 *
 ****************************************************************************/

void stm32_board_initialize(void);

#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32N6_STM32N6570_DK_INCLUDE_BOARD_H */
