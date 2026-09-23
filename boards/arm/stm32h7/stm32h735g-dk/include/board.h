/****************************************************************************
 * boards/arm/stm32h7/stm32h735g-dk/include/board.h
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

#ifndef __BOARDS_ARM_STM32H7_STM32H735G_DK_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32H7_STM32H735G_DK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/* Do not include STM32 H7 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The board has a 25 MHz HSE oscillator and a 32.768 kHz LSE crystal.
 * PLL1 supplies a 400 MHz CPU clock, 200 MHz AHB and 100 MHz APB clocks.
 */

/* rcc_reset() leaves the 64 MHz HSI oscillator divided by four. */

#define STM32_HSI_FREQUENCY      16000000ul
#define STM32_LSI_FREQUENCY      32000
#define STM32_HSE_FREQUENCY      25000000ul
#define STM32_LSE_FREQUENCY      32768

#define STM32_BOARD_USEHSE
#define STM32_HSEBYP_ENABLE

#define STM32_PLLCFG_PLLSRC      RCC_PLLCKSELR_PLLSRC_HSE

/* PLL1, wide 4 - 8 MHz input, enable DIVP, DIVQ, DIVR
 *
 *   PLL1_VCO = (25,000,000 / 5) * 160 = 800 MHz
 *
 *   PLL1P = PLL1_VCO/2  = 800 MHz / 2   = 400 MHz
 *   PLL1Q = PLL1_VCO/4  = 800 MHz / 4   = 200 MHz
 *   PLL1R = PLL1_VCO/8  = 800 MHz / 8   = 100 MHz
 */

#define STM32_PLLCFG_PLL1CFG     (RCC_PLLCFGR_PLL1VCOSEL_WIDE | \
                                  RCC_PLLCFGR_PLL1RGE_4_8_MHZ | \
                                  RCC_PLLCFGR_DIVP1EN | \
                                  RCC_PLLCFGR_DIVQ1EN | \
                                  RCC_PLLCFGR_DIVR1EN)
#define STM32_PLLCFG_PLL1M       RCC_PLLCKSELR_DIVM1(5)
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(160)
#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR_P1(2)
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(4)
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR_R1(8)

#define STM32_VCO1_FREQUENCY     ((STM32_HSE_FREQUENCY / 5) * 160)
#define STM32_PLL1P_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)
#define STM32_PLL1Q_FREQUENCY    (STM32_VCO1_FREQUENCY / 4)

/* PLL2 is unused. */

#define STM32_PLLCFG_PLL2CFG     0
#define STM32_PLLCFG_PLL2M       0
#define STM32_PLLCFG_PLL2N       0
#define STM32_PLLCFG_PLL2P       0
#define STM32_PLLCFG_PLL2Q       0
#define STM32_PLLCFG_PLL2R       0
#ifdef CONFIG_STM32_LTDC
/* PLL3R supplies the LCD pixel clock: 800 MHz / 83 = 9.64 MHz. */

#  define STM32_PLLCFG_PLL3CFG   (RCC_PLLCFGR_PLL3VCOSEL_WIDE | \
                                 RCC_PLLCFGR_PLL3RGE_4_8_MHZ | \
                                 RCC_PLLCFGR_DIVR3EN)
#  define STM32_PLLCFG_PLL3M     RCC_PLLCKSELR_DIVM3(5)
#  define STM32_PLLCFG_PLL3N     RCC_PLL3DIVR_N3(160)
#  define STM32_PLLCFG_PLL3P     RCC_PLL3DIVR_P3(2)
#  define STM32_PLLCFG_PLL3Q     RCC_PLL3DIVR_Q3(2)
#  define STM32_PLLCFG_PLL3R     RCC_PLL3DIVR_R3(83)
#else
#  define STM32_PLLCFG_PLL3CFG   0
#  define STM32_PLLCFG_PLL3M     0
#  define STM32_PLLCFG_PLL3N     0
#  define STM32_PLLCFG_PLL3P     0
#  define STM32_PLLCFG_PLL3Q     0
#  define STM32_PLLCFG_PLL3R     0
#endif

/* SYSCLK = PLL1P = 400 MHz
 * CPUCLK = SYSCLK / 1 = 400 MHz
 */

#define STM32_RCC_D1CFGR_D1CPRE  (RCC_D1CFGR_D1CPRE_SYSCLK)
#define STM32_SYSCLK_FREQUENCY   (STM32_PLL1P_FREQUENCY)
#define STM32_CPUCLK_FREQUENCY   (STM32_SYSCLK_FREQUENCY / 1)

/* Configure Clock Assignments */

/* AHB clock (HCLK) is SYSCLK/2 (200 MHz max)
 * HCLK1 = HCLK2 = HCLK3 = HCLK4
 */

#define STM32_RCC_D1CFGR_HPRE   RCC_D1CFGR_HPRE_SYSCLKd2        /* HCLK  = SYSCLK / 2 */
#define STM32_HCLK_FREQUENCY    (STM32_SYSCLK_FREQUENCY / 2)    /* HCLK in D2, HCLK4 in D3 */

/* APB1 clock (PCLK1) is HCLK/2 (100 MHz) */

#define STM32_RCC_D2CFGR_D2PPRE1  RCC_D2CFGR_D2PPRE1_HCLKd2       /* PCLK1 = HCLK / 2 */
#define STM32_PCLK1_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB2 clock (PCLK2) is HCLK/2 (100 MHz) */

#define STM32_RCC_D2CFGR_D2PPRE2  RCC_D2CFGR_D2PPRE2_HCLKd2       /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB3 clock (PCLK3) is HCLK/2 (100 MHz) */

#define STM32_RCC_D1CFGR_D1PPRE   RCC_D1CFGR_D1PPRE_HCLKd2        /* PCLK3 = HCLK / 2 */

/* APB4 clock (PCLK4) is HCLK/2 (100 MHz) */

#define STM32_RCC_D3CFGR_D3PPRE   RCC_D3CFGR_D3PPRE_HCLKd2       /* PCLK4 = HCLK / 2 */
#define STM32_PCLK4_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* Timer clock frequencies */

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_TIM15_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_TIM16_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_TIM17_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Four flash wait states at 200 MHz ACLK. */

#define BOARD_FLASH_WAITSTATES  4

/* ST-LINK virtual COM port: USART3 TX on PD8, RX on PD9. */

#define GPIO_USART3_RX         (GPIO_USART3_RX_3 | GPIO_SPEED_100MHz)
#define GPIO_USART3_TX         (GPIO_USART3_TX_3 | GPIO_SPEED_100MHz)

/* I2C4: touchscreen on PF14/PF15. */

#define STM32_RCC_D3CCIPR_I2C4SRC RCC_D3CCIPR_I2C4SEL_HSI
#define GPIO_I2C4_SCL            (GPIO_I2C4_SCL_2 | GPIO_SPEED_50MHz)
#define GPIO_I2C4_SDA            (GPIO_I2C4_SDA_2 | GPIO_SPEED_50MHz)

/* RK043FN48H 480x272 RGB panel. */

#define GPIO_LTDC_B0        (GPIO_LTDC_B0_2 | GPIO_SPEED_100MHz)     /* PG14 */
#define GPIO_LTDC_B1        (GPIO_LTDC_B1_4 | GPIO_SPEED_100MHz)     /* PD0 */
#define GPIO_LTDC_B2        (GPIO_LTDC_B2_2 | GPIO_SPEED_100MHz)     /* PD6 */
#define GPIO_LTDC_B3        (GPIO_LTDC_B3_1 | GPIO_SPEED_100MHz)     /* PA8 */
#define GPIO_LTDC_B4        (GPIO_LTDC_B4_2 | GPIO_SPEED_100MHz)     /* PE12 */
#define GPIO_LTDC_B5        (GPIO_LTDC_B5_1 | GPIO_SPEED_100MHz)     /* PA3 */
#define GPIO_LTDC_B6        (GPIO_LTDC_B6_1 | GPIO_SPEED_100MHz)     /* PB8 */
#define GPIO_LTDC_B7        (GPIO_LTDC_B7_1 | GPIO_SPEED_100MHz)     /* PB9 */
#define GPIO_LTDC_CLK       (GPIO_LTDC_CLK_2 | GPIO_SPEED_100MHz)    /* PG7 */
#define GPIO_LTDC_G0        (GPIO_LTDC_G0_1 | GPIO_SPEED_100MHz)     /* PB1 */
#define GPIO_LTDC_G1        (GPIO_LTDC_G1_1 | GPIO_SPEED_100MHz)     /* PB0 */
#define GPIO_LTDC_G2        (GPIO_LTDC_G2_1 | GPIO_SPEED_100MHz)     /* PA6 */
#define GPIO_LTDC_G3        (GPIO_LTDC_G3_2 | GPIO_SPEED_100MHz)     /* PE11 */
#define GPIO_LTDC_G4        (GPIO_LTDC_G4_2 | GPIO_SPEED_100MHz)     /* PH15 */
#define GPIO_LTDC_G5        (GPIO_LTDC_G5_4 | GPIO_SPEED_100MHz)     /* PH4 */
#define GPIO_LTDC_G6        (GPIO_LTDC_G6_1 | GPIO_SPEED_100MHz)     /* PC7 */
#define GPIO_LTDC_G7        (GPIO_LTDC_G7_1 | GPIO_SPEED_100MHz)     /* PD3 */
#define GPIO_LTDC_HSYNC     (GPIO_LTDC_HSYNC_1 | GPIO_SPEED_100MHz)  /* PC6 */
#define GPIO_LTDC_R0        (GPIO_LTDC_R0_4 | GPIO_SPEED_100MHz)     /* PE0 */
#define GPIO_LTDC_R1        (GPIO_LTDC_R1_2 | GPIO_SPEED_100MHz)     /* PH3 */
#define GPIO_LTDC_R2        (GPIO_LTDC_R2_3 | GPIO_SPEED_100MHz)     /* PH8 */
#define GPIO_LTDC_R3        (GPIO_LTDC_R3_1 | GPIO_SPEED_100MHz)     /* PH9 */
#define GPIO_LTDC_R4        (GPIO_LTDC_R4_3 | GPIO_SPEED_100MHz)     /* PH10 */
#define GPIO_LTDC_R5        (GPIO_LTDC_R5_4 | GPIO_SPEED_100MHz)     /* PH11 */
#define GPIO_LTDC_R6        (GPIO_LTDC_R6_5 | GPIO_SPEED_100MHz)     /* PE1 */
#define GPIO_LTDC_R7        (GPIO_LTDC_R7_1 | GPIO_SPEED_100MHz)     /* PE15 */
#define GPIO_LTDC_VSYNC     (GPIO_LTDC_VSYNC_1 | GPIO_SPEED_100MHz)  /* PA4 */

/* The panel uses HSYNC/VSYNC mode.  Hold DE low as in the ST BSP. */

#define GPIO_LTDC_DE       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                            GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN13)

#define BOARD_LTDC_WIDTH        480
#define BOARD_LTDC_HEIGHT       272
#define BOARD_LTDC_OUTPUT_BPP   24
#define BOARD_LTDC_HFP          32
#define BOARD_LTDC_HBP          13
#define BOARD_LTDC_VFP          2
#define BOARD_LTDC_VBP          2
#define BOARD_LTDC_HSYNC        41
#define BOARD_LTDC_VSYNC        10
#define BOARD_LTDC_GCR_PCPOL    0
#define BOARD_LTDC_GCR_DEPOL    0
#define BOARD_LTDC_GCR_VSPOL    0
#define BOARD_LTDC_GCR_HSPOL    0

/* User LEDs: LD1 green (PC3), LD2 red (PC2), both active low. */

#define BOARD_LED_GREEN        0
#define BOARD_LED_RED          1
#define BOARD_NLEDS            2
#define BOARD_LED_GREEN_BIT    (1 << BOARD_LED_GREEN)
#define BOARD_LED_RED_BIT      (1 << BOARD_LED_RED)

/* Green indicates successful startup; red blinks on a fatal error. */

#define LED_STARTED            0
#define LED_HEAPALLOCATE       0
#define LED_IRQSENABLED         0
#define LED_STACKCREATED       1
#define LED_INIRQ              2
#define LED_SIGNAL             2
#define LED_ASSERTION          2
#define LED_PANIC              3
#define LED_IDLE               2

/* User button B2 on PC13. */

#define BUTTON_USER            0
#define BUTTON_USER_BIT        (1 << BUTTON_USER)

#endif /* __BOARDS_ARM_STM32H7_STM32H735G_DK_INCLUDE_BOARD_H */
