/****************************************************************************
 * boards/arm/stm32u5/b-u585i-iot02a/include/board.h
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

#ifndef __BOARDS_ARM_STM32U5_B_U585I_IOT02A_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32U5_B_U585I_IOT02A_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stm32_gpio.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The B-U585I-IOT02A board supports both HSE and LSE crystals (X1 and X2).
 * However, as shipped, the X1 crystal is not populated.  Therefore the board
 * will need to run off the 32kHz-sync'ed MSIS.
 *
 *   System Clock source : PLL (MSIS)
 *   SYSCLK(Hz)          : 160000000   Determined by PLL configuration
 *   HCLK(Hz)            : 160000000
 *   AHB Prescaler       : 1            (STM32_RCC_CFGR2_HPRE)  (160MHz)
 *   APB1 Prescaler      : 1            (STM32_RCC_CFGR2_PPRE1) (160MHz)
 *   APB2 Prescaler      : 1            (STM32_RCC_CFGR2_PPRE2) (160MHz)
 *   APB3 Prescaler      : 1            (STM32_RCC_CFGR3_PPRE3) (160MHz)
 *   MSIS Frequency(Hz)  : 4000000      (nominal)
 *   MSIK Frequency(Hz)  : 4000000      (nominal)
 *   PLL_MBOOST          : 1            (Embedded power distribution booster)
 *   PLLM                : 1            (STM32_PLLCFG_PLLM)
 *   PLLN                : 80           (STM32_PLLCFG_PLLN)
 *   PLLP                : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                : 2            (STM32_PLLCFG_PLLQ)
 *   PLLR                : 2            (STM32_PLLCFG_PLLR)
 *   Flash Latency(WS)   : 4
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * MSI - 4 MHz, autotrimmed via LSE
 * HSE - not installed
 * LSE - 32.768 kHz installed
 */

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_LSE_FREQUENCY     32768

#define STM32_BOARD_USEMSIS     1
#define STM32_BOARD_MSISRANGE   RCC_ICSCR1_MSISRANGE_4MHZ
#define STM32_BOARD_MSIKRANGE   RCC_ICSCR1_MSIKRANGE_4MHZ

/* PLL1 config; we use this to generate our system clock */

#define STM32_RCC_PLL1CFGR_PLL1M          RCC_PLL1CFGR_PLL1M(1)
#define STM32_RCC_PLL1DIVR_PLL1N          RCC_PLL1DIVR_PLL1N(80)
#define STM32_RCC_PLL1DIVR_PLL1P          0
#undef  STM32_RCC_PLL1CFGR_PLL1P_ENABLED
#define STM32_RCC_PLL1DIVR_PLL1Q          0
#undef  STM32_RCC_PLL1CFGR_PLL1Q_ENABLED
#define STM32_RCC_PLL1DIVR_PLL1R          RCC_PLL1DIVR_PLL1R(2)
#define STM32_RCC_PLL1CFGR_PLL1R_ENABLED

#define STM32_SYSCLK_FREQUENCY  160000000ul

/* Enable LSE (for the RTC and for MSIS autotrimming) */

#define STM32_USE_LSE           1

/* Configure the HCLK divisor (for the AHB bus, core, memory, and DMA */

#define STM32_RCC_CFGR2_HPRE    RCC_CFGR2_HPRE_SYSCLK     /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* Configure the APB1 prescaler */

#define STM32_RCC_CFGR2_PPRE1   RCC_CFGR2_PPRE1_HCLK      /* PCLK1 = HCLK / 1 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY / 1)

#define STM32_APB1_TIM2_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (STM32_PCLK1_FREQUENCY)

/* Configure the APB2 prescaler */

#define STM32_RCC_CFGR2_PPRE2   RCC_CFGR2_PPRE2_HCLK       /* PCLK2 = HCLK / 1 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY / 1)

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM15_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM16_CLKIN  (STM32_PCLK2_FREQUENCY)

/* Configure the APB3 prescaler */

#define STM32_RCC_CFGR3_PPRE3   RCC_CFGR3_PPRE3_HCLK       /* PCLK3 = HCLK / 1 */
#define STM32_PCLK3_FREQUENCY   (STM32_HCLK_FREQUENCY / 1)

/* The timer clock frequencies are automatically defined by hardware.  If the
 * APB prescaler equals 1, the timer clock frequencies are set to the same
 * frequency as that of the APB domain. Otherwise they are set to twice.
 * Note: TIM1,15,16 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM3_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM4_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM5_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM6_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM7_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM15_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM16_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_LPTIM1_FREQUENCY  STM32_HCLK_FREQUENCY
#define BOARD_LPTIM2_FREQUENCY  STM32_HCLK_FREQUENCY

/* DMA Channel/Stream Selections ********************************************/

/* Alternate function pin selections ****************************************/

/* USART1: Connected to STLink VCP and to CN9. */

#define GPIO_USART1_RX   GPIO_USART1_RX_1    /* PA10 */
#define GPIO_USART1_TX   GPIO_USART1_TX_1    /* PA9  */

/* SPI1: Arduino Connector CN13 */

#define GPIO_SPI1_NSS   (GPIO_OUTPUT|GPIO_SPEED_2MHZ| \
                         GPIO_PUSHPULL|GPIO_OUTPUT_SET| \
                         GPIO_PORTE|GPIO_PIN12)             /* PE12 */
#define GPIO_SPI1_SCK   (GPIO_SPI1_SCK_4|GPIO_SPEED_25MHZ)  /* PE13 */
#define GPIO_SPI1_MISO  (GPIO_SPI1_MISO_4)                  /* PE14 */
#define GPIO_SPI1_MOSI  (GPIO_SPI1_MOSI_4|GPIO_SPEED_25MHZ) /* PE15 */

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
 *   All STM32 architectures must provide the following entry point.
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
#endif  /* __BOARDS_ARM_STM32U5_B_U585I_IOT02A_INCLUDE_BOARD_H */
