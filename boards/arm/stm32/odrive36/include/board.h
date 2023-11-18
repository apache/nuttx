/****************************************************************************
 * boards/arm/stm32/odrive36/include/board.h
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

#ifndef __BOARDS_ARM_STM32_ODRIVE36_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_ODRIVE36_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif
#include "stm32_rcc.h"
#include "stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC (30-60KHz, uncalibrated)
 * HSE - On-board crystal frequency is 8MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (8,000,000 / 8) * 336
 *         = 336,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 336,000,000 / 2 = 168,000,000
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 48,000,000
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(336)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(7)

#define STM32_SYSCLK_FREQUENCY  168000000ul

/* AHB clock (HCLK) is SYSCLK (168MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK  /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/4 (42MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd4     /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/4)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (84MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM3_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM4_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM5_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM6_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM7_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM8_FREQUENCY    STM32_HCLK_FREQUENCY

/* DMA Channel/Stream Selections ********************************************/

/* ADC 1 */

#define ADC1_DMA_CHAN           DMAMAP_ADC1_1

/* Alternate function pin selections ****************************************/

/* ADC */

#define GPIO_ADC1_IN4      GPIO_ADC1_IN4_0
#define GPIO_ADC1_IN5      GPIO_ADC1_IN5_0
#define GPIO_ADC1_IN6      GPIO_ADC1_IN6_0
#define GPIO_ADC1_IN15     GPIO_ADC1_IN15_0

#define GPIO_ADC2_IN10     GPIO_ADC2_IN10_0
#define GPIO_ADC2_IN11     GPIO_ADC2_IN11_0
#define GPIO_ADC2_IN12     GPIO_ADC2_IN13_0

#define GPIO_ADC3_IN12     GPIO_ADC3_IN12_0
#define GPIO_ADC3_IN13     GPIO_ADC3_IN13_0

/* USART2:
 *  USART2_TX - PA2 - GPIO_3
 *  USART2_RX - PA3 - GPIO_4
 */

#define GPIO_USART2_RX     (GPIO_USART2_RX_1|GPIO_SPEED_100MHz)
#define GPIO_USART2_TX     (GPIO_USART2_TX_1|GPIO_SPEED_100MHz)

/* CAN:
 *   CAN_R - PB8
 *   CAN_T - PB9
 */

#define GPIO_CAN1_RX       (GPIO_CAN1_RX_2|GPIO_SPEED_50MHz)
#define GPIO_CAN1_TX       (GPIO_CAN1_TX_2|GPIO_SPEED_50MHz)

/* SPI3 - connected to DRV8301
 *   SPI3_SCK  - PC10
 *   SPI3_MISO - PC11
 *   SPI3_MOSI - PC12
 */

#define GPIO_SPI3_SCK      (GPIO_SPI3_SCK_2|GPIO_SPEED_50MHz)
#define GPIO_SPI3_MISO     (GPIO_SPI3_MISO_2|GPIO_SPEED_50MHz)
#define GPIO_SPI3_MOSI     (GPIO_SPI3_MOSI_2|GPIO_SPEED_50MHz)

/* USBDEV */

#define GPIO_OTGFS_DM      (GPIO_OTGFS_DM_0|GPIO_SPEED_100MHz)
#define GPIO_OTGFS_DP      (GPIO_OTGFS_DP_0|GPIO_SPEED_100MHz)
#define GPIO_OTGFS_ID      (GPIO_OTGFS_ID_0|GPIO_SPEED_100MHz)

/* Dual FOC configuration */

/* TIM1 configuration *******************************************************/

#define GPIO_TIM1_CH1OUT   (GPIO_TIM1_CH1OUT_1|GPIO_SPEED_100MHz) /* TIM1 CH1  - PA8  - U high */
#define GPIO_TIM1_CH1NOUT  (GPIO_TIM1_CH1N_2|GPIO_SPEED_100MHz)   /* TIM1 CH1N - PB13 - U low */
#define GPIO_TIM1_CH2OUT   (GPIO_TIM1_CH2OUT_1|GPIO_SPEED_100MHz) /* TIM1 CH2  - PA9  - V high */
#define GPIO_TIM1_CH2NOUT  (GPIO_TIM1_CH2N_2|GPIO_SPEED_100MHz)   /* TIM1 CH2N - PB14 - V low */
#define GPIO_TIM1_CH3OUT   (GPIO_TIM1_CH3OUT_1|GPIO_SPEED_100MHz) /* TIM1 CH3  - PA10 - W high */
#define GPIO_TIM1_CH3NOUT  (GPIO_TIM1_CH3N_2|GPIO_SPEED_100MHz)   /* TIM1 CH3N - PB15 - W low */
#define GPIO_TIM1_CH4OUT   0                                      /* not used as output */

/* TIM8 configuration *******************************************************/

#define GPIO_TIM8_CH1OUT   (GPIO_TIM8_CH1OUT_1|GPIO_SPEED_100MHz) /* TIM8 CH1  - PC6  - U high */
#define GPIO_TIM8_CH1NOUT  (GPIO_TIM8_CH1N_2|GPIO_SPEED_100MHz)   /* TIM8 CH1N - PA7 - U low */
#define GPIO_TIM8_CH2OUT   (GPIO_TIM8_CH2OUT_1|GPIO_SPEED_100MHz) /* TIM8 CH2  - PC7  - V high */
#define GPIO_TIM8_CH2NOUT  (GPIO_TIM8_CH2N_1|GPIO_SPEED_100MHz)   /* TIM8 CH2N - PB0 - V low */
#define GPIO_TIM8_CH3OUT   (GPIO_TIM8_CH3OUT_1|GPIO_SPEED_100MHz) /* TIM8 CH3  - PC8 - W high */
#define GPIO_TIM8_CH3NOUT  (GPIO_TIM8_CH3N_1|GPIO_SPEED_100MHz)   /* TIM8 CH3N - PB1 - W low */
#define GPIO_TIM8_CH4OUT   0                                      /* not used as output */

/* QEN3 configuration *******************************************************/

#define GPIO_TIM3_CH1IN    (GPIO_TIM3_CH1IN_2|GPIO_SPEED_50MHz) /* TIM3 CH1IN - PB4 */
#define GPIO_TIM3_CH2IN    (GPIO_TIM3_CH2IN_2|GPIO_SPEED_50MHz) /* TIM3 CH2IN - PB5 */

/* QEN4 configuration *******************************************************/

#define GPIO_TIM4_CH1IN    (GPIO_TIM4_CH1IN_1|GPIO_SPEED_50MHz) /* TIM4 CH1IN - PB6 */
#define GPIO_TIM4_CH2IN    (GPIO_TIM4_CH2IN_1|GPIO_SPEED_50MHz) /* TIM4 CH2IN - PB7 */

#endif /* __BOARDS_ARM_STM32_ODRIVE36_INCLUDE_BOARD_H */
