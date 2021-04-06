/****************************************************************************
 * boards/arm/stm32/olimex-stm32-h407/include/board.h
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

#ifndef __BOARDS_ARM_STM32_OLIMEX_STM32_H407_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_OLIMEX_STM32_H407_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#ifdef __KERNEL__
#  include "stm32_rcc.h"
#  include "stm32_sdio.h"
#  include "stm32.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The Olimex-STM32-H407 board features a 12MHz crystal and
 * a 32kHz RTC backup crystal.
 *
 * This is the canonical configuration:
 *   System Clock source       : PLL (HSE)
 *   SYSCLK(Hz)                : 168000000    Determined by PLL configuration
 *   HCLK(Hz)                  : 168000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler             : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler            : 4            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler            : 2            (STM32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)         : 8000000      (STM32_BOARD_XTAL)
 *   PLLM                      : 8            (STM32_PLLCFG_PLLM)
 *   PLLN                      : 336          (STM32_PLLCFG_PLLN)
 *   PLLP                      : 2            (STM32_PLLCFG_PLLP)
 *   PLLQ                      : 7            (STM32_PLLCFG_PLLQ)
 *   Main regulator output
 *              voltage        : Scale1 mode  Needed for high speed SYSCLK
 *   Flash Latency(WS)         : 5
 *   Prefetch Buffer           : OFF
 *   Instruction cache         : ON
 *   Data cache                : ON
 *   Require 48MHz for
 *   USB OTG FS,
 *   SDIO and RNG clock        : Enabled
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC (30-60KHz, uncalibrated)
 * HSE - On-board crystal frequency is 12MHz
 * LSE - 32.768 kHz
 * STM32F407ZGT6 - too 168Mhz
 */

#define STM32_BOARD_XTAL        12000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE
 * PLL_VCO = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 *         = (25,000,000 / 12) * 360
 *         = 240,000,000
 * SYSCLK  = PLL_VCO / PLLP
 *         = 240,000,000 / 2 = 120,000,000
 * USB OTG FS, SDIO and RNG Clock
 *         =  PLL_VCO / PLLQ
 *         = 240,000,000 / 5 = 48,000,000
 *         = 48,000,000
 *
 *  Xtal     /M  *n  /P         SysClk AHB HCLK  APB1     PCLK1
 * 12Mhz HSE /12 336 /2 PLLCLK 168Mhz  /1  168    /4     42Mhz
 * 12Mhz HSE /6  168 /2 PLLCLK 168Mhz  /1  168    /4     42Mhz
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(3)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(84)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(5)
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

/* APB2 clock (PCLK2) is HCLK/2  */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLKd2     /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2 * STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2 * STM32_PCLK2_FREQUENCY)

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

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(118+2)=400 KHz
 */

#define SDIO_INIT_CLKDIV        (118 << SDIO_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV    (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_MMCXFR_CLKDIV    (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV     (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV     (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* LED definitions **********************************************************/

/* If CONFIG_ARCH_LEDS is not defined, then the user can control the status
 * LED in any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_STATUS  0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_LED_STATUS_BIT  (1 << BOARD_LED_STATUS)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the status LED of
 * the Olimex STM32-H405.
 * The following definitions describe how NuttX controls the LEDs:
 */

#define LED_STARTED       0  /* LED_STATUS on */
#define LED_HEAPALLOCATE  1  /* no change */
#define LED_IRQSENABLED   2  /* no change */
#define LED_STACKCREATED  3  /* no change */
#define LED_INIRQ         4  /* no change */
#define LED_SIGNAL        5  /* no change */
#define LED_ASSERTION     6  /* LED_STATUS off */
#define LED_PANIC         7  /* LED_STATUS blinking */

/* Button definitions *******************************************************/

/* The Olimex STM32-H405 supports one buttons: */

#define BUTTON_BUT     0
#define NUM_BUTTONS    1

#define BUTTON_BUT_BIT (1 << BUTTON_BUT)

/* Alternate function pin selections ****************************************/

/* USART3: */

#if 0
#define GPIO_USART3_RX    GPIO_USART3_RX_1  /* PB11 */
#define GPIO_USART3_TX    GPIO_USART3_TX_1  /* PB10 */
#define GPIO_USART3_CTS   GPIO_USART3_CTS_1 /* PB13 */
#define GPIO_USART3_RTS   GPIO_USART3_RTS_1 /* PB14 */
#endif

/* USART2: */

#define GPIO_USART2_RX    GPIO_USART2_RX_1
#define GPIO_USART2_TX    GPIO_USART2_TX_1
#define GPIO_USART2_CTS   GPIO_USART2_CTS_1
#define GPIO_USART2_RTS   GPIO_USART2_RTS_1

/* USART6: (UEXT connector) */

#define GPIO_USART6_RX    GPIO_USART6_RX_1
#define GPIO_USART6_TX    GPIO_USART6_TX_1

/* GPIO_USART6_CTS and GPIO_USART6_RTS aren't used for UEXT */

/* CAN: */

#define GPIO_CAN1_RX      GPIO_CAN1_RX_2    /* PB8 */
#define GPIO_CAN1_TX      GPIO_CAN1_TX_2    /* PB9 */
#define GPIO_CAN2_RX      GPIO_CAN1_RX_2    /* PB5 */
#define GPIO_CAN2_TX      GPIO_CAN1_TX_2    /* PB6 */

/* DMA Channel/Stream Selections ********************************************/

/* Stream selections are arbitrary for now but might become important in the
 * future if we set aside more DMA channels/streams.
 *
 * SDIO DMA
 *   DMAMAP_SDIO_1 = Channel 4, Stream 3
 *   DMAMAP_SDIO_2 = Channel 4, Stream 6
 */

#define DMAMAP_SDIO DMAMAP_SDIO_1

#endif /* __BOARDS_ARM_STM32_OLIMEX_STM32_H407_INCLUDE_BOARD_H */
