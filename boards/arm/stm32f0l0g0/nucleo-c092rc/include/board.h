/****************************************************************************
 * boards/arm/stm32f0l0g0/nucleo-c092rc/include/board.h
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

#ifndef __BOARDS_ARM_STM32F0L0G0_NUCLEO_C092RC_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32F0L0G0_NUCLEO_C092RC_INCLUDE_BOARD_H

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

/* HSI - Internal 48 MHz RC Oscillator
 * LSI - 32 KHz RC
 * HSE - 8 MHz from MCO output of ST-LINK (disabled by default)
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul          /* 8MHz */

#define STM32_HSI_FREQUENCY     48000000ul         /* 48MHz */
#define STM32_LSI_FREQUENCY     32000              /* Between 30kHz and 60kHz */
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768              /* X2 on board */

/* Configure HSI48 clock division factor (48 MHz) */

#define STM32_RCC_HSIDIV        RCC_CR_HSIDIV_HSI

/* Use the HSI as SYSCLK source (48 MHz) */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_HSI
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_HSI
#define STM32_SYSCLK_FREQUENCY  (STM32_HSI_FREQUENCY)

/* AHB clock (HCLK) is SYSCLK (48 MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK) is HCLK (48 MHz) */

#define STM32_RCC_CFGR_PPRE     RCC_CFGR_PPRE_HCLK
#define STM32_PCLK1_FREQUENCY   STM32_HCLK_FREQUENCY

/* FDCAN1 clock is PCLK (48 MHz) */

#define STM32_FDCAN1_SEL       RCC_CCIPR1_FDCAN1SEL_PCLK
#define STM32_FDCAN_FREQUENCY  STM32_PCLK1_FREQUENCY

/* All timers on PCLK x1 (48 MHz) */

#define STM32_APB2_TIM1_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM2_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM3_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_APB2_TIM15_CLKIN  STM32_PCLK1_FREQUENCY
#define STM32_APB2_TIM16_CLKIN  STM32_PCLK1_FREQUENCY
#define STM32_APB2_TIM17_CLKIN  STM32_PCLK1_FREQUENCY

/* LED definitions **********************************************************/

/* LED index values for use with board_userled() */

#define BOARD_LED1       0 /* User LD1 */
#define BOARD_LED2       1 /* User LD2 */
#define BOARD_NLEDS      2

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT   (1 << BOARD_LED1)
#define BOARD_LED2_BIT   (1 << BOARD_LED2)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on the
 * board. The following definitions describe how NuttX controls
 * the LED:
 *
 *   SYMBOL              Meaning                  LED1 state
 *   ------------------  -----------------------  ----------
 *   LED_STARTED         NuttX has been started   OFF
 *   LED_HEAPALLOCATE    Heap has been allocated  OFF
 *   LED_IRQSENABLED     Interrupts enabled       OFF
 *   LED_STACKCREATED    Idle stack created       ON
 *   LED_INIRQ           In an interrupt          No change
 *   LED_SIGNAL          In a signal handler      No change
 *   LED_ASSERTION       An assertion failed      No change
 *   LED_PANIC           The system has crashed   Blinking
 *   LED_IDLE            STM32 is is sleep mode   Not used
 */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED 1
#define LED_INIRQ        2
#define LED_SIGNAL       2
#define LED_ASSERTION    2
#define LED_PANIC        1

/* Button definitions *******************************************************/

/* Nucleo C092RC board supports two buttons; only one button is controllable
 * by software:
 *
 *   B1 USER:  user button connected to STM32 I/O PC13.
 *   B2 RESET: push button connected to NRST; used to RESET the MCU.
 */

#define BUTTON_USER      0  /* User B1 */
#define NUM_BUTTONS      1

#define BUTTON_USER_BIT  (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/* USART */

/* USART1 at arduino D0/D1:
 *   USART1_RX - PB6
 *   USART1_TX - PB7
 */

#define GPIO_USART1_RX      (GPIO_USART2_RX_2|GPIO_SPEED_HIGH)    /* PB6 */
#define GPIO_USART1_TX      (GPIO_USART2_TX_2|GPIO_SPEED_HIGH)    /* PB7 */

/* USART1 RS485_DIR - PA8 (arduino D7)
 * (compatible with RS485 Waveshare shield)
 */

#define GPIO_USART1_RS485_DIR (GPIO_OUTPUT | GPIO_PUSHPULL |          \
                               GPIO_SPEED_HIGH | GPIO_OUTPUT_CLEAR | \
                               GPIO_PORTA | GPIO_PIN8)

/* By default the USART2 is connected to STLINK Virtual COM Port:
 *   USART2_RX - PA3
 *   USART2_TX - PA2
 */

#define GPIO_USART2_RX      (GPIO_USART2_RX_1|GPIO_SPEED_HIGH)    /* PA3 */
#define GPIO_USART2_TX      (GPIO_USART2_TX_1|GPIO_SPEED_HIGH)    /* PA2 */

/* FDCAN */

#define GPIO_FDCAN1_RX      (GPIO_FDCAN1_RX_8|GPIO_SPEED_HIGH)    /* PD0 */
#define GPIO_FDCAN1_TX      (GPIO_FDCAN1_TX_9|GPIO_SPEED_HIGH)    /* PD1 */

/* DMA channels *************************************************************/

/* ADC */

#define ADC1_DMA_CHAN DMAMAP_DMA1_ADC1     /* DMA1 */

#endif /* __BOARDS_ARM_STM32F0L0G0_NUCLEO_C092RC_INCLUDE_BOARD_H */
