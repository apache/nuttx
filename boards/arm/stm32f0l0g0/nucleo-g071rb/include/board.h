/****************************************************************************
 * boards/arm/stm32f0l0g0/nucleo-g071rb/include/board.h
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

#ifndef __BOARDS_ARM_STM32F0L0G0_NUCLEO_G071RB_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32F0L0G0_NUCLEO_G071RB_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - Internal 16 MHz RC Oscillator
 * LSI - 32 KHz RC
 * HSE - 8 MHz from MCO output of ST-LINK (disabled by default)
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000            /* Between 30kHz and 60kHz */
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768            /* X2 on board */

/* Main PLL Configuration.
 *
 * PLL source is HSI = 16,000,000
 *
 * PLL_VCOx = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 * Subject to:
 *
 *     1 <= PLLM <= 8
 *     8 <= PLLN <= 86
 *   4 MHz <= PLL_IN <= 16MHz
 *   64 MHz <= PLL_VCO <= 344MHz
 * SYSCLK  = PLLRCLK = PLL_VCO / PLLR
 *
 */

/* PLL source is HSI, PLLN=50, PLLM=4
 * PLLP enable, PLLQ enable, PLLR enable
 *
 *   2 <= PLLP <= 32
 *   2 <= PLLQ <= 8
 *   2 <= PLLR <= 8
 *
 *   PLLR <= 64MHz
 *   PLLQ <= 128MHz
 *   PLLP <= 128MHz
 *
 *   PLL_VCO = (16,000,000 / 4) * 50 = 200 MHz
 *
 *   PLLP = PLL_VCO/4 = 200 MHz / 4 = 40 MHz
 *   PLLQ = PLL_VCO/4 = 200 MHz / 4 = 40 MHz
 *   PLLR = PLL_VCO/4 = 200 MHz / 4 = 40 MHz
 */

#define STM32_PLLCFG_PLLSRC     RCC_PLLCFG_PLLSRC_HSI
#define STM32_PLLCFG_PLLCFG     (RCC_PLLCFG_PLLPEN | \
                                 RCC_PLLCFG_PLLQEN | \
                                 RCC_PLLCFG_PLLREN)

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(4)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(50)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP(4)
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(4)
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(4)

#define STM32_VCO_FREQUENCY     ((STM32_HSE_FREQUENCY / 2) * 50)
#define STM32_PLLP_FREQUENCY    (STM32_VCO_FREQUENCY / 4)
#define STM32_PLLQ_FREQUENCY    (STM32_VCO_FREQUENCY / 4)
#define STM32_PLLR_FREQUENCY    (STM32_VCO_FREQUENCY / 4)

/* Use the PLL and set the SYSCLK source to be the PLLR (40MHz) */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY  (STM32_PLLR_FREQUENCY)

/* AHB clock (HCLK) is SYSCLK (40MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/2 (20MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* TODO: timers */

/* LED definitions **********************************************************/

/* The Nucleo LO73RZ board has three LEDs.  Two of these are controlled by
 * logic on the board and are not available for software control:
 *
 * LD1 COM:  LD1 default status is red.  LD1 turns to green to indicate that
 *           communications are in progress between the PC and the
 *           ST-LINK/V2-1.
 * LD3 PWR:  red LED indicates that the board is powered.
 *
 * And one can be controlled by software:
 *
 * User LD2: green LED is a user LED connected to the I/O PA5 of the
 *           STM32LO73RZ.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LED in
 * any way.  The following definition is used to access the LED.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1       0 /* User LD2 */
#define BOARD_NLEDS      1

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT   (1 << BOARD_LED1)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the Nucleo LO73RZ.  The following definitions describe how NuttX controls
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

/* The Nucleo LO73RZ supports two buttons; only one button is controllable
 * by software:
 *
 *   B1 USER:  user button connected to the I/O PC13 of the STM32LO73RZ.
 *   B2 RESET: push button connected to NRST is used to RESET the
 *             STM32LO73RZ.
 */

#define BUTTON_USER      0
#define NUM_BUTTONS      1

#define BUTTON_USER_BIT  (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/* USART */

/* By default the USART2 is connected to STLINK Virtual COM Port:
 * USART2_RX - PA3
 * USART2_TX - PA2
 */

#define GPIO_USART2_RX (GPIO_USART2_RX_1|GPIO_SPEED_HIGH) /* PA3 */
#define GPIO_USART2_TX (GPIO_USART2_TX_1|GPIO_SPEED_HIGH) /* PA2 */

/* DMA channels *************************************************************/

/* ADC */

#define ADC1_DMA_CHAN DMACHAN_ADC1     /* DMA1_CH1 */

#endif /* __BOARDS_ARM_STM32F0L0G0_NUCLEO_LO73RZ_INCLUDE_BOARD_H */
