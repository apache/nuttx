/****************************************************************************
 * boards/arm/stm32u0/stm32u083c-dk/include/board.h
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

#ifndef __BOARDS_ARM_STM32U0_STM32U083C_DK_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32U0_STM32U083C_DK_INCLUDE_BOARD_H

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

/* HSI - Internal 16 MHz RC Oscillator
 * MSI - Internal multispeed RC Oscillator (100 kHz - 48 MHz)
 * LSI - 32 kHz RC
 * HSE - 8 MHz crystal (not fitted by default)
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul          /* 8MHz */

#define STM32_HSI_FREQUENCY     16000000ul         /* 16MHz */
#define STM32_LSI_FREQUENCY     32000              /* Between 30kHz and 60kHz */
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768              /* X2 on board */

/* PLL configuration
 *
 *   PLL source is HSI16
 *   VCO input frequency  = 16 MHz / PLLM = 16 MHz (must be 2.66 - 16 MHz)
 *   VCO output frequency = 16 MHz * PLLN = 112 MHz (must be 96 - 344 MHz)
 *   PLLP output          = 112 MHz / PLLP = 56 MHz
 *   PLLQ output          = 112 MHz / PLLQ = 56 MHz
 *   PLLR output (SYSCLK) = 112 MHz / PLLR = 56 MHz
 */

#define STM32_PLLCFG_PLLSRC     RCC_PLLCFG_PLLSRC_HSI
#define STM32_PLLCFG_PLLCFG     (RCC_PLLCFG_PLLPEN | RCC_PLLCFG_PLLREN)

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(1)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(7)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP(2)
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(2)
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(2)

#define STM32_VCO_FREQUENCY     ((STM32_HSI_FREQUENCY / 1) * 7)
#define STM32_PLLP_FREQUENCY    (STM32_VCO_FREQUENCY / 2)
#define STM32_PLLQ_FREQUENCY    (STM32_VCO_FREQUENCY / 2)
#define STM32_PLLR_FREQUENCY    (STM32_VCO_FREQUENCY / 2)

/* Use the PLL as SYSCLK source (56 MHz) */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY  STM32_PLLR_FREQUENCY

/* AHB clock (HCLK) is SYSCLK (56 MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB clock (PCLK) is HCLK (56 MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK
#define STM32_PCLK1_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_PCLK2_FREQUENCY   STM32_PCLK1_FREQUENCY

/* All timers on PCLK x1 (56 MHz) */

#define STM32_TIM1_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_TIM2_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_TIM3_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_TIM6_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_TIM7_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_TIM15_CLKIN  STM32_PCLK1_FREQUENCY
#define STM32_TIM16_CLKIN  STM32_PCLK1_FREQUENCY

/* LED definitions **********************************************************/

/* The STM32U083C-DK board has three user LEDs:
 *
 *   User LD3:   green LED connected to the I/O PC13.
 *   User LD4:   blue LED connected to the I/O PA5.
 *   User LD5:   red LED connected to the I/O PB2.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1       0 /* User LD3 (green) */
#define BOARD_LED2       1 /* User LD4 (blue) */
#define BOARD_LED3       2 /* User LD5 (red) */
#define BOARD_NLEDS      3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT   (1 << BOARD_LED1)
#define BOARD_LED2_BIT   (1 << BOARD_LED2)
#define BOARD_LED3_BIT   (1 << BOARD_LED3)

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
 *   LED_IDLE            STM32 is in sleep mode   Not used
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

/* The STM32U083C-DK board has no GPIO user buttons.  The board provides
 * a RESET push-button connected to NRST, a 4-direction joystick with
 * selection connected to an ADC1 input (resistor ladder) and a touchkey
 * connected to the TSC.
 */

/* Alternate function pin selections ****************************************/

/* USART */

/* By default the USART2 is connected to STLINK Virtual COM Port:
 *   USART2_RX - PA3
 *   USART2_TX - PA2
 */

#define GPIO_USART2_RX      (GPIO_USART2_RX_1|GPIO_SPEED_HIGH)    /* PA3 */
#define GPIO_USART2_TX      (GPIO_USART2_TX_1|GPIO_SPEED_HIGH)    /* PA2 */

/* USART3 at arduino D0/D1:
 *   USART3_RX - PC5
 *   USART3_TX - PC4
 */

#define GPIO_USART3_RX      (GPIO_USART3_RX_4|GPIO_SPEED_HIGH)    /* PC5 */
#define GPIO_USART3_TX      (GPIO_USART3_TX_4|GPIO_SPEED_HIGH)    /* PC4 */

/* PWM on TIM1:
 *   TIM1_CH1 - PA8 (arduino D5)
 */

#define GPIO_TIM1_CH1OUT    (GPIO_TIM1_CH1OUT_1|GPIO_SPEED_HIGH)  /* PA8 */

/* DMA channels *************************************************************/

/* ADC */

#define ADC1_DMA_CHAN DMAMAP_DMA1_ADC1     /* DMA1 */

#endif /* __BOARDS_ARM_STM32U0_STM32U083C_DK_INCLUDE_BOARD_H */
