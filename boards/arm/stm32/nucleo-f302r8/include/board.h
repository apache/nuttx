/****************************************************************************
 * boards/arm/stm32/nucleo-f302r8/include/board.h
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

#ifndef __BOARDS_ARM_STM32_NUCLEO_F302R8_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_NUCLEO_F302R8_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#ifdef __KERNEL__
#  include "stm32.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - Internal 8 MHz RC Oscillator
 * LSI - 32 KHz RC
 * HSE - On-board crystal frequency is 8MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul        /* X1 on board */

#define STM32_HSI_FREQUENCY     8000000ul
#define STM32_LSI_FREQUENCY     40000            /* Between 30kHz and 60kHz */
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768            /* X2 on board */

/* PLL source is HSE/1, PLL multipler is 9: PLL frequency is
 * 8MHz (XTAL) x 9 = 72MHz
 */

#define STM32_CFGR_PLLSRC       RCC_CFGR_PLLSRC
#define STM32_CFGR_PLLXTPRE     0
#define STM32_CFGR_PLLMUL       RCC_CFGR_PLLMUL_CLKx9
#define STM32_PLL_FREQUENCY     (9*STM32_BOARD_XTAL)

/* Use the PLL and set the SYSCLK source to be the PLL */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY  STM32_PLL_FREQUENCY

/* AHB clock (HCLK) is SYSCLK (72MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB2 clock (PCLK2) is HCLK (72MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_APB2_CLKIN        (STM32_PCLK2_FREQUENCY)   /* Timers 1 and 8, 15-17 */

/* APB2 timers 1 and 8, 15-17 will receive PCLK2. */

/* Timers driven from APB2 will be PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB1_TIM15_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB1_TIM16_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB1_TIM17_CLKIN  (STM32_PCLK2_FREQUENCY)

/* APB1 clock (PCLK1) is HCLK/2 (36MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* APB1 timers 2-7 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)

/* USB divider -- Divide PLL clock by 1.5 */

#define STM32_CFGR_USBPRE       0

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY   (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM3_FREQUENCY   (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM4_FREQUENCY   (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM5_FREQUENCY   (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM6_FREQUENCY   (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM7_FREQUENCY   (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM8_FREQUENCY   STM32_HCLK_FREQUENCY

/* LED definitions **********************************************************/

/* The Nucleo F302R8 board has three LEDs.  Two of these are controlled by
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
 *           STM32F302R8T6.
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
 * the Nucleo F302R8.  The following definitions describe how NuttX controls
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

/* The Nucleo F302R8 supports two buttons; only one button is controllable
 * by software:
 *
 *   B1 USER:  user button connected to the I/O PC13 of the STM32F302R8T6.
 *   B2 RESET: push button connected to NRST is used to RESET the
 *             STM32F302R8T6.
 */

#define BUTTON_USER      0
#define NUM_BUTTONS      1

#define BUTTON_USER_BIT  (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/* TIM2 input ***************************************************************/

#define GPIO_TIM2_CH1IN (GPIO_TIM2_CH1IN_2 | GPIO_PULLUP) /* PA15 */
#define GPIO_TIM2_CH2IN (GPIO_TIM2_CH2IN_2 | GPIO_PULLUP) /* PB3 */

/* USART */

/* By default the USART2 is connected to STLINK Virtual COM Port:
 * USART2_RX - PA3
 * USART2_TX - PA2
 */

#define GPIO_USART2_RX GPIO_USART2_RX_2 /* PA3 */
#define GPIO_USART2_TX GPIO_USART2_TX_2 /* PA2 */

/* USART1 */

#define GPIO_USART1_RX GPIO_USART1_RX_2 /* PB7 */
#define GPIO_USART1_TX GPIO_USART1_TX_2 /* PB6 */

/* CAN */

#define GPIO_CAN1_RX     GPIO_CAN_RX_3 /* PB8 */
#define GPIO_CAN1_TX     GPIO_CAN_TX_3 /* PB9 */

/* PWM configuration ********************************************************/

/* TIM1 PWM */

#define STM32_TIM1_TRGO 0

#define GPIO_TIM1_CH1OUT  GPIO_TIM1_CH1OUT_2  /* PA8 */
#define GPIO_TIM1_CH1NOUT GPIO_TIM1_CH1N_3    /* PA11 */
#define GPIO_TIM1_CH2OUT  GPIO_TIM1_CH2OUT_2  /* PA9 */
#define GPIO_TIM1_CH2NOUT GPIO_TIM1_CH2N_2    /* PA12 */
#define GPIO_TIM1_CH3OUT  GPIO_TIM1_CH3OUT_2  /* PA10 */
#define GPIO_TIM1_CH3NOUT GPIO_TIM1_CH3N_3    /* PB1 */

/* TIM2 PWM */

#define GPIO_TIM2_CH1OUT  GPIO_TIM2_CH1_ETR_1 /* PA0 */
#define GPIO_TIM2_CH2OUT  GPIO_TIM2_CH2OUT_1  /* PA1 */
#define GPIO_TIM2_CH3OUT  GPIO_TIM2_CH3OUT_1  /* PA9 */

/* DMA channels *************************************************************/

/* ADC */

#define ADC1_DMA_CHAN DMACHAN_ADC1     /* DMA1_CH1 */

#ifdef CONFIG_BOARD_STM32_IHM07M1

/* Configuration specific for the X-NUCLEO-IHM07M1 expansion board with
 * the L6230 gate drivers.
 */

/* TIM1 configuration *******************************************************/

#  define GPIO_TIM1_CH1OUT   GPIO_TIM1_CH1OUT_2 /* TIM1 CH1  - PA8  - U high */
#  define GPIO_TIM1_CH2OUT   GPIO_TIM1_CH2OUT_2 /* TIM1 CH2  - PA9  - V high */
#  define GPIO_TIM1_CH3OUT   GPIO_TIM1_CH3OUT_2 /* TIM1 CH3  - PA10 - W high */
#  define GPIO_TIM1_CH4OUT   0                  /* not used as output */

/* UVW ENABLE */

#  define GPIO_FOC_EN_U (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|  \
                         GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN10)
#  define GPIO_FOC_EN_V (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|  \
                         GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN11)
#  define GPIO_FOC_EN_W (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|  \
                         GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN12)

/* DIAG/ENABLE */

#  define GPIO_FOC_DIAGEN (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz| \
                           GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN11)

#  define GPIO_FOC_LED2   (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                           GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN2)

/* Debug pins */

#  define GPIO_FOC_DEBUG0 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                           GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN8)
#  define GPIO_FOC_DEBUG1 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                           GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN9)
#  define GPIO_FOC_DEBUG2 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                           GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN6)
#  define GPIO_FOC_DEBUG3 (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz| \
                           GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN5)

#endif  /* CONFIG_BOARD_STM32_IHM07M1 */

#endif /* __BOARDS_ARM_STM32_NUCLEO_F302R8_INCLUDE_BOARD_H */
