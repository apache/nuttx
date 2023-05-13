/****************************************************************************
 * boards/arm/stm32/stm32f3discovery/include/board.h
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

#ifndef __BOARDS_ARM_STM32_STM32F3DISCOVERY_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_STM32F3DISCOVERY_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

#include "stm32_rcc.h"
#include "stm32.h"

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

/* PLL source is HSE/1,
 * PLL multipler is 9:
 * PLL frequency is 8MHz (XTAL) x 9 = 72MHz
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
#define STM32_HCLK_FREQUENCY    STM32_PLL_FREQUENCY

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

#define BOARD_TIM1_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM3_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM4_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM5_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM6_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM7_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define BOARD_TIM8_FREQUENCY    STM32_HCLK_FREQUENCY

/* LED definitions **********************************************************/

/* The STM32F3Discovery board has ten LEDs.  Two of these are controlled by
 * logic on the board and are not available for software control:
 *
 * LD1 PWR:   red LED indicates that the board is powered.
 * LD2 COM:   LD2 default status is red. LD2 turns to green to indicate that
 *            communications are in progress between the PC and the
 *            ST-LINK/V2.
 *
 * And eight can be controlled by software:
 *
 * User LEDs connected to the I/O of the STM32F303VCT6.
 * User LD3:  red LED is a user LED connected to the PE9 I/O.
 * User LD4:  blue LED is a user LED connected to the PE8 I/O.
 * User LD5:  orange LED is a user LED connected to the PE10 I/O.
 * User LD6:  green LED is a user LED connected to the PE15 I/O.
 * User LD7:  green LED is a user LED connected to the PE11 I/O.
 * User LD8:  orange LED is a user LED connected to the PE14 I/O.
 * User LD9:  blue LED is a user LED connected to the PE12 I/O.
 * User LD10: red LED is a user LED connected to the PE13 I/O.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0 /* User LD3 */
#define BOARD_LED2        1 /* User LD4 */
#define BOARD_LED3        2 /* User LD5 */
#define BOARD_LED4        3 /* User LD6 */
#define BOARD_LED5        4 /* User LD7 */
#define BOARD_LED6        5 /* User LD8 */
#define BOARD_LED7        6 /* User LD9 */
#define BOARD_LED8        7 /* User LD10 */
#define BOARD_NLEDS       8

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)
#define BOARD_LED4_BIT    (1 << BOARD_LED4)
#define BOARD_LED5_BIT    (1 << BOARD_LED5)
#define BOARD_LED6_BIT    (1 << BOARD_LED6)
#define BOARD_LED7_BIT    (1 << BOARD_LED7)
#define BOARD_LED8_BIT    (1 << BOARD_LED8)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 8 LEDs on
 * board the stm32f3discovery.
 * The following definitions describe how NuttX controls the LEDs:
 *
 *   SYMBOL                Meaning                 LED state
 *                                              Initially all LEDs are OFF
 *   -------------------  -----------------------  ------------- ------------
 *   LED_STARTED          NuttX has been started   LD3 ON
 *   LED_HEAPALLOCATE     Heap has been allocated  LD4 ON
 *   LED_IRQSENABLED      Interrupts enabled       LD4 ON
 *   LED_STACKCREATED     Idle stack created       LD6 ON
 *   LED_INIRQ            In an interrupt          LD7 should glow
 *   LED_SIGNAL           In a signal handler      LD8 might glow
 *   LED_ASSERTION        An assertion failed      LD9 ON while handling
 *                                                 the assertion
 *   LED_PANIC            The system has crashed   LD10 Blinking at 2Hz
 *   LED_IDLE             STM32 is is sleep mode   (Optional, not used)
 */

#define LED_STARTED       0
#define LED_HEAPALLOCATE  1
#define LED_IRQSENABLED   2
#define LED_STACKCREATED  3
#define LED_INIRQ         4
#define LED_SIGNAL        5
#define LED_ASSERTION     6
#define LED_PANIC         7

/* Button definitions *******************************************************/

/* The STM32F3Discovery supports two buttons; only one button is controllable
 * by software:
 *
 *   B1 USER:
 *      user and wake-up button connected to the I/O PA0 of the
 *      STM32F303VCT6.
 *   B2 RESET:
 *      pushbutton connected to NRST is used to RESET the STM32F303VCT6.
 */

#define BUTTON_USER        0

#define NUM_BUTTONS        1

#define BUTTON_USER_BIT    (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/* USART
 *
 *  USART1: Hardwired to embedded STLinkV2 hardware debugger
 *    RX (PC5)
 *    TX (PC4)
 *
 *  USART2:
 *    Connect to an external UART<->RS232 transceiver for use as console.
 *    RX (PA3)
 *    TX (PA2)
 */

#define GPIO_USART2_RX GPIO_USART2_RX_2
#define GPIO_USART2_TX GPIO_USART2_TX_2

/* SPI
 *
 *  SPI1: Hardwired to ST L3GD20 MEMS device
 *    MISO (PA6)
 *    MSOI (PA7)
 *    SCK (PA5)
 */

#define GPIO_SPI1_MISO GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK  GPIO_SPI1_SCK_1

/* I2C
 *
 * I2C1: Accessible via expansion headers
 *   SCL (PA15)
 *   SDA (PA14)
 *   SMBA (PB5)
 *
 * I2C2: Accessible via expansion headers
 *   SCL (PA9)
 *   SDA (PA10)
 *   SMBA (PB12)
 */

#ifdef CONFIG_STM32_I2C1
#define GPIO_I2C1_SCL  GPIO_I2C1_SCL_1
#define GPIO_I2C1_SDA  GPIO_I2C1_SDA_1
#endif

#ifdef CONFIG_STM32_I2C2
#define GPIO_I2C2_SCL  GPIO_I2C2_SCL_1
#define GPIO_I2C2_SDA  GPIO_I2C2_SDA_1
#endif

#endif /* __BOARDS_ARM_STM32_STM32F3DISCOVERY_INCLUDE_BOARD_H */
