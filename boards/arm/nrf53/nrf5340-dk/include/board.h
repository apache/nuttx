/****************************************************************************
 * boards/arm/nrf53/nrf5340-dk/include/board.h
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

#ifndef __BOARDS_ARM_NRF53_NRF5340_DK_INCLUDE_BOARD_H
#define __BOARDS_ARM_NRF53_NRF5340_DK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_NRF53_GPIOTE)
#  include <nuttx/irq.h>
#endif

#include "hardware/nrf53_osc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#define BOARD_SYSTICK_CLOCK         (64000000)
#define BOARD_OSC_XOSC32KI_INTCAP   (OSC_XOSC32KI_INTCAP_C7PF)

/* LED definitions **********************************************************/

/* A low output illuminates the LED.
 *
 * LED index values for use with board_userled()
 */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_LED4        3
#define BOARD_NLEDS       4

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)
#define BOARD_LED4_BIT    (1 << BOARD_LED4)

/* If CONFIG_ARCH_LEDS is defined, the LED will be controlled as follows
 * for NuttX debug functionality (where NC means "No Change").
 */

#define LED_STARTED                0  /* OFF      */
#define LED_HEAPALLOCATE           0  /* OFF      */
#define LED_IRQSENABLED            0  /* OFF      */
#define LED_STACKCREATED           1  /* ON       */
#define LED_INIRQ                  2  /* NC       */
#define LED_SIGNAL                 2  /* NC       */
#define LED_ASSERTION              2  /* NC       */
#define LED_PANIC                  3  /* Flashing */

/* If CONFIG_ARCH_LEDS is not defined, then the LEDs are completely under
 * control of the application.  The following interfaces are then available
 * for application control of the LEDs:
 *
 *  uint32_t board_userled_initialize(void);
 *  void board_userled(int led, bool ledon);
 *  void board_userled_all(uint32_t ledset);
 */

/* Button definitions *******************************************************/

/* Board supports 4 buttons. */

#define BUTTON_BTN1       0
#define BUTTON_BTN2       1
#define BUTTON_BTN3       2
#define BUTTON_BTN4       3
#define NUM_BUTTONS       4

#define BUTTON_BTN1_BIT  (1 << BUTTON_BTN1)
#define BUTTON_BTN2_BIT  (1 << BUTTON_BTN2)
#define BUTTON_BTN3_BIT  (1 << BUTTON_BTN3)
#define BUTTON_BTN4_BIT  (1 << BUTTON_BTN4)

/* UART Pins ****************************************************************/

/* The following definitions must be provided so that the NRF53 serial
 * driver can set up the UART for the serial console properly.
 */

#ifdef CONFIG_ARCH_CHIP_NRF5340_CPUAPP
#  define BOARD_UART0_RX_PIN  (GPIO_MCUSEL_APP | GPIO_INPUT  | \
                               GPIO_PORT1 | GPIO_PIN(0))
#  define BOARD_UART0_TX_PIN  (GPIO_MCUSEL_APP | GPIO_OUTPUT | \
                               GPIO_VALUE_ONE | GPIO_PORT1 | GPIO_PIN(1))

/* Declarations for the Net core */

#  define BOARD_NET_UART0_RX_PIN  (GPIO_MCUSEL_NET | GPIO_INPUT  |  \
                                   GPIO_PORT0 | GPIO_PIN(22))
#  define BOARD_NET_UART0_TX_PIN  (GPIO_MCUSEL_NET | GPIO_OUTPUT |      \
                                   GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(20))
#endif

#ifdef CONFIG_ARCH_CHIP_NRF5340_CPUNET
#  define BOARD_UART0_RX_PIN  (GPIO_MCUSEL_NET | GPIO_INPUT  | \
                               GPIO_PORT0 | GPIO_PIN(22))
#  define BOARD_UART0_TX_PIN  (GPIO_MCUSEL_NET | GPIO_OUTPUT | \
                               GPIO_VALUE_ONE | GPIO_PORT0 | GPIO_PIN(20))
#endif

/* PWM Pins *****************************************************************/

/* PWM 0:
 *   PWM0 CH0 - P1.10
 *   PWM0 CH1 - P1.11
 *   PWM0 CH2 - P1.12
 *   PWM0 CH3 - P1.13
 */

#define NRF53_PWM0_CH0_PIN (GPIO_MCUSEL_APP | GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN(10))
#define NRF53_PWM0_CH1_PIN (GPIO_MCUSEL_APP | GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN(11))
#define NRF53_PWM0_CH2_PIN (GPIO_MCUSEL_APP | GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN(12))
#define NRF53_PWM0_CH3_PIN (GPIO_MCUSEL_APP | GPIO_OUTPUT | GPIO_PORT1 | GPIO_PIN(13))

/* ADC Pins *****************************************************************/

/* ADC
 *   ADC CH0 - P0.04 - AIN1
 *   ADC CH1 - P0.05 - AIN2
 *   ADC CH2 - P0.06 - AIN4
 *   ADC CH3 - P0.07 - AIN5
 */

#define NRF53_ADC_CH0_PIN (GPIO_MCUSEL_APP | GPIO_INPUT | GPIO_PORT0 | GPIO_PIN(4))
#define NRF53_ADC_CH1_PIN (GPIO_MCUSEL_APP | GPIO_INPUT | GPIO_PORT0 | GPIO_PIN(5))
#define NRF53_ADC_CH2_PIN (GPIO_MCUSEL_APP | GPIO_INPUT | GPIO_PORT0 | GPIO_PIN(6))
#define NRF53_ADC_CH3_PIN (GPIO_MCUSEL_APP | GPIO_INPUT | GPIO_PORT0 | GPIO_PIN(7))

#endif /* __BOARDS_ARM_NRF53_NRF5340_DK_INCLUDE_BOARD_H */
