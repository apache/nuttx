/****************************************************************************
 * boards/risc-v/gd32vw55x/gd32vw553-hmq/include/board.h
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

#ifndef __BOARDS_RISCV_GD32VW55X_GD32VW553_HMQ_INCLUDE_BOARD_H
#define __BOARDS_RISCV_GD32VW55X_GD32VW553_HMQ_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking: 40 MHz HXTAL -> PLLDIG -> 160 MHz SYSCLK (see
 * gd32vw55x_clockconfig.c).  These are informational for board code.
 */

#define BOARD_HXTAL_FREQUENCY   40000000
#define BOARD_IRC32K_FREQUENCY  32000     /* Internal 32 kHz RC (FWDGT, RTC) */
#define BOARD_SYSCLK_FREQUENCY  160000000

/* UART pin muxing.  The GD32VW553-HMQ wires UART0 (PA8/PB15) to the
 * on-board UBS/Serial port.  Pin/AF assignments follow the
 * vendor SDK (plf/src/uart/uart.h, ASIC boards).
 */

#define BOARD_UART2_TX_GPIO     GD32VW55X_GPIOA_BASE
#define BOARD_UART2_TX_PIN      6
#define BOARD_UART2_TX_AF       10
#define BOARD_UART2_RX_GPIO     GD32VW55X_GPIOA_BASE
#define BOARD_UART2_RX_PIN      7
#define BOARD_UART2_RX_AF       8

#define BOARD_UART1_TX_GPIO     GD32VW55X_GPIOB_BASE
#define BOARD_UART1_TX_PIN      15
#define BOARD_UART1_TX_AF       7
#define BOARD_UART1_RX_GPIO     GD32VW55X_GPIOA_BASE
#define BOARD_UART1_RX_PIN      8
#define BOARD_UART1_RX_AF       3

/* This is the console: PA8 as RX and PB15 ad TX */

#define BOARD_USART0_TX_GPIO    GD32VW55X_GPIOB_BASE
#define BOARD_USART0_TX_PIN     15
#define BOARD_USART0_TX_AF      8
#define BOARD_USART0_RX_GPIO    GD32VW55X_GPIOA_BASE
#define BOARD_USART0_RX_PIN     8
#define BOARD_USART0_RX_AF      2

/* Peripheral pin selection *************************************************/

/* The pin options come from arch/risc-v/src/gd32vw55x/hardware/
 * gd32vw55x_pinmap.h.  Avoid PA6/PA7: they are the UART2 console.
 */

#ifdef CONFIG_GD32VW55X_SPI
#  define GPIO_SPI_SCK   GPIO_SPI_SCK_1    /* PA2, AF5 */
#  define GPIO_SPI_MISO  GPIO_SPI_MISO_1   /* PA1, AF5 */
#  define GPIO_SPI_MOSI  GPIO_SPI_MOSI_1   /* PA0, AF5 */

/* SPI0 chip select for the SD card: a plain GPIO output on PA4 (J1),
 * driven by gd32_spi0select().  Active low, idles high.
 */

#  define GPIO_SPI0_CSPIN (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE | \
                           GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                           GPIO_CFG_OUTPUT_SET | GPIO_CFG_PORT_A | \
                           GPIO_CFG_PIN_4)
#endif

/* I2C0 uses PA2 (SCL) / PA3 (SDA) on AF4, the pins broken out on the J1
 * header (datasheet Table 2-5), so a sensor such as the SHT3x can be wired
 * there directly.  When SPI is enabled it claims PA2 (SPI SCK), so I2C0
 * falls back to PB0/PB1 (AF6); PB1 (SDA) is not broken out on J1/J2, so that
 * pairing only registers the bus (the periph case).
 */

#ifdef CONFIG_GD32VW55X_I2C0
#  ifdef CONFIG_GD32VW55X_SPI
#    define GPIO_I2C0_SCL  GPIO_I2C0_SCL_3   /* PB0, AF6 */
#    define GPIO_I2C0_SDA  GPIO_I2C0_SDA_3   /* PB1, AF6 (not on a header) */
#  else
#    define GPIO_I2C0_SCL  GPIO_I2C0_SCL_1   /* PA2, AF4 (J1) */
#    define GPIO_I2C0_SDA  GPIO_I2C0_SDA_1   /* PA3, AF4 (J1) */
#  endif
#endif

#ifdef CONFIG_GD32VW55X_I2C1
#  define GPIO_I2C1_SCL  GPIO_I2C1_SCL_4   /* PB12, AF6 */
#  define GPIO_I2C1_SDA  GPIO_I2C1_SDA_4   /* PB13, AF6 */
#endif

/* PWM.  The board bringup registers /dev/pwm0 on TIMER1.  Route its channel
 * 0 (TIMER1_CH0) to PA0 on the J1 header (AF1; TIMER1 lives on AF1) so the
 * output can be probed there.  The other channels stay unrouted.
 */

#ifdef CONFIG_GD32VW55X_PWM
#  define GPIO_TIMER1_CH0OUT (GPIO_CFG_MODE_AF | GPIO_CFG_PUPD_NONE | \
                              GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                              GPIO_CFG_AF_1 | GPIO_CFG_PORT_A | GPIO_CFG_PIN_0)
#endif

/* ADC.  Route ADC channel 8 (ADC_IN8) to PB0 on the J1 header (datasheet
 * Table 2-5) so an analog signal can be applied there.  When SPI is enabled
 * PB0 is instead claimed by the I2C0 fallback (see above), so the analog pin
 * is only routed when SPI is off.  The board bringup samples this channel
 * when the macro is defined.
 */

#if defined(CONFIG_GD32VW55X_ADC) && !defined(CONFIG_GD32VW55X_SPI)
#  define GPIO_ADC_IN8 (GPIO_CFG_MODE_ANALOG | GPIO_CFG_PUPD_NONE | \
                        GPIO_CFG_PORT_B | GPIO_CFG_PIN_0)
#endif

/* LEDs *********************************************************************/

/* The GD32VW553-HMQ has one LED on GPIO PC13, driven push-pull and
 * active HIGH (the vendor demo turns them on with gpio_bit_set(); see
 * MSDK/plf/src/gd32vw55x_platform.h, where they are named LED_RUN,
 * LED_SLEEP and LED_RX).
 *
 *   LED1  PC13
 */

#define GPIO_LED1 (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE | GPIO_CFG_PP | \
                   GPIO_CFG_SPEED_MAX | GPIO_CFG_OUTPUT_RESET | \
                   GPIO_CFG_PORT_C | GPIO_CFG_PIN_0)

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)

/* If CONFIG_ARCH_LEDS is defined, the LEDs are used by the OS to signal
 * its state, and are not available to the application.  Otherwise they are
 * fully under application control.
 *
 *   SYMBOL              MEANING                   LED1  LED2  LED3
 *   ------------------  ------------------------  ----  ----  ----
 */

#define LED_STARTED       0  /* NuttX has been started  ON    */
#define LED_HEAPALLOCATE  1  /* Heap has been allocated OFF   */
#define LED_IRQSENABLED   2  /* Interrupts enabled      OFF   */
#define LED_STACKCREATED  3  /* Idle stack created      OFF   */
#define LED_INIRQ         4  /* In an interrupt         OFF   */
#define LED_SIGNAL        5  /* In a signal handler     OFF   */
#define LED_ASSERTION     6  /* An assertion failed     OFF   */
#define LED_PANIC         7  /* The system has crashed  BLINK */

#endif /* __BOARDS_RISCV_GD32VW55X_GD32VW553_HMQ_INCLUDE_BOARD_H */
