/****************************************************************************
 * boards/arm/gd32w51x/gd-xd-w515-eval/include/board.h
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

#ifndef __BOARDS_ARM_GD32W51X_GD_XD_W515_EVAL_INCLUDE_BOARD_H
#define __BOARDS_ARM_GD32W51X_GD_XD_W515_EVAL_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking.  The GD-xD-W515-EVAL has a 25 MHz crystal (measured: the
 * factory firmware runs the PLL with PSC = 25 at 180 MHz).  The PLL
 * brings SYSCLK to 180 MHz (see gd32w51x_clockconfig.c).
 */

#define BOARD_HXTAL_FREQUENCY   25000000
#define BOARD_IRC32K_FREQUENCY  32000     /* 32 kHz RC (FWDGT, RTC) */
#define BOARD_SYSCLK_FREQUENCY  180000000

/* Pin assignments, from the board schematic (GD-xD-W515-EVAL V1.0,
 * GigaDevice, 2023-05) and checked on the hardware.
 */

/* LEDs: LED1..LED4 of the board schematic, push-pull outputs, active
 * high.  LED3 (PB0) is also TIMER2_CH1.
 */

#define BOARD_NLEDS             4

#define BOARD_LED1              0
#define BOARD_LED2              1
#define BOARD_LED3              2
#define BOARD_LED4              3

#define GPIO_LED1               (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE | \
                                 GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                 GPIO_CFG_OUTPUT_RESET | GPIO_CFG_PORT_C | \
                                 GPIO_CFG_PIN_5)
#define GPIO_LED2               (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE | \
                                 GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                 GPIO_CFG_OUTPUT_RESET | GPIO_CFG_PORT_C | \
                                 GPIO_CFG_PIN_4)
#define GPIO_LED3               (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE | \
                                 GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                 GPIO_CFG_OUTPUT_RESET | GPIO_CFG_PORT_B | \
                                 GPIO_CFG_PIN_0)
#define GPIO_LED4               (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE | \
                                 GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                 GPIO_CFG_OUTPUT_RESET | GPIO_CFG_PORT_B | \
                                 GPIO_CFG_PIN_1)

/* CHRG_EN of the GD30BC2416 charger (PC3, active high).  It is held high:
 * the charge is controlled through the I2C registers of the charger.
 */

#define GPIO_CHRG_EN            (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE | \
                                 GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                 GPIO_CFG_OUTPUT_SET | GPIO_CFG_PORT_C | \
                                 GPIO_CFG_PIN_3)

#define BOARD_LED1_BIT          (1 << BOARD_LED1)
#define BOARD_LED2_BIT          (1 << BOARD_LED2)
#define BOARD_LED3_BIT          (1 << BOARD_LED3)
#define BOARD_LED4_BIT          (1 << BOARD_LED4)

/* I2C0: SCL PB15, SDA PA8 */

#define GPIO_I2C0_SCL           GPIO_I2C0_SCL_1
#define GPIO_I2C0_SDA           GPIO_I2C0_SDA_1

/* SPI0 drives the ST7735S LCD (160x80, write only): SCK PA11, MOSI PA9.
 * PA10 is the data/command line, PB13 the chip select and PC6 the reset.
 */

#define GPIO_SPI0_SCK           GPIO_SPI0_SCK_3
#define GPIO_SPI0_MOSI          GPIO_SPI0_MOSI_3

#define GPIO_LCD_DC             (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE | \
                                 GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                 GPIO_CFG_OUTPUT_SET | GPIO_CFG_PORT_A | \
                                 GPIO_CFG_PIN_10)
#define GPIO_LCD_CS             (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE | \
                                 GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                 GPIO_CFG_OUTPUT_SET | GPIO_CFG_PORT_B | \
                                 GPIO_CFG_PIN_13)
#define GPIO_LCD_RST            (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE | \
                                 GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                 GPIO_CFG_OUTPUT_RESET | GPIO_CFG_PORT_C | \
                                 GPIO_CFG_PIN_6)

/* SPI1 drives the GSL6157 fingerprint sensor: SCK PC7, MISO PB6, MOSI
 * PB9.  PB8 is its chip select, PB5 its SHUTDOWN (reset, active low) line
 * and PA12 its IRQ output (/dev/gpio2).  Found with a logic analyzer on
 * the connector of the sensor.
 */

#define GPIO_SPI1_SCK           GPIO_SPI1_SCK_4
#define GPIO_SPI1_MISO          GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI          GPIO_SPI1_MOSI_2

#define GPIO_FP_CS              (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE | \
                                 GPIO_CFG_PP | GPIO_CFG_SPEED_MAX | \
                                 GPIO_CFG_OUTPUT_SET | GPIO_CFG_PORT_B | \
                                 GPIO_CFG_PIN_8)
#define GPIO_FP_SHUTDOWN        (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE | \
                                 GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                 GPIO_CFG_OUTPUT_SET | GPIO_CFG_PORT_B | \
                                 GPIO_CFG_PIN_5)

/* QSPI: the GD25Q128E NOR flash (U6, 16 MB) on PA4 (SCK), PA5 (CS),
 * PA6/PA7 (IO0/IO1) and PB3/PB4 (IO2/IO3), AF3.
 */

#define GPIO_QSPI_SCK           GPIO_QSPI_SCK_1
#define GPIO_QSPI_CS            GPIO_QSPI_CS_1
#define GPIO_QSPI_IO0           GPIO_QSPI_IO0_1
#define GPIO_QSPI_IO1           GPIO_QSPI_IO1_1
#define GPIO_QSPI_IO2           GPIO_QSPI_IO2_1
#define GPIO_QSPI_IO3           GPIO_QSPI_IO3_1

/* Touch key (TSI group 2): PB12 (G2P1) to the sampling capacitor, PB14
 * (G2P3) to the "TSI" electrode.  It is exposed as button 0
 * (/dev/buttons).
 */

#define NUM_BUTTONS             1
#define BUTTON_TSI              0
#define BUTTON_TSI_BIT          (1 << BUTTON_TSI)

#define GPIO_TSI_SAMPCAP        (GPIO_TSI_G2_IO1_1 | GPIO_CFG_OD)
#define GPIO_TSI_KEY            GPIO_TSI_G2_IO3_1
#define BOARD_TSI_GROUP         2
#define BOARD_TSI_SAMPLE_PIN    1
#define BOARD_TSI_KEY_PIN       3

/* PWM: TIMER1_CH0 on PA0 drives the vibration motor */

#define GPIO_TIMER1_CH0OUT      GPIO_TIMER1_CH0OUT_1

/* GPIOs exposed through /dev/gpioN (CONFIG_DEV_GPIO):
 *
 *   /dev/gpio0  PB7   output     Heater coil (COIL_EN, P-MOSFET), active
 *                                low on the pin: write 1 to switch it on
 *   /dev/gpio1  PC1   interrupt  USB power detection (high with USB)
 *   /dev/gpio2  PA12  interrupt  IRQ of the GSL6157 fingerprint sensor
 *   /dev/gpio3  PB2   interrupt  IRQ of the GD30BC2416 battery charger,
 *                                only without its driver (CONFIG_GD30BC2416)
 *
 * The interrupts trigger on both edges.  The charger pulses its IRQ low
 * for 8 ms on each event (USB plugged or unplugged, charge end, fault);
 * through the PNP Q3, PB2 idles low and goes high during the pulse.
 */

#define BOARD_NGPIOOUT          1
#ifdef CONFIG_GD30BC2416
#  define BOARD_NGPIOINT        2
#else
#  define BOARD_NGPIOINT        3
#endif

#define GPIO_HEATER             (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE | \
                                 GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                 GPIO_CFG_OUTPUT_SET | GPIO_CFG_PORT_B | \
                                 GPIO_CFG_PIN_7)
#define GPIO_CHARGER_IRQ        (GPIO_CFG_MODE_INPUT | GPIO_CFG_PUPD_NONE | \
                                 GPIO_CFG_PORT_B | GPIO_CFG_PIN_2)
#define GPIO_USB_DETECT         (GPIO_CFG_MODE_INPUT | GPIO_CFG_PUPD_NONE | \
                                 GPIO_CFG_PORT_C | GPIO_CFG_PIN_1)
#define GPIO_FP_IRQ             (GPIO_CFG_MODE_INPUT | GPIO_CFG_PUPD_NONE | \
                                 GPIO_CFG_PORT_A | GPIO_CFG_PIN_12)

/* Console: USART2 (UART2 in NuttX) on PB10/PB11 (AF7), wired to the
 * CH340E USB-serial bridge behind the micro-USB connector.  Found by
 * pulsing the pins from the debugger; the boot ROM also uses USART2 on
 * these pins.
 */

#define BOARD_UART2_TX_GPIO     GD32W51X_GPIOB_BASE
#define BOARD_UART2_TX_PIN      10
#define BOARD_UART2_TX_AF       7
#define BOARD_UART2_RX_GPIO     GD32W51X_GPIOB_BASE
#define BOARD_UART2_RX_PIN      11
#define BOARD_UART2_RX_AF       7

#endif /* __BOARDS_ARM_GD32W51X_GD_XD_W515_EVAL_INCLUDE_BOARD_H */
