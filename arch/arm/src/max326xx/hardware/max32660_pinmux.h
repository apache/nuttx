/************************************************************************************
 * arch/arm/src/max326xx/hardware/max32660_pinmux.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_PINMUX_H
#define __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_PINMUX_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Alternate Pin Functions.
 *
 * Alternative pin selections are provided with a numeric suffix like _1, _2, etc.
 * Drivers, however, will use the pin selection without the numeric suffix.
 * Additional definitions are required in the board.h file.  For example, if
 * UART1 RX connects vis P0.7 on some board, then the following definition should
 * appear in the board.h header file for that board:
 *
 * #define GPIO_UART1_RX GPIO_UART1_RX_2
 *
 * The driver will then automatically configure P0.7 as the UART1 RX pin.
 */

/* WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!! WARNING!!!
 * Additional effort is required to select specific GPIO options such as frequency,
 * open-drain/push-pull, and pull-up/down!  Just the basics are defined for most
 * pins in this file.
 */

/* 32KHz Clock Output */

#define GPIO_32KCAL      (GPIO_ALT3 | GPIO_PORT0 | GPIO_PIN2)

/* I2C */

#define GPIO_I2C0_SCL    (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN8)
#define GPIO_I2C0_SDA    (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN9)

#define GPIO_I2C1_SCL    (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN2)
#define GPIO_I2C1_SDA    (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN3)

/* SPIMMS/I2S */

#define GPIO_I2S_BCLK_1  (GPIO_ALT2 | GPIO_PORT0 | GPIO_PIN2)
#define GPIO_I2S_BCLK_2  (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN12) /* 20-TQFN only */
#define GPIO_I2S_LRCLK_1 (GPIO_ALT2 | GPIO_PORT0 | GPIO_PIN3)
#define GPIO_I2S_LRCLK_2 (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN13) /* 20-TQFN only */
#define GPIO_I2S_SDI_1   (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN0)
#define GPIO_I2S_SDI_2   (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN10) /* 20-TQFN only */
#define GPIO_I2S_SDO_1   (GPIO_ALT2 | GPIO_PORT0 | GPIO_PIN1)
#define GPIO_I2S_SDO_2   (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN11) /* 20-TQFN only */

/* SPI */

#define GPIO_SPI0_MISO   (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN4)
#define GPIO_SPI0_MOSI   (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN5)
#define GPIO_SPI0_SCK    (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN6)
#define GPIO_SPI0_SS0    (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN7)

#define GPIO_SPI1_MISO_1 (GPIO_ALT2 | GPIO_PORT0 | GPIO_PIN0)
#define GPIO_SPI1_MISO_2 (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN10) /* 20-TQFN only */
#define GPIO_SPI1_MOSI_1 (GPIO_ALT2 | GPIO_PORT0 | GPIO_PIN1)
#define GPIO_SPI1_MOSI_2 (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN11) /* 20-TQFN only */
#define GPIO_SPI1_SCK_1  (GPIO_ALT2 | GPIO_PORT0 | GPIO_PIN2)
#define GPIO_SPI1_SCK_2  (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN12) /* 20-TQFN only */
#define GPIO_SPI1_SS0_1  (GPIO_ALT2 | GPIO_PORT0 | GPIO_PIN3)
#define GPIO_SPI1_SS0_2  (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN13) /* 20-TQFN only */

/* JTAG/SWD */

#define GPIO_SWDCLK_1    (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN1)
#define GPIO_SWDCLK_2    (GPIO_ALT2 | GPIO_PORT0 | GPIO_PIN9)
#define GPIO_SWDIO_1     (GPIO_ALT1 | GPIO_PORT0 | GPIO_PIN0)
#define GPIO_SWDIO_2     (GPIO_ALT2 | GPIO_PORT0 | GPIO_PIN8)

/* Timer */

#define GPIO_TMR0        (GPIO_ALT3 | GPIO_PORT0 | GPIO_PIN3)

/* UARTs */

#define GPIO_UART0_CTS   (GPIO_ALT2 | GPIO_PORT0 | GPIO_PIN6)
#define GPIO_UART0_RTS   (GPIO_ALT2 | GPIO_PORT0 | GPIO_PIN7)
#define GPIO_UART0_RX    (GPIO_ALT2 | GPIO_PORT0 | GPIO_PIN5)
#define GPIO_UART0_TX    (GPIO_ALT2 | GPIO_PORT0 | GPIO_PIN4)

#define GPIO_UART1_CTS   (GPIO_ALT2 | GPIO_PORT0 | GPIO_PIN12) /* 20-TQFN only */
#define GPIO_UART1_RTS   (GPIO_ALT2 | GPIO_PORT0 | GPIO_PIN13) /* 20-TQFN only */
#define GPIO_UART1_RX_1  (GPIO_ALT3 | GPIO_PORT0 | GPIO_PIN1)
#define GPIO_UART1_RX_2  (GPIO_ALT3 | GPIO_PORT0 | GPIO_PIN7)
#define GPIO_UART1_RX_3  (GPIO_ALT2 | GPIO_PORT0 | GPIO_PIN11) /* 20-TQFN only */
#define GPIO_UART1_TX_1  (GPIO_ALT3 | GPIO_PORT0 | GPIO_PIN0)
#define GPIO_UART1_TX_2  (GPIO_ALT3 | GPIO_PORT0 | GPIO_PIN6)
#define GPIO_UART1_TX_3  (GPIO_ALT2 | GPIO_PORT0 | GPIO_PIN10) /* 20-TQFN only */

#endif /* __ARCH_ARM_SRC_MAX326XX_HARDWARE_MAX32660_PINMUX_H */
