/************************************************************************************
 * arch/arm/src/lpc11xx/chip/lpc111x_pinconfig.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC11XX_CHIP_LPC111X_PINCONFIG_H
#define __ARCH_ARM_SRC_LPC11XX_CHIP_LPC111X_PINCONFIG_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* GPIO pin definitions *************************************************************/
/* NOTE that functions have a alternate pins that can be selected.  These alternates
 * are identified with a numeric suffix like _1, _2, or _3.  Your board.h file
 * should select the correct alternative for your board by including definitions
 * such as:
 *
 * #define GPIO_UART1_RXD GPIO_UART1_RXD_1
 *
 * (without the suffix)
 */

#ifdef CONFIG_ARCH_CHIP_LPC1115

#define GPIO_CLKOUT        (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN1)
#define GPIO_CT32B0_MAT2   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN1)
#define GPIO_SPI0_SSEL     (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN2)
#define GPIO_CT16B0_CAP0   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN2)
#define GPIO_I2C0_SCL      (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN4)
#define GPIO_I2C0_SDA      (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN5)
#define GPIO_SPI0_SCK_1    (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN6)
#define GPIO_UART0_CTS     (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN7)
#define GPIO_SPI0_MISO     (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN8)
#define GPIO_CT16B0_MAT0   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN8)
#define GPIO_SPI0_MOSI     (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN9)
#define GPIO_CT16B0_MAT1   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN9)
#define GPIO_JTAG_SWCLK    (GPIO_ALT0 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN10)
#define GPIO_PIO0_10       (GPIO_ALT_GPIO | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN10)
#define GPIO_SPI0_SCK      (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN10)
#define GPIO_CT16B0_MAT2   (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN10)
#define GPIO_PIO0_11       (GPIO_ALT_GPIO | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN11)
#define GPIO_AD_inp0       (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN11)
#define GPIO_CT32B0_MAT3   (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT0 | GPIO_PIN11)
#define GPIO_PIO1_0        (GPIO_ALT_GPIO | GPIO_PULLUP  | GPIO_PORT1 | GPIO_PIN0)
#define GPIO_AD_inp1       (GPIO_ALT2 | GPIO_PULLUP  | GPIO_PORT1 | GPIO_PIN0)
#define GPIO_CT32B1_CAP0   (GPIO_ALT3 | GPIO_PULLUP  | GPIO_PORT1 | GPIO_PIN0)
#define GPIO_PIO1_1        (GPIO_ALT_GPIO | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN1)
#define GPIO_AD_inp2       (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN1)
#define GPIO_CT32B1_MAT0   (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN1)
#define GPIO_PIO1_2        (GPIO_ALT_GPIO | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN2)
#define GPIO_AD_inp3       (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN2)
#define GPIO_CT32B1_MAT1   (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN2)
#define GPIO_JTAG_SWDIO    (GPIO_ALT0 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN3)
#define GPIO_PIO1_3        (GPIO_ALT_GPIO | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN3)
#define GPIO_AD_inp4       (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN3)
#define GPIO_CT32B1_MAT2   (GPIO_ALT3 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN3)
#define GPIO_AD_inp5       (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN4)
#define GPIO_CT32B1_MAT3   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN4)
#define GPIO_UART0_RTS     (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN5)
#define GPIO_CT32B0_CAP0   (GPIO_ALT2 | GPIO_PULLDN | GPIO_PORT1 | GPIO_PIN5)
#define GPIO_UART0_RXD     (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN6)
#define GPIO_CT32B0_MAT0   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN6)
#define GPIO_UART0_TXD     (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN7)
#define GPIO_CT32B0_MAT1   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN7)
#define GPIO_CT16B1_CAP0   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN8)
#define GPIO_CT16B1_MAT0   (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN9)
#define GPIO_AD_inp6       (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN10)
#define GPIO_CT16B1_MAT1   (GPIO_ALT2 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN10)
#define GPIO_AD_inp7       (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT1 | GPIO_PIN11)
#define GPIO_UART0_DTR     (GPIO_ALT1 | GPIO_PULLUP | GPIO_PORT2 | GPIO_PIN0)

#endif /* CONFIG_ARCH_CHIP_LPC1115 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC11XX_CHIP_LPC116X_PINCONFIG_H */
