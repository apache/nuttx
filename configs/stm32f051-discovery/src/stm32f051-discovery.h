/****************************************************************************************************
 * configs/stm32f051-discovery/src/stm32f051-discovery.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Alan Carvalho de Assis <acassis@gmail.com>
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
 ****************************************************************************************************/

#ifndef __CONFIGS_STM32F051_DISCOVERY_SRC_STM32F051_DISCOVERY_H
#define __CONFIGS_STM32F051_DISCOVERY_SRC_STM32F051_DISCOVERY_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "stm32f0_gpio.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Configuration ************************************************************************************/
/* How many SPI modules does this chip support? */

#if STM32F0_NSPI < 1
#  undef CONFIG_STM32F0_SPI1
#  undef CONFIG_STM32F0_SPI2
#  undef CONFIG_STM32F0_SPI3
#elif STM32F0_NSPI < 2
#  undef CONFIG_STM32F0_SPI2
#  undef CONFIG_STM32F0_SPI3
#elif STM32F0_NSPI < 3
#  undef CONFIG_STM32F0_SPI3
#endif

/* STM32F0Discovery GPIOs ***************************************************************************/
/* The STM32F0Discovery board has four LEDs.  Two of these are controlled by logic on the board and
 * are not available for software control:
 *
 * LD1 COM:   LD2 default status is red. LD2 turns to green to indicate that communications are in
 *            progress between the PC and the ST-LINK/V2.
 * LD2 PWR:   Red LED indicates that the board is powered.
 *
 * And two LEDs can be controlled by software:
 *
 * User LD3:  Green LED is a user LED connected to the I/O PB7 of the STM32L152 MCU.
 * User LD4:  Blue LED is a user LED connected to the I/O PB6 of the STM32L152 MCU.
 *
 * The other side of the LED connects to ground so high value will illuminate the LED.
 */

#define GPIO_LED1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_10MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN9)
#define GPIO_LED2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_10MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN8)

/* Button definitions *******************************************************************************/
/* The STM32F0Discovery supports two buttons; only one button is controllable by software:
 *
 *   B1 USER: user and wake-up button connected to the I/O PA0 of the STM32F303VCT6.
 *   B2 RESET: pushbutton connected to NRST is used to RESET the STM32F303VCT6.
 *
 * NOTE that  EXTI interrupts are configured
 */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER   (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | GPIO_PORTA | GPIO_PIN0)

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_STM32F051_DISCOVERY_SRC_STM32F051_DISCOVERY_H */
