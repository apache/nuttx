/****************************************************************************
 * boards/arm/stm32/stm32f3discovery/src/stm32f3discovery.h
 *
 *   Copyright (C) 2013, 2016 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32_STM32F3DISCOVERY_SRC_STM32F3DISCOVERY_H
#define __BOARDS_ARM_STM32_STM32F3DISCOVERY_SRC_STM32F3DISCOVERY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/compiler.h>

#include <arch/stm32/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* How many SPI modules does this chip support? */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 3
#  undef CONFIG_STM32_SPI3
#endif

/* STM32F3Discovery GPIOs ***************************************************/

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
 * User LD3:  red LED is a user LED connected to PE9
 * User LD4:  blue LED is a user LED connected to PE8
 * User LD5:  orange LED is a user LED connected to PE10
 * User LD6:  green LED is a user LED connected to PE15
 * User LD7:  green LED is a user LED connected to PE11
 * User LD8:  orange LED is a user LED connected to PE14
 * User LD9:  blue LED is a user LED connected to PE12
 * User LD10: red LED is a user LED connected to PE13
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs
 * in any way.  The following definitions are used to access individual LEDs.
 */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN9)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN8)
#define GPIO_LED3       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN10)
#define GPIO_LED4       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN15)
#define GPIO_LED5       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN11)
#define GPIO_LED6       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN14)
#define GPIO_LED7       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN12)
#define GPIO_LED8       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN13)

/* Button definitions *******************************************************/

/* The STM32F3Discovery supports two buttons; only one button is controllable
 * by software:
 *
 *   B1 USER: user and wake-up button connected to PA0
 *   B2 RESET: pushbutton connected to NRST is used to RESET
 *             the STM32F303VCT6.
 *
 * NOTE that  EXTI interrupts are configured
 */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN0)

/* SPI - There is a ST MEMS L3GD20 device on SPI1 using these pins: */

#define GPIO_MEMS_CS    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#define GPIO_MEMS_INT1  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN0)
#define GPIO_MEMS_INT2  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTE|GPIO_PIN1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the stm32f3discovery
 *   board.
 *
 ****************************************************************************/

void weak_function stm32_spidev_initialize(void);

/****************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in inialization to setup
 *   USB-related GPIO pins for the STM32F3Discovery board.
 *
 ****************************************************************************/

#ifdef CONFIG_STM32_USB
void weak_function stm32_usbinitialize(void);
#endif

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

#ifdef CONFIG_PWM
int stm32_pwm_setup(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32_STM32F3DISCOVERY_SRC_STM32F3DISCOVERY_H */
