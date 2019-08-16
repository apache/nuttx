/****************************************************************************
 * boards/arm/tiva/dk-tm4c129x/src/dk-tm4c129x.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_ARM_TIVA_DK_TM4C129X_DK_TM4C129X_H
#define __BOARDS_ARM_TIVA_DK_TM4C129X_DK_TM4C129X_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "chip.h"
#include "tiva_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* How many SSI modules does this chip support? */

#if TIVA_NSSI < 1
#  undef CONFIG_TIVA_SSI0
#  undef CONFIG_TIVA_SSI0
#elif TIVA_NSSI < 2
#  undef CONFIG_TIVA_SSI0
#endif

/* Do we need to register I2C drivers on behalf of the I2C tool? */

#define HAVE_I2CTOOL 1
#if !defined(CONFIG_SYSTEM_I2CTOOL) || !defined(CONFIG_I2C_DRIVER)
#  undef HAVE_I2CTOOL
#endif

/* LED definitions **********************************************************/

/* The TMC4C123G LaunchPad has a single RGB LED.
 * There is only one visible LED which will vary in color.
 * But, from the standpoint of the firmware, this appears as
 * three LEDs:
 *
 *   --- ------------ -----------------
 *   Pin Pin Function Jumper
 *   --- ------------ -----------------
 *   PN5 Red LED      J36 pins 1 and 2
 *   PQ4 Blue LED     J36 pins 3 and 4
 *   PQ7 Green LED    J36 pins 5 and 6
 *   --- ------------ -----------------
 *
 * A high output illuminates the LED.
 */

#define GPIO_LED_R   (GPIO_FUNC_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTN | GPIO_PIN_5)
#define GPIO_LED_G   (GPIO_FUNC_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTQ | GPIO_PIN_7)
#define GPIO_LED_B   (GPIO_FUNC_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTQ | GPIO_PIN_4)

/* Button definitions *******************************************************/

/* There are three push buttons on the board.
 *
 *   --- ------------ -----------------
 *   Pin Pin Function Jumper
 *   --- ------------ -----------------
 *   PP1 Select SW4   J37 pins 1 and 2
 *   PN3 Up SW2       J37 pins 3 and 4
 *   PE5 Down SW3     J37 pins 5 and 6
 *   --- ------------ -----------------
 *
 * Interrupts are supported only on port P and Q so only SW4 interrupts are
 * supported.
 */

#define GPIO_SW2   (GPIO_FUNC_INPUT | GPIO_PORTE | GPIO_PIN_5)
#define GPIO_SW3   (GPIO_FUNC_INPUT | GPIO_PORTN | GPIO_PIN_3)
#define GPIO_SW4   (GPIO_FUNC_INPUT | GPIO_INT_BOTHEDGES | GPIO_PORTP | GPIO_PIN_1)

#define IRQ_SW4    TIVA_IRQ_GPIOP_1

/* SPI Chip selects *********************************************************/

/*   SSI0: PA3 is used for SSI0 chip select to the second booster pack
 *         (No pull-ups)
 *   SSI3: PH4 selects the SD card and PQ1 selects the on-board SPI flash.
 *         Both pulled up on board.
 */

#define GPIO_BSTR2_CS (GPIO_FUNC_OUTPUT | GPIO_PADTYPE_STDWPU | GPIO_STRENGTH_4MA | \
                       GPIO_VALUE_ONE | GPIO_PORTA | GPIO_PIN_3)
#define GPIO_FLASH_CS (GPIO_FUNC_OUTPUT | GPIO_PADTYPE_STD | GPIO_STRENGTH_4MA | \
                       GPIO_VALUE_ONE | GPIO_PORTH | GPIO_PIN_4)
#define GPIO_SD_CS    (GPIO_FUNC_OUTPUT | GPIO_PADTYPE_STD | GPIO_STRENGTH_4MA | \
                       GPIO_VALUE_ONE | GPIO_PORTH | GPIO_PIN_4)

/* I2C **********************************************************************/

/*   I2C3: PG4-5 are provide to the BoostPack 1 interface
 *   I2C7: PA4-5 are provide to the BoostPack 2 interface
 *   I2C6: PB6-7 are used for I2C to the TMP100 and the EM connector.
 *         J18 and J20 must be closed to connect the TMP100.
 *         I2C address is 0x4A
 */

#define TMP100_I2CBUS  6
#define TMP100_I2CADDR 0x4a

/* Speaker outputs **********************************************************/

/* PB2/PD4 are used for the speaker output */

/* Touchscreen **************************************************************/

/* PE7/PP7/PT2-3 are used for the touch screen */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: tm4c_ssidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the DK-TM4C129X.
 *
 ****************************************************************************/

void weak_function tm4c_ssidev_initialize(void);

/****************************************************************************
 * Name: tm4c_led_initialize
 *
 * Description:
 *   Called to initialize the on-board LEDs.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void tm4c_led_initialize(void);
#endif

/****************************************************************************
 * Name: tm4c_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int tm4c_bringup(void);

/****************************************************************************
 * Name: tiva_timer_configure
 *
 * Description:
 *   Configure the timer driver
 *
 ****************************************************************************/

#ifdef CONFIG_DK_TM4C129X_TIMER
int tiva_timer_configure(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_TIVA_DK_TM4C129X_DK_TM4C129X_H */
