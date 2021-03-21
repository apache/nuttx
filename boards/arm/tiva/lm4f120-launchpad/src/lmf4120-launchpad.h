/****************************************************************************
 * boards/arm/tiva/lm4f120-launchpad/src/lmf4120-launchpad.h
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

#ifndef __BOARDS_ARM_TIVA_LM4F120_LAUNCHPAD_LM4F120_LAUNCHPAD_H
#define __BOARDS_ARM_TIVA_LM4F120_LAUNCHPAD_LM4F120_LAUNCHPAD_H

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

/* How many SSI modules does this chip support? The LM3S6965 supports 1 SSI
 * module (others may support more than 2 -- in such case, the following must
 * be expanded).
 */

#if TIVA_NSSI < 1
#  undef CONFIG_TIVA_SSI0
#  undef CONFIG_TIVA_SSI1
#elif TIVA_NSSI < 2
#  undef CONFIG_TIVA_SSI1
#endif

/* LM4F LaunchPad ***********************************************************/

/* The LM4F120 LaunchPad has a single RGB LED.
 * There is only one visible LED which will vary in color.
 * But, from the standpoint of the firmware, this appears as three LEDs:
 *
 *   BOARD_LED_R    -- Connected to PF1
 *   BOARD_LED_G    -- Connected to PF3
 *   BOARD_LED_B    -- Connected to PF2
 *
 * If CONFIG_ARCH_LEDS is defined, then automated support for the LaunchPad
 * LEDs will be included in the build:
 *
 * OFF:
 * - OFF means that the OS is still initializing. Initialization is very fast
 *   so if you see this at all, it probably means that the system is hanging
 *   up somewhere in the initialization phases.
 *
 * GREEN or GREEN-ish
 * - This means that the OS completed initialization.
 *
 * Bluish:
 * - Whenever and interrupt or signal handler is entered, the BLUE LED is
 *   illuminated and extinguished when the interrupt or signal handler exits.
 *   This will add a BLUE-ish tinge to the LED.
 *
 * Redish:
 * - If a recovered assertion occurs, the RED component will be illuminated
 *   briefly while the assertion is handled.
 *   You will probably never see this.
 *
 * Flashing RED:
 * - In the event of a fatal crash, the BLUE and GREEN components will be
 *   extinguished and the RED component will FLASH at a 2Hz rate.
 *
 *                          RED  GREEN BLUE
 * LED_STARTED       0      OFF  OFF   OFF
 * LED_HEAPALLOCATE  0      OFF  OFF   OFF
 * LED_IRQSENABLED   0      OFF  OFF   OFF
 * LED_STACKCREATED  1      OFF  ON    OFF
 * LED_INIRQ         2      NC   NC    ON  (momentary)
 * LED_SIGNAL        2      NC   NC    ON  (momentary)
 * LED_ASSERTION     3      ON   NC    NC  (momentary)
 * LED_PANIC         3      ON   OFF   OFF (flashing 2Hz)
 */

#define GPIO_LED_R   (GPIO_FUNC_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTF | GPIO_PIN_1)
#define GPIO_LED_G   (GPIO_FUNC_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTF | GPIO_PIN_3)
#define GPIO_LED_B   (GPIO_FUNC_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTF | GPIO_PIN_2)

/* The LM4F120 LaunchPad has a two buttons:
 *
 *   BOARD_SW1    -- Connected to PF4
 *   BOARD_SW2    -- Connected to PF0
 */

#define GPIO_SW1     (GPIO_FUNC_INTERRUPT | GPIO_INT_BOTHEDGES | GPIO_PORTF | GPIO_PIN_1)
#define GPIO_SW2     (GPIO_FUNC_INTERRUPT | GPIO_INT_BOTHEDGES | GPIO_PORTF | GPIO_PIN_1)

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: lm4f_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the LM4F LaunchPad.
 *
 ****************************************************************************/

void weak_function lm4f_spidev_initialize(void);

/****************************************************************************
 * Name: lm4f_led_initialize
 *
 * Description:
 *   Called to initialize the on-board LEDs.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void lm4f_led_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_TIVA_LM4F120_LAUNCHPAD_LM4F120_LAUNCHPAD_H */
