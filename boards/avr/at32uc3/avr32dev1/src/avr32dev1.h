/****************************************************************************
 * boards/avr/at32uc3/avr32dev1/src/avr32dev1.h
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

#ifndef __BOARDS_AVR_AT32UC3_AVR32DEV1_SRC_AVR32DEV1_H
#define __BOARDS_AVR_AT32UC3_AVR32DEV1_SRC_AVR32DEV1_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include "at32uc3_config.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#if (CONFIG_AVR32_GPIOIRQSETB & 4) == 1
#  define CONFIG_AVR32DEV_BUTTON1_IRQ 1
#endif

#if (CONFIG_AVR32_GPIOIRQSETB & 8) == 1
#  define CONFIG_AVR32DEV_BUTTON2_IRQ 1
#endif

/* AVRDEV1 GPIO Pin Definitions *********************************************/

/* LEDs
 *
 * The AVR32DEV1 board has 3 LEDs,
 * two of which can be controlled through GPIO pins.
 *
 * PIN 13  PA7  LED1
 * PIN 14  PA8  LED2
 */

#define PINMUX_GPIO_LED1 (GPIO_ENABLE | GPIO_OUTPUT | GPIO_LOW | GPIO_PORTA | 7)
#define PINMUX_GPIO_LED2 (GPIO_ENABLE | GPIO_OUTPUT | GPIO_LOW | GPIO_PORTA | 8)

/* BUTTONs
 *
 * The AVR32DEV1 board has 3 BUTTONs,
 * two of which can be sensed through GPIO pins.
 *
 * PIN 24  PB2  KEY1
 * PIN 25  PB3  KEY2
 */

#if CONFIG_AVR32DEV_BUTTON1_IRQ
#  define PINMUX_GPIO_BUTTON1 (GPIO_ENABLE | GPIO_INPUT | GPIO_INTR | \
                               GPIO_INTMODE_BOTH | GPIO_GLITCH | GPIO_PORTB | 2)
#  define GPIO_BUTTON1_IRQ    AVR32_IRQ_GPIO_PB2
#else
#  define PINMUX_GPIO_BUTTON1 (GPIO_ENABLE | GPIO_INPUT | GPIO_GLITCH | \
                               GPIO_PORTB | 2)
#endif

#if CONFIG_AVR32DEV_BUTTON2_IRQ
#  define PINMUX_GPIO_BUTTON2 (GPIO_ENABLE | GPIO_INPUT | GPIO_INTR | \
                               GPIO_INTMODE_BOTH | GPIO_GLITCH | GPIO_PORTB | 3)
#  define GPIO_BUTTON2_IRQ    AVR32_IRQ_GPIO_PB3
#else
#  define PINMUX_GPIO_BUTTON2 (GPIO_ENABLE | GPIO_INPUT | GPIO_GLITCH | \
                               GPIO_PORTB | 3)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_AVR_AT32UC3_AVR32DEV1_SRC_AVR32DEV1_H */
