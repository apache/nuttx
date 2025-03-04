/****************************************************************************
 * boards/avr/at32uc3/mizar32a/src/mizar32a.h
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

#ifndef __BOARDS_AVR_AT32UC3_MIZAR32A_SRC_MIZAR32A_H
#define __BOARDS_AVR_AT32UC3_MIZAR32A_SRC_MIZAR32A_H

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
#  define CONFIG_MIZAR32A_BUTTON1_IRQ 1
#endif

#if (CONFIG_AVR32_GPIOIRQSETB & 8) == 1
#  define CONFIG_MIZAR32A_BUTTON2_IRQ 1
#endif

/* AVRDEV1 GPIO Pin Definitions *********************************************/

/* LEDs
 *
 * The Mizar32-A board has 2 LEDs,
 * one of which can be controlled through GPIO pins.
 *
 * PIN 20  PB29  LED1
 */

#define PINMUX_GPIO_LED1 (GPIO_ENABLE | GPIO_OUTPUT | GPIO_LOW | GPIO_PORTB | 29)

/* BUTTONs
 *
 * The Mizar32-A board has 2 BUTTONs,
 * one of which can be sensed through GPIO pins.
 *
 * PIN 61  PX16  KEY1
 */

#if CONFIG_MIZAR32A_BUTTON1_IRQ
#  define PINMUX_GPIO_BUTTON1 (GPIO_ENABLE | GPIO_INPUT | GPIO_INTR | \
                               GPIO_INTMODE_BOTH | GPIO_GLITCH | GPIO_PORTB | 2)
#  define GPIO_BUTTON1_IRQ    AVR32_IRQ_GPIO_PB2
#else
#  define PINMUX_GPIO_BUTTON1 (GPIO_ENABLE | GPIO_INPUT | GPIO_GLITCH | \
                               GPIO_PORTB | 2)
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
#endif /* __BOARDS_AVR_AT32UC3_MIZAR32A_SRC_MIZAR32A_H */
