/****************************************************************************
 * boards/arm/rzv2h/rzv2h-rdk/src/rzv2h_autoleds.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "rzv2h_gpio.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array maps LED numbers to GPIO configurations */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_dumppins
 ****************************************************************************/

#ifdef LED_VERBOSE
static void led_dumppins(const char *msg)
{
  (void)msg;
}
#else
#  define led_dumppins(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 *
 * Description:
 *   Initialize LED GPIOs so that LEDs can be controlled.
 *
 ****************************************************************************/

void board_autoled_initialize(void)
{
  ;
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Set the LED configuration into the ON condition for the state provided
 *   by the led parameter.  This may be one of:
 *
 *   LED_STARTED       - NuttX has been started
 *   LED_HEAPALLOCATE  - Heap has been allocated
 *   LED_IRQSENABLED   - Interrupts enabled
 *   LED_STACKCREATED  - Idle stack created
 *   LED_INIRQ         - In an interrupt handler
 *   LED_SIGNAL        - In a signal handler
 *   LED_ASSERTION     - An assertion failed
 *   LED_PANIC         - The system has crashed
 *
 ****************************************************************************/

void board_autoled_on(int led)
{
  /* Active low LEDs: write 0 to turn ON */

  (void)led;
}

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Set the LED configuration into the OFF condition for the state provided
 *   by the led parameter.
 *
 ****************************************************************************/

void board_autoled_off(int led)
{
  /* Active low LEDs: write 1 to turn OFF */

  (void)led;
}

#endif /* CONFIG_ARCH_LEDS */

/****************************************************************************
 * Name: board_userled_initialize
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is not defined, then the board can provide
 *   application-controlled LED functionality via these functions.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
uint32_t board_userled_initialize(void)
{
  return 0;
}

/****************************************************************************
 * Name: board_userled
 *
 * Description:
 *   Set the LED to the on or off state
 *
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  (void)led;
  (void)ledon;
}

/****************************************************************************
 * Name: board_userled_all
 *
 * Description:
 *   Set the state of all LEDs
 *
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  (void)ledset;
}

#endif /* !CONFIG_ARCH_LEDS */
