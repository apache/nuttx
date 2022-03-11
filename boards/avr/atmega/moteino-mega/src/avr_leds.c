/****************************************************************************
 * boards/avr/atmega/moteino-mega/src/avr_leds.c
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

#include <stdbool.h>
#include <debug.h>
#include <avr/io.h>

#include <nuttx/board.h>

#include "up_internal.h"
#include "atmega.h"
#include "moteino_mega.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_ncoff;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: atmega_led_initialize
 ****************************************************************************/

void atmega_led_initialize(void)
{
  /* The MoteinoMEGA's single LED is on Port D, Pin 7.
   * Configure this pin as an output and turn it OFF.
   * The "other" side of the LED is onnected to ground through a resistor.
   * Therefore, a logic value of 0 should turn the LED off.
   */

  DDRD   |= (1 << 7);
  PORTD  &= ~(1 << 7);
  g_ncoff = true;
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  /*                         ON      OFF
   * LED_STARTED        0    OFF     ON  (never happens)
   * LED_HEAPALLOCATE   0    OFF     ON  (never happens)
   * LED_IRQSENABLED    0    OFF     ON  (never happens)
   * LED_STACKCREATED   1    ON      ON  (never happens)
   * LED_INIRQ          2    OFF     NC  (momentary)
   * LED_SIGNAL         2    OFF     NC  (momentary)
   * LED_ASSERTION      2    OFF     NC  (momentary)
   * LED_PANIC          0    OFF     ON  (1Hz flashing)
   */

  switch (led)
    {
    case 0:

      /* The steady state is OFF */

      g_ncoff = true;

    case 2:

      /* Turn the LED off */

      PORTD &= ~(1 << 7);
      break;

    case 1:

      /* The steady state is ON */

      PORTD |= (1 << 7);
      g_ncoff = false;
      break;

    default:
      return;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  /*                         ON      OFF
   * LED_STARTED        0    OFF     ON  (never happens)
   * LED_HEAPALLOCATE   0    OFF     ON  (never happens)
   * LED_IRQSENABLED    0    OFF     ON  (never happens)
   * LED_STACKCREATED   1    ON      ON  (never happens)
   * LED_INIRQ          2    OFF     NC  (momentary)
   * LED_SIGNAL         2    OFF     NC  (momentary)
   * LED_ASSERTION      2    OFF     NC  (momentary)
   * LED_PANIC          0    OFF     ON  (1Hz flashing)
   */

  switch (led)
    {
    case 2:

      /* If the "no-change" state is OFF, then turn the LED off */

      if (g_ncoff)
        {
          PORTD &= ~(1 << 7);
          break;
        }

      /* Otherwise, fall through to turn the LED ON */

    case 0:
    case 1:

      /* Turn the LED on */

      PORTD |= (1 << 7);
      break;

    default:
      return;
    }
}

#endif /* CONFIG_ARCH_LEDS */
