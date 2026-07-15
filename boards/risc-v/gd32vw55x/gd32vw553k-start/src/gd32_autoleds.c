/****************************************************************************
 * boards/risc-v/gd32vw55x/gd32vw553k-start/src/gd32_autoleds.c
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

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "gd32vw55x_gpio.h"
#include "gd32vw553k-start.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The three LEDs are on GPIOC and are active HIGH.  When CONFIG_ARCH_LEDS is
 * selected they show the OS state and are not available to the application;
 * the meaning of each state is tabulated in board.h.
 */

static const uint32_t g_led_map[BOARD_NLEDS] =
{
  GPIO_LED1,
  GPIO_LED2,
  GPIO_LED3
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void led_set(int led, bool ledon)
{
  gd32_gpio_write(g_led_map[led], ledon);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  int i;

  for (i = 0; i < BOARD_NLEDS; i++)
    {
      gd32_gpio_config(g_led_map[i]);
    }
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      case LED_IRQSENABLED:
        led_set(BOARD_LED1, true);
        break;

      case LED_STACKCREATED:
        led_set(BOARD_LED1, true);
        led_set(BOARD_LED2, true);
        break;

      case LED_ASSERTION:
      case LED_PANIC:
        led_set(BOARD_LED3, true);
        break;

      case LED_INIRQ:
      case LED_SIGNAL:
      default:

        /* These states leave the LEDs alone: toggling them on every
         * interrupt would make the display useless.
         */

        break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
      case LED_ASSERTION:
      case LED_PANIC:
        led_set(BOARD_LED3, false);
        break;

      case LED_INIRQ:
      case LED_SIGNAL:
      default:
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
