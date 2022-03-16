/****************************************************************************
 * boards/arm/tiva/lm3s8962-ek/src/lm_leds.c
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

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "tiva_gpio.h"
#include "lm3s8962-ek.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define led_dumpgpio(m) tiva_dumpgpio(LED_GPIO, m)
#else
#  define led_dumpgpio(m)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_nest;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_initialize(void)
{
  ledinfo("Initializing\n");

  /* Configure Port E, Bit 1 as an output, initial value=OFF */

  led_dumpgpio("board_autoled_initialize before tiva_configgpio()");
  tiva_configgpio(LED_GPIO);
  led_dumpgpio("board_autoled_initialize after tiva_configgpio()");
  g_nest = 0;
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
      default:
        break;

      case LED_INIRQ:
      case LED_SIGNAL:
      case LED_ASSERTION:
      case LED_PANIC:
        g_nest++;
      case LED_IRQSENABLED:
      case LED_STACKCREATED:
        led_dumpgpio("board_autoled_on: before tiva_gpiowrite()");
        tiva_gpiowrite(LED_GPIO, false);
        led_dumpgpio("board_autoled_on: after tiva_gpiowrite()");
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
      case LED_IRQSENABLED:
      case LED_STACKCREATED:
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      default:
        break;

      case LED_INIRQ:
      case LED_SIGNAL:
      case LED_ASSERTION:
      case LED_PANIC:
        if (--g_nest <= 0)
          {
            led_dumpgpio("board_autoled_off: before tiva_gpiowrite()");
            tiva_gpiowrite(LED_GPIO, true);
            led_dumpgpio("board_autoled_off: after tiva_gpiowrite()");
          }
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
