/****************************************************************************
 * boards/arm/lpc17xx_40xx/pnev5180b/src/lpc17_40_leds.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "lpc17_40_gpio.h"
#include "pnev5180b.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void pnev5180b_autoled_initialize(void)
{
  /* Configure all LED GPIO lines */

  lpc17_40_configgpio(PNEV5180B_LED_RED);
  lpc17_40_configgpio(PNEV5180B_LED_ORANGE);
  lpc17_40_configgpio(PNEV5180B_LED_BLUE);
  lpc17_40_configgpio(PNEV5180B_LED_GREEN);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      case LED_STARTED:
        lpc17_40_gpiowrite(PNEV5180B_LED_RED, true);
        lpc17_40_gpiowrite(PNEV5180B_LED_ORANGE, true);
        lpc17_40_gpiowrite(PNEV5180B_LED_BLUE, true);
        lpc17_40_gpiowrite(PNEV5180B_LED_GREEN, true);
        break;

      case LED_HEAPALLOCATE:
        lpc17_40_gpiowrite(PNEV5180B_LED_RED, false);
        lpc17_40_gpiowrite(PNEV5180B_LED_ORANGE, false);
        lpc17_40_gpiowrite(PNEV5180B_LED_BLUE, false);
        lpc17_40_gpiowrite(PNEV5180B_LED_GREEN, true);
        break;

      case LED_IRQSENABLED:
        lpc17_40_gpiowrite(PNEV5180B_LED_RED, false);
        lpc17_40_gpiowrite(PNEV5180B_LED_ORANGE, false);
        lpc17_40_gpiowrite(PNEV5180B_LED_BLUE, true);
        lpc17_40_gpiowrite(PNEV5180B_LED_GREEN, false);
        break;

      case LED_STACKCREATED:
        lpc17_40_gpiowrite(PNEV5180B_LED_RED, false);
        lpc17_40_gpiowrite(PNEV5180B_LED_ORANGE, false);
        lpc17_40_gpiowrite(PNEV5180B_LED_BLUE, false);
        lpc17_40_gpiowrite(PNEV5180B_LED_GREEN, false);
        break;

      case LED_INIRQ:
        lpc17_40_gpiowrite(PNEV5180B_LED_GREEN, true);
        break;

      case LED_SIGNAL:
        lpc17_40_gpiowrite(PNEV5180B_LED_BLUE, true);
        break;

      case LED_ASSERTION:
        lpc17_40_gpiowrite(PNEV5180B_LED_ORANGE, true);
        break;

      case LED_PANIC:
        lpc17_40_gpiowrite(PNEV5180B_LED_RED, true);
        break;

      default:
        DEBUGPANIC();
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
      case LED_INIRQ:
        lpc17_40_gpiowrite(PNEV5180B_LED_GREEN, false);
        break;

      case LED_SIGNAL:
        lpc17_40_gpiowrite(PNEV5180B_LED_BLUE, false);
        break;

      case LED_ASSERTION:
        lpc17_40_gpiowrite(PNEV5180B_LED_ORANGE, false);
        break;

      case LED_PANIC:
        lpc17_40_gpiowrite(PNEV5180B_LED_RED, false);
        break;

      default:
        DEBUGPANIC();
    }
}

#endif /* CONFIG_ARCH_LEDS */
