/****************************************************************************
 * boards/arm/ra8m1/ek-ra8m1/src/ra8m1_autoleds.c
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

#include "ra_gpio.h"
#include "ek_ra8m1.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure the LED GPIOs for output, initially off */

  ra_configgpio(GPIO_LED1);
  ra_configgpio(GPIO_LED2);
  ra_configgpio(GPIO_LED3);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      /* 0: LED_STARTED, LED_HEAPALLOCATE, LED_IRQSENABLED: all LEDs off.
       * Since the LEDs were initially all off and since this state only
       * occurs one time, nothing need be done.
       */

      default:
      case 0:
        break;

      /* 1: LED_STACKCREATED: LED1 on */

      case 1:
        ra_gpiowrite(GPIO_LED1, true);
        break;

      /* 2: LED_INIRQ, LED_SIGNAL, LED_ASSERTION: LED2 on */

      case 2:
        ra_gpiowrite(GPIO_LED2, true);
        break;

      /* 3: LED_PANIC: LED3 on */

      case 3:
        ra_gpiowrite(GPIO_LED3, true);
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
      /* 0 and 1 should never happen: they are only signalled once. */

      default:
      case 0:
      case 1:
        break;

      /* 2: LED_INIRQ, LED_SIGNAL, LED_ASSERTION: LED2 off */

      case 2:
        ra_gpiowrite(GPIO_LED2, false);
        break;

      /* 3: LED_PANIC: LED3 off */

      case 3:
        ra_gpiowrite(GPIO_LED3, false);
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
