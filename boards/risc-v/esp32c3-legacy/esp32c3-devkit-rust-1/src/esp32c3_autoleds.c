/****************************************************************************
 * boards/risc-v/esp32c3-legacy/esp32c3-devkit-rust-1/src/esp32c3_autoleds.c
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

#include <debug.h>

#include <arch/board/board.h>

#include "esp32c3_gpio.h"
#include "hardware/esp32c3_gpio_sigmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IO7 is connected to LED D2 */

#define GPIO_LED   7

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void set_led(bool v)
{
  ledinfo("Turn LED %s\n", v ? "on":"off");
  esp32c3_gpiowrite(GPIO_LED, v);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure the LED's pin to be used as output */

  esp32c3_gpio_matrix_out(GPIO_LED, SIG_GPIO_OUT_IDX, 0, 0);
  esp32c3_configgpio(GPIO_LED, OUTPUT_FUNCTION_1 | INPUT_FUNCTION_1);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  ledinfo("board_autoled_on(%d)\n", led);

  switch (led)
    {
      case LED_STARTED:
      case LED_HEAPALLOCATE:

        /* As the board provides only one soft controllable LED, we simply
         * turn it on when the board boots.
         */

        set_led(true);
        break;

      case LED_PANIC:

        /* For panic state, the LED is blinking */

        set_led(true);
        break;

      default:
        ledinfo("Not handled LED state: %d\n", led);
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
      case LED_PANIC:

        /* For panic state, the LED is blinking */

        set_led(false);
        break;

      default:
        ledinfo("Not handled LED state: %d\n", led);
        break;
    }
}
