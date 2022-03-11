/****************************************************************************
 * boards/arm/stm32/stm32f411-minimum/src/stm32_autoleds.c
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
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32.h"
#include "stm32f411-minimum.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void set_led(bool v)
{
  ledinfo("Turn LED %s\n", v? "on":"off");
  stm32_gpiowrite(GPIO_LED1, !v);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_initialize(void)
{
  /* Configure LED GPIO for output */

  stm32_configgpio(GPIO_LED1);
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
    }
}

#endif /* CONFIG_ARCH_LEDS */
