/****************************************************************************
 * boards/arm/tiva/dk-tm4c129x/src/tm4c_autoleds.c
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

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "tiva_gpio.h"
#include "dk-tm4c129x.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tm4c_led_initialize
 *
 * Description:
 *   Called to initialize the on-board LEDs.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void tm4c_led_initialize(void)
{
  /* Configure LED PIOs for output */

  tiva_configgpio(GPIO_LED_R);
  tiva_configgpio(GPIO_LED_G);
  tiva_configgpio(GPIO_LED_B);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  /* --------------- ------- ---- ----- --------------------
   * STATE           VALUE   RED  GREEN BLUE
   * --------------- ------- ---- ----- --------------------
   * LED_STARTED       0     OFF  OFF   ON
   * LED_HEAPALLOCATE  1     NC   NC    NC
   * LED_IRQSENABLED   1     NC   NC    NC
   * LED_STACKCREATED  2     OFF  ON    OFF
   * LED_INIRQ         1     NC   NC    NC
   * LED_SIGNAL        1     NC   NC    NC
   * LED_ASSERTION     1     NC   NC    NC
   * LED_PANIC         3     ON   OFF   OFF (flashing 2Hz)
   * --------------- ------- ---- ----- --------------------
   *
   * A high output illuminates the LED.
   */

  switch (led)
    {
    case 0: /* R=OFF, G=OFF, B=ON */

      /* Previous state was all OFF */

      tiva_gpiowrite(GPIO_LED_B, true);
      break;

    default:
    case 1: /* No change */
      break;

    case 2: /* R=OFF, G=ON, B=OFF */

      /* Previous state was all: R=OFF, G=OFF, B=ON */

      tiva_gpiowrite(GPIO_LED_G, true);
      tiva_gpiowrite(GPIO_LED_B, false);
      break;

    case 3: /* R=ON, G=OFF, B=OFF */

      /* Previous state was all: R=OFF, G=Unknown, B=Unknown */

      tiva_gpiowrite(GPIO_LED_R, true);
      tiva_gpiowrite(GPIO_LED_G, false);
      tiva_gpiowrite(GPIO_LED_B, false);
      break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  /* --------------- ------- ---- ----- --------------------
   * STATE           VALUE   RED  GREEN BLUE
   * --------------- ------- ---- ----- --------------------
   * LED_STARTED       0     OFF  OFF   ON
   * LED_HEAPALLOCATE  1     NC   NC    NC
   * LED_IRQSENABLED   1     NC   NC    NC
   * LED_STACKCREATED  2     OFF  ON    OFF
   * LED_INIRQ         1     NC   NC    NC
   * LED_SIGNAL        1     NC   NC    NC
   * LED_ASSERTION     1     NC   NC    NC
   * LED_PANIC         3     ON   OFF   OFF (flashing 2Hz)
   * --------------- ------- ---- ----- --------------------
   *
   * A high output illuminates the LED.
   */

  switch (led)
    {
    case 0: /* Will not happen */
    case 1: /* No change */
    case 2: /* Will not happen */
    default:
      break;

    case 3: /* R=OFF, G=OFF, B=OFF */

      /* Previous state was all: R=ON, G=OFF, B=OFF */

      tiva_gpiowrite(GPIO_LED_R, false);
    }
}

#endif /* CONFIG_ARCH_LEDS */
