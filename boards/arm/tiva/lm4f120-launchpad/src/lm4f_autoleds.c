/****************************************************************************
 * boards/arm/tiva/lm4f120-launchpad/src/lm4f_autoleds.c
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
#include "lmf4120-launchpad.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The LM4F120 LaunchPad has a single RGB LED.  There is only one visible LED
 * which will vary in color.  But, from the standpoint of the firmware, this
 * appears as three LEDs:
 *
 *   BOARD_LED_R    -- Connected to PF1
 *   BOARD_LED_G    -- Connected to PF3
 *   BOARD_LED_B    -- Connected to PF2
 *
 * If CONFIG_ARCH_LEDS is defined, then automated support for the LaunchPad
 * LEDs will be included in the build:
 *
 * OFF:
 * - OFF means that the OS is still initializing. Initialization is very fast
 *   so if you see this at all, it probably means that the system is hanging
 *   up somewhere in the initialization phases.
 *
 * GREEN or GREEN-ish
 * - This means that the OS completed initialization.
 *
 * Bluish:
 * - Whenever and interrupt or signal handler is entered, the BLUE LED is
 *   illuminated and extinguished when the interrupt or signal handler exits.
 *   This will add a BLUE-ish tinge to the LED.
 *
 * Redish:
 * - If a recovered assertion occurs, the RED component will be illuminated
 *   briefly while the assertion is handled.
 *   You will probably never see this.
 *
 * Flashing RED:
 * - In the event of a fatal crash, the BLUE and GREEN components will be
 *   extinguished and the RED component will FLASH at a 2Hz rate.
 *
 *                          RED  GREEN BLUE
 * LED_STARTED       0      OFF  OFF   OFF
 * LED_HEAPALLOCATE  0      OFF  OFF   OFF
 * LED_IRQSENABLED   0      OFF  OFF   OFF
 * LED_STACKCREATED  1      OFF  ON    OFF
 * LED_INIRQ         2      NC   NC    ON  (momentary)
 * LED_SIGNAL        2      NC   NC    ON  (momentary)
 * LED_ASSERTION     3      ON   NC    NC  (momentary)
 * LED_PANIC         4      ON   OFF   OFF (flashing 2Hz)
 */

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define led_dumpgpio(m) tiva_dumpgpio(LED_GPIO, m)
#else
#  define led_dumpgpio(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm4f_led_initialize
 *
 * Description:
 *   Called to initialize the on-board LEDs.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void lm4f_led_initialize(void)
{
  ledinfo("Initializing\n");

  /* Configure Port E, Bit 1 as an output, initial value=OFF */

  led_dumpgpio("lm4f_led_initialize before tiva_configgpio()");
  tiva_configgpio(GPIO_LED_R);
  tiva_configgpio(GPIO_LED_G);
  tiva_configgpio(GPIO_LED_B);
  led_dumpgpio("lm4f_led_initialize after tiva_configgpio()");
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      /* All components stay off until the file initialization step */

      default:
      case 0:
        break;

      /* The GREEN component is illuminated at the final initialization
       * step
       */

      case 1:
        tiva_gpiowrite(GPIO_LED_G, false);
        break;

      /* These will illuminate the BLUE component with on effect no RED
       * and GREEN
       */

      case 2:
        tiva_gpiowrite(GPIO_LED_B, false);
        break;

      /* This will turn off RED and GREEN and turn RED on */

      case 4:
        tiva_gpiowrite(GPIO_LED_G, true);
        tiva_gpiowrite(GPIO_LED_B, true);

      /* This will illuminate the RED component with no effect on RED
       * and GREEN
       */

      case 3:
        tiva_gpiowrite(GPIO_LED_R, false);
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
      /* These should not happen and are ignored */

      default:
      case 0:
      case 1:
        break;

      /* These will extinguish the BLUE component with no effect on RED
       * and GREEN
       */

      case 2:
        tiva_gpiowrite(GPIO_LED_B, true);
        break;

      /* These will extinguish the RED component with on effect on RED
       * and GREEN
       */

      case 3:
      case 4:
        tiva_gpiowrite(GPIO_LED_R, true);
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
