/****************************************************************************
 * boards/arm/tiva/dk-tm4c129x/src/tm4c_autoleds.c
 *
 *   Copyright (C) 2014-2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
