/****************************************************************************
 * configs/lm4f120-launchpad/src/lm4f_leds.c
 * arch/arm/src/board/lm4f_leds.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "lm_gpio.h"
#include "lmf4120-launchpad.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* The LM32F120 has a single RGB LED.  There is only one visible LED which will vary
 * in color.  But, from the standpoint of the firmware, this appears as three LEDs:
 *
 *   BOARD_LED_R    -- Connected to PF1
 *   BOARD_LED_G    -- Connected to PF3
 *   BOARD_LED_B    -- Connected to PF2
 *
 * If CONFIG_ARCH_LEDS is defined, then automated support for the LaunchPad LEDs
 * will be included in the build:
 *
 * OFF:
 * - OFF means that the OS is still initializing. Initialization is very fast so
 *   if you see this at all, it probably means that the system is hanging up
 *   somewhere in the initialization phases.
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
 *   briefly while the assertion is handled.  You will probably never see this.
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
 * LED_PANIC         3      ON   OFF   OFF (flashing 2Hz)
 */

/* CONFIG_DEBUG_LEDS enables debug output from this file (needs CONFIG_DEBUG
 * with CONFIG_DEBUG_VERBOSE too)
 */

#ifdef CONFIG_DEBUG_LEDS
#  define leddbg  lldbg
#  define ledvdbg llvdbg
#else
#  define leddbg(x...)
#  define ledvdbg(x...)
#endif

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_LEDS
#  define led_dumpgpio(m) lm_dumpgpio(LED_GPIO, m)
#else
#  define led_dumpgpio(m)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm4f_ledinit
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void lm4f_ledinit(void)
{
  leddbg("Initializing\n");

  /* Configure Port E, Bit 1 as an output, initial value=OFF */

  led_dumpgpio("lm4f_ledinit before lm_configgpio()");
  lm_configgpio(GPIO_LED_R);
  lm_configgpio(GPIO_LED_G);
  lm_configgpio(GPIO_LED_B);
  led_dumpgpio("lm4f_ledinit after lm_configgpio()");
}

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void up_ledon(int led)
{
  switch (led)
    {
      /* All components stay off until the file initialization step */

      default:
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      case LED_IRQSENABLED:
      default:
        break;

      /* The GREEN component is illuminated at the final initialization step */

      case LED_STACKCREATED:
        lm_gpiowrite(GPIO_LED_GREEN, false);
        break;

      /* These will illuminate the BLUE component with on effect no RED and GREEN */

      case LED_INIRQ:
      case LED_SIGNAL:
        lm_gpiowrite(GPIO_LED_BLUE, false);
        break;

      /* This will turn off RED and GREEN and turn RED on */

      case LED_PANIC:
        lm_gpiowrite(GPIO_LED_GREEN, true);
        lm_gpiowrite(GPIO_LED_BLUE, true);

      /* This will illuminate the RED component with no effect on RED and GREEN */

      case LED_ASSERTION:
        lm_gpiowrite(GPIO_LED_RED, false);
        break;
    }
}

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

void up_ledoff(int led)
{
  switch (led)
    {
      /* These should not happen and are ignored */

      default:
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      case LED_IRQSENABLED:
      case LED_STACKCREATED:
      default:
        break;

      /* These will extinguish the BLUE component with no effect on RED and GREEN */

      case LED_INIRQ:
      case LED_SIGNAL:
        lm_gpiowrite(GPIO_LED_BLUE, true);
        break;

      /* These will extinguish the RED component with on effect on RED and GREEN */

      case LED_INIRQ:
      case LED_SIGNAL:
        lm_gpiowrite(GPIO_LED_RED, true);
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
