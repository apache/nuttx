/****************************************************************************
 * configs/cc3200/src/cc3200_autoleds.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Jim Ewing <jim@droidifi.com>
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

#include "cc3200_launchpad.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The CC3200 LaunchPad has three RGB LEDs.
 *
 *   BOARD_LED_R    -- Connected to PF1
 *   BOARD_LED_G    -- Connected to PF3
 *   BOARD_LED_Y    -- Connected to PF2
 *
 * If CONFIG_ARCH_LEDS is defined, then automated support for the LaunchPad LEDs
 * will be included in the build:
 *
 * OFF:
 * - OFF means that the OS is still initializing. Initialization is very fast so
 *   if you see this at all, it probably means that the system is hanging up
 *   somewhere in the initialization phases.
 *
 * GREEN
 * - This means that the OS completed initialization.
 *
 * YELLOW:
 * - Whenever and interrupt or signal handler is entered, the YELLOW LED is
 *   illuminated and extinguished when the interrupt or signal handler exits.
 *
 * RED:
 * - If a recovered assertion occurs, the RED component will be illuminated
 *   briefly while the assertion is handled.  You will probably never see this.
 *
 * Flashing RED:
 * - In the event of a fatal crash, the YELLOW and GREEN components will be
 *   extinguished and the RED component will FLASH at a 2Hz rate.
 *
 *                          RED  YELLOW BLUE
 * LED_STARTED       0      OFF  OFF   OFF
 * LED_HEAPALLOCATE  0      OFF  OFF   OFF
 * LED_IRQSENABLED   0      OFF  OFF   OFF
 * LED_STACKCREATED  1      OFF  ON    OFF
 * LED_INIRQ         2      NC   NC    ON  (momentary)
 * LED_SIGNAL        2      NC   NC    ON  (momentary)
 * LED_ASSERTION     3      ON   NC    NC  (momentary)
 * LED_PANIC         4      ON   OFF   OFF (flashing 2Hz)
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
 * Name: cc3200_ledinit
 *
 * Description:
 *   Called to initialize the on-board LEDs.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Name: up_ledon
 ****************************************************************************/

void board_led_on(int led)
{
  switch (led)
    {
      /* All components stay off until the file initialization step */

      default:
      case 0:
        break;

      /* The GREEN component is illuminated at the final initialization step */

      case 1:
        cc3200_ledon(1);
        break;

      /* These will illuminate the YELLOW component with on effect no RED and GREEN */

      case 2:
        cc3200_ledon(2);
        break;

      /* This will turn off YELLOW and GREEN and turn RED on */

      case 4:
        cc3200_ledoff(1);
        cc3200_ledoff(2);

      /* This will illuminate the RED component with no effect on YELLOW and GREEN */

      case 3:
        cc3200_ledon(3);
        break;
    }
}

/****************************************************************************
 * Name: up_ledoff
 ****************************************************************************/

void board_led_off(int led)
{
  switch (led)
    {
      /* These should not happen and are ignored */

      default:
      case 0:
      case 1:
        break;

      /* These will extinguish the YELLOW component with no effect on RED and GREEN */

      case 2:
        cc3200_ledoff(2);
        break;

      /* These will extinguish the RED component with on effect on RED and GREEN */

      case 3:
      case 4:
        cc3200_ledoff(3);
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
