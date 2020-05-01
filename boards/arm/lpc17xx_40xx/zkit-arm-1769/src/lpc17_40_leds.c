/****************************************************************************
 * boards/arm/lpc17xx_40xx/zkit-arm-1769/src/lpc17_40_leds.c
 *
 *   Copyright (C) 2013 Zilogic Systems. All rights reserved.
 *   Author: BabuSubashChandar <code@zilogic.com>
 *
 * Based on boards/lpcxpresso-lpc1768/src/lpc17_40_leds.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "arm_internal.h"

#include "lpc17_40_gpio.h"
#include "zkit-arm-1769.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* LED definitions **********************************************************/

/* The ZKit-ARM-1769 has 2 LEDs along the bottom of the board. Red or off.
 * If CONFIG_ARCH_LEDS is defined, the LEDs will be controlled as follows for
 * NuttX debug functionality (where NC means "No Change").
 *
 * During the boot phases.  LED1 and LED2 will show boot status.
 *
 *                LED1   LED2
 * STARTED         OFF    OFF
 * HEAPALLOCATE   BLUE    OFF
 * IRQSENABLED     OFF   BLUE
 * STACKCREATED    OFF    OFF
 *
 * After the system is booted, this logic will no longer use LED 1.  LED 1
 * is available for use by applications using lpc17_40_led (prototyped below)
 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_initialized;
static int  g_nestcount;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure all LED GPIO lines */

  lpc17_40_configgpio(ZKITARM_LED1);
  lpc17_40_configgpio(ZKITARM_LED2);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  /* We will control LED1 and LED2 not yet completed the boot sequence. */

  if (!g_initialized)
    {
      bool led1 = false;
      bool led2 = false;

      switch (led)
        {
        case LED_STACKCREATED:
          g_initialized = true;
        case LED_STARTED:
        default:
          break;

        case LED_HEAPALLOCATE:
          led1 = true;
          break;

        case LED_IRQSENABLED:
          led2 = true;
          break;
        }

      lpc17_40_gpiowrite(ZKITARM_LED1, led1);
      lpc17_40_gpiowrite(ZKITARM_LED2, led2);
    }

  /* We will always control the HB LED */

  switch (led)
    {
    case LED_INIRQ:
    case LED_SIGNAL:
    case LED_ASSERTION:
    case LED_PANIC:
      lpc17_40_gpiowrite(ZKITARM_LED2, false);
      g_nestcount++;

    default:
      break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  /* In all states, OFF can only mean turning off the HB LED */

  if (g_nestcount <= 1)
    {
      lpc17_40_gpiowrite(ZKITARM_LED2, true);
      g_nestcount = 0;
    }
  else
    {
      g_nestcount--;
    }
}

/****************************************************************************
 * Name: lpc17_40_led
 *
 * Description:
 *   Once the system has booted, this functions can be used to control LED 1
 *
 ****************************************************************************/

void lpc17_40_led(int lednum, int state)
{
  lpc17_40_gpiowrite(ZKITARM_LED1, state);
}

#endif /* CONFIG_ARCH_LEDS */
