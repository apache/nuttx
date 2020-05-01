/****************************************************************************
 * boards/arm/tiva/lm3s6432-s2e/src/lm_leds.c
 *
 *   Copyright (C) 2010, 2013, 2015 Gregory Nutt. All rights reserved.
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

#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"
#include "tiva_gpio.h"
#include "lm3s6432-s2e.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define led_dumpgpio(m) tiva_dumpgpio(LED_GPIO, m)
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

static uint8_t g_nest;

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_initialize(void)
{
  ledinfo("Initializing\n");

  /* Configure Port F, Bit 2 as an output, initial value=OFF */

  led_dumpgpio("board_autoled_initialize before tiva_configgpio()");
  tiva_configgpio(LED0_GPIO);
  tiva_configgpio(LED1_GPIO);
  led_dumpgpio("board_autoled_initialize after tiva_configgpio()");
  g_nest = 0;
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      default:
        break;

      case LED_INIRQ:
      case LED_SIGNAL:
      case LED_ASSERTION:
      case LED_PANIC:
        g_nest++;
      case LED_IRQSENABLED:
      case LED_STACKCREATED:
        led_dumpgpio("board_autoled_on: before tiva_gpiowrite()");
        tiva_gpiowrite(LED1_GPIO, false);
        led_dumpgpio("board_autoled_on: after tiva_gpiowrite()");
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
      case LED_IRQSENABLED:
      case LED_STACKCREATED:
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      default:
        break;

      case LED_INIRQ:
      case LED_SIGNAL:
      case LED_ASSERTION:
      case LED_PANIC:
        if (--g_nest <= 0)
          {
            led_dumpgpio("board_autoled_off: before tiva_gpiowrite()");
            tiva_gpiowrite(LED1_GPIO, true);
            led_dumpgpio("board_autoled_off: after tiva_gpiowrite()");
          }
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
