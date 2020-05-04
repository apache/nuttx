/****************************************************************************
 * boards/arm/lpc17xx_40xx/pnev5180b/src/lpc17_40_leds.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Michael Jung <mijung@gmx.net>
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
#include <assert.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "arm_internal.h"

#include "lpc17_40_gpio.h"
#include "pnev5180b.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void pnev5180b_autoled_initialize(void)
{
  /* Configure all LED GPIO lines */

  lpc17_40_configgpio(PNEV5180B_LED_RED);
  lpc17_40_configgpio(PNEV5180B_LED_ORANGE);
  lpc17_40_configgpio(PNEV5180B_LED_BLUE);
  lpc17_40_configgpio(PNEV5180B_LED_GREEN);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      case LED_STARTED:
        lpc17_40_gpiowrite(PNEV5180B_LED_RED, true);
        lpc17_40_gpiowrite(PNEV5180B_LED_ORANGE, true);
        lpc17_40_gpiowrite(PNEV5180B_LED_BLUE, true);
        lpc17_40_gpiowrite(PNEV5180B_LED_GREEN, true);
        break;

      case LED_HEAPALLOCATE:
        lpc17_40_gpiowrite(PNEV5180B_LED_RED, false);
        lpc17_40_gpiowrite(PNEV5180B_LED_ORANGE, false);
        lpc17_40_gpiowrite(PNEV5180B_LED_BLUE, false);
        lpc17_40_gpiowrite(PNEV5180B_LED_GREEN, true);
        break;

      case LED_IRQSENABLED:
        lpc17_40_gpiowrite(PNEV5180B_LED_RED, false);
        lpc17_40_gpiowrite(PNEV5180B_LED_ORANGE, false);
        lpc17_40_gpiowrite(PNEV5180B_LED_BLUE, true);
        lpc17_40_gpiowrite(PNEV5180B_LED_GREEN, false);
        break;

      case LED_STACKCREATED:
        lpc17_40_gpiowrite(PNEV5180B_LED_RED, false);
        lpc17_40_gpiowrite(PNEV5180B_LED_ORANGE, false);
        lpc17_40_gpiowrite(PNEV5180B_LED_BLUE, false);
        lpc17_40_gpiowrite(PNEV5180B_LED_GREEN, false);
        break;

      case LED_INIRQ:
        lpc17_40_gpiowrite(PNEV5180B_LED_GREEN, true);
        break;

      case LED_SIGNAL:
        lpc17_40_gpiowrite(PNEV5180B_LED_BLUE, true);
        break;

      case LED_ASSERTION:
        lpc17_40_gpiowrite(PNEV5180B_LED_ORANGE, true);
        break;

      case LED_PANIC:
        lpc17_40_gpiowrite(PNEV5180B_LED_RED, true);
        break;

      default:
        DEBUGPANIC();
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
      case LED_INIRQ:
        lpc17_40_gpiowrite(PNEV5180B_LED_GREEN, false);
        break;

      case LED_SIGNAL:
        lpc17_40_gpiowrite(PNEV5180B_LED_BLUE, false);
        break;

      case LED_ASSERTION:
        lpc17_40_gpiowrite(PNEV5180B_LED_ORANGE, false);
        break;

      case LED_PANIC:
        lpc17_40_gpiowrite(PNEV5180B_LED_RED, false);
        break;

      default:
        DEBUGPANIC();
    }
}

#endif /* CONFIG_ARCH_LEDS */
