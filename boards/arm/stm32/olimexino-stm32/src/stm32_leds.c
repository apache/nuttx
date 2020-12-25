/****************************************************************************
 * boards/arm/stm32/olimexino-stm32/src/stm32_leds.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           David Sidrane <david_s5@nscdg.com>
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "stm32.h"
#include "olimexino-stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Dump GPIO registers */

#ifdef CONFIG_DEBUG_LEDS_INFO
#  define led_dumpgpio(m) stm32_dumpgpio(GPIO_LED_GREEN, m)
#else
#  define led_dumpgpio(m)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
static bool g_initialized = false;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_led_initialize/board_userled_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
uint32_t stm32_led_initialize(void)
#else
uint32_t board_userled_initialize(void)
#endif
{
  /* Configure all LED GPIO lines */

  led_dumpgpio("board_*led_initialize() Entry)");

  stm32_configgpio(GPIO_LED_YELLOW);
  stm32_configgpio(GPIO_LED_GREEN);

  led_dumpgpio("board_*led_initialize() Exit");
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void board_userled(int led, bool ledon)
{
  if (led == BOARD_LED_GREEN)
    {
      stm32_gpiowrite(GPIO_LED_GREEN, !ledon);
    }
  else if (led == BOARD_LED_YELLOW)
    {
      stm32_gpiowrite(GPIO_LED_YELLOW, !ledon);
    }
}
#endif

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void board_userled_all(uint32_t ledset)
{
  stm32_gpiowrite(GPIO_LED_GREEN, (ledset & BOARD_LED_YELLOW_BIT) == 0);
  stm32_gpiowrite(GPIO_LED_YELLOW, (ledset & BOARD_LED_YELLOW_BIT) == 0);
}
#endif

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_on(int led)
{
  switch (led)
    {
      default:
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      case LED_IRQSENABLED:
        stm32_gpiowrite(GPIO_LED_GREEN, false);
        stm32_gpiowrite(GPIO_LED_YELLOW, false);
        break;

      case LED_STACKCREATED:
        stm32_gpiowrite(GPIO_LED_GREEN, true);
        stm32_gpiowrite(GPIO_LED_YELLOW, false);
        g_initialized = true;
        break;

      case LED_INIRQ:
      case LED_SIGNAL:
      case LED_ASSERTION:
      case LED_PANIC:
        stm32_gpiowrite(GPIO_LED_YELLOW, true);
        break;

      case LED_IDLE : /* IDLE */
        stm32_gpiowrite(GPIO_LED_GREEN, false);
        break;
    }
}
#endif

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_off(int led)
{
  switch (led)
    {
      default:
      case LED_STARTED:
      case LED_HEAPALLOCATE:
      case LED_IRQSENABLED:
      case LED_STACKCREATED:
        stm32_gpiowrite(GPIO_LED_GREEN, false);

      case LED_INIRQ:
      case LED_SIGNAL:
      case LED_ASSERTION:
      case LED_PANIC:
        stm32_gpiowrite(GPIO_LED_YELLOW, false);
        break;

      case LED_IDLE: /* IDLE */
        stm32_gpiowrite(GPIO_LED_GREEN, g_initialized);
        break;
    }
}
#endif
