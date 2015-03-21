/****************************************************************************
 * configs/stm32f3discovery/src/stm32_autoleds.c
 *
 *   Copyright (C) 2013, 2015 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "stm32.h"
#include "stm32f3discovery-internal.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array maps an LED number to GPIO pin configuration */

static const uint32_t g_ledcfg[BOARD_NLEDS] =
{
  GPIO_LED1, GPIO_LED2, GPIO_LED3, GPIO_LED4,
  GPIO_LED5, GPIO_LED6, GPIO_LED7, GPIO_LED8
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_led_onoff
 ****************************************************************************/

void board_led_onoff(int led, bool state)
{
  if ((unsigned)led < BOARD_NLEDS)
    {
      stm32_gpiowrite(g_ledcfg[led], state);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_led_initialize
 ****************************************************************************/

void board_led_initialize(void)
{
  int i;

  /* Configure LED1-8 GPIOs for output */

  for (i = 0; i < BOARD_NLEDS; i++)
    {
      stm32_configgpio(g_ledcfg[i]);
    }
}

/****************************************************************************
 * Name: board_led_on
 ****************************************************************************/

void board_led_on(int led)
{
  board_led_onoff(led, true);
}

/****************************************************************************
 * Name: board_led_off
 ****************************************************************************/

void board_led_off(int led)
{
  board_led_onoff(led, false);
}

#endif /* CONFIG_ARCH_LEDS */
