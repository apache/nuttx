/****************************************************************************
 * configs/spark/src/up_leds.c
 *
 *   Copyright (C) 2011, 2013 Gregory Nutt. All rights reserved.
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

#include <arch/board/board.h>
#include <nuttx/power/pm.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "stm32.h"
#include "spark.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Definitions
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

static uint32_t g_ledcfg[BOARD_NLEDS] =
{
  GPIO_LED1, GPIO_LED2, GPIO_LED3, GPIO_LED4
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_led_initialize
 ****************************************************************************/

void board_led_initialize(void)
{
   /* Configure LED1-4 GPIOs for output */

   stm32_configgpio(GPIO_LED1);
   stm32_configgpio(GPIO_LED2);
   stm32_configgpio(GPIO_LED3);
   stm32_configgpio(GPIO_LED4);
}

/****************************************************************************
 * Name: up_setled
 ****************************************************************************/

void up_setled(int led, bool ledon)
{
  if ((unsigned)led < BOARD_NLEDS)
    {
      bool active_low = (LED_ACTIVE_LOW & (1 << ledon)) != 0;
      stm32_gpiowrite(g_ledcfg[led], active_low ? !ledon : ledon);
    }
}

/****************************************************************************
 * Name: up_setleds
 * Description:
 *   This function will be called to set the state of the Leds on the board
 *
 * Paramaters:
 *   ledset: is a bit set of 1s for the LEDs to effect
 *   led_states_set: a bit set of 1 for on 0 for off
 *     N.B. The led_states_set terms are in true logic, the led polarity is
 *     dealt herein
 *
 ****************************************************************************/

void up_setleds(uint8_t ledset, uint8_t led_states_set)
{
 led_states_set ^= LED_ACTIVE_LOW;
  if ((ledset & BOARD_USR_LED_BIT) == 0)
    {
      stm32_gpiowrite(GPIO_LED1, (led_states_set & BOARD_USR_LED_BIT) == 0);
    }

  if ((ledset & BOARD_RED_LED_BIT) == 0)
    {
      stm32_gpiowrite(GPIO_LED2, (led_states_set & BOARD_RED_LED_BIT) == 0);
    }

  if ((ledset & BOARD_BLUE_LED_BIT) == 0)
    {
      stm32_gpiowrite(GPIO_LED3, (led_states_set & BOARD_BLUE_LED_BIT) == 0);
    }

  if ((ledset & BOARD_GREEN_LED_BIT) == 0)
    {
      stm32_gpiowrite(GPIO_LED4, (led_states_set & BOARD_GREEN_LED_BIT) == 0);
    }
}

#endif /* !CONFIG_ARCH_LEDS */
