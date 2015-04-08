/****************************************************************************
 * configs/arduino-due/src/sam_userleds.c
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "sam_gpio.h"
#include "arduino-due.h"

#ifndef CONFIG_ARCH_LEDS

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

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

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
 * Name: sam_ledinit
 ****************************************************************************/

void sam_ledinit(void)
{
  /* Configure LED1-2 GPIOs for output */

  sam_configgpio(GPIO_LED_L);
  sam_configgpio(GPIO_LED_RX);
  sam_configgpio(GPIO_LED_TX);
}

/****************************************************************************
 * Name: sam_setled
 ****************************************************************************/

void sam_setled(int led, bool ledon)
{
  uint32_t ledcfg;

  if (led == BOARD_LED_L)
    {
      ledcfg = GPIO_LED_RX;
    }
  else if (led == BOARD_LED_RX)
    {
      ledcfg = GPIO_LED_RX;
      ledon = !ledon;
    }
  else if (led == BOARD_LED_TX)
    {
      ledcfg = GPIO_LED_TX;
      ledon = !ledon;
    }
  else
    {
      return;
    }

  sam_gpiowrite(ledcfg, ledon);
}

/****************************************************************************
 * Name: sam_setleds
 ****************************************************************************/

void sam_setleds(uint8_t ledset)
{
  bool ledon;

  ledon = ((ledset & BOARD_LED_L_BIT) != 0);
  sam_gpiowrite(GPIO_LED_L, ledon);

  ledon = ((ledset & BOARD_LED_RX_BIT) != 0);
  sam_gpiowrite(GPIO_LED_RX, ledon);

  ledon = ((ledset & BOARD_LED_TX_BIT) != 0);
  sam_gpiowrite(GPIO_LED_TX, ledon);
}

#endif /* !CONFIG_ARCH_LEDS */
