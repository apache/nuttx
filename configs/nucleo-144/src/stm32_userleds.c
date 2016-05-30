/****************************************************************************
 * configs/nucleo-144/src/stm32_userleds.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Author: Mark Olsson <post@markolsson.se>
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

#include "stm32_gpio.h"
#include "nucleo-144.h"

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

/* This array maps an LED number to GPIO pin configuration */

static const uint32_t g_ledcfg[3] =
{
  GPIO_LD1, GPIO_LD2, GPIO_LD3
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then the
 *   board_userled_initialize() is available to initialize the LED from user
 *   application logic.
 *
 ****************************************************************************/

void board_userled_initialize(void)
{
  int i;

  /* Configure LED1-3 GPIOs for output */

  for (i = 0; i < 3; i++)
    {
      stm32_configgpio(g_ledcfg[i]);
    }
}

/****************************************************************************
 * Name: board_userled
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *  LEDs.  If CONFIG_ARCH_LEDS is not defined, then the board_userled() is
 *  available to control the LED from user application logic.
 *
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if ((unsigned)led < 3)
  {
    stm32_gpiowrite(g_ledcfg[led], ledon);
  }
}

/****************************************************************************
 * Name: board_userled_all
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *  LEDs.  If CONFIG_ARCH_LEDS is not defined, then the board_userled_all() is
 *  available to control the LED from user application logic.  NOTE:  since
 *  there is only a single LED on-board, this is function is not very useful.
 *
 ****************************************************************************/

void board_userled_all(uint8_t ledset)
{
  int i;

  /* Configure LED1-3 GPIOs for output */

  for (i = 0; i < 3; i++)
    {
      stm32_gpiowrite(g_ledcfg[i], (ledset & (1 << i)) != 0);
    }
}

#endif /* !CONFIG_ARCH_LEDS */
