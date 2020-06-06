/****************************************************************************
 * boards/arm/tiva/tm4c1294-launchpad/src/tm4c_userleds.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

/* The EK-TM4C1294XL has a four green LEDs.
 *
 *   --- ------------
 *   Pin Pin Function
 *   --- ------------
 *   PN1 Green LED D1
 *   PN0 Green LED D2
 *   PF4 Green LED D3
 *   PF0 Green LED D4
 *   --- ------------
 *
 * A high output illuminates the LED.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "tiva_gpio.h"
#include "tm4c1294-launchpad.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure LED PIOs for output */

  tiva_configgpio(GPIO_LED_D1);
  tiva_configgpio(GPIO_LED_D2);
  tiva_configgpio(GPIO_LED_D3);
  tiva_configgpio(GPIO_LED_D4);
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  uint32_t ledcfg;

  if (led == BOARD_LED_D1)
    {
      ledcfg = GPIO_LED_D1;
    }
  else if (led == BOARD_LED_D2)
    {
      ledcfg = GPIO_LED_D2;
    }
  else if (led == BOARD_LED_D3)
    {
      ledcfg = GPIO_LED_D3;
    }
  else if (led == BOARD_LED_D4)
    {
      ledcfg = GPIO_LED_D4;
    }
  else
    {
      return;
    }

  tiva_gpiowrite(ledcfg, ledon);
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  bool ledon;

  ledon = ((ledset & BOARD_LED_D1_BIT) != 0);
  tiva_gpiowrite(GPIO_LED_D1, ledon);

  ledon = ((ledset & BOARD_LED_D2_BIT) != 0);
  tiva_gpiowrite(GPIO_LED_D2, ledon);

  ledon = ((ledset & BOARD_LED_D3_BIT) != 0);
  tiva_gpiowrite(GPIO_LED_D3, ledon);

  ledon = ((ledset & BOARD_LED_D4_BIT) != 0);
  tiva_gpiowrite(GPIO_LED_D4, ledon);
}

#endif /* !CONFIG_ARCH_LEDS */
