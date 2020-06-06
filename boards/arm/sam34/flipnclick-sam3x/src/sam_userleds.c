/****************************************************************************
 * boards/arm/sam34/flipnclick-sam3x/src/sam_userleds.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

/* There are four LEDs on the top, blue side of the board.  Only one can be
 * controlled by software:
 *
 *   LED L - PB27 (PWM13)
 *
 * There are also four LEDs on the back, white side of the board:
 *
 *   LED A - PC6
 *   LED B - PC5
 *   LED C - PC7
 *   LED D - PC8
 *
 * A high output value illuminates the LEDs.
 */

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
#include "flipnclick-sam3x.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
#ifndef CONFIG_ARCH_LEDS
  /* Configure LED GPIOs for output */

  sam_configgpio(GPIO_LED_L);
  sam_configgpio(GPIO_LED_A);
  sam_configgpio(GPIO_LED_B);
  sam_configgpio(GPIO_LED_C);
  sam_configgpio(GPIO_LED_D);
#endif
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  uint32_t ledcfg;

  switch (led)
    {
#ifndef CONFIG_ARCH_LEDS
      case BOARD_LED_L:
        ledcfg = GPIO_LED_L;
        break;
#endif

      case BOARD_LED_A:
        ledcfg = GPIO_LED_A;
        break;

      case BOARD_LED_B:
        ledcfg = GPIO_LED_B;
        break;

      case BOARD_LED_C:
        ledcfg = GPIO_LED_C;
        break;

      case BOARD_LED_D:
        ledcfg = GPIO_LED_D;
        break;

      default:
        return;
    }

  sam_gpiowrite(ledcfg, ledon);
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  bool ledon;

#ifndef CONFIG_ARCH_LEDS
  ledon = ((ledset & BOARD_LED_L_BIT) != 0);
  sam_gpiowrite(GPIO_LED_L, ledon);
#endif

  ledon = ((ledset & BOARD_LED_A_BIT) != 0);
  sam_gpiowrite(GPIO_LED_A, ledon);

  ledon = ((ledset & BOARD_LED_B_BIT) != 0);
  sam_gpiowrite(GPIO_LED_B, ledon);

  ledon = ((ledset & BOARD_LED_C_BIT) != 0);
  sam_gpiowrite(GPIO_LED_C, ledon);

  ledon = ((ledset & BOARD_LED_D_BIT) != 0);
  sam_gpiowrite(GPIO_LED_D, ledon);
}

#endif /* !CONFIG_ARCH_LEDS */
