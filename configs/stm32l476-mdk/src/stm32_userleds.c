/****************************************************************************
 * configs/stm32l476-mdk/src/sam_userleds.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#include "stm32l4_gpio.h"
#include "stm32l476-mdk.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

void board_userled_initialize(void)
{
#ifndef CONFIG_ARCH_LEDS
  /* Configure LED GPIOs for output */

  stm32l4_configgpio(GPIO_LED_RED);
  stm32l4_configgpio(GPIO_LED_GREEN);
  stm32l4_configgpio(GPIO_LED_WHITE);
#endif
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if (led == BOARD_RED_LED)
    {
      stm32l4_gpiowrite(GPIO_LED_RED, !ledon); /* Low illuminates */
    }
  else if (led == BOARD_GREEN_LED)
    {
      stm32l4_gpiowrite(GPIO_LED_GREEN, !ledon); /* Low illuminates */
    }
#ifndef CONFIG_ARCH_LEDS
  else if (led == BOARD_WHITE_LED)
    {
      stm32l4_gpiowrite(GPIO_LED_WHITE, !ledon); /* Low illuminates */
    }
#endif
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint8_t ledset)
{
  /* Low illuminates */

  stm32l4_gpiowrite(GPIO_LED_RED,   (ledset & BOARD_RED_LED_BIT)   == 0);
  stm32l4_gpiowrite(GPIO_LED_GREEN, (ledset & BOARD_GREEN_LED_BIT) == 0);
#ifndef CONFIG_ARCH_LEDS
  stm32l4_gpiowrite(GPIO_LED_WHITE, (ledset & BOARD_WHITE_LED_BIT) == 0);
#endif
}
