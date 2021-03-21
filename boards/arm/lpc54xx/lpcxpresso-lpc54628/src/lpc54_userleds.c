/****************************************************************************
 * boards/arm/lpc54xx/lpcxpresso-lpc54628/src/lpc54_userleds.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* The LPCXpress-LPC54628 has three user LEDs: D9, D11, and D12.  These
 * LEDs are for application use. They are illuminated when the driving
 * signal from the LPC546xx is low. The LEDs are driven by ports P2-2 (D9),
 * P3-3 (D11) and P3-14 (D12).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "lpc54_gpio.h"
#include "lpcxpresso-lpc54628.h"

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

  lpc54_gpio_config(GPIO_LED_D9);
  lpc54_gpio_config(GPIO_LED_D11);
  lpc54_gpio_config(GPIO_LED_D12);
#endif
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if (led == BOARD_D9)
    {
      lpc54_gpio_write(GPIO_LED_D9, !ledon); /* Low illuminates */
    }
  else if (led == BOARD_D11)
    {
      lpc54_gpio_write(GPIO_LED_D11, !ledon); /* Low illuminates */
    }
#ifndef CONFIG_ARCH_LEDS
  else if (led == BOARD_D12)
    {
      lpc54_gpio_write(GPIO_LED_D12, !ledon); /* Low illuminates */
    }
#endif
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  /* Low illuminates */

  lpc54_gpio_write(GPIO_LED_D9,  (ledset & BOARD_D9_BIT)  == 0);
  lpc54_gpio_write(GPIO_LED_D11, (ledset & BOARD_D11_BIT) == 0);
#ifndef CONFIG_ARCH_LEDS
  lpc54_gpio_write(GPIO_LED_D12, (ledset & BOARD_D12_BIT) == 0);
#endif
}
