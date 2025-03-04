/****************************************************************************
 * boards/arm/ra4/arduino-r4-minima/src/ra4m1_userleds.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include "chip.h"
#include "ra_gpio.h"
#include "arduino-r4-minima.h"

#include <arch/board/board.h>

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure LED GPIOs for output */

  ra_configgpio(GPIO_L_LED);
  ra_configgpio(GPIO_RX_LED);
  ra_configgpio(GPIO_TX_LED);
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  gpio_pinset_t ledcfg;

  if (led == BOARD_LED_L)
    {
      ledcfg = GPIO_L_LED;
      ledon = ledon;
    }
  else if (led == BOARD_LED_RX)
    {
      ledcfg = GPIO_RX_LED;
      ledon = !ledon;
    }
  else if (led == BOARD_LED_TX)
    {
      ledcfg = GPIO_TX_LED;
      ledon = !ledon;
    }
  else
    {
      return;
    }

  ra_gpiowrite(ledcfg, ledon);
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  bool ledon;

  ledon = ((ledset & BOARD_LED_L_BIT) != 0);
  ra_gpiowrite(GPIO_L_LED, ledon);

  ledon = ((ledset & BOARD_LED_RX_BIT) != 0);
  ra_gpiowrite(GPIO_RX_LED, ledon);

  ledon = ((ledset & BOARD_LED_TX_BIT) != 0);
  ra_gpiowrite(GPIO_TX_LED, ledon);
}

#endif /* !CONFIG_ARCH_LEDS */
