/****************************************************************************
 * boards/arm/sam34/arduino-due/src/sam_userleds.c
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

#include <arch/board/board.h>

#include "chip.h"
#include "sam_gpio.h"
#include "arduino-due.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure LED1-2 GPIOs for output */

  sam_configgpio(GPIO_LED_L);
  sam_configgpio(GPIO_LED_RX);
  sam_configgpio(GPIO_LED_TX);
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
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
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
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
