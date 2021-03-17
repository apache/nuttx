/****************************************************************************
 * boards/arm/tiva/dk-tm4c129x/src/tm4c_userleds.c
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

/* The development board has one tri-color user LED.
 *
 *   --- ------------ -----------------
 *   Pin Pin Function Jumper
 *   --- ------------ -----------------
 *   PN5 Red LED      J36 pins 1 and 2
 *   PQ4 Blue LED     J36 pins 3 and 4
 *   PQ7 Green LED    J36 pins 5 and 6
 *   --- ------------ -----------------
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
#include "dk-tm4c129x.h"

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

  tiva_configgpio(GPIO_LED_R);
  tiva_configgpio(GPIO_LED_G);
  tiva_configgpio(GPIO_LED_B);
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  uint32_t ledcfg;

  if (led == BOARD_LED_R)
    {
      ledcfg = GPIO_LED_R;
    }
  else if (led == BOARD_LED_B)
    {
      ledcfg = GPIO_LED_B;
    }
  else if (led == BOARD_LED_G)
    {
      ledcfg = GPIO_LED_G;
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

  ledon = ((ledset & BOARD_LED_R_BIT) != 0);
  tiva_gpiowrite(GPIO_LED_R, ledon);

  ledon = ((ledset & BOARD_LED_G_BIT) != 0);
  tiva_gpiowrite(GPIO_LED_G, ledon);

  ledon = ((ledset & BOARD_LED_B_BIT) != 0);
  tiva_gpiowrite(GPIO_LED_B, ledon);
}

#endif /* !CONFIG_ARCH_LEDS */
