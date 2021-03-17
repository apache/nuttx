/****************************************************************************
 * boards/arm/tiva/tm4c1294-launchpad/src/tm4c_userleds.c
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
