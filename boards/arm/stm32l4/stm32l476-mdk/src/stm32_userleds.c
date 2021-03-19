/****************************************************************************
 * boards/arm/stm32l4/stm32l476-mdk/src/stm32_userleds.c
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

#include "stm32l4_gpio.h"
#include "stm32l476-mdk.h"

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

  stm32l4_configgpio(GPIO_LED_RED);
  stm32l4_configgpio(GPIO_LED_GREEN);
  stm32l4_configgpio(GPIO_LED_WHITE);
#endif
  return BOARD_NLEDS;
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

void board_userled_all(uint32_t ledset)
{
  /* Low illuminates */

  stm32l4_gpiowrite(GPIO_LED_RED,   (ledset & BOARD_RED_LED_BIT)   == 0);
  stm32l4_gpiowrite(GPIO_LED_GREEN, (ledset & BOARD_GREEN_LED_BIT) == 0);
#ifndef CONFIG_ARCH_LEDS
  stm32l4_gpiowrite(GPIO_LED_WHITE, (ledset & BOARD_WHITE_LED_BIT) == 0);
#endif
}
