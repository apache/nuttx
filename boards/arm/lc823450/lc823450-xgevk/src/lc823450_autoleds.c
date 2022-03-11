/****************************************************************************
 * boards/arm/lc823450/lc823450-xgevk/src/lc823450_autoleds.c
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

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "lc823450_gpio.h"
#include "lc823450-xgevk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LED0_PIN (GPIO_PORT0 | GPIO_PIN0)
#define LED1_PIN (GPIO_PORT2 | GPIO_PINF)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      case LED_CPU0:
        lc823450_gpio_write(LED0_PIN, 1);
        break;

      case LED_CPU1:
        lc823450_gpio_write(LED1_PIN, 1);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
      case LED_CPU0:
        lc823450_gpio_write(LED0_PIN, 0);
        break;

      case LED_CPU1:
        lc823450_gpio_write(LED1_PIN, 0);
        break;

      default:
        break;
    }
}
