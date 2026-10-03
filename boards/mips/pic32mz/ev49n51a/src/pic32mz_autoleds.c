/****************************************************************************
 * boards/mips/pic32mz/ev49n51a/src/pic32mz_autoleds.c
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

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "mips_internal.h"
#include "pic32mz_gpio.h"
#include "ev49n51a.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pic32mz_led_initialize
 ****************************************************************************/

void pic32mz_led_initialize(void)
{
  pic32mz_configgpio(GPIO_LED_RED);
  pic32mz_configgpio(GPIO_LED_GREEN);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      case LED_STACKCREATED:
        pic32mz_gpiowrite(GPIO_LED_GREEN, true);
        break;

      case LED_INIRQ:
        pic32mz_gpiowrite(GPIO_LED_RED, true);
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
  if (led == LED_INIRQ)
    {
      pic32mz_gpiowrite(GPIO_LED_RED, false);
    }
}

#endif /* CONFIG_ARCH_LEDS */
