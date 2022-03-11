/****************************************************************************
 * boards/mips/pic32mz/chipkit-wifire/src/pic32mz_userleds.c
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

/* There are four LEDs on the top side of the board:
 *
 *   LED LD1      - RG6
 *   LED LD2      - RD4
 *   LED LD3      - RB11
 *   LED LD4      - RG15
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

#include "mips_internal.h"
#include "pic32mz_gpio.h"
#include "chipkit-wifire.h"

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

  pic32mz_configgpio(GPIO_LED_LD1);
  pic32mz_configgpio(GPIO_LED_LD2);
  pic32mz_configgpio(GPIO_LED_LD3);
  pic32mz_configgpio(GPIO_LED_LD4);
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
      case BOARD_LED_LD1:
        ledcfg = GPIO_LED_LD1;
        break;

      case BOARD_LED_LD2:
        ledcfg = GPIO_LED_LD2;
        break;

      case BOARD_LED_LD3:
        ledcfg = GPIO_LED_LD3;
        break;

      case BOARD_LED_LD4:
        ledcfg = GPIO_LED_LD4;
        break;

      default:
        return;
    }

  pic32mz_gpiowrite(ledcfg, ledon);
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  bool ledon;

  ledon = ((ledset & BOARD_LED_LD1_BIT) != 0);
  pic32mz_gpiowrite(GPIO_LED_LD1, ledon);

  ledon = ((ledset & BOARD_LED_LD2_BIT) != 0);
  pic32mz_gpiowrite(GPIO_LED_LD2, ledon);

  ledon = ((ledset & BOARD_LED_LD3_BIT) != 0);
  pic32mz_gpiowrite(GPIO_LED_LD3, ledon);

  ledon = ((ledset & BOARD_LED_LD4_BIT) != 0);
  pic32mz_gpiowrite(GPIO_LED_LD4, ledon);
}

#endif /* !CONFIG_ARCH_LEDS */
