/****************************************************************************
 * boards/mips/pic32mz/flipnclick-pic32mz/src/pic32mz_userleds.c
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

/* There are four LEDs on the top, red side of the board.  Only one can be
 * controlled by software:
 *
 *   LED L      - RB14 (SPI3_SCK)
 *
 * There are also four LEDs on the back, white side of the board:
 *
 *   LED A      - RA6
 *   LED B      - RA7
 *   LED C      - RE0
 *   LED D      - RE1
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

#include "mips_arch.h"
#include "mips_internal.h"

#include "pic32mz_gpio.h"
#include "flipnclick-pic32mz.h"

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

  pic32mz_configgpio(GPIO_LED_L);
  pic32mz_configgpio(GPIO_LED_A);
  pic32mz_configgpio(GPIO_LED_B);
  pic32mz_configgpio(GPIO_LED_C);
  pic32mz_configgpio(GPIO_LED_D);
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
#ifndef CONFIG_ARCH_LEDS
      case BOARD_LED_L:
        ledcfg = GPIO_LED_L;
        break;
#endif

      case BOARD_LED_A:
        ledcfg = GPIO_LED_A;
        break;

      case BOARD_LED_B:
        ledcfg = GPIO_LED_B;
        break;

      case BOARD_LED_C:
        ledcfg = GPIO_LED_C;
        break;

      case BOARD_LED_D:
        ledcfg = GPIO_LED_D;
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

#ifndef CONFIG_ARCH_LEDS
  ledon = ((ledset & BOARD_LED_L_BIT) != 0);
  pic32mz_gpiowrite(GPIO_LED_L, ledon);
#endif

  ledon = ((ledset & BOARD_LED_A_BIT) != 0);
  pic32mz_gpiowrite(GPIO_LED_A, ledon);

  ledon = ((ledset & BOARD_LED_B_BIT) != 0);
  pic32mz_gpiowrite(GPIO_LED_B, ledon);

  ledon = ((ledset & BOARD_LED_C_BIT) != 0);
  pic32mz_gpiowrite(GPIO_LED_C, ledon);

  ledon = ((ledset & BOARD_LED_D_BIT) != 0);
  pic32mz_gpiowrite(GPIO_LED_D, ledon);
}

#endif /* !CONFIG_ARCH_LEDS */
