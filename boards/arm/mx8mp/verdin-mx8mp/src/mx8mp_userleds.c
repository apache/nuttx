/****************************************************************************
 * boards/arm/mx8mp/verdin-mx8mp/src/mx8mp_userleds.c
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

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "verdin-mx8mp.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: led_dumppins
 ****************************************************************************/

#ifdef LED_VERBOSE
static void led_dumppins(const char *msg)
{
  #warning Missing logic
}
#else
#  define led_dumppins(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  mx8mp_iomuxc_config(IOMUX_LED_1);
  mx8mp_iomuxc_config(IOMUX_LED_2);
  mx8mp_iomuxc_config(IOMUX_LED_3);
  mx8mp_iomuxc_config(IOMUX_LED_4);

  mx8mp_gpio_config(GPIO_LED_1);
  mx8mp_gpio_config(GPIO_LED_2);
  mx8mp_gpio_config(GPIO_LED_3);
  mx8mp_gpio_config(GPIO_LED_4);

  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool on)
{
  gpio_pinset_t gpio;

  switch (led)
    {
      case BOARD_LED_1:
        {
          gpio = GPIO_LED_1;
        }
      break;

      case BOARD_LED_2:
        {
          gpio = GPIO_LED_2;
        }
      break;

      case BOARD_LED_3:
        {
          gpio = GPIO_LED_3;
        }
      break;

      case BOARD_LED_4:
        {
          gpio = GPIO_LED_4;
        }
      break;

      default:
        return;
    }

    mx8mp_gpio_write(gpio, on);
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  mx8mp_gpio_write(GPIO_LED_1, (ledset & BOARD_LED_1_BIT) == 0);
  mx8mp_gpio_write(GPIO_LED_2, (ledset & BOARD_LED_2_BIT) == 0);
  mx8mp_gpio_write(GPIO_LED_3, (ledset & BOARD_LED_3_BIT) == 0);
  mx8mp_gpio_write(GPIO_LED_4, (ledset & BOARD_LED_4_BIT) == 0);
}

#endif /* !CONFIG_ARCH_LEDS */
