/****************************************************************************
 * boards/arm/mx8mp/verdin-mx8mp/src/mx8mp_autoleds.c
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

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "mx8mp_gpio.h"
#include "verdin-mx8mp.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  mx8mp_iomuxc_config(IOMUX_LED_1);
  mx8mp_iomuxc_config(IOMUX_LED_2);
  mx8mp_iomuxc_config(IOMUX_LED_3);
  mx8mp_iomuxc_config(IOMUX_LED_4);

  mx8mp_gpio_config(GPIO_LED_1);
  mx8mp_gpio_config(GPIO_LED_2);
  mx8mp_gpio_config(GPIO_LED_3);
  mx8mp_gpio_config(GPIO_LED_4);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      case LED_HEAPALLOCATE:
        {
          mx8mp_gpio_write(GPIO_LED_1, true);
          mx8mp_gpio_write(GPIO_LED_2, false);
        }
        break;

      case LED_IRQSENABLED:
        {
          mx8mp_gpio_write(GPIO_LED_1, false);
          mx8mp_gpio_write(GPIO_LED_2, true);
        }
        break;

      case LED_STACKCREATED:
        {
          mx8mp_gpio_write(GPIO_LED_1, true);
          mx8mp_gpio_write(GPIO_LED_2, true);
        }
        break;

      case LED_ASSERTION:
        {
          mx8mp_gpio_write(GPIO_LED_3, true);
        }
        break;

      case LED_PANIC:
        {
          mx8mp_gpio_write(GPIO_LED_1, true);
          mx8mp_gpio_write(GPIO_LED_2, true);
        }
        break;

      case LED_IDLE:
        {
          mx8mp_gpio_write(GPIO_LED_4, true);
        }
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
      case LED_PANIC:
        {
          mx8mp_gpio_write(GPIO_LED_1, false);
          mx8mp_gpio_write(GPIO_LED_2, false);
        }
        break;

      case LED_IDLE:
        {
          mx8mp_gpio_write(GPIO_LED_4, false);
        }
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
