/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_leds.c
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

#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS_CPU_ACTIVITY
static inline void led_clrbits(unsigned int clrbits)
{
  if ((clrbits & BOARD_LED1_BIT) != 0)
    {
      cxd56_gpio_write(GPIO_LED1, false);
    }

  if ((clrbits & BOARD_LED2_BIT) != 0)
    {
      cxd56_gpio_write(GPIO_LED2, false);
    }

  if ((clrbits & BOARD_LED3_BIT) != 0)
    {
      cxd56_gpio_write(GPIO_LED3, false);
    }

  if ((clrbits & BOARD_LED4_BIT) != 0)
    {
      cxd56_gpio_write(GPIO_LED4, false);
    }
}

static inline void led_setbits(unsigned int setbits)
{
  if ((setbits & BOARD_LED1_BIT) != 0)
    {
      cxd56_gpio_write(GPIO_LED1, true);
    }

  if ((setbits & BOARD_LED2_BIT) != 0)
    {
      cxd56_gpio_write(GPIO_LED2, true);
    }

  if ((setbits & BOARD_LED3_BIT) != 0)
    {
      cxd56_gpio_write(GPIO_LED3, true);
    }

  if ((setbits & BOARD_LED4_BIT) != 0)
    {
      cxd56_gpio_write(GPIO_LED4, true);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  cxd56_gpio_config(GPIO_LED1, false);
  cxd56_gpio_config(GPIO_LED2, false);
  cxd56_gpio_config(GPIO_LED3, false);
  cxd56_gpio_config(GPIO_LED4, false);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
#ifdef CONFIG_ARCH_LEDS_CPU_ACTIVITY
  switch (led)
    {
      case LED_CPU0:
        cxd56_gpio_write(GPIO_LED1, 1);
        break;

      case LED_CPU1:
        cxd56_gpio_write(GPIO_LED2, 1);
        break;

      case LED_CPU2:
        cxd56_gpio_write(GPIO_LED3, 1);
        break;

      case LED_CPU3:
        cxd56_gpio_write(GPIO_LED4, 1);
        break;

      default:
        break;
    }
#else
  led_clrbits(BOARD_LED1_BIT | BOARD_LED2_BIT |
              BOARD_LED3_BIT | BOARD_LED4_BIT);
  led_setbits(led);
#endif
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
#ifdef CONFIG_ARCH_LEDS_CPU_ACTIVITY
  switch (led)
    {
      case LED_CPU0:
        cxd56_gpio_write(GPIO_LED1, 0);
        break;

      case LED_CPU1:
        cxd56_gpio_write(GPIO_LED2, 0);
        break;

      case LED_CPU2:
        cxd56_gpio_write(GPIO_LED3, 0);
        break;

      case LED_CPU3:
        cxd56_gpio_write(GPIO_LED4, 0);
        break;

      default:
        break;
    }
#else
  led_clrbits(led);
#endif
}

#endif /* CONFIG_ARCH_LEDS */
