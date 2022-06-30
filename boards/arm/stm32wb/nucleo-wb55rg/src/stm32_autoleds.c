/****************************************************************************
 * boards/arm/stm32wb/nucleo-wb55rg/src/stm32_autoleds.c
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

#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32wb.h"
#include "nucleo-wb55rg.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LEDs GPIO for output. Initial state is OFF */

  stm32wb_configgpio(GPIO_LED1);
  stm32wb_configgpio(GPIO_LED2);
  stm32wb_configgpio(GPIO_LED3);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      default:
        break;

      case BOARD_LED1:
        stm32wb_gpiowrite(GPIO_LED1, true);
        break;

      case BOARD_LED2:
        stm32wb_gpiowrite(GPIO_LED2, true);
        break;

      case BOARD_LED3:
        stm32wb_gpiowrite(GPIO_LED3, true);
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
      default:
        break;

      case BOARD_LED1:
        stm32wb_gpiowrite(GPIO_LED1, false);
        break;

      case BOARD_LED2:
        stm32wb_gpiowrite(GPIO_LED2, false);
        break;

      case BOARD_LED3:
        stm32wb_gpiowrite(GPIO_LED3, false);
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
