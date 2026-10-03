/****************************************************************************
 * boards/arm/stm32n6/stm32n6570-dk/src/stm32_autoleds.c
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

#include "stm32_gpio.h"
#include "stm32n6570-dk.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 *
 * Description:
 *   Configure both user LEDs as outputs, initially off.
 *
 ****************************************************************************/

void board_autoled_initialize(void)
{
  stm32_configgpio(GPIO_LD1);
  stm32_configgpio(GPIO_LD2);
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Indicate successful startup with the green LED and panic with red.
 *   Interrupt and signal events leave the LEDs unchanged.
 *
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      case LED_STARTED:
        stm32_gpiowrite(GPIO_LD1, false);
        stm32_gpiowrite(GPIO_LD2, true);
        break;

      case LED_STACKCREATED:
        stm32_gpiowrite(GPIO_LD1, true);
        break;

      case LED_PANIC:
        stm32_gpiowrite(GPIO_LD1, false);
        stm32_gpiowrite(GPIO_LD2, false);
        break;

      default:
        break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Turn off the red LED between panic indications.
 *
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led == LED_PANIC)
    {
      stm32_gpiowrite(GPIO_LD2, true);
    }
}

#endif /* CONFIG_ARCH_LEDS */
