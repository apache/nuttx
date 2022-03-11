/****************************************************************************
 * boards/arm/stm32/olimex-stm32-h407/src/stm32_autoleds.c
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
#include "stm32.h"
#include "olimex-stm32-h407.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED_STATUS GPIO for output */

  stm32_configgpio(GPIO_LED_STATUS);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (led == LED_STARTED)
    {
      stm32_gpiowrite(GPIO_LED_STATUS, true);
    }

  if (led == LED_ASSERTION || led == LED_PANIC)
    {
      stm32_gpiowrite(GPIO_LED_STATUS, false);
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led == LED_STARTED)
    {
      stm32_gpiowrite(GPIO_LED_STATUS, false);
    }

  if (led == LED_ASSERTION || led == LED_PANIC)
    {
      stm32_gpiowrite(GPIO_LED_STATUS, true);
    }
}

#endif /* CONFIG_ARCH_LEDS */
