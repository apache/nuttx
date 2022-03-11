/****************************************************************************
 * boards/arm/stm32/nucleo-f412zg/src/stm32_autoleds.c
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
#include "nucleo-f412zg.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  stm32_configgpio(GPIO_LD1);
  stm32_configgpio(GPIO_LD2);
  stm32_configgpio(GPIO_LD3);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
    switch (led)
    {
    case 1:
        stm32_gpiowrite(GPIO_LD1, true);
        break;
    case 2:
        stm32_gpiowrite(GPIO_LD1, true);
        stm32_gpiowrite(GPIO_LD2, true);
        break;
    case 3:
        stm32_gpiowrite(GPIO_LD1, true);
        stm32_gpiowrite(GPIO_LD2, true);
        stm32_gpiowrite(GPIO_LD3, true);
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
    case 1:
        stm32_gpiowrite(GPIO_LD1, false);
        break;
    case 2:
        stm32_gpiowrite(GPIO_LD1, false);
        stm32_gpiowrite(GPIO_LD2, false);
        break;
    case 3:
        stm32_gpiowrite(GPIO_LD1, false);
        stm32_gpiowrite(GPIO_LD2, false);
        stm32_gpiowrite(GPIO_LD3, false);
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
