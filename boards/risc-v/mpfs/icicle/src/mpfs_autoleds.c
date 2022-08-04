/****************************************************************************
 * boards/risc-v/mpfs/icicle/src/mpfs_autoleds.c
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
#include <mpfs_gpio.h>
#include "board_config.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 *
 * Description:
 *    Init the LEDs.
 *
 ****************************************************************************/

void board_autoled_initialize(void)
{
  mpfs_configgpio(ICICLE_GPIO_LED1);
  mpfs_configgpio(ICICLE_GPIO_LED2);
  mpfs_configgpio(ICICLE_GPIO_LED3);
  mpfs_configgpio(ICICLE_GPIO_LED4);
  mpfs_gpiowrite(ICICLE_GPIO_LED1, false);
  mpfs_gpiowrite(ICICLE_GPIO_LED2, false);
  mpfs_gpiowrite(ICICLE_GPIO_LED3, false);
  mpfs_gpiowrite(ICICLE_GPIO_LED4, false);
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *    Turn on the LED specificed.
 *
 * Input Parameters:
 *   led - The LED which is under this control
 *
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
    case LED_STARTED:
      mpfs_gpiowrite(ICICLE_GPIO_LED1, true);
      mpfs_gpiowrite(ICICLE_GPIO_LED2, false);
      mpfs_gpiowrite(ICICLE_GPIO_LED3, false);
      mpfs_gpiowrite(ICICLE_GPIO_LED4, false);
      break;
    case LED_HEAPALLOCATE:
      mpfs_gpiowrite(ICICLE_GPIO_LED2, true);
      break;
    case LED_IRQSENABLED:
      mpfs_gpiowrite(ICICLE_GPIO_LED1, true);
      mpfs_gpiowrite(ICICLE_GPIO_LED2, true);
      break;
    case LED_STACKCREATED:
      mpfs_gpiowrite(ICICLE_GPIO_LED3, true);
      mpfs_gpiowrite(ICICLE_GPIO_LED1, false);
      mpfs_gpiowrite(ICICLE_GPIO_LED2, false);
      break;
    case LED_INIRQ:
      mpfs_gpiowrite(ICICLE_GPIO_LED1, true);
      mpfs_gpiowrite(ICICLE_GPIO_LED2, false);
      mpfs_gpiowrite(ICICLE_GPIO_LED3, false);
      break;
    case LED_SIGNAL:
      mpfs_gpiowrite(ICICLE_GPIO_LED3, true);
      mpfs_gpiowrite(ICICLE_GPIO_LED1, false);
      mpfs_gpiowrite(ICICLE_GPIO_LED2, false);
      break;
    case LED_ASSERTION:
      mpfs_gpiowrite(ICICLE_GPIO_LED3, true);
      mpfs_gpiowrite(ICICLE_GPIO_LED2, true);
      mpfs_gpiowrite(ICICLE_GPIO_LED1, true);
      break;
    case LED_PANIC:
      mpfs_gpiowrite(ICICLE_GPIO_LED4, true);
      break;

    default:
      break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *    Turn off the LED specificed.
 *
 * Input Parameters:
 *   led - The LED which is under this control
 *
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
    case LED_INIRQ:
      mpfs_gpiowrite(ICICLE_GPIO_LED1, false);
      break;
    case LED_SIGNAL:
      mpfs_gpiowrite(ICICLE_GPIO_LED3, false);
      break;
    case LED_ASSERTION:
      mpfs_gpiowrite(ICICLE_GPIO_LED3, false);
      mpfs_gpiowrite(ICICLE_GPIO_LED2, false);
      mpfs_gpiowrite(ICICLE_GPIO_LED1, false);
      break;
    case LED_PANIC:
      mpfs_gpiowrite(ICICLE_GPIO_LED4, false);
      break;

    default:
      break;
    }
}
