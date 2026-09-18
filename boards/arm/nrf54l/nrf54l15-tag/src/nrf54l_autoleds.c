/****************************************************************************
 * boards/arm/nrf54l/nrf54l15-tag/src/nrf54l_autoleds.c
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

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "nrf54l15-tag.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const nrf54l_pinset_t g_ledcfg[BOARD_NLEDS] =
{
  GPIO_LED1,
  GPIO_LED2,
  GPIO_LED3,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 *
 * Description:
 *   Configure the GPIO pins used for operating system LED indications.
 *
 ****************************************************************************/

void board_autoled_initialize(void)
{
  unsigned int i;

  for (i = 0; i < BOARD_NLEDS; i++)
    {
      nrf54l_gpio_config(g_ledcfg[i]);
    }
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *   Set the LED indication for the requested operating system event.
 *
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (led == LED_STACKCREATED || led == LED_PANIC)
    {
      nrf54l_gpio_write(g_ledcfg[BOARD_LED1], false);
    }
}

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *   Clear the LED indication for the requested operating system event.
 *
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led == LED_PANIC)
    {
      nrf54l_gpio_write(g_ledcfg[BOARD_LED1], true);
    }
}

#endif /* CONFIG_ARCH_LEDS */
