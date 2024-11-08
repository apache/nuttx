/****************************************************************************
 * boards/arm/stm32h5/nucleo-h563zi/src/stm32_autoleds.c
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
#include <debug.h>

#include <sys/param.h>

#include <nuttx/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_gpio.h"
#include "nucleo-h563zi.h"

#include <arch/board/board.h>

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Indexed by BOARD_LED_<color> */

static const uint32_t g_ledmap[BOARD_NLEDS] =
{
  GPIO_LD1,
  GPIO_LD2,
  GPIO_LD3,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void phy_set_led(int led, bool state)
{
  /* Active High */

  stm32_gpiowrite(g_ledmap[led], state);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  int i;

  /* Configure the LD1, LD2, and LD3 GPIOs for output. Initial state is OFF */

  for (i = 0; i < nitems(g_ledmap); i++)
    {
      stm32_configgpio(g_ledmap[i]);
      stm32_gpiowrite(g_ledmap[i], false);
    }
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
    case LED_HEAPALLOCATE:
      phy_set_led(BOARD_LED_GREEN, false);
      phy_set_led(BOARD_LED_YELLOW, false);
      phy_set_led(BOARD_LED_RED, false);
      break;

    case LED_IDLE:
      phy_set_led(BOARD_LED_GREEN, true);
      phy_set_led(BOARD_LED_YELLOW, false);
      phy_set_led(BOARD_LED_RED, false);
     break;

    case LED_IRQSENABLED:
      phy_set_led(BOARD_LED_GREEN, false);
      phy_set_led(BOARD_LED_YELLOW, true);
      phy_set_led(BOARD_LED_RED, false);
      break;

    case LED_STACKCREATED:
      phy_set_led(BOARD_LED_GREEN, true);
      phy_set_led(BOARD_LED_YELLOW, true);
      phy_set_led(BOARD_LED_RED, false);
     break;

    case LED_INIRQ:
      phy_set_led(BOARD_LED_GREEN, false);
      phy_set_led(BOARD_LED_YELLOW, false);
      phy_set_led(BOARD_LED_RED, true);
      break;

    case LED_SIGNAL:
      phy_set_led(BOARD_LED_GREEN, true);
      phy_set_led(BOARD_LED_YELLOW, false);
      phy_set_led(BOARD_LED_RED, true);
      break;

    case LED_ASSERTION:
      phy_set_led(BOARD_LED_GREEN, false);
      phy_set_led(BOARD_LED_YELLOW, true);
      phy_set_led(BOARD_LED_RED, true);
      break;

    case LED_PANIC:
      phy_set_led(BOARD_LED_GREEN, true);
      phy_set_led(BOARD_LED_YELLOW, true);
      phy_set_led(BOARD_LED_RED, true);
      break;

    default:
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
    case LED_SIGNAL:
      phy_set_led(BOARD_LED_RED, false);
      phy_set_led(BOARD_LED_GREEN, false);
      break;

    case LED_INIRQ:
      phy_set_led(BOARD_LED_RED, false);
      break;

    case LED_ASSERTION:
      phy_set_led(BOARD_LED_YELLOW, false);
      break;

    case LED_PANIC:
      phy_set_led(BOARD_LED_GREEN, false);
      phy_set_led(BOARD_LED_RED, false);
      phy_set_led(BOARD_LED_YELLOW, false);
      break;

    case LED_IDLE:
      phy_set_led(BOARD_LED_GREEN, false);
      break;

    default:
      break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
