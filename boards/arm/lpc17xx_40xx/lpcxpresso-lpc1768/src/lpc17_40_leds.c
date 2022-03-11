/****************************************************************************
 * boards/arm/lpc17xx_40xx/lpcxpresso-lpc1768/src/lpc17_40_leds.c
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

#include "arm_internal.h"
#include "lpc17_40_gpio.h"
#include "lpcxpresso-lpc1768.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_ncstate;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure all LED GPIO lines */

  lpc17_40_configgpio(LPCXPRESSO_LED);
  g_ncstate = true;
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  bool off;

  switch (led)
    {
    case 0:
    case 2:
      off = true;
      break;

    case 1:
      off       = false;
      g_ncstate = false;
      break;

    default:
      return;
    }

  lpc17_40_gpiowrite(LPCXPRESSO_LED, off);
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  bool off;

  switch (led)
    {
    case 0:
    case 1:
      off = false;
      break;

    case 2:
      off = g_ncstate;
      break;

    default:
      return;
    }

  lpc17_40_gpiowrite(LPCXPRESSO_LED, off);
}

#endif /* CONFIG_ARCH_LEDS */
