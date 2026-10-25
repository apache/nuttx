/****************************************************************************
 * boards/risc-v/ab32vg1/ab32vg1-generic/src/ab32vg1_autoleds.c
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

#include "ab32vg1-generic.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void ab32vg1_setleds(bool red, bool green, bool blue)
{
  ab32vg1_gpiowrite(GPIO_LED_RED, !red);       /* Active low */
  ab32vg1_gpiowrite(GPIO_LED_GREEN, !green);
  ab32vg1_gpiowrite(GPIO_LED_BLUE, !blue);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  ab32vg1_configgpio(GPIO_LED_RED);
  ab32vg1_configgpio(GPIO_LED_GREEN);
  ab32vg1_configgpio(GPIO_LED_BLUE);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      case 0:                                   /* LED_STARTED .. */
        ab32vg1_setleds(false, false, true);
        break;

      case 1:                                   /* LED_STACKCREATED */
        ab32vg1_setleds(false, true, false);
        break;

      case 3:                                   /* LED_ASSERTION */
      case 4:                                   /* LED_PANIC */
        ab32vg1_setleds(true, false, false);
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
  if (led == 4)                                 /* LED_PANIC */
    {
      ab32vg1_setleds(false, false, false);
    }
}

#endif /* CONFIG_ARCH_LEDS */
