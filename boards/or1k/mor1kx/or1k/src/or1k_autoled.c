/****************************************************************************
 * boards/or1k/mor1kx/or1k/src/or1k_autoled.c
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

#include "or1k_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LED_BASE      (0x91000000)
#define LED_DATA      (LED_BASE)
#define LED_DIRECTION (LED_BASE+4)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t *led_data = (uint32_t *)LED_DATA;
static uint32_t *led_dir  = (uint32_t *)LED_DIRECTION;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_initialize(void)
{
  /* Set the or1k GPIO direction register to output */

  /* The Terasic C5G has 18 LEDs on GPIO0[0:17] */

  *led_dir = 0x3fff;
  *led_data = 0x0;
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  *led_data |= led & 0xff;
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  *led_data &= ~(led & 0xff);
}

#endif /* CONFIG_ARCH_LEDS */
