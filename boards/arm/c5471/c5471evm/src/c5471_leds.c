/****************************************************************************
 * boards/arm/c5471/c5471evm/src/c5471_leds.c
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
#include <nuttx/board.h>

#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CS2  *(volatile uint32_t*)0xffff2e08
#define LEDS *(volatile uint32_t*)0x01000000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_ledstate;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_initialize(void)
{
  /* Enable access to LEDs */

  CS2 = 0x000013db;

  /* Turn LED 1-7 off; turn LED 0 on */

  g_ledstate = 0x000000fe;
  LEDS       = g_ledstate;
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (led < 8)
    {
      g_ledstate &= ~(1 << led);
      LEDS        = g_ledstate;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led < 8)
    {
      g_ledstate |= (1 << led);
      LEDS        = g_ledstate;
    }
}

#endif /* CONFIG_ARCH_LEDS */
