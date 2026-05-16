/****************************************************************************
 * boards/arm/stm32/alientek-m144z-m4/src/stm32_userleds.c
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

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32.h"
#include "alientek-m144z-m4.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This array maps an LED index to its GPIO pin configuration.
 * LEDs on this board are wired active LOW (cathode -> GPIO,
 * anode -> 510R -> 3.3V), so the gpio write value is inverted in
 * board_userled() / board_userled_all().
 */

static const uint32_t g_ledcfg[BOARD_NLEDS] =
{
  GPIO_LED0,
  GPIO_LED1,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  int i;

  for (i = 0; i < BOARD_NLEDS; i++)
    {
      stm32_configgpio(g_ledcfg[i]);
    }

  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if ((unsigned)led < BOARD_NLEDS)
    {
      stm32_gpiowrite(g_ledcfg[led], !ledon);
    }
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  stm32_gpiowrite(g_ledcfg[BOARD_LED0], (ledset & BOARD_LED0_BIT) == 0);
  stm32_gpiowrite(g_ledcfg[BOARD_LED1], (ledset & BOARD_LED1_BIT) == 0);
}

#endif /* !CONFIG_ARCH_LEDS */
