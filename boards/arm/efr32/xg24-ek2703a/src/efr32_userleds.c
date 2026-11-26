/****************************************************************************
 * boards/arm/efr32/xg24-ek2703a/src/efr32_userleds.c
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

#include "efr32_gpio.h"
#include "xg24-ek2703a.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const gpio_pinset_t g_ledcfg[BOARD_NLEDS] =
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
      efr32_configgpio(g_ledcfg[i]);
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
      efr32_gpiowrite(g_ledcfg[led], ledon);
    }
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  int i;

  for (i = 0; i < BOARD_NLEDS; i++)
    {
      efr32_gpiowrite(g_ledcfg[i], (ledset & (1 << i)) != 0);
    }
}

/****************************************************************************
 * Name: board_userled_getall
 ****************************************************************************/

void board_userled_getall(uint32_t *ledset)
{
  uint32_t value = 0;
  int i;

  for (i = 0; i < BOARD_NLEDS; i++)
    {
      if (efr32_gpioread(g_ledcfg[i]))
        {
          value |= 1 << i;
        }
    }

  *ledset = value;
}

#endif /* !CONFIG_ARCH_LEDS */
