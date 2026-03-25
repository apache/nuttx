/****************************************************************************
 * boards/arm/ht32f491x3/esk32/src/ht32_userleds.c
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

#include <stdbool.h>
#include <stdint.h>

#include <arch/board/board.h>

#include "arm_internal.h"

#include "ht32f491x3_gpio.h"

#include "hardware/ht32f491x3_crm.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ht32_ledcfg_s
{
  uint32_t clken;
  uintptr_t base;
  uint8_t pin;
  bool active_low;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ht32_ledcfg_s g_ledcfg[BOARD_NLEDS] =
{
  {BOARD_LED2_GPIO_CLKEN, BOARD_LED2_GPIO_BASE, BOARD_LED2_GPIO_PIN, true},
  {BOARD_LED3_GPIO_CLKEN, BOARD_LED3_GPIO_BASE, BOARD_LED3_GPIO_PIN, true},
  {BOARD_LED4_GPIO_CLKEN, BOARD_LED4_GPIO_BASE, BOARD_LED4_GPIO_PIN, true},
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  unsigned int i;

  for (i = 0; i < BOARD_NLEDS; i++)
    {
      modifyreg32(HT32_CRM_AHBEN1, 0, g_ledcfg[i].clken);
      ht32f491x3_gpioconfig(g_ledcfg[i].base, g_ledcfg[i].pin,
                            HT32_GPIO_MODE_OUTPUT, false,
                            HT32_GPIO_DRIVE_MEDIUM,
                            HT32_GPIO_PULL_NONE, 0);
      ht32f491x3_gpiowrite(g_ledcfg[i].base, g_ledcfg[i].pin,
                           g_ledcfg[i].active_low);
    }

  return BOARD_NLEDS;
}

void board_userled(int led, bool ledon)
{
  if (led >= 0 && led < BOARD_NLEDS)
    {
      ht32f491x3_gpiowrite(g_ledcfg[led].base, g_ledcfg[led].pin,
                           g_ledcfg[led].active_low ? !ledon : ledon);
    }
}

void board_userled_all(uint32_t ledset)
{
  unsigned int i;

  for (i = 0; i < BOARD_NLEDS; i++)
    {
      board_userled(i, (ledset & (1u << i)) != 0);
    }
}

#endif /* !CONFIG_ARCH_LEDS */
