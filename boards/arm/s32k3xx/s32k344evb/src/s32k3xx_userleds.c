/****************************************************************************
 * boards/arm/s32k3xx/s32k344evb/src/s32k3xx_userleds.c
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

/* Copyright 2022 NXP */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/board.h>

#include "s32k3xx_pin.h"

#include <arch/board/board.h>

#include "s32k344evb.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure LED GPIOs for output */

  s32k3xx_pinconfig(GPIO_LED0_R);
  s32k3xx_pinconfig(GPIO_LED0_G);
  s32k3xx_pinconfig(GPIO_LED0_B);

  s32k3xx_pinconfig(GPIO_LED1_R);
  s32k3xx_pinconfig(GPIO_LED1_G);
  s32k3xx_pinconfig(GPIO_LED1_B);

  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  uint32_t ledcfg;

  switch (ledcfg)
    {
      case BOARD_LED0_R:
        ledcfg = GPIO_LED0_R;
        break;

      case BOARD_LED0_G:
        ledcfg = GPIO_LED0_G;
        break;

      case BOARD_LED0_B:
        ledcfg = GPIO_LED0_B;
        break;

      case BOARD_LED1_R:
        ledcfg = GPIO_LED1_R;
        break;

      case BOARD_LED1_G:
        ledcfg = GPIO_LED1_G;
        break;

      case BOARD_LED1_B:
        ledcfg = GPIO_LED1_B;
        break;

      default:
        return;
    }

  /* An output of '1' illuminates the LED */

  s32k3xx_gpiowrite(ledcfg, ledon);
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  /* An output of '1' illuminates the LED */

  s32k3xx_gpiowrite(GPIO_LED0_R, (ledset & BOARD_LED0_R_BIT) != 0);
  s32k3xx_gpiowrite(GPIO_LED0_G, (ledset & BOARD_LED0_G_BIT) != 0);
  s32k3xx_gpiowrite(GPIO_LED0_B, (ledset & BOARD_LED0_B_BIT) != 0);

  s32k3xx_gpiowrite(GPIO_LED1_R, (ledset & BOARD_LED1_R_BIT) != 0);
  s32k3xx_gpiowrite(GPIO_LED1_G, (ledset & BOARD_LED1_G_BIT) != 0);
  s32k3xx_gpiowrite(GPIO_LED1_B, (ledset & BOARD_LED1_B_BIT) != 0);
}

#endif /* !CONFIG_ARCH_LEDS */
