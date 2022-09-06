/****************************************************************************
 * boards/arm/gd32f4/gd32f450zk-eval/src/gd32f4xx_userleds.c
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
#include <arch/board/board.h>

#include "gd32f4xx_gpio.h"
#include "gd32f450z_eval.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ARRAYSIZE(x) (sizeof((x)) / sizeof((x)[0]))

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* LED index */

static const uint32_t g_led_map[BOARD_LEDS] =
{
  LED1,
  LED2,
  LED3
};

static const uint32_t g_led_setmap[BOARD_LEDS] =
{
  BOARD_LED1_BIT,
  BOARD_LED2_BIT,
  BOARD_LED3_BIT
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Turn on selected led */

static void gd32_eval_led_on(led_typedef_enum led_num)
{
  gd32_gpio_write(g_led_map[led_num], true);
}

/* Turn off selected led */

static void gd32_eval_led_off(led_typedef_enum led_num)
{
  gd32_gpio_write(g_led_map[led_num], false);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then the
 *   board_userled_initialize() is available to initialize the LED from user
 *   application logic.
 *
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  int i;

  /* Configure the LED GPIO for output. */

  for (i = 0; i < ARRAYSIZE(g_led_map); i++)
    {
      gd32_gpio_config(g_led_map[i]);
    }

  return BOARD_LEDS;
}

/****************************************************************************
 * Name: board_userled
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *  LEDs.  If CONFIG_ARCH_LEDS is not defined, then the board_userled() is
 *  available to control the LED from user application logic.
 *
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if ((unsigned)led < ARRAYSIZE(g_led_map))
    {
      gd32_gpio_write(g_led_map[led], ledon);
    }
}

/****************************************************************************
 * Name: board_userled_all
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *  LEDs.  If CONFIG_ARCH_LEDS is not defined, then the board_userled_all()
 *  is available to control the LED from user application logic.
 *  NOTE:  since there is only a single LED on-board, this is function
 *  is not very useful.
 *
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  int i;

  /* Configure LED1-3 GPIOs for output */

  for (i = 0; i < ARRAYSIZE(g_led_map); i++)
    {
      gd32_gpio_write(g_led_map[i], (ledset & g_led_setmap[i]) != 0);
    }
}

#endif /* !CONFIG_ARCH_LEDS */
