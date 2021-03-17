/****************************************************************************
 * boards/arm/tiva/launchxl-cc1312r1/src/cc1312_userleds.c
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

#include <arch/board/board.h>

#include "tiva_gpio.h"
#include "launchxl-cc1312r1.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  tiva_configgpio(&g_gpio_gled);
  tiva_configgpio(&g_gpio_rled);
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  const struct cc13xx_pinconfig_s *pinconfig;

  if (led == BOARD_GLED)
    {
      pinconfig = &g_gpio_gled;
    }
  else if (led = BOARD_RLED)
    {
      pinconfig = &g_gpio_rled;
    }
  else
    {
      return;
    }

  tiva_gpiowrite(pinconfig, ledon);  /* High output illuminates */
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  board_userled(BOARD_GLED, (ledset & BOARD_GLED_BIT) != 0);
  board_userled(BOARD_RLED, (ledset & BOARD_RLED_BIT) != 0);
}
