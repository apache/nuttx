/****************************************************************************
 * boards/arm/rm57/rm57l843-launchxl2/src/rm57_userleds.c
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
#include <nuttx/debug.h>

#include <arch/board/board.h>

#include "rm57_gio.h"
#include "rm57l843-launchxl2.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure LED GIOs for output */

  rm57_configgio(GIO_LED_B6);
  rm57_configgio(GIO_LED_B7);
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if (led == BOARD_LED_B6)
    {
      rm57_giowrite(GIO_LED_B6, ledon); /* High illuminates */
    }
  else if (led == BOARD_LED_B7)
    {
      rm57_giowrite(GIO_LED_B7, ledon);
    }
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  /* High illuminates */

  rm57_giowrite(GIO_LED_B6, (ledset & BOARD_LED_B6_BIT) != 0);
  rm57_giowrite(GIO_LED_B7, (ledset & BOARD_LED_B7_BIT) != 0);
}
