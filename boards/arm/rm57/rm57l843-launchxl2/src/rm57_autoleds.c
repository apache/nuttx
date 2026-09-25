/****************************************************************************
 * boards/arm/rm57/rm57l843-launchxl2/src/rm57_autoleds.c
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

/* LEDs
 *
 * The LAUNCHXL2-RM57L has two user LEDs, labeled B6 and B7 on the board
 * silkscreen, driven by GIOB[6] and GIOB[7] (see rm57l843-launchxl2.h).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "rm57_gio.h"
#include "rm57l843-launchxl2.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* Configure LED GIOs for output */

  rm57_configgio(GIO_LED_B6);
  rm57_configgio(GIO_LED_B7);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (led == LED_STACKCREATED || led == LED_PANIC)
    {
      rm57_giowrite(GIO_LED_B6, true); /* High illuminates */
      rm57_giowrite(GIO_LED_B7, true);
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led == LED_STACKCREATED || led == LED_PANIC)
    {
      rm57_giowrite(GIO_LED_B6, false); /* Low extinguishes */
      rm57_giowrite(GIO_LED_B7, false);
    }
}

#endif /* CONFIG_ARCH_LEDS */
