/****************************************************************************
 * boards/z16/z16f/z16f2800100zcog/src/z16f_leds.c
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

/* The z16f2800100zcog board has four LEDs:
 *
 * - Green LED D1 which illuminates in the presence of Vcc
 * - Red LED D2 connected to chip port PA0_T0IN
 * - Yellow LED D3 connected to chip port PA1_T0OUT
 * - Green LED D4 connected to chip port PA2_DE0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "z16_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_initialize(void)
{
  /* The following is performed z16f_board_initialize() as well */

  putreg8(getreg8(Z16F_GPIOA_OUT) | 0x07, Z16F_GPIOA_OUT);
  putreg8(getreg8(Z16F_GPIOA_DD) & 0xf8, Z16F_GPIOA_DD);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if ((unsigned)led <= 7)
    {
       putreg8(((getreg8(Z16F_GPIOA_OUT) & 0xf8) | led), Z16F_GPIOA_OUT);
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led >= 1)
    {
      board_autoled_on(led - 1);
    }
}
#endif /* CONFIG_ARCH_LEDS */
