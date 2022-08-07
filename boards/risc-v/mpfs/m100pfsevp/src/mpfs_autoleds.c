/****************************************************************************
 * boards/risc-v/mpfs/m100pfsevp/src/mpfs_autoleds.c
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

#include <nuttx/board.h>

#include <arch/board/board.h>
#include <mpfs_gpio.h>
#include "board_config.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 *
 * Description:
 *    Init the LEDs.
 *
 ****************************************************************************/

void board_autoled_initialize(void)
{
  /* TODO */
}

/****************************************************************************
 * Name: board_autoled_on
 *
 * Description:
 *    Turn on the LED specified.
 *
 * Input Parameters:
 *   led - The LED which is under this control
 *
 ****************************************************************************/

void board_autoled_on(int led)
{
  switch (led)
    {
      /* TODO */

    default:
      break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 *
 * Description:
 *    Turn off the LED specified.
 *
 * Input Parameters:
 *   led - The LED which is under this control
 *
 ****************************************************************************/

void board_autoled_off(int led)
{
  switch (led)
    {
      /* TODO */

    default:
      break;
    }
}
