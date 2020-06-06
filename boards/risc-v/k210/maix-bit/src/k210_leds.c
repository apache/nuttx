/****************************************************************************
 * boards/risc-v/k210/maix-bit/src/k210_leds.c
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

#include "k210_fpioa.h"
#include "k210_gpiohs.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void board_autoled_initialize(void)
{
  k210_fpioa_config(BOARD_LED_PAD, BOARD_LED_IO_FUNC | K210_IOFLAG_GPIOHS);
  k210_gpiohs_set_direction(BOARD_LED_IO, true);
  k210_gpiohs_set_value(BOARD_LED_IO, true); /* LED off */
}

void board_autoled_on(int led)
{
  if (led == LED_PANIC)
    {
      k210_gpiohs_set_value(BOARD_LED_IO, false);
    }
}

void board_autoled_off(int led)
{
  if (led == LED_PANIC)
    {
      k210_gpiohs_set_value(BOARD_LED_IO, true);
    }
}
