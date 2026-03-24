/****************************************************************************
 * boards/risc-v/k210/maix-bit/src/k210_userleds.c
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

#include "k210_fpioa.h"
#include "k210_gpiohs.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* LED configuration table */

struct k210_led_cfg_s
{
  uint32_t pad;   /* FPIOA physical pad number */
  uint32_t func;  /* FPIOA function + flags */
  uint32_t io;    /* GPIOHS IO number */
};

static const struct k210_led_cfg_s g_led_cfg[BOARD_LEDS] =
{
  { BOARD_USERLED1_PAD, K210_IO_FUNC_GPIOHS1 | K210_IOFLAG_GPIOHS,
    BOARD_USERLED1_IO },
  { BOARD_USERLED2_PAD, K210_IO_FUNC_GPIOHS2 | K210_IOFLAG_GPIOHS,
    BOARD_USERLED2_IO }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  board_userled_initialize
 *
 * Description:
 *   This function may called from application-specific logic during its
 *   to perform board-specific initialization of LED resources.  This
 *   includes such things as, for example, configure GPIO pins to drive the
 *   LEDs and also putting the LEDs in their correct initial state.
 *
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then this interfaces may be
 *   available to control the LEDs directly from user board logic or
 *   indirectly user applications (via the common LED character driver).
 *
 *   Most boards have only a few LEDs and in those cases all LEDs may be
 *   used by the NuttX LED logic exclusively and may not be available for
 *   use by user logic if CONFIG_ARCH_LEDS=y.
 *
 *   NOTE: The LED number is returned.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Number of LEDs on board
 *
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  int i;

  /* Configure the LED GPIO for output.
   * LEDs are active low (false=on, true=off), so set them
   * high (off) initially.
   */

  for (i = 0; i < BOARD_LEDS; i++)
    {
      /* Configure pin for GPIOHS function */

      k210_fpioa_config(g_led_cfg[i].pad, g_led_cfg[i].func);

      /* Set direction to output, initial value high (LED off) */

      k210_gpiohs_set_direction(g_led_cfg[i].io, true);
      k210_gpiohs_set_value(g_led_cfg[i].io, true);
    }

  return BOARD_LEDS;
}

/****************************************************************************
 * Name:  board_userled
 *
 * Description:
 *   This interface may be used by application specific logic to set the
 *   state of a single LED.  Definitions for the led identification are
 *   provided in the board-specific board.h header file that may be included
 *   like:
 *
 *     #included <arch/board/board.h>
 *
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then this interfaces may be
 *   available to control the LEDs directly from user board logic or
 *   indirectly user applications (via the common LED character driver).
 *
 *   Most boards have only a few LEDs and in those cases all LEDs may be
 *   used by the NuttX LED logic exclusively and may not be available for
 *   use by user logic if CONFIG_ARCH_LEDS=y.
 *
 * Input Parameters:
 *   led   - LED number
 *   ledon - True if LED should be turned on; False to turn off
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if ((unsigned)led < BOARD_LEDS)
    {
      /* LEDs are active low: false = on, true = off */

      k210_gpiohs_set_value(g_led_cfg[led].io, !ledon);
    }
}

/****************************************************************************
 * Name:  board_userled_all
 *
 * Description:
 *   This interface may be used by application specific logic to set the
 *   state of all board LED.  Definitions for the led set member
 *   identification is provided in the board-specific board.h header file
 *   that may be includedlike:
 *
 *     #included <arch/board/board.h>
 *
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then this interfaces may be
 *   available to control the LEDs directly from user board logic or
 *   indirectly user applications (via the common LED character driver).
 *
 *   Most boards have only a few LEDs and in those cases all LEDs may be
 *   used by the NuttX LED logic exclusively and may not be available for
 *   use by user logic if CONFIG_ARCH_LEDS=y.
 *
 * Input Parameters:
 *   ledset - Bitset of LEDs to be turned on and off
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  int i;

  for (i = 0; i < BOARD_LEDS; i++)
    {
      board_userled(i, (ledset & (1 << i)) != 0);
    }
}
