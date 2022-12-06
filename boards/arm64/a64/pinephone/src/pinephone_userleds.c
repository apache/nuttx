/****************************************************************************
 * boards/arm64/a64/pinephone/src/pinephone_userleds.c
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

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm64_internal.h"
#include "pinephone.h"

#ifdef CONFIG_USERLED

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
  int ret;

  /* Configure the LED GPIO for output. */

  for (i = 0; i < ARRAYSIZE(g_led_map); i++)
    {
      ret = a64_pio_config(g_led_map[i]);
      DEBUGASSERT(ret == OK);
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
  if ((unsigned)led < ARRAYSIZE(g_led_map))
    {
      a64_pio_write(g_led_map[led], ledon);
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

  /* Configure LED1-3 GPIOs for output */

  for (i = 0; i < ARRAYSIZE(g_led_map); i++)
    {
      a64_pio_write(g_led_map[i], (ledset & g_led_setmap[i]) != 0);
    }
}

#endif /* CONFIG_USERLED */
