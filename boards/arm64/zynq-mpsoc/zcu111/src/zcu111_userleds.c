/****************************************************************************
 * boards/arm64/zynq-mpsoc/zcu111/src/zcu111_userleds.c
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

#include <sys/param.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm64_internal.h"
#include "zcu111.h"

#ifdef CONFIG_USERLED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USER LED definitions *****************************************************/

/* LED index values for use with board_userled() */

typedef enum
{
    USER_LED1 = 0, /* DS11 */
    USER_LED2 = 1, /* DS12 */
    USER_LED3 = 2, /* DS13 */
    USER_LED4 = 3, /* DS14 */
    USER_LED5 = 4, /* DS15 */
    USER_LED6 = 5, /* DS16 */
    USER_LED7 = 6, /* DS17 */
    USER_LED8 = 7, /* DS18 */
    USER_LEDS      /* Number of LEDs */
} led_typedef_enum;

/* LED bits for use with board_userled_all() */

#define USER_LED1_BIT    (1 << USER_LED1)
#define USER_LED2_BIT    (1 << USER_LED2)
#define USER_LED3_BIT    (1 << USER_LED3)
#define USER_LED4_BIT    (1 << USER_LED4)
#define USER_LED5_BIT    (1 << USER_LED5)
#define USER_LED6_BIT    (1 << USER_LED6)
#define USER_LED7_BIT    (1 << USER_LED7)
#define USER_LED8_BIT    (1 << USER_LED8)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* LED index */

static const uint32_t g_led_map[USER_LEDS] =
{
  78, /* MIO78(EMIO) */
  79, /* MIO79(EMIO) */
  80, /* MIO80(EMIO) */
  81, /* MIO81(EMIO) */
  82, /* MIO82(EMIO) */
  83, /* MIO83(EMIO) */
  84, /* MIO84(EMIO) */
  85  /* MIO85(EMIO) */
};

static const uint32_t g_led_setmap[USER_LEDS] =
{
  USER_LED1_BIT, /* BANK 3 Pin 0 */
  USER_LED2_BIT, /* BANK 3 Pin 1 */
  USER_LED3_BIT, /* BANK 3 Pin 2 */
  USER_LED4_BIT, /* BANK 3 Pin 3 */
  USER_LED5_BIT, /* BANK 3 Pin 4 */
  USER_LED6_BIT, /* BANK 3 Pin 5 */
  USER_LED7_BIT, /* BANK 3 Pin 6 */
  USER_LED8_BIT  /* BANK 3 Pin 7 */
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

  /* Configure the LED GPIO for output. */

  for (i = 0; i < nitems(g_led_map); i++)
    {
      /* Configure LED GPIOs for output */

      zynq_mio_setdirpin(g_led_map[i], 1);
      zynq_mio_setoutenpin(g_led_map[i], 1);
      zynq_mio_writepin(g_led_map[i], false);
    }

  return USER_LEDS;
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
  if ((unsigned)led < nitems(g_led_map))
    {
      zynq_mio_writepin(g_led_map[led], ledon);
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

  for (i = 0; i < nitems(g_led_map); i++)
    {
      zynq_mio_writepin(g_led_map[i], (ledset & g_led_setmap[i]) != 0);
    }
}

#endif /* CONFIG_USERLED */
