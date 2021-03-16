/****************************************************************************
 * boards/z80/z8/z8encore000zco/src/z8_leds.c
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

#include <stdint.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "z80_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Port G:  Anode Bit Assignments (1 enables) */

#define Z8_PORTG_ANODE_MASK      0x7f
#define Z8_PORTG_ANODE_ROW0      0x01
#define Z8_PORTG_ANODE_ROW1      0x02
#define Z8_PORTG_ANODE_ROW2      0x04
#define Z8_PORTG_ANODE_ROW3      0x08
#define Z8_PORTG_ANODE_ROW4      0x10
#define Z8_PORTG_ANODE_ROW5      0x20
#define Z8_PORTG_ANODE_ROW6      0x40

/* Port E: Cathode Bit Assignments (0 enables) */

#define Z8_PORTE_CATHODE_MASK    0x1f
#define Z8_PORTE_CATHODE_COLUMN0 0x01
#define Z8_PORTE_CATHODE_COLUMN1 0x02
#define Z8_PORTE_CATHODE_COLUMN2 0x04
#define Z8_PORTE_CATHODE_COLUMN3 0x08
#define Z8_PORTE_CATHODE_COLUMN4 0x10

/* Port E: LED Addressing */

#define Z8_PORTE_LED_MASK        0xe0
#define Z8_PORTE_LED_D3          0x20
#define Z8_PORTE_LED_D4          0x40
#define Z8_PORTE_LED_D1          0x80

/* Port G: LED Addressing */

#define Z8_PORTG_LED_MASK        0x80
#define Z8_PORTG_LED_D2          0x80

/* Special values for display */

#define LED_ALLON                { Z8_PORTG_ANODE_MASK, 0x00 }
#define LED_ALLOFF               { 0x00, Z8_PORTE_CATHODE_MASK }
#define LED_LEVEL1               { 0x10, 0x00 }
#define LED_LEVEL2               { 0x08, 0x00 }
#define LED_LEVEL3               { 0x04, 0x00 }
#define LED_LEVEL4               { 0x02, 0x00 }
#define LED_LEVEL1I              { 0x14, 0x00 }
#define LED_LEVEL2I              { 0x0c, 0x00 }
#define LED_LEVEL3I              { 0x04, 0x00 }
#define LED_LEVEL4I              { 0x06, 0x00 }
#define LED_LEVEL1S              { 0x11, 0x00 }
#define LED_LEVEL2S              { 0x09, 0x00 }
#define LED_LEVEL3S              { 0x05, 0x00 }
#define LED_LEVEL4S              { 0x03, 0x00 }
#define LED_LEVEL1A              { 0x10, 0x11 }
#define LED_LEVEL2A              { 0x08, 0x11 }
#define LED_LEVEL3A              { 0x04, 0x11 }
#define LED_LEVEL4A              { 0x02, 0x11 }
#define LED_SNAKEEYES            { 0x06, 0x11 }

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct z8_ledbits_s
{
  uint8_t anode;
  uint8_t cathode;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct z8_ledbits_s g_ledarray[10][4] =
{
  { LED_ALLON,     LED_ALLON,     LED_ALLON,     LED_ALLON     },
  { LED_ALLOFF,    LED_ALLOFF,    LED_ALLOFF,    LED_ALLOFF    },
  { LED_LEVEL1,    LED_ALLOFF,    LED_ALLOFF,    LED_ALLOFF    },
  { LED_LEVEL1,    LED_LEVEL2,    LED_ALLOFF,    LED_ALLOFF    },
  { LED_LEVEL1,    LED_LEVEL2,    LED_LEVEL3,    LED_ALLOFF    },
  { LED_LEVEL1,    LED_LEVEL2,    LED_LEVEL3,    LED_LEVEL4    },
  { LED_LEVEL1I,   LED_LEVEL2I,   LED_LEVEL3I,   LED_LEVEL4I   },
  { LED_LEVEL1S,   LED_LEVEL2S,   LED_LEVEL3S,   LED_LEVEL4S   },
  { LED_LEVEL1A,   LED_LEVEL2A,   LED_LEVEL3A,   LED_LEVEL4A   },
  { LED_SNAKEEYES, LED_SNAKEEYES, LED_SNAKEEYES, LED_SNAKEEYES }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: z8_putled134
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
static void z8_putled134(FAR const struct z8_ledbits_s *bits, uint8_t addr)
{
  uint8_t porte;

  porte = bits->cathode;
  putreg8(porte, PEOD);          /* Load porte data */
  putreg8(bits->anode, PGOD);    /* Load portg data */

  porte |= addr;
  putreg8(porte, PEOD);          /* Latch data */
}
#endif

/****************************************************************************
 * Name: z8_lputed2
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
static void z8_putled2(FAR const struct z8_ledbits_s *bits, uint8_t addr)
{
  uint8_t portg;

  putreg8(bits->cathode, PEOD);  /* Load porte data */
  portg = bits->anode;
  putreg8(porte, PGOD);          /* Load portg data */

  portg |= addr;
  putreg8(portg, PGOD);          /* Latch data */
}
#endif

/****************************************************************************
 * Name: z8_putarray
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
static void z8_putarray(FAR const struct z8_ledarray_s *array)
{
  z8_putled134(&array->led[0], Z8_PORTE_LED_D1);
  z8_putled2(&array->led[1], Z8_PORTG_LED_D2);
  z8_putled134(&array->led[2], Z8_PORTE_LED_D3);
  z8_putled134(&array->led[3], Z8_PORTE_LED_D4);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_initialize(void)
{
  putreg8(0x00, PEAF);          /* PE Alt func = Port */
  putreg8(0x00, PGAF);          /* PG Alt func = Port */

  putreg8(0x00, PEOC);          /* PE Out Ctrl = push-pull */
  putreg8(0x00, PGOC);          /* PG Out Ctrl = push-pull */

  /* putreg8(0x00, PEDD);           * PA Data Dir = output */

  putreg8(0x01, PEADDR);        /* PA Data Dir = output */
  putreg8(0x00, PECTL);         /* OUTPUT */

  /* putreg8(0x00, PGDD);           * PA Data Dir = output */

  putreg8(0x01, PGADDR);        /* PA Data Dir = output */
  putreg8(0x00, PGCTL);         /* OUTPUT */

  z8_putarray(&g_ledarray[0][0]);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if ((unsigned)led <= 8)
    {
      z8_putarray(&g_ledarray[led + 1][0]);
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
