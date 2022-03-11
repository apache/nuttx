/****************************************************************************
 * boards/renesas/m16c/skp16c26/src/m16c_leds.c
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

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "up_internal.h"
#include "chip.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SKP62C26 has 3 LEDs control by bits
 * 0 and 2 in port 7 and bit 0 in port 8.
 */

#define GREEN_LED            (1 << 2)   /* Bit 2, port 7 */
#define YELLOW_LED           (1 << 4)   /* Bit 4, port 7 */
#define RED_LED              (1 << 0)   /* Bit 0, port 8 */

#define GREEN_LED_ON         0
#define GREEN_LED_OFF        GREEN_LED
#define GREEN_LED_MASK       GREEN_LED
#define GREEN_LED_PORT       M16C_P7

#define YELLOW_LED_ON        0
#define YELLOW_LED_OFF       YELLOW_LED
#define YELLOW_LED_MASK      YELLOW_LED
#define YELLOW_LED_PORT      M16C_P7

#define GREENYELLOW_LED_MASK (GREEN_LED_MASK|YELLOW_LED_MASK)
#define GREENYELLOW_LED_PORT M16C_P7
#define GREENYELLOW_DIR_PORT M16C_PD7

#define RED_LED_ON           0
#define RED_LED_OFF          RED_LED
#define RED_LED_MASK         RED_LED
#define RED_LED_PORT         M16C_P8
#define RED_DIR_PORT         M16C_PD8

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_ledstate[7] =
{
  (GREEN_LED_OFF | YELLOW_LED_OFF | RED_LED_OFF), /* LED_STARTED */
  (GREEN_LED_ON  | YELLOW_LED_OFF | RED_LED_OFF), /* LED_HEAPALLOCATE */
  (GREEN_LED_OFF | YELLOW_LED_ON  | RED_LED_OFF), /* LED_IRQSENABLED */
  (GREEN_LED_ON  | YELLOW_LED_ON  | RED_LED_OFF), /* LED_STACKCREATED */
  (GREEN_LED_ON  | YELLOW_LED_OFF | RED_LED_ON),  /* LED_INIRQ */
  (GREEN_LED_OFF | YELLOW_LED_ON  | RED_LED_ON),  /* LED_SIGNAL */
  (GREEN_LED_ON  | YELLOW_LED_ON  | RED_LED_ON)   /* LED_ASSERTION */
};

static uint8_t g_prevled[3];
static uint8_t g_nestlevel;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: m16c_setleds
 ****************************************************************************/

static void m16c_setleds(uint8_t gybits, uint8_t rbit)
{
  uint8_t regval;

  regval  = getreg8(GREENYELLOW_LED_PORT);
  regval &= ~GREENYELLOW_LED_MASK;
  regval |= gybits;
  putreg8(regval, GREENYELLOW_LED_PORT);

  regval  = getreg8(RED_LED_PORT);
  regval &= ~RED_LED_MASK;
  regval |= rbit;
  putreg8(regval, RED_LED_PORT);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  register uint8_t regval;

  /* Make sure that the LEDs are in the OFF state */

  regval  = getreg8(GREENYELLOW_LED_PORT);
  regval |= (GREEN_LED_OFF | YELLOW_LED_OFF);
  putreg8(regval, GREENYELLOW_LED_PORT);

  regval  = getreg8(RED_LED_PORT);
  regval |=  RED_LED_OFF;
  putreg8(regval, RED_LED_PORT);

  /* Set the direction to output */

  regval  = getreg8(GREENYELLOW_DIR_PORT);
  regval |= (GREEN_LED | YELLOW_LED);
  putreg8(regval, GREENYELLOW_DIR_PORT);

  regval  = getreg8(RED_DIR_PORT);
  regval |=  RED_LED;
  putreg8(regval, RED_DIR_PORT);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  uint8_t ledset;

  /* If this is the ASSERTION led, preserve the Y&G bits from the last
   * setting and set the RED LED on.
   */

  if (led == LED_ASSERTION)
    {
      ledset = g_ledstate[g_prevled[g_nestlevel]];
      m16c_setleds(ledset & GREENYELLOW_LED_MASK, RED_LED_ON);
    }
  else if (led < LED_ASSERTION)
    {
      /* Otherwise, just show the LEDs corresponding to this state */

      ledset = g_ledstate[led];
      m16c_setleds(ledset & GREENYELLOW_LED_MASK, ledset & RED_LED_MASK);

      /* If this was a nested states (INIRQ, SIGNAL, or ASSERTION) then
       * stack up the previous value.
       */

      if (led > LED_STACKCREATED)
        {
          g_nestlevel++;
        }

      g_prevled[g_nestlevel] = led;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  uint8_t ledset;

  /* If this is the ASSERTION led then what we do depends on
   * the previous state
   */

  if (led == LED_ASSERTION)
    {
      /* If the previous state was one of the nested states
       * (INIRQ, SIGNAL, or ASSERTION),
       * then turn the green and yellow LEDs all off.
       * That way we can distinguish that case from the simple cases.
       */

      if (g_nestlevel > 0)
        {
          ledset = 0;
        }
      else
        {
          ledset = g_ledstate[g_prevled[0]];
        }

      m16c_setleds(ledset & GREENYELLOW_LED_MASK, RED_LED_OFF);
    }
  else if (led > 0 && led < LED_ASSERTION)
    {
      /* If this was one of the nested states,
       * then we want to back to the LED setting
       * before entering that nested statel.
       */

      if (g_nestlevel > 0)
        {
          g_nestlevel--;
          led = g_prevled[g_nestlevel];
        }
      else if (led > LED_STACKCREATED)
        {
          /* This shouldn't happen */

          led--;
        }

      ledset = g_ledstate[led];
      m16c_setleds(ledset & GREENYELLOW_LED_MASK, ledset & RED_LED_MASK);
      g_prevled[g_nestlevel] = led;
    }
}

#endif /* CONFIG_ARCH_LEDS */
