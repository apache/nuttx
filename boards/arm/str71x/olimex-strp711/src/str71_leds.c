/****************************************************************************
 * boards/arm/str71x/olimex-strp711/src/str71_leds.c
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

#include "chip.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* There are two LEDs are connected to  P1.8 & 9 */

#if defined(CONFIG_ARCH_LEDS) && !defined(CONFIG_STR71X_GPIO1)
#  error "LEDs require GPIO1"
#endif

#define STR71X_LED1GPIO1_BIT (0x0100)
#define STR71X_LED2GPIO1_BIT (0x0200)
#define STR71X_LEDGPIO1_BITS (STR71X_LED1GPIO1_BIT|STR71X_LED2GPIO1_BIT)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint16_t g_led2set;
static uint16_t g_led2clr;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: str71_setleds
 ****************************************************************************/

static void str71_setleds(uint16_t setbits, uint16_t clearbits)
{
  uint16_t reg16;

  /* Save the state of LED2 for later */

  g_led2set = setbits & STR71X_LED2GPIO1_BIT;
  g_led2clr = clearbits & STR71X_LED2GPIO1_BIT;

  /* Set and clear bits as directed */

  reg16  = getreg16(STR71X_GPIO1_PD);
  reg16 &= ~clearbits;
  reg16 |= setbits;
  putreg16(reg16, STR71X_GPIO1_PD);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_initialize(void)
{
  uint16_t reg16;

  /* Set normal function output */

  reg16  = getreg16(STR71X_GPIO1_PC0);
  reg16 |= STR71X_LEDGPIO1_BITS;
  putreg16(reg16, STR71X_GPIO1_PC0);

  reg16  = getreg16(STR71X_GPIO1_PC1);
  reg16 &= ~STR71X_LEDGPIO1_BITS;
  putreg16(reg16, STR71X_GPIO1_PC1);

  reg16  = getreg16(STR71X_GPIO1_PC2);
  reg16 |= STR71X_LEDGPIO1_BITS;
  putreg16(reg16, STR71X_GPIO1_PC2);

  /* Clear the LEDs (1 clears; 0 sets) */

  reg16  = getreg16(STR71X_GPIO1_PD);
  reg16 |= STR71X_LEDGPIO1_BITS;
  putreg16(reg16, STR71X_GPIO1_PD);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  /* The Olimex board has only two LEDs, so following states are faked as
   * follows
   *
   *                       SET         CLEAR
   *  LED_STARTED          (none)      (n/a)
   *  LED_HEAPALLOCATE     LED1        (n/a)
   *  LED_IRQSENABLED      LED1        (n/a)
   *  LED_STACKCREATED     LED1        (n/a)
   *  LED_INIRQ            LED1+LED2   LED1
   *  LED_SIGNAL           LED1+LED2   LED1
   *  LED_ASSERTION        LED1+LED2   LED1
   *  LED_PANIC            LED1+LED2*  LED1
   *
   *                      *The previous state of LED2 will be retained
   */

  switch (led)
    {
    default:
    case LED_STARTED:
      str71_setleds(0, STR71X_LED1GPIO1_BIT | STR71X_LED2GPIO1_BIT); /* Clear LED1&2 */
      break;

    case LED_HEAPALLOCATE:
    case LED_IRQSENABLED:
    case LED_STACKCREATED:
      str71_setleds(STR71X_LED1GPIO1_BIT, STR71X_LED2GPIO1_BIT); /* Set LED1, clear LED2 */
      break;

    case LED_INIRQ:
    case LED_SIGNAL:
    case LED_ASSERTION:
      str71_setleds(STR71X_LED1GPIO1_BIT | STR71X_LED2GPIO1_BIT, 0); /* Set LED1&2 */
      break;

    case LED_PANIC:
      str71_setleds(STR71X_LED2GPIO1_BIT | g_led2set, g_led2set); /* Set LED1, preserve LED2 */
      break;
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  /* The Olimex board has only two LEDs, so following states are faked as
   * follows
   *
   *                       SET         CLEAR
   *  LED_STARTED          (none)      (n/a)
   *  LED_HEAPALLOCATE     LED1        (n/a)
   *  LED_IRQSENABLED      LED1        (n/a)
   *  LED_STACKCREATED     LED1        (n/a)
   *  LED_INIRQ            LED1+LED2   LED1
   *  LED_SIGNAL           LED1+LED2   LED1
   *  LED_ASSERTION        LED1+LED2   LED1
   *  LED_PANIC            LED1+LED2*  LED1
   *
   *                      *The previous state of LED2 will be retained
   */

  switch (led)
    {
    default:
    case LED_STARTED:
    case LED_HEAPALLOCATE:
    case LED_IRQSENABLED:
    case LED_STACKCREATED:
      break;

    case LED_INIRQ:
    case LED_SIGNAL:
    case LED_ASSERTION:
      str71_setleds(STR71X_LED1GPIO1_BIT, STR71X_LED2GPIO1_BIT); /* Set LED1, clear LED2 */
      break;

    case LED_PANIC:
      str71_setleds(g_led2set, STR71X_LED1GPIO1_BIT | g_led2clr); /* Clear LED1, preserve LED2 */
      break;
    }
}
#endif /* CONFIG_ARCH_LEDS */
