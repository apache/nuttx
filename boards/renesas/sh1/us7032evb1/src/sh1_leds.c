/****************************************************************************
 * boards/renesas/sh1/us7032evb1/src/sh1_leds.c
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

#include "chip.h"
#include "renesas_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SH1_LPEVB only a single LED controlled by either port A, pin 15, or
 * port B, pin 15 (selectable via JP8).  In this file, we assume the portB
 * setup.
 */

#define SH1_PBDR_LED  0x8000
#define SH1_PBIOR_LED 0x8000
#define SH1_PBCR2_LED 0xc000

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

  /* Setup port B, pin 15 as an output */

  reg16  = getreg16(SH1_PFC_PBIOR);
  reg16 |= SH1_PBIOR_LED;
  putreg16(reg16, SH1_PFC_PBIOR);

  /* Setup port B, pin 15 as a normal I/O register */

  reg16  = getreg16(SH1_PFC_PBCR1);
  reg16 &= ~SH1_PBCR2_LED;
  putreg16(reg16, SH1_PFC_PBCR1);

  /* Turn the LED off */

  reg16  = getreg16(SH1_PORTB_DR);
  reg16 &= ~SH1_PBDR_LED;
  putreg16(reg16, SH1_PORTB_DR);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  uint16_t reg16;

  if (led)
    {
      /* Turn the LED on */

      reg16  = getreg16(SH1_PORTB_DR);
      reg16 |= SH1_PBDR_LED;
      putreg16(reg16, SH1_PORTB_DR);
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  uint16_t reg16;

  if (led)
    {
      /* Turn the LED off */

      reg16  = getreg16(SH1_PORTB_DR);
      reg16 &= ~SH1_PBDR_LED;
      putreg16(reg16, SH1_PORTB_DR);
    }
}
#endif /* CONFIG_ARCH_LEDS */
