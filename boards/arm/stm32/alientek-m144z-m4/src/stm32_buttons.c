/****************************************************************************
 * boards/arm/stm32/alientek-m144z-m4/src/stm32_buttons.c
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
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "alientek-m144z-m4.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Pin configuration for each button. Indexed by BUTTON_* in board.h.
 *
 * Why we need a runtime "baseline + XOR" approach on this board:
 *
 *   - KEY0 (PE4) shares the line with BOOT0 via a BAT54C dual-Schottky to
 *     allow the well-known "KEY0 + RESET = enter UART ISP" trick. The BOOT0
 *     side has a strong (~10 k) external pull-down to GND, which dominates
 *     the STM32 internal pull-up (~30-50 k) on PE4. Result: PE4 reads LOW
 *     in the un-pressed state, NOT HIGH as a textbook active-LOW button
 *     would suggest.
 *
 *   - WK_UP (PA0) is a stand-by wake-up input that the silkscreen marks as
 *     "active HIGH", but external biasing on the minimal-system board is
 *     not explicit on the schematic; depending on the rev it can sit at
 *     either rail.
 *
 * Rather than hard-coding a per-pin polarity (and risking the user having
 * to recompile every time we discover something new on the schematic), we
 * snapshot the GPIO level once during initialization and treat any later
 * change from that baseline as "pressed". The implicit assumption is that
 * NO button is held at power-on, which is essentially always true outside
 * of the UART-ISP entry sequence (and even then, the user has released
 * KEY0 long before nsh runs).
 */

static const uint32_t g_buttons[NUM_BUTTONS] =
{
  GPIO_BTN_KEY0,  /* BUTTON_KEY0 (PE4) */
  GPIO_BTN_WKUP,  /* BUTTON_WKUP (PA0) */
};

/* Idle-state snapshot captured by board_button_initialize().
 * board_buttons() returns the XOR against this baseline.
 */

static bool g_idle_level[NUM_BUTTONS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  int i;

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      stm32_configgpio(g_buttons[i]);
    }

  /* Give the internal pull resistor a few microseconds to settle the line
   * to its true idle level, then capture the baseline.
   */

  up_udelay(50);

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      g_idle_level[i] = stm32_gpioread(g_buttons[i]);
    }

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   board_buttons() returns a bit-set: bit i = 1 means button i is currently
 *   pressed. Polarity is derived at runtime from the baseline captured in
 *   board_button_initialize(); see the comment block above g_buttons[].
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  int i;

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      if (stm32_gpioread(g_buttons[i]) != g_idle_level[i])
        {
          ret |= (1u << i);
        }
    }

  return ret;
}

/****************************************************************************
 * Button support.
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   32-bit bit set with each bit associated with a button.  See the
 *   BUTTON_*_BIT definitions in board.h for the meaning of each bit.
 *
 *   board_button_irq() may be called to register an interrupt handler that
 *   will be called when a button is depressed or released.  The ID value is
 *   a button enumeration value that uniquely identifies a button resource.
 *   See the BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  int ret = -EINVAL;

  /* The following should be atomic */

  if (id >= MIN_IRQBUTTON && id <= MAX_IRQBUTTON)
    {
      ret = stm32_gpiosetevent(g_buttons[id], true, true, true, irqhandler,
                               arg);
    }

  return ret;
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
