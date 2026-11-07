/****************************************************************************
 * boards/xtensa/esp32/esp32-m5-stickc-plus/src/esp32_buttons.c
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

#include <assert.h>
#include <nuttx/debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <arch/irq.h>
#include <arch/board/board.h>

#include "espressif/esp_gpio.h"

#include "esp32-m5-stickc-plus.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Button ID to GPIO mapping, indexed by BUTTON_*_ID from board.h */

static const int g_buttons[NUM_BUTTONS] =
{
  BUTTON_A,
  BUTTON_B
};

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

  /* GPIO37/39 are input-only and have no internal pull resistors; the
   * board fits external pull-ups, so only the input function is set.
   */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      esp_configgpio(g_buttons[i], INPUT_FUNCTION_3 | CHANGE);
    }

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns a
 *   bit set with each bit associated with a button.  See the BUTTON_*_BIT
 *   definitions in board.h for the meaning of each bit.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  int i;

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      /* Low value means that the button is pressed */

      if (!esp_gpioread(g_buttons[i]))
        {
          ret |= 1 << i;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
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
  if (id < 0 || id >= NUM_BUTTONS)
    {
      return -EINVAL;
    }

  return esp_gpio_irq(g_buttons[id], irqhandler, arg);
}
#endif
