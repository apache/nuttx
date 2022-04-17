/****************************************************************************
 * boards/arm/nrf52/nrf52832-dk/src/nrf52_buttons.c
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

#include "nrf52_gpio.h"

#include "nrf52832-dk.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Pin configuration for each nRF52832-DK button.  This array is indexed by
 * the BUTTON_* definitions in board.h
 */

static const uint32_t g_buttons[NUM_BUTTONS] =
{
  GPIO_BUTTON1,
  GPIO_BUTTON2,
  GPIO_BUTTON3,
  GPIO_BUTTON4
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

  /* Configure the GPIO pins as inputs. */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      nrf52_gpio_config(g_buttons[i]);
    }

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;

  /* Check that state of each key */

  if (!nrf52_gpio_read(g_buttons[BUTTON_BTN1]))
    {
      ret |= BUTTON_BTN1_BIT;
    }

  if (!nrf52_gpio_read(g_buttons[BUTTON_BTN2]))
    {
      ret |= BUTTON_BTN2_BIT;
    }

  if (!nrf52_gpio_read(g_buttons[BUTTON_BTN3]))
    {
      ret |= BUTTON_BTN3_BIT;
    }

  if (!nrf52_gpio_read(g_buttons[BUTTON_BTN4]))
    {
      ret |= BUTTON_BTN4_BIT;
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
  int ret = -ENOSYS;

#warning Missing Implementation!

  return ret;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
