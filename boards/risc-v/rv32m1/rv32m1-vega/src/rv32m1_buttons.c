/****************************************************************************
 * boards/risc-v/rv32m1/rv32m1-vega/src/rv32m1_buttons.c
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
#include <nuttx/board.h>
#include <nuttx/arch.h>

#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include "rv32m1_gpio.h"
#include "rv32m1.h"
#include "rv32m1-vega.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t g_buttons[BOARD_NBUTTON] =
{
  BUTTON_SW2,
  BUTTON_SW3,
  BUTTON_SW4,
  BUTTON_SW5
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

  for (i = 0; i < BOARD_NBUTTON; ++i)
    {
      rv32m1_gpio_config(g_buttons[i]);
    }

  return BOARD_NBUTTON;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   32-bit bit set with each bit associated with a button.  See the
 *   BUTTON_*_BIT  definitions in board.h for the meaning of each bit.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  int i = 0;

  for (i = 0; i < BOARD_NBUTTON; ++i)
    {
      /* Low value means that the button is pressed */

      if (!rv32m1_gpio_read(g_buttons[i]))
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
  int ret = -EINVAL;

  if (id < 0 || id >= BOARD_NBUTTON)
    {
      return -EINVAL;
    }

  if (NULL != irqhandler)
    {
      /* Attach the new button handler. */

      ret = rv32m1_gpio_irqattach(g_buttons[id], irqhandler, arg);

      /* Then make sure that interrupts are enabled on the pin */

      rv32m1_gpio_irqenable(g_buttons[id]);
    }
  else
    {
      rv32m1_gpio_irqdisable(g_buttons[id]);
      ret = 0;
    }

  return ret;
}
#endif
