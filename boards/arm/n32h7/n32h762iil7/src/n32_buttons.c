/****************************************************************************
 * boards/arm/n32h7/n32h762iil7/src/n32_buttons.c
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

#include <stddef.h>
#include <errno.h>

#include <sys/param.h>

#include <nuttx/irq.h>
#include <nuttx/board.h>

#include "n32_gpio.h"
#include "n32h762iil7.h"
#include <arch/board/board.h>

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static const uint32_t g_btncfg[NUM_BUTTONS] =
{
  GPIO_BTN_WKUP,
  GPIO_BTN_KEY1,
  GPIO_BTN_KEY2,
  GPIO_BTN_KEY3,
};

static const bool g_btnactive[NUM_BUTTONS] =
{
  true,
  false,
  false,
  false,
};

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

  /* Configure button GPIOs for input */

  for (i = 0; i < nitems(g_btncfg); i++)
    {
      n32_configgpio(g_btncfg[i]);
    }

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  int i;

  for (i = 0; i < nitems(g_btncfg); i++)
    {
      ret |= ((n32_gpioread(g_btncfg[i]) == g_btnactive[i] ? 1 : 0) ?
             (1 << i) : 0);
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
 *   called to collect the state of all buttons.  board_buttons() returns a
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

  if (id >= 0 && id < nitems(g_btncfg))
    {
      if (irqhandler != NULL)
        {
          ret = n32_gpiosetevent(g_btncfg[id], true, true, true,
                             irqhandler, arg);
        }
      else
        {
          ret = n32_gpiosetevent(g_btncfg[id], false, false, false,
                             irqhandler, arg);
        }
    }

  return ret;
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
