/****************************************************************************
 * boards/arm/tiva/tm4c129e-launchpad/src/tm4c_buttons.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "tm4c129e-launchpad.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Pin configuration for each STM3210E-EVAL button.  This array is indexed by
 * the BUTTON_* and JOYSTICK_* definitions in board.h
 */

static const uint32_t g_buttons[NUM_BUTTONS] =
{
  GPIO_SW1, GPIO_SW2
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
      tiva_configgpio(g_buttons[i]);
    }

#ifdef CONFIG_ARCH_IRQBUTTONS
  /* Configure GPIO interrupts */

  tiva_gpioirqinitialize();
#endif
  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  int i;

  /* Check that state of each key */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      /* A LOW value means that the key is pressed. */

      bool released = tiva_gpioread(g_buttons[i]);

      /* Accumulate the set of depressed (not released) keys */

      if (!released)
        {
          ret |= (1 << i);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  int ret;

  if (id < 0 || id >= NUM_BUTTONS)
    {
      ret = -EINVAL;
    }
  else
    {
      /* Are we attaching or detaching? */

      if (irqhandler != NULL)
        {
          ret = tiva_gpioirqattach(g_buttons[id], irqhandler, arg);
        }
      else
        {
          ret = tiva_gpioirqdetach(g_buttons[id]);
        }
    }

  return ret;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
