/****************************************************************************
 * boards/mips/pic32mz/pic32mz-starterkit/src/pic32mz_buttons.c
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

#include "pic32mz_gpio.h"
#include "pic32mz-starterkit.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Pin configuration for each start kit switch.  This array is indexed by
 * the BUTTON_* definitions in board.h
 */

static const pinset_t g_buttons[NUM_BUTTONS] =
{
  GPIO_SW_1, GPIO_SW_2, GPIO_SW_3
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

  /* Configure the GPIO pins as inputs.  NOTE that EXTI interrupts are
   * configured for all pins.
   */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      pic32mz_configgpio(g_buttons[i]);
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

  /* Check that state of each key */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      /* A LOW value means that the key is pressed. */

      bool released = pic32mz_gpioread(g_buttons[i]);

      /* Accumulate the set of depressed (not released) keys */

      if (!released)
        {
            ret |= (1 << i);
        }
    }

  return ret;
}

/****************************************************************************
 * Button interrupt support.
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupthandlers.
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
#ifdef CONFIG_PIC32MZ_GPIOIRQ_PORTB
  int ret = OK;

  if ((unsigned)id < NUM_BUTTONS)
    {
      /* Perform the attach/detach operation */

      ret = pic32mz_gpioattach(g_buttons[id], irqhandler, arg);

      /* The interrupt is now disabled.  Are we attaching or detaching from
       * button interrupt?
       */

      if (ret >= 0)
        {
          /* Attaching... enable button interrupts now */

          pic32mz_gpioirqenable(g_buttons[id]);
        }
    }

  return ret;
#else
  return -ENOSYS;
#endif
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
