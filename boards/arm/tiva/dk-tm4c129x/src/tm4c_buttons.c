/****************************************************************************
 * boards/arm/tiva/dk-tm4c129x/src/tm4c_buttons.c
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

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "dk-tm4c129x.h"

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
  GPIO_SW2, GPIO_SW3, GPIO_SW4
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
 * Button support.
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.
 *   board_buttons() returns an 32-bit bit set with each bit associated with
 *   a button.
 *   See the BUTTON_*_BIT and JOYSTICK_*_BIT definitions in board.h for the
 *    meaning of each bit.
 *
 *   board_button_irq() may be called to register an interrupt handler that
 *   will be called when a button is depressed or released.  The ID value is
 *   a button enumeration value that uniquely identifies a button resource.
 *   See the BUTTON_* and JOYSTICK_* definitions in board.h for the meaning
 *   of enumeration value.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_TIVA_GPIOP_IRQS)
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  irqstate_t flags;
  int ret = -EINVAL;

  /* Interrupts are supported only on ports P and Q and,
   * hence, only on button SW4
   */

  if (id == BUTTON_SW4)
    {
      /* The following should be atomic */

      flags = enter_critical_section();

      /* Detach and disable the button interrupt */

      up_disable_irq(IRQ_SW4);
      irq_detach(IRQ_SW4);

      /* Attach the new handler if so requested */

      if (irqhandler != NULL)
        {
          ret = irq_attach(IRQ_SW4, irqhandler, arg);
          if (ret == OK)
            {
              up_enable_irq(IRQ_SW4);
            }
        }

      leave_critical_section(flags);
    }

  return ret;
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
