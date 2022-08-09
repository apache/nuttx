/****************************************************************************
 * boards/arm/lpc17xx_40xx/zkit-arm-1769/src/lpc17_40_buttons.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "lpc17_40_gpio.h"
#include "zkit-arm-1769.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Pin configuration for each zkit-arm-1769 button.
 * This array is indexed by NUM_BUTTONS in board.h
 */

static const uint16_t g_buttons[NUM_BUTTONS] =
{
  ZKITARM_KEY1, ZKITARM_KEY2, ZKITARM_KEY3, ZKITARM_KEY4, ZKITARM_KEY5
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
   * configured for some pins but NOT used in this file
   */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      lpc17_40_configgpio(g_buttons[i]);
    }

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  bool released;
  int i;

  /* Check that state of each key */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      released = lpc17_40_gpioread(g_buttons[i]);

      /* Accumulate set of depressed keys */

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
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   32-bit bit set with each bit associated with a button.
 *   See the BUTTON_*_BIT and JOYSTICK_*_BIT definitions in board.h for the
 *   meaning of each bit.
 *
 *   board_button_irq() may be called to register an interrupt handler that
 *   will be called when a button is depressed or released.
 *   The ID value is a button enumeration value that uniquely identifies a
 *   button resource.
 *   See the BUTTON_* and JOYSTICK_* definitions in board.h for the meaning
 *   of enumeration value.
 *
 ****************************************************************************/

#if defined CONFIG_ARCH_IRQBUTTONS && CONFIG_LPC17_40_GPIOIRQ
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  irqstate_t flags;
  int ret = -EINVAL;

  /* Interrupts are supported on KEY5 only */

  if (id == BOARD_BUTTON_5)
    {
      flags = enter_critical_section();

      /* Attach or detach the interrupt handler for KEY5. */

      if (irqhandler)
        {
          /* Configure KEY5 as an interrupting input */

          lpc17_40_configgpio(ZKITARM_INT_KEY5);

          /* Attach the new interrupt handler and enable the interrupt */

          ret = irq_attach(ZKITARM_KEY5_IRQ, irqhandler, arg);
          if (ret == OK)
            {
              up_enable_irq(ZKITARM_KEY5_IRQ);
            }
        }
      else
        {
          /* Disable the interrupt and detach the handler */

          up_disable_irq(ZKITARM_KEY5_IRQ);
          irq_detach(ZKITARM_KEY5_IRQ);

          /* Configure KEY5 as a non-interrupting input */

          lpc17_40_configgpio(ZKITARM_KEY5);
          ret = OK;
        }

      leave_critical_section(flags);
    }

  return ret;
}

#endif
#endif /* CONFIG_ARCH_BUTTONS */
