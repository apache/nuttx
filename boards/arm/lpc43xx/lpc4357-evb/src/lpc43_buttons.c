/****************************************************************************
 * boards/arm/lpc43xx/lpc4357-evb/src/lpc43_buttons.c
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
#include <stdbool.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>

#include <arch/board/board.h>

#include "lpc4357-evb.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Pin configuration for each LPC4357-EVB button.  This array is indexed by
 * the BUTTON_* definitions in board.h
 */

#if 0 /* Not yet used */
static const uint16_t g_buttoncfg[NUM_BUTTONS] =
{
};

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_LPC43_GPIO_IRQ)
/* This array provides the mapping from button ID numbers to button IRQ
 * numbers.
 */

static uint8_t g_buttonirq[NUM_BUTTONS] =
{
};
#endif
#endif /* Not yet used */

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
#if 0 /* Not yet implemented */
  int i;

  /* Configure the GPIO pins as interrupting inputs. */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      lpc43_configgpio(g_buttoncfg[i]);
    }
#else
  return 0;
#endif
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons.
 *
 *   board_buttons() may be called at any time to harvest the state of every
 *   button.  The state of the buttons is returned as a bitset with one
 *   bit corresponding to each button:  If the bit is set, then the button
 *   is pressed.  See the BOARD_BUTTON_*_BIT and BOARD_JOYSTICK_*_BIT
 *   definitions in board.h for the meaning of each bit.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
#if 0 /* Not yet implemented */
  uint32_t ret = 0;
  int i;

  /* Check that state of each key */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      /* A LOW value means that the key is pressed. */

      bool released = lpc43_gpio_read(g_buttoncfg[i]);

      /* Accumulate the set of depressed (not released) keys */

      if (!released)
        {
            ret |= (1 << i);
        }
    }

  return ret;
#else
  return 0;
#endif /* Not yet implemented */
}

/****************************************************************************
 * Button support.
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 *   board_button_irq() may be called to register an interrupt handler that
 *   will be called when a button is depressed or released.
 *   The ID value is a button enumeration value that uniquely identifies a
 *   button resource.
 *   See the BOARD_BUTTON_* and BOARD_JOYSTICK_* definitions in board.h for
 *   the meaning of enumeration values.
 *
 *   Note that board_button_irq() also enables button interrupts.
 *   Button interrupts will remain enabled after the interrupt handler
 *   is attached.
 *   Interrupts may be disabled (and detached) by calling board_button_irq
 *   with irqhandler equal to NULL.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_LPC43_GPIO_IRQ)
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
#if 0 /* Not yet implemented */
  irqstate_t flags;
  int ret = -EINVAL;
  int irq;

  /* Verify that the button ID is within range */

  if ((unsigned)id < NUM_BUTTONS)
    {
      /* Disable interrupts until we are done */

      flags = enter_critical_section();

      /* Configure the interrupt.  Either attach and enable the new
       * interrupt or disable and detach the old interrupt handler.
       */

      irq = g_buttonirq[id];
      if (irqhandler)
        {
          /* Attach then enable the new interrupt handler */

          irq_attach(irq, irqhandler, arg);
          up_enable_irq(irq);
        }
      else
        {
          /* Disable then detach the old interrupt handler */

          up_disable_irq(irq);
          irq_detach(irq);
        }

      leave_critical_section(flags);
      ret = OK;
    }

  return ret;
#else
  return -ENOSYS;
#endif /* Not yet implemented */
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
