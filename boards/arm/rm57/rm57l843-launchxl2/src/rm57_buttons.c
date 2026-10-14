/****************************************************************************
 * boards/arm/rm57/rm57l843-launchxl2/src/rm57_buttons.c
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
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "rm57_gio.h"
#include "rm57l843-launchxl2.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef HAVE_IRQBUTTONS
#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_RM57_GIO_IRQ)
#  define HAVE_IRQBUTTONS 1
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_IRQBUTTONS
static xcpt_t g_irq_button_b4;
static xcpt_t g_irq_button_b5;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_irqx
 *
 * Description:
 *   This function implements the core of the board_button_irq() logic.
 *
 ****************************************************************************/

#ifdef HAVE_IRQBUTTONS
static int board_button_irqx(gio_pinset_t pinset, int irq,
                              xcpt_t irqhandler, xcpt_t *store, void *arg)
{
  irqstate_t flags;

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = enter_critical_section();

  /* Get the old button interrupt handler and save the new one */

  *store = irqhandler;

  /* Are we attaching or detaching? */

  if (irqhandler != NULL)
    {
      /* Configure the interrupt */

      rm57_gioirq(pinset);
      irq_attach(irq, irqhandler, arg);
      rm57_gioirqenable(irq);
    }
  else
    {
      /* Detach and disable the interrupt */

      irq_detach(irq);
      rm57_gioirqdisable(irq);
    }

  leave_critical_section(flags);

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  /* Configure button GIOs for input */

  rm57_configgio(GIO_BUTTON_B4);
  rm57_configgio(GIO_BUTTON_B5);
  return BOARD_NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;

  /* Active-low: a pin reading false (0V) means the button is pressed. */

  if (!rm57_gioread(GIO_BUTTON_B4))
    {
      ret |= BOARD_BUTTON_B4_BIT;
    }

  if (!rm57_gioread(GIO_BUTTON_B5))
    {
      ret |= BOARD_BUTTON_B5_BIT;
    }

  return ret;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is one
 *   of the BOARD_BUTTON_* definitions in board.h.  The interrupt fires on
 *   both edges (press and release); the handler must re-read the button
 *   state (e.g. via board_buttons()) to tell which edge occurred.
 *
 * Configuration Notes:
 *   CONFIG_ARCH_IRQBUTTONS and CONFIG_RM57_GIO_IRQ must both be selected
 *   to use this interface.
 *
 ****************************************************************************/

int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
#ifdef HAVE_IRQBUTTONS
  if (id == BOARD_BUTTON_B4)
    {
      return board_button_irqx(GIO_BUTTON_B4, RM57_IRQ_GIOB4,
                                irqhandler, &g_irq_button_b4, arg);
    }
  else if (id == BOARD_BUTTON_B5)
    {
      return board_button_irqx(GIO_BUTTON_B5, RM57_IRQ_GIOB5,
                                irqhandler, &g_irq_button_b5, arg);
    }
#endif

  return -EINVAL;
}

#endif /* CONFIG_ARCH_BUTTONS */
