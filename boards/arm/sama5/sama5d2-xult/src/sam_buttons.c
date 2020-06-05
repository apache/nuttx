/****************************************************************************
 *  boards/arm/sama5/sama5d2-xult/src/sam_buttons.c
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

/* A single button, PB_USER (PB6), is available on the SAMA5D2-XULT
 *
 *  ------------------------------ ------------------- ----------------------
 *  SAMA5D2 PIO                    SIGNAL              USAGE
 *  ------------------------------ ------------------- ----------------------
 *  PB6                            USER_PB_PB6         PB_USER push button
 *  ------------------------------ ------------------- ----------------------
 *
 *  Closing PB_USER will bring PB6 to ground so 1) PB6 should have a weak
 *  pull-up,  and 2) when PB_USER is pressed, a low value will be senses.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "sam_pio.h"
#include "sama5d2-xult.h"

#ifdef CONFIG_ARCH_BUTTONS

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
  sam_configpio(PIO_BTN_USER);
  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   32-bit bit set with each bit associated with a button.  See the BUTTON*
 *   definitions above for the meaning of each bit in the returned value.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  return sam_pioread(PIO_BTN_USER) ? 0 : BUTTON_USER_BIT;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is one
 *   of the BUTTON* definitions provided above.
 *
 * Configuration Notes:
 *   Configuration CONFIG_SAMA5_PIO_IRQ must be selected to enable the
 *   overall PIO IRQ feature and CONFIG_SAMA5_PIOB_IRQ must be enabled to
 *   select PIOs to support interrupts on PIOE.
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_PIOB_IRQ) && defined(CONFIG_ARCH_IRQBUTTONS)
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
  int ret = -EINVAL;

  if (id == BUTTON_USER)
    {
      irqstate_t flags;

      /* Disable interrupts until we are done.  This guarantees that the
       * following operations are atomic.
       */

      flags = enter_critical_section();

      /* Are we attaching or detaching? */

      if (irqhandler != NULL)
        {
          /* Configure the interrupt */

          sam_pioirq(PIO_BTN_USER);
          irq_attach(IRQ_BTN_USER, irqhandler, arg);
          sam_pioirqenable(IRQ_BTN_USER);
        }
      else
        {
          /* Disable and detach the interrupt */

          sam_pioirqdisable(IRQ_BTN_USER);
          irq_detach(IRQ_BTN_USER);
        }

      leave_critical_section(flags);
      ret = OK;
    }

  return ret;
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
