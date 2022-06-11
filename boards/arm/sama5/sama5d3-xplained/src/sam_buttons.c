/****************************************************************************
 * boards/arm/sama5/sama5d3-xplained/src/sam_buttons.c
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

/* There are five push button switches on the SAMA5D3-Xplained base board:
 *
 *   1. One Reset, board reset (BP1)
 *   2. One Wake up, push button to bring the processor out of low power mode
 *     (BP2)
 *   3. One User momentary Push Button
 *   4. One Disable CS Push Button
 *
 * Only the momentary push button is controllable by software (labeled
 * "PB_USER1" on the board):
 *
 *   - PE27.  Pressing the switch connect PE27 to grounded.  Therefore, PE27
 *     must be pulled high internally.  When the button is pressed the SAMA5
 *     will sense "0" is on PE27.
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
#include "sama5d3-xplained.h"

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
  sam_configpio(PIO_USER);
  return 1;
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
  return sam_pioread(PIO_USER) ? 0 : BUTTON_USER_BIT;
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
 *   overall PIO IRQ feature and CONFIG_SAMA5_PIOE_IRQ must be enabled to
 *   select PIOs to support interrupts on PIOE.
 *
 ****************************************************************************/

#if defined(CONFIG_SAMA5_PIOE_IRQ) && defined(CONFIG_ARCH_IRQBUTTONS)
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
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

          sam_pioirq(PIO_USER);
          irq_attach(IRQ_USER1, irqhandler, arg);
          sam_pioirqenable(IRQ_USER1);
        }
      else
        {
          /* Disable and detach the interrupt */

          sam_pioirqdisable(IRQ_USER1);
          irq_detach(IRQ_USER1);
        }

      leave_critical_section(flags);
      ret = OK;
    }

  return ret;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
