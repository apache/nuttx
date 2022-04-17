/****************************************************************************
 * boards/arm/samd2l2/saml21-xplained/src/sam_buttons.c
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

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "sam_port.h"
#include "saml21-xplained.h"

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
  sam_configport(PORT_SW0);
  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called,
 *   board_buttons() may be called to collect the state of all buttons.
 *   board_buttons() returns an 32-bit bit set with each bit associated
 *   with a button.  See the BUTTON* definitions above for the meaning of
 *   each bit in the returned value.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  return sam_portread(PORT_SW0) ? 0 : BUTTON_SW0_BIT;
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
 *   Configuration CONFIG_AVR32_PORTIRQ must be selected to enable the
 *   overall PORT IRQ feature and CONFIG_AVR32_PORTIRQSETA and/or
 *   CONFIG_AVR32_PORTIRQSETB must be enabled to select PORTs to support
 *   interrupts on.  For button support, bits 2 and 3 must be set in
 *   CONFIG_AVR32_PORTIRQSETB (PB2 and PB3).
 *
 ****************************************************************************/

#if defined(CONFIG_PORTA_IRQ) && defined(CONFIG_ARCH_IRQBUTTONS)
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  int ret = -EINVAL;

  if (id == BUTTON_SW0)
    {
      irqstate_t flags;

      /* Disable interrupts until we are done.  This guarantees that the
       * following operations are atomic.
       */

      flags = enter_critical_section();

      /* Configure the interrupt */

      sam_portirq(IRQ_SW0);
      irq_attach(IRQ_SW0, irqhandler, arg);
      sam_portirqenable(IRQ_SW0);

      leave_critical_section(flags);
      ret = OK;
    }

  /* Return the old button handler (so that it can be restored) */

  return ret;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
