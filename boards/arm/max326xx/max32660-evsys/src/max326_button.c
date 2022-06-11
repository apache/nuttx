/****************************************************************************
 * boards/arm/max326xx/max32660-evsys/src/max326_button.c
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
#include <nuttx/spinlock.h>

#include <nuttx/irq.h>

#include "max326_gpio.h"
#include "max32660-evsys.h"

#include <arch/board/board.h>

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
  max326_gpio_config(GPIO_BUTTON);
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
  return !max326_gpio_read(GPIO_BUTTON) ? 0 : BUTTON_SW2_BIT;
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
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  int ret = -EINVAL;

  if (id == BUTTON_SW2)
    {
      irqstate_t flags;

      /* Disable interrupts until we are done.  This guarantees that the
       * following operations are atomic.
       */

      flags = spin_lock_irqsave(NULL);

      /* Are we attaching or detaching? */

      if (irqhandler != NULL)
        {
          /* Configure the interrupt */

          irq_attach(BUTTON_IRQ, irqhandler, arg);
          up_enable_irq(BUTTON_IRQ);
        }
      else
        {
          /* Disable and detach the interrupt */

          up_disable_irq(BUTTON_IRQ);
          irq_detach(BUTTON_IRQ);
        }

      spin_unlock_irqrestore(NULL, flags);
      ret = OK;
    }

  return ret;
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
