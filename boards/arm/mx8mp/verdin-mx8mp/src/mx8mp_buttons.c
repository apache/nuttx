/****************************************************************************
 * boards/arm/mx8mp/verdin-mx8mp/src/mx8mp_buttons.c
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

#include <sys/types.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "mx8mp_gpio.h"
#include "verdin-mx8mp.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_irqx
 *
 * Description:
 *   This function implements the core of the board_button_irq() logic.
 *
 ****************************************************************************/

static int board_button_irqx(gpio_pinset_t pinset, int irq,
                             xcpt_t irqhandler, void *arg)
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

      irq_attach(irq, irqhandler, arg);
      mx8mp_gpio_irq_enable (pinset);

      /* Then make sure that interrupts are enabled on the pin */

      up_enable_irq(irq);
    }
  else
    {
      /* Detach and disable the interrupt */

      irq_detach(irq);
      mx8mp_gpio_irq_disable(pinset);
    }

  leave_critical_section(flags);
  return OK;
}

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
  /* Configure the buttons as input */

  mx8mp_iomuxc_config(BUTTON_1_IOMUX);
  mx8mp_gpio_config(BUTTON_1_GPIO);

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   8-bit bit set with each bit associated with a button.  See the
 *   BUTTON_*_BIT  definitions in board.h for the meaning of each bit.
 *
 ****************************************************************************/

uint8_t board_buttons(void)
{
  uint8_t ret = 0;

  if (!mx8mp_gpio_read(BUTTON_1_GPIO))
    {
      ret |= BUTTON_1_BIT;
    }

  return ret;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
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
  /* The button has already been configured as an interrupting input (by
   * board_button_initialize() above).
   *
   * Attach the new button handler.
   */

  switch (id)
    {
      case BUTTON_1:
        return board_button_irqx(BUTTON_1_GPIO, BUTTON_1_IRQ,
                                 irqhandler, arg);

      default:
        return -EINVAL;
    }
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
