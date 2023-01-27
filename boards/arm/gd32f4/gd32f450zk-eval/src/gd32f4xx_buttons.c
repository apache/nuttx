/****************************************************************************
 * boards/arm/gd32f4/gd32f450zk-eval/src/gd32f4xx_buttons.c
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

#include <stddef.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/board.h>

#include <arch/board/board.h>

#include "gd32f4xx.h"
#include "gd32f450z_eval.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t g_button_boards[NUM_BUTTONS] =
{
  GPIO_BTN_USER, GPIO_BTN_TAMPER, GPIO_BTN_WAKEUP
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

  /* Configure each button */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      gd32_gpio_config(g_button_boards[i]);
    }

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   32-bit bit set with each bit associated with a button.  See the BUTTON*
 *   definitions above for the meaning of each bit in the returned value.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret;
  int i;

  /* Check each button */

  for (i = 0, ret = 0; i < NUM_BUTTONS; i++)
    {
      /* The low value is sensed when the button is pressed */

      if (!gd32_gpio_read(g_button_boards[i]))
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
 *   After that, board_buttons() may be called to collect the current
 *   state of all buttons or board_button_irq() may be called to register
 *   button interrupt handlers.
 *
 *   After board_button_initialize() has been called, board_buttons()
 *   may be called to collect the state of all buttons.  board_buttons()
 *   returns an 32-bit bit set with each bit associated with a button.
 *   See the BUTTON_*_BIT definitions in board.h for the meaning of each
 *   bit.
 *
 *   board_button_irq() may be called to register an interrupt handler that
 *    will be called when a button is depressed or released.  The ID value
 *   is a button enumeration value that uniquely identifies a button
 *   resource. See the BUTTON_* definitions in board.h for the meaning of
 *   enumeration value.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  if (id >= 0 && id < NUM_BUTTONS)
    {
      uint8_t gpio_irq;
      uint8_t gpio_irqnum;
      irqstate_t flags;
      int ret;

      /* Disable interrupts until we are done.  This guarantees that the
       * following operations are atomic.
       */

      flags = enter_critical_section();

      /* Get gpio irq numbers */

      ret = gd32_gpio_exti_irqnum_get(g_button_boards[id], &gpio_irqnum);

      if (ret < 0)
        {
          leave_critical_section(flags);
          return ret;
        }

      /* Are we attaching or detaching? */

      if (irqhandler != NULL)
        {
          ret = gd32_exti_gpioirq_init(g_button_boards[id], EXTI_INTERRUPT,
                                       EXTI_TRIG_RISING, &gpio_irq);

          if (ret < 0)
            {
              leave_critical_section(flags);
              return ret;
            }

          /* Attach and enable the interrupt */

          gd32_exti_gpio_irq_attach(gpio_irq, irqhandler, arg);
          up_enable_irq(gpio_irqnum);
        }
      else
        {
          /* Disable and detach the interrupt */

          up_disable_irq(gpio_irqnum);
          irq_detach(gpio_irqnum);
        }

      leave_critical_section(flags);
    }

  /* Return the old button handler (so that it can be restored) */

  return OK;
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
