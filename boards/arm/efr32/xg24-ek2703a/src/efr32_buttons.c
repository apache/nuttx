/****************************************************************************
 * boards/arm/efr32/xg24-ek2703a/src/efr32_buttons.c
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
#include <stdbool.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "efr32_gpio.h"
#include "xg24-ek2703a.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const gpio_pinset_t g_buttoncfg[NUM_BUTTONS] =
{
  GPIO_BTN0,
  GPIO_BTN1,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button
 *   resources.  After that, board_buttons() may be called to collect the
 *   current state of all buttons.
 *
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  int i;

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      efr32_configgpio(g_buttoncfg[i]);
    }

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   Return the state of all buttons, one bit per button, set while the
 *   button is pressed.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  int i;

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      /* The buttons are low when pressed */

      if (!efr32_gpioread(g_buttoncfg[i]))
        {
          ret |= 1 << i;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   Attach an interrupt handler to a button, called on press and on
 *   release.  A NULL handler disables the interrupt.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  irqstate_t flags;
  int irq;
  int ret;

  if (id < 0 || id >= NUM_BUTTONS)
    {
      return -EINVAL;
    }

  irq = EFR32_IRQ_EXTI(GPIO_PINNO(g_buttoncfg[id]));

  flags = enter_critical_section();
  if (irqhandler != NULL)
    {
      ret = efr32_gpioirq(g_buttoncfg[id]);
      if (ret >= 0)
        {
          irq_attach(irq, irqhandler, arg);
          up_enable_irq(irq);
        }
    }
  else
    {
      up_disable_irq(irq);
      irq_detach(irq);
      ret = OK;
    }

  leave_critical_section(flags);
  return ret;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
