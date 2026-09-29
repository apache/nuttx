/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/src/imxrt_buttons.c
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

#include "imxrt_gpio.h"
#include "imxrt1180-evk.h"

#ifdef CONFIG_ARCH_BUTTONS

#if defined(CONFIG_ARCH_IRQBUTTONS) && !defined(CONFIG_IMXRT_GPIO_IRQ)
#  error "CONFIG_ARCH_IRQBUTTONS requires CONFIG_IMXRT_GPIO_IRQ"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const gpio_pinset_t g_buttons[NUM_BUTTONS] =
{
  GPIO_SW8,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   Configure the button pins as interrupting inputs.  The interrupts stay
 *   disabled until a handler is attached with board_button_irq().
 *
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  int i;

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      imxrt_config_gpio(g_buttons[i]);
    }

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   Return the current state of the buttons, one bit per button, set when
 *   the button is pressed.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  int i;

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      /* The buttons are active low */

      if (!imxrt_gpio_read(g_buttons[i]))
        {
          ret |= (1 << i);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   Attach (irqhandler != NULL) or detach (irqhandler == NULL) an interrupt
 *   handler to a button.  The interrupt fires on both edges, i.e. on press
 *   and on release.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  gpio_pinset_t pinset;
  int ret;

  if (id < 0 || id >= NUM_BUTTONS)
    {
      return -EINVAL;
    }

  pinset = g_buttons[id];

  if (irqhandler != NULL)
    {
      ret = imxrt_gpioirq_attach(pinset, irqhandler, arg);
      if (ret == OK)
        {
          ret = imxrt_gpioirq_enable(pinset);
        }
    }
  else
    {
      ret = imxrt_gpioirq_disable(pinset);
      if (ret == OK)
        {
          ret = imxrt_gpioirq_attach(pinset, NULL, NULL);
        }
    }

  return ret;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
