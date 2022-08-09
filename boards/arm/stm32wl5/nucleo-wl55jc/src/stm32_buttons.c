/****************************************************************************
 * boards/arm/stm32wl5/nucleo-wl55jc/src/stm32_buttons.c
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
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "nucleo-wl55jc.h"

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
  /* Configure the single button as an input.  NOTE that EXTI interrupts are
   * also configured for the pin.
   */

  stm32wl5_configgpio(GPIO_BUTTON1);
  stm32wl5_configgpio(GPIO_BUTTON2);
#ifndef CONFIG_ARCH_BOARD_NUCLEO_WL55JC_DEMO_LED_IRQ
  stm32wl5_configgpio(GPIO_BUTTON3);
  return 3; /* number of buttons */
#else
  return 2; /* number of buttons */
#endif
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t state;

  /* Check that state of each USER button.
   * A LOW value means that the key is pressed.
   */

  state = 0;

  if (stm32wl5_gpioread(GPIO_BUTTON1) == 0)
    {
      state |= BUTTON1_BIT;
    }

  if (stm32wl5_gpioread(GPIO_BUTTON2) == 0)
    {
      state |= BUTTON2_BIT;
    }

#ifndef CONFIG_ARCH_BOARD_NUCLEO_WL55JC_DEMO_LED_IRQ
  if (stm32wl5_gpioread(GPIO_BUTTON3) == 0)
    {
      state |= BUTTON3_BIT;
    }
#endif

  return state;
}

/****************************************************************************
 * Button support.
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns a
 *   32-bit bit set with each bit associated with a button.  See the
 *   BUTTON_*_BIT definitions in board.h for the meaning of each bit.
 *
 *   board_button_irq() may be called to register an interrupt handler that
 *   will be called when a button is depressed or released. The ID value is a
 *   button enumeration value that uniquely identifies a button resource. See
 *   the BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  int ret = -EINVAL;

  if (id == BOARD_BUTTON1)
    {
      ret = stm32wl5_gpiosetevent(GPIO_BUTTON1, true, true, false,
                                  irqhandler, arg);
    }

  if (id == BOARD_BUTTON2)
    {
      ret = stm32wl5_gpiosetevent(GPIO_BUTTON2, true, true, false,
                                  irqhandler, arg);
    }

#ifndef CONFIG_ARCH_BOARD_NUCLEO_WL55JC_DEMO_LED_IRQ
  if (id == BOARD_BUTTON3)
    {
      ret = stm32wl5_gpiosetevent(GPIO_BUTTON3, true, true, false,
                                  irqhandler, arg);
    }
#endif

  return ret;
}
#endif
