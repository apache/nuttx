/****************************************************************************
 * boards/arm/s32k1xx/s32k144evb/src/s32k1xx_buttons.c
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

/* The S32K144EVB supports two buttons:
 *
 *   SW2  PTC12
 *   SW3  PTC13
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>

#include <nuttx/board.h>

#include "s32k1xx_pin.h"

#include <arch/board/board.h>

#include "s32k144evb.h"

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
  /* Configure the GPIO pins as interrupting inputs */

  s32k1xx_pinconfig(GPIO_SW2);
  s32k1xx_pinconfig(GPIO_SW3);

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;

  if (s32k1xx_gpioread(GPIO_SW2))
    {
      ret |= BUTTON_SW2_BIT;
    }

  if (s32k1xx_gpioread(GPIO_SW3))
    {
      ret |= BUTTON_SW3_BIT;
    }

  return ret;
}

#ifdef CONFIG_ARCH_IRQBUTTONS
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
 *   will be called when a button is pressed or released.  The ID value is a
 *   button enumeration value that uniquely identifies a button resource.
 *   See the BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 ****************************************************************************/

int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  uint32_t pinset;
  int ret;

  /* Map the button id to the GPIO bit set */

  if (id == BUTTON_SW2)
    {
      pinset = GPIO_SW2;
    }
  else if (id == BUTTON_SW3)
    {
      pinset = GPIO_SW3;
    }
  else
    {
      return -EINVAL;
    }

  /* The button has already been configured as an interrupting input (by
   * board_button_initialize() above).
   *
   * Attach the new button handler.
   */

  ret = s32k1xx_pinirqattach(pinset, irqhandler, NULL);
  if (ret >= 0)
    {
      /* Then make sure that interrupts are enabled on the pin */

      s32k1xx_pinirqenable(pinset);
    }

  return ret;
}
#endif /* CONFIG_ARCH_IRQBUTTONS */
#endif /* CONFIG_ARCH_BUTTONS */
