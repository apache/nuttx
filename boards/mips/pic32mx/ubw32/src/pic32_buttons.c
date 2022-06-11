/****************************************************************************
 * boards/mips/pic32mx/ubw32/src/pic32_buttons.c
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
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "chip.h"
#include "mips_internal.h"
#include "pic32mx.h"
#include "ubw32.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The UBW32 board has three buttons.
 *
 * PROGRAM RE7  Pulled high, Grounded/low when depressed
 * USER    RE6  Pulled high, Grounded/low when depressed
 * RESET        Not software accessible
 */

#define GPIO_PROGRAM (GPIO_INPUT|GPIO_INT|GPIO_PORTE|GPIO_PIN7)
#define GPIO_USER    (GPIO_INPUT|GPIO_INT|GPIO_PORTE|GPIO_PIN6)

/* Change notification numbers:
 *
 * RE7 -> No change notification associated with RE7
 * RE6 -> No change notification associated with RE6
 */

#define CN_PROGRAM
#define CN_USER

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Pin configuration for each button */

static const uint16_t g_buttonset[NUM_BUTTONS] =
{
  GPIO_PROGRAM GPIO_USER
}

/* Change notification number for each button */

#ifdef CONFIG_ARCH_IRQBUTTONS
static const uint8_t g_buttoncn[NUM_BUTTONS] =
{
  CN_PROGRAM, CN_USER
}
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

  /* Configure input pins */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      pic32mx_configgpio(g_buttonset[i]);
    }

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  int id;

  /* Configure input pins */

  for (id = 0; id < NUM_BUTTONS; id++)
    {
      if (pic32mx_gpioread(g_buttonset[id]))
        {
          ret |= (1 << id);
        }
    }

  return ret;
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
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   32-bit bit set with each bit associated with a button.
 *   See the BUTTON_*_BIT definitions in board.h for the meaning of each bit.
 *
 *   board_button_irq() may be called to register an interrupt handler that
 *   will be called when a button is depressed or released.  The ID value is
 *   a button enumeration value that uniquely identifies a button resource.
 *   See the BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 *   Interrupts are automatically enabled when the button handler is attached
 *   and automatically disabled when the button handler is detached.
 *
 *   When an interrupt occurs, it is due to a change on the GPIO input pin
 *   associated with the button.  In that case, all attached change
 *   notification handlers will be called.  Each handler must maintain state
 *   and determine if the unlying GPIO button input value changed.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  int ret = -EINVAL;

  if (id < NUM_BUTTONS)
    {
      pic32mx_gpioirqdisable(g_buttoncn[id]);
      ret = pic32mx_gpioattach(g_buttonset[id], g_buttoncn[id],
                               irqhandler, arg);
      if (ret >= 0)
        {
          pic32mx_gpioirqenable(g_buttoncn[id]);
        }
    }

  return ret;
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
