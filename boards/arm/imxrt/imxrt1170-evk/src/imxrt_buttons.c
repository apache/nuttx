/****************************************************************************
 * boards/arm/imxrt/imxrt1170-evk/src/imxrt_buttons.c
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
#include "imxrt_config.h"
#include "imxrt_irq.h"
#include "imxrt_gpio.h"
#include "imxrt1170-evk.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The IMXRT1170-EVK board has seven (sets of) switches or buttons:
 *
 *   - External boot configuration switches (SW1)
 *   - External boot configuration switches (SW2)
 *   - MCU reset button (SW3)
 *   - System Reset Button / POR_BUTTON (SW4)
 *   - 5V power switch (SW5)
 *   - CPU ONOFF Button (SW6)
 *   - CPU Wakeup Button / USER_BUTTON (SW7)
 *
 * Only one button, SW7, can be used for user input.
 */

const uint32_t gpio_pins[NUM_BUTTONS]     =
                                            {
                                              GPIO_SW7
                                            };
const uint32_t gpio_pins_int[NUM_BUTTONS] =
                                            {
                                              GPIO_SW7_INT
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
  uint32_t i;

  /* Configure the buttons as input */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      imxrt_config_gpio(gpio_pins[i]);
    }

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
  uint8_t i   = 0;

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      ret |= ((!imxrt_gpio_read(gpio_pins[i])) << i);
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
  int ret = -EINVAL;

  /* The button has already been configured as an interrupting input (by
   * board_button_initialize() above).
   *
   * Attach the new button handler.
   */

  if (id < NUM_BUTTONS)
    {
      uint32_t irqnum = gpio_pins_int[id];
      if (irqhandler)
        {
          ret = irq_attach(irqnum, irqhandler, arg);
          imxrt_gpioirq_enable (irqnum);

          /* Then make sure that interrupts are enabled on the pin */

          up_enable_irq(irqnum);
        }
      else
        {
          up_disable_irq(irqnum);
          imxrt_gpioirq_disable(irqnum);
          ret = OK;
        }
    }

  return ret;
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
