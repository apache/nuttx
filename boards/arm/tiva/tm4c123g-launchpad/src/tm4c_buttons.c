/****************************************************************************
 * boards/arm/tiva/tm4c123g-launchpad/src/tm4c_buttons.c
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

#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/irq.h>
#include <arch/board/board.h>

#include "tiva_gpio.h"
#include "tm4c123g-launchpad.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
  /* Unlock GPIOF from NMI to use it w/ buttons see Register 19 of GPIOs
   * and see table 10-10 in datasheet for pins with special considerations.
   */

  tiva_gpio_lockport(GPIO_SW2, false);

  /* Configure the GPIO pins as inputs.  NOTE that EXTI interrupts are
   * configured for some pins but NOT used in this file
   */

  tiva_configgpio(GPIO_SW1);
  tiva_configgpio(GPIO_SW2);

  /* These pins need to be set to logic high (3.3V) so that the buttons
   * can pull them to logic low (0V)
   */

  tiva_gpiowrite(GPIO_SW1, true);
  tiva_gpiowrite(GPIO_SW2, true);

  /* Configure GPIO interrupts */

#ifdef CONFIG_ARCH_IRQBUTTONS
  tiva_gpioirqinitialize();
#endif
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
  uint32_t ret = 0;

  /* Check that state of each key.  A LOW value means that the key is
   * pressed.
   */

  if (!tiva_gpioread(GPIO_SW1))
    {
      ret |= BUTTON_SW1_BIT;
    }

  if (!tiva_gpioread(GPIO_SW2))
    {
      ret |= BUTTON_SW2_BIT;
    }

  return ret;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is one
 *   of the BUTTON* definitions provided above.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  uint32_t pinset = 0;
  int ret;

  /* Determine which switch to set the irq handler for */

  switch (id)
    {
      case BUTTON_SW1:
        pinset = GPIO_SW1;
        break;

      case BUTTON_SW2:
        pinset = GPIO_SW2;
        break;

      default:
        return -EINVAL;
    }

    /* Are we attaching or detaching? */

    if (irqhandler != NULL)
      {
        ret = tiva_gpioirqattach(pinset, irqhandler, arg);
      }
    else
      {
        ret = tiva_gpioirqdetach(pinset);
      }

  return ret;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
