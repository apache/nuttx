/****************************************************************************
 * boards/arm/stm32h7/devebox-stm32h743/src/stm32_buttons.c
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
#include <nuttx/board.h>

#include "stm32_gpio.h"
#include "devebox-stm32h743.h"

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
  /* Configure the buttons as an input.  NOTE that EXTI interrupts are
   * also configured for the pin.
   */

  stm32_configgpio(USER_BTN1);
  stm32_configgpio(USER_BTN2);
  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   Returns current button states as a bitmask. Bit 0 = USER_BTN1, bit 1 =
 *   USER_BTN2. A set bit means the button is pressed (active low logic).
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;

  /* Button 1 – check if low (pressed) */

  if (!stm32_gpioread(USER_BTN1))
    {
      ret |= (1 << 0);
    }

  /* Button 2 – check if low (pressed) */

  if (!stm32_gpioread(USER_BTN2))
    {
      ret |= (1 << 1);
    }

  return ret;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   Attach or detach an interrupt handler to a button. The driver calls this
   * function when it needs to enable/disable interrupts for a specific
   * button.
 *   The 'id' parameter is the button index (0 for button 1, 1 for button 2).
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  int ret = -EINVAL;

  switch (id)
    {
      case 0:  /* Button 1 */
        ret = stm32_gpiosetevent(USER_BTN1, true, true, true,
                                 irqhandler, arg);
        break;
      case 1:  /* Button 2 */
        ret = stm32_gpiosetevent(USER_BTN2, true, true, true,
                                 irqhandler, arg);
        break;

      default:
        break;
    }

  return ret;
}
#endif /* CONFIG_ARCH_IRQBUTTONS */

#endif /* CONFIG_ARCH_BUTTONS */
