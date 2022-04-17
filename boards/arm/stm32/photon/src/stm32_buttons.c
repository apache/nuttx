/****************************************************************************
 * boards/arm/stm32/photon/src/stm32_buttons.c
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

#include <debug.h>
#include <errno.h>

#include <arch/board/board.h>
#include "photon.h"

#include "stm32_gpio.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  /* Configure Photon button gpio as input */

  stm32_configgpio(GPIO_BUTTON1);
  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  /* Check the state of the only button */

  if (stm32_gpioread(GPIO_BUTTON1))
    {
      return BOARD_BUTTON1_BIT;
    }

  return 0;
}

/****************************************************************************
 * Name: board_button_irq
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  if (id != BOARD_BUTTON1)
    {
      /* Invalid button id */

      return -EINVAL;
    }

  /* Configure interrupt on falling edge only */

  return stm32_gpiosetevent(GPIO_BUTTON1, false, true, false,
                            irqhandler, arg);
}
#endif /* CONFIG_ARCH_IRQBUTTONS */
