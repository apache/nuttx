/****************************************************************************
 * boards/arm/stm32g4/nucleo-g491re/src/stm32_buttons.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>

#include <nuttx/board.h>

#include "nucleo-g491re.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  stm32_configgpio(GPIO_BTN_USER);
  return NUM_BUTTONS;
}

uint32_t board_buttons(void)
{
  return stm32_gpioread(GPIO_BTN_USER) ? BUTTON_USER_BIT : 0;
}

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  if (id != BUTTON_USER)
    {
      return -EINVAL;
    }

  return stm32_gpiosetevent(GPIO_BTN_USER, true, true, true,
                            irqhandler, arg);
}
#endif
