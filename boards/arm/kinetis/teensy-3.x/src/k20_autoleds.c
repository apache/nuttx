/****************************************************************************
 * boards/arm/kinetis/teensy-3.x/src/k20_autoleds.c
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

#include <nuttx/board.h>

#include "kinetis.h"
#include "teensy-3x.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 *
 * Description:
 *   Initialize LED GPIOs so that LEDs can be controlled.
 *
 ****************************************************************************/

void board_autoled_initialize(void)
{
  kinetis_pinconfig(GPIO_LED);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (led != 2)
    {
      kinetis_gpiowrite(GPIO_LED, (led != 0));
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led != 2)
    {
      kinetis_gpiowrite(GPIO_LED, false);
    }
}

#endif /* CONFIG_ARCH_LEDS */
