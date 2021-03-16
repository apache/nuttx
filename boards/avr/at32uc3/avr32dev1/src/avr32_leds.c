/****************************************************************************
 * boards/avr/at32uc3/avr32dev1/src/avr32_leds.c
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

#include <nuttx/board.h>

#include "at32uc3.h"
#include "avr32dev1.h"

#ifdef CONFIG_ARCH_LEDS

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
 * Name: board_autoled_initializeialize
 ****************************************************************************/

void board_autoled_initializeialize(void)
{
  at32uc3_configgpio(PINMUX_GPIO_LED1);
  at32uc3_configgpio(PINMUX_GPIO_LED2);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  at32uc3_gpiowrite(PINMUX_GPIO_LED1, (led != 0));
  at32uc3_gpiowrite(PINMUX_GPIO_LED2, (led == 2));
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  at32uc3_gpiowrite(PINMUX_GPIO_LED1, (led == 2));
  at32uc3_gpiowrite(PINMUX_GPIO_LED2, false);
}
#endif /* CONFIG_ARCH_LEDS */
