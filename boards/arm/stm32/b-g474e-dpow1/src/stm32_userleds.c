/****************************************************************************
 *  boards/arm/stm32/b-g474e-dpow1/src/stm32_userleds.c
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
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "stm32.h"
#include "b-g474e-dpow1.h"

#if !defined(CONFIG_ARCH_LEDS)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 *
 * Description:
 *   Initialize the user LEDs before use. Note: For this function to be
 *   available to user application logic, CONFIG_ARCH_LEDS must not be
 *   defined.
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure LED GPIOs for output */

  stm32_configgpio(GPIO_LED1);
  stm32_configgpio(GPIO_LED2);
  stm32_configgpio(GPIO_LED3);
  stm32_configgpio(GPIO_LED4);
  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 *
 * Description:
 *   Allow user application logic to control LEDs one at a time. Note: For
 *   this function to be available to user application logic,
 *   CONFIG_ARCH_LEDS must not be defined.
 *
 * Parameters:
 *   led: Index to the LED, which may be one of the defines BOARD_LED1,
 *        BOARD_LED2, BOARD_LED3, or BOARD_LED4.
 *   ledon: true to turn the LED on, false to turn it off.
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  switch (led)
    {
      case BOARD_LED1:
        stm32_gpiowrite(GPIO_LED1, ledon);
        break;

      case BOARD_LED2:
        stm32_gpiowrite(GPIO_LED2, ledon);
        break;

      case BOARD_LED3:
        stm32_gpiowrite(GPIO_LED3, ledon);
        break;

      case BOARD_LED4:
        stm32_gpiowrite(GPIO_LED4, ledon);
        break;
    }
}

/****************************************************************************
 * Name: board_userled_all
 *
 * Description:
 *   Allow user application logic to control all LEDs in one function call.
 *   Note: For this function to be available to user application logic,
 *   CONFIG_ARCH_LEDS must not be defined.
 *
 * Parameters:
 *   ledset: Bitmask indicating the new state for all LEDs, where a set bit
 *           indicates LED on and a clear bit indicates LED off. To
 *           construct the bitmask, using a bitwise OR of the defines
 *           BOARD_LED1_BIT, BOARD_LED2_BIT, BOARD_LED3_BIT, and/or
 *           BOARD_LED4_BIT.
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  stm32_gpiowrite(GPIO_LED1, (ledset & BOARD_LED1_BIT) != 0);
  stm32_gpiowrite(GPIO_LED2, (ledset & BOARD_LED2_BIT) != 0);
  stm32_gpiowrite(GPIO_LED3, (ledset & BOARD_LED3_BIT) != 0);
  stm32_gpiowrite(GPIO_LED4, (ledset & BOARD_LED4_BIT) != 0);
}

#endif /* !CONFIG_ARCH_LEDS */
