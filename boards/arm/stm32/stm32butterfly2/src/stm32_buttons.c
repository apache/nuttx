/****************************************************************************
 * boards/arm/stm32/stm32butterfly2/src/stm32_buttons.c
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

#include "stm32_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NUM_BUTTONS 5

#define GPIO_JOY_O (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT |\
                    GPIO_PORTC | GPIO_PIN7)
#define GPIO_JOY_U (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT |\
                    GPIO_PORTC | GPIO_PIN8)
#define GPIO_JOY_D (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT |\
                    GPIO_PORTC | GPIO_PIN9)
#define GPIO_JOY_R (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT |\
                    GPIO_PORTC | GPIO_PIN10)
#define GPIO_JOY_L (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT |\
                    GPIO_PORTC | GPIO_PIN11)

/****************************************************************************
 * Private Declarations
 ****************************************************************************/

static const uint32_t buttons[NUM_BUTTONS] =
{
  GPIO_JOY_O, GPIO_JOY_U, GPIO_JOY_D, GPIO_JOY_R, GPIO_JOY_L
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *      Initializes gpio pins for joystick buttons
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  int i;

  for (i = 0; i != NUM_BUTTONS; ++i)
    {
      stm32_configgpio(buttons[i]);
    }

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *      Reads keys
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t rv = 0;
  int i;

  for (i = 0; i != NUM_BUTTONS; ++i)
    {
      if (stm32_gpioread(buttons[i]) == 0)
        {
          rv |= 1 << i;
        }
    }

  return rv;
}
