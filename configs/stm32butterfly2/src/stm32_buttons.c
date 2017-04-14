/*****************************************************************************
 * configs/stm32butterfly2/src/stm32_buttons.c
 *
 *   Copyright (C) 2016 Michał Łyszczek. All rights reserved.
 *   Author: Michał Łyszczek <michal.lyszczek@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/

/*****************************************************************************
 * Public Includes
 ****************************************************************************/

#include "stm32_gpio.h"

/*****************************************************************************
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

/*****************************************************************************
 * Private Declarations
 ****************************************************************************/

static const uint32_t buttons[NUM_BUTTONS] =
{
  GPIO_JOY_O, GPIO_JOY_U, GPIO_JOY_D, GPIO_JOY_R, GPIO_JOY_L
};

/*****************************************************************************
 * Public Functions
 ****************************************************************************/

/*****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *      Initializes gpio pins for joystick buttons
 ****************************************************************************/

void board_button_initialize(void)
{
  int i;

  for (i = 0; i != NUM_BUTTONS; ++i)
    {
      stm32_configgpio(buttons[i]);
    }
}

/*****************************************************************************
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
