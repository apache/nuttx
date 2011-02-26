/****************************************************************************
 * configs/stm3210e-eval/src/up_buttons.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>
#include "stm3210e-internal.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint16_t g_buttons[NUM_BUTTONS] =
{
  GPIO_BTN_WAKEUP, GPIO_BTN_TAMPER, GPIO_BTN_KEY,   GPIO_JOY_KEY,
  GPIO_JOY_DOWN,   GPIO_JOY_LEFT,   GPIO_JOY_RIGHT, GPIO_JOY_UP
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_buttoninit
 ****************************************************************************/

void up_buttoninit(void)
{
  int i;

  /* Configure the GPIO pins as inputs.  NOTE that EXTI interrupts are 
   * configured for some pins but NOT used in this file
   */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      stm32_configgpio(g_buttons[i]);
    }
}

/****************************************************************************
 * Name: up_buttons
 ****************************************************************************/

uint8_t up_buttons(void)
{
  uint8_t ret = 0;
  int i;

  /* Check that state of each key */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
       /* A LOW value means that the key is pressed */

       if (!stm32_gpioread(g_buttons[i]))
         {
           ret |= (1 << i);
         }
    }

  return ret;
}

#endif /* CONFIG_ARCH_BUTTONS */
