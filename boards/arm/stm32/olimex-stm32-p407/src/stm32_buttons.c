/****************************************************************************
 * boards/arm/stm32/olimex-stm32-p407/src/stm32_buttons.c
 *
 *   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "stm32_gpio.h"

#include "olimex-stm32-p407.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Pin configuration for each STM32F4 Discovery button. This array is indexed
 * by the BUTTON_* definitions in board.h
 */

static const uint32_t g_buttons[NUM_BUTTONS] =
{
  GPIO_BTN_TAMPER,
  GPIO_BTN_WKUP,

  /* The Joystick is treated like the other buttons unless CONFIG_DJOYSTICK
   * is defined, then it is assumed that they should be used by the discrete
   * joystick driver.
   */

#ifndef CONFIG_DJOYSTICK
  GPIO_JOY_RIGHT,
  GPIO_JOY_UP,
  GPIO_JOY_LEFT,
  GPIO_JOY_DOWN,
  GPIO_JOY_CENTER
#endif
};

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
  int i;

  /* Configure the GPIO pins as inputs.  NOTE that EXTI interrupts are
   * configured for all pins.
   */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      stm32_configgpio(g_buttons[i]);
    }

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;

  /* Check that state of each key */

  if (!stm32_gpioread(g_buttons[BUTTON_TAMPER]))
    {
      ret |= BUTTON_TAMPER_BIT;
    }

  if (stm32_gpioread(g_buttons[BUTTON_WKUP]))
    {
      ret |= BUTTON_WKUP_BIT;
    }

#ifndef CONFIG_DJOYSTICK
  if (stm32_gpioread(g_buttons[JOYSTICK_RIGHT]))
    {
      ret |= JOYSTICK_RIGHT_BIT;
    }

  if (stm32_gpioread(g_buttons[JOYSTICK_UP]))
    {
      ret |= JOYSTICK_UP_BIT;
    }

  if (stm32_gpioread(g_buttons[JOYSTICK_LEFT]))
    {
      ret |= JOYSTICK_LEFT_BIT;
    }

  if (stm32_gpioread(g_buttons[JOYSTICK_DOWN]))
    {
      ret |= JOYSTICK_DOWN_BIT;
    }

  if (stm32_gpioread(g_buttons[JOYSTICK_CENTER]))
    {
      ret |= JOYSTICK_CENTER_BIT;
    }
#endif

  return ret;
}

/****************************************************************************
 * Button support.
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   32-bit bit set with each bit associated with a button.  See the
 *   BUTTON_*_BIT definitions in board.h for the meaning of each bit.
 *
 *   board_button_irq() may be called to register an interrupt handler that
 *   will be called when a button is depressed or released. The ID value is a
 *   button enumeration value that uniquely identifies a button resource. See
 *   the BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
  int ret = -EINVAL;

  /* The following should be atomic */

  if (id >= MIN_IRQBUTTON && id <= MAX_IRQBUTTON)
    {
      ret = stm32_gpiosetevent(g_buttons[id], true, true, true,
                               irqhandler, arg);
    }

  return ret;
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
