/****************************************************************************
 * boards/arm/stm32l4/stm32l4r9ai-disco/src/stm32_buttons.c
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
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>
#include <nuttx/power/pm.h>

#include "stm32l4r9ai-disco.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_PM_BUTTON_ACTIVITY
#  define CONFIG_PM_BUTTON_ACTIVITY 10
#endif

#define PM_IDLE_DOMAIN  0 /* Revisit */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Button Power Management */

#ifdef CONFIG_PM
static void button_pm_notify(struct pm_callback_s *cb, int domain,
                             enum pm_state_e pmstate);
static int button_pm_prepare(struct pm_callback_s *cb, int domain,
                             enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Pin configuration for each STM32L476 Discovery button.  This array is
 * indexed by the BUTTON_* definitions in board.h
 */

static const uint32_t g_buttons[NUM_BUTTONS] =
{
  GPIO_BTN_CENTER,
  GPIO_BTN_LEFT,
  GPIO_BTN_DOWN,
  GPIO_BTN_RIGHT,
  GPIO_BTN_UP
};

#ifdef CONFIG_PM
static struct pm_callback_s g_buttonscb =
{
  .notify  = button_pm_notify,
  .prepare = button_pm_prepare,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: button_pm_notify
 *
 * Description:
 *   Notify the driver of new power state. This callback is called after
 *   all drivers have had the opportunity to prepare for the new power state.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void button_pm_notify(struct pm_callback_s *cb, int domain,
                             enum pm_state_e pmstate)
{
  switch (pmstate)
    {
      case(PM_NORMAL):
        {
          /* Restore normal buttons operation
           * XXX turn on any GPIO
           */
        }
        break;

      case(PM_IDLE):
        {
          /* Entering IDLE mode - buttons
           * XXX turn on any GPIO
           */
        }
        break;

      case(PM_STANDBY):
        {
          /* Entering STANDBY mode - Logic for PM_STANDBY goes here
           * XXX turn off any GPIO
           */
        }
        break;

      case(PM_SLEEP):
        {
          /* Entering SLEEP mode - Logic for PM_SLEEP goes here
           * XXX turn off any GPIO
           */
        }
        break;

      default:
        {
          /* Should not get here */
        }
        break;
    }
}
#endif

/****************************************************************************
 * Name: button_handler
 *
 * Description:
 *   Handle a button wake-up interrupt
 *
 ****************************************************************************/

/* XXX it's not completely clear to me if this is appropriate; on the one
 * hand, it seems to make sense that this would be the module to have the ISR
 * for the buttons.  On the other hand, it will conflict with things done in
 * the buttons example, which registers it's own ISR, and warns if it sees
 * one already there.  I don't know if 'buttons' is overstepping it's bounds
 * in the interest of providing a compact example, (like the I2C app directly
 * talking to the bus), or if really that should be an expected thing to do.
 */

#if 0
#ifdef CONFIG_ARCH_IRQBUTTONS
static int button_handler(int irq, void *context, void *arg)
{
#ifdef CONFIG_PM
  /* At this point the MCU should have already awakened.  The state
   * change will be handled in the IDLE loop when the system is re-awakened
   * The button interrupt handler should be totally ignorant of the PM
   * activities and should report button activity as if nothing
   * special happened.
   */

  pm_activity(PM_IDLE_DOMAIN, CONFIG_PM_BUTTON_ACTIVITY);
#endif
  return OK;
}
#endif /* CONFIG_ARCH_IRQBUTTONS */
#endif

/****************************************************************************
 * Name: button_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int button_pm_prepare(struct pm_callback_s *cb, int domain,
                             enum pm_state_e pmstate)
{
  /* No preparation to change power modes is required by the Buttons driver.
   * We always accept the state change by returning OK.
   */

  return OK;
}
#endif

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
      stm32l4_configgpio(g_buttons[i]);

      /* It's not clear if this is correct; I think so, but then there are
       * conflicts with the 'buttons' sample app.
       */

#if 0
#ifdef CONFIG_ARCH_IRQBUTTONS
      board_button_irq(i, button_handler);
#endif
#endif
    }

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  int i;

  /* Check that state of each key */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      /* A HIGH value means that the key is pressed. */

      bool pressed = stm32l4_gpioread(g_buttons[i]);

      /* Accumulate the set of depressed (not released) keys */

      if (pressed)
        {
          ret |= (1 << i);
        }
    }

#ifdef CONFIG_PM
  /* if the user pressed any buttons, notify power management system we are
   * active
   */

  if (0 != ret)
    {
      pm_activity(PM_IDLE_DOMAIN, CONFIG_PM_BUTTON_ACTIVITY);
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
 *   will be called when a button is depressed or released.  The ID value is
 *   a button enumeration value that uniquely identifies a button resource.
 *   See the BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  int ret = -EINVAL;

  /* The following should be atomic */

  if (id >= MIN_IRQBUTTON && id <= MAX_IRQBUTTON)
    {
      ret = stm32l4_gpiosetevent(g_buttons[id], true, true, true,
                                 irqhandler, arg);
    }

  return ret;
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
