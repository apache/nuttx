/****************************************************************************
 * boards/arm/lpc54xx/lpcxpresso-lpc54628/src/lpc54_buttons.c
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
#include <nuttx/irq.h>

#include <nuttx/irq.h>

#include "lpc54_gpio.h"
#include "lpcxpresso-lpc54628.h"

#include <arch/board/board.h>

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_LPC54_GPIOIRQ) && defined(CONFIG_ARCH_IRQBUTTONS)
static uint8_t g_button_irq;
static xcpt_t  g_button_handler;
static void   *g_button_arg;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_interrupt
 *
 * Description:
 *   This function intermediates the interrupt provided to the application
 *   logic that attached the interrupt.  This is necessary to properly
 *   clear the pending button interrupts.
 *
 ****************************************************************************/

static int board_button_interrupt(int irq, void *context, void *arg)
{
  /* Acknowledge the button interrupt */

  lpc54_gpio_ackedge(irq);

  /* Transfer control to the attached interrupt handler */

  if (g_button_handler != NULL)
    {
      return g_button_handler(irq, context, arg);
    }

  return OK;
}

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
  int ret;

  /* Configure the button GPIO interrupt */

  ret = lpc54_gpio_config(GPIO_BUTTON_USER);
  if (ret >= 0)
    {
#if defined(CONFIG_LPC54_GPIOIRQ) && defined(CONFIG_ARCH_IRQBUTTONS)
      /* Get the IRQ that is associated with the PIN interrupt and attach the
       * intermediate button interrupt handler to that interrupt.
       */

      g_button_irq = lpc54_gpio_irqno(GPIO_BUTTON_USER);
      irq_attach(g_button_irq, board_button_interrupt, NULL);
#endif
    }

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   32-bit bit set with each bit associated with a button.  See the BUTTON*
 *   definitions above for the meaning of each bit in the returned value.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  return lpc54_gpio_read(GPIO_BUTTON_USER) ? 0 : BUTTON_USER_BIT;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is one
 *   of the BUTTON* definitions provided above.
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR32_GPIOIRQ must be selected to enable the
 *   overall GPIO IRQ feature and CONFIG_AVR32_GPIOIRQSETA and/or
 *   CONFIG_AVR32_GPIOIRQSETB must be enabled to select GPIOs to support
 *   interrupts on.  For button support, bits 2 and 3 must be set in
 *   CONFIG_AVR32_GPIOIRQSETB (PB2 and PB3).
 *
 ****************************************************************************/

#if defined(CONFIG_LPC54_GPIOIRQ) && defined(CONFIG_ARCH_IRQBUTTONS)
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  int ret = -EINVAL;

  if (id == BUTTON_USER)
    {
      /* Are we attaching or detaching? */

      if (irqhandler != NULL)
        {
          /* Yes.. Attach and enable the interrupt */

          g_button_handler = irqhandler;
          g_button_arg     = arg;
          up_enable_irq(g_button_irq);
        }
      else
        {
          /* No.. Disable and detach the interrupt */

          up_disable_irq(g_button_irq);
          g_button_handler = NULL;
          g_button_arg     = NULL;
        }

      ret = OK;
    }

  return ret;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
