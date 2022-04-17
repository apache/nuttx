/****************************************************************************
 * boards/arm/sam34/sam4e-ek/src/sam_buttons.c
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
#include <arch/board/board.h>

#include "sam_gpio.h"
#include "sam4e-ek.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_irqx
 *
 * Description:
 *   This function implements the core of the board_button_irq() logic.
 *
 ****************************************************************************/

#if defined(CONFIG_SAM34_GPIOA_IRQ) && defined(CONFIG_ARCH_IRQBUTTONS)
static int board_button_irqx(gpio_pinset_t pinset, int irq,
                             xcpt_t irqhandler, void *arg)
{
  irqstate_t flags;

  /* Disable interrupts until we are done.  This guarantees that the
   * following operations are atomic.
   */

  flags = enter_critical_section();

  /* Are we attaching or detaching? */

  if (irqhandler != NULL)
    {
      /* Configure the interrupt */

      sam_gpioirq(pinset);
      irq_attach(irq, irqhandler, arg);
      sam_gpioirqenable(irq);
    }
  else
    {
      /* Detach and disable the interrupt */

      irq_detach(irq);
      sam_gpioirqdisable(irq);
    }

  leave_critical_section(flags);
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
  sam_configgpio(GPIO_SCROLLUP);
  sam_configgpio(GPIO_SCROLLDWN);
  sam_configgpio(GPIO_WAKU);
  sam_configgpio(GPIO_TAMP);
  return 4;
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
  uint32_t retval;

  retval  = sam_gpioread(GPIO_SCROLLUP)  ? 0 : BUTTON_SCROLLUP;
  retval |= sam_gpioread(GPIO_SCROLLDWN) ? 0 : BUTTON_SCROLLDOWN;
  retval |= sam_gpioread(GPIO_WAKU)      ? 0 : BUTTON_WAKU;
  retval |= sam_gpioread(GPIO_TAMP)      ? 0 : BUTTON_TAMP;

  return retval;
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

#if defined(CONFIG_SAM34_GPIOA_IRQ) && defined(CONFIG_ARCH_IRQBUTTONS)
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  switch (id)
    {
      case BUTTON_SCROLLUP:
        return board_button_irqx(GPIO_SCROLLUP, IRQ_SCROLLUP,
                                 irqhandler, arg);

      case BUTTON_SCROLLDOWN:
        return board_button_irqx(GPIO_SCROLLDWN, IRQ_SCROLLDWN,
                                 irqhandler, arg);

      case BUTTON_WAKU:
        return board_button_irqx(GPIO_WAKU, IRQ_WAKU,
                                 irqhandler, arg);

      case BUTTON_TAMP:
        return board_button_irqx(GPIO_TAMP, IRQ_TAMP,
                                 irqhandler, arg);

      default:
        return -EINVAL;
    }
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
