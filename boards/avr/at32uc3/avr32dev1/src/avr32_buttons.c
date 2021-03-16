/****************************************************************************
 * boards/avr/at32uc3/avr32dev1/src/avr32_buttons.c
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
#include "at32uc3_config.h"

#include <sys/types.h>
#include <stdint.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "at32uc3.h"
#include "avr32dev1.h"

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

#if defined(CONFIG_AVR32_GPIOIRQ) && defined(CONFIG_ARCH_IRQBUTTONS) && \
   (defined(CONFIG_AVR32DEV_BUTTON1_IRQ) || defined(CONFIG_AVR32DEV_BUTTON2_IRQ))
static int board_button_irqx(int irq, xcpt_t irqhandler, void *arg)
{
  /* Attach the handler */

  int ret = gpio_irqattach(irq, irqhandler, &oldhandler, arg);
  if (ret >= 0)
    {
      /* Enable/disable the interrupt */

      if (irqhandler != NULL)
        {
          gpio_irqenable(irq);
        }
      else
        {
         gpio_irqdisable(irq);
        }
    }

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
  at32uc3_configgpio(PINMUX_GPIO_BUTTON1);
  at32uc3_configgpio(PINMUX_GPIO_BUTTON2);
  return 2;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   32-bit bit set with each bit associated with a button.  See the BUTTON*
 *   definitions in the board.h header file for the meaning of each bit in
 *   the returned value.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t retval;

  retval  = at32uc3_gpioread(PINMUX_GPIO_BUTTON1) ? 0 : BUTTON1;
  retval |= at32uc3_gpioread(PINMUX_GPIO_BUTTON2) ? 0 : BUTTON2;

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

#if defined(CONFIG_AVR32_GPIOIRQ) && defined(CONFIG_ARCH_IRQBUTTONS)
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
  int ret;

#ifdef CONFIG_AVR32DEV_BUTTON1_IRQ
  if (id == BUTTON1)
    {
      ret = board_button_irqx(GPIO_BUTTON1_IRQ, irqhandler, arg);
    }
  else
#endif
#ifdef CONFIG_AVR32DEV_BUTTON2_IRQ
  if (id == BUTTON2)
    {
      ret = board_button_irqx(GPIO_BUTTON2_IRQ, irqhandler, arg);
    }
  else
#endif
    {
      ret = -EINVAL;
    }

  return ret;
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
