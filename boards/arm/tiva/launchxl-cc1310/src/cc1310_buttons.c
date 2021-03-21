/****************************************************************************
 * boards/arm/tiva/launchxl-cc1310/src/cc1310_buttons.c
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

#include "tiva_gpio.h"
#include "launchxl-cc1310.h"

#include <arch/board/board.h>

#ifdef CONFIG_ARCH_BUTTONS

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
  tiva_configgpio(&g_gpio_sw1);
  tiva_configgpio(&g_gpio_sw2);
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
  uint32_t ret = 0;

  /* When the button is pressed, a low value will be sensed */

  if (!tiva_gpioread(&g_gpio_sw1))
    {
      ret |= BUTTON_SW1_BIT;
    }

  if (!tiva_gpioread(&g_gpio_sw2))
    {
      ret |= BUTTON_SW2_BIT;
    }

  return ret;
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
 *   Configuration CONFIG_SAMA5_PIO_IRQ must be selected to enable the
 *   overall PIO IRQ feature and CONFIG_SAMA5_PIOB_IRQ must be enabled to
 *   select PIOs to support interrupts on PIOE.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_TIVA_GPIOIRQS)
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
  irqstate_t flags;
  int irq;
  int ret;

  if (id == BUTTON_SW1)
    {
      irq = CC1310_SW1_IRQ;
    }
  else if (id == BUTTON_SW2)
    {
      irq = CC1310_SW2_IRQ;
    }
  else
    {
      return -EINVAL
    }

  /* The following should be atomic */

  flags = enter_critical_section();

  /* Detach and disable the button interrupt */

  up_disable_irq(irq);
  irq_detach(irq);

  /* Attach the new handler if so requested */

  ret = OK;
  if (irqhandler != NULL)
    {
      ret = irq_attach(irq, irqhandler, arg);
      if (ret == OK)
        {
          up_enable_irq(irq);
        }
    }

  leave_critical_section(flags);
  return ret;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
