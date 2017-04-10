/****************************************************************************
 * configs/olimex-efm32g880f128-stk/src/efm32_buttons.c
 *
 *   Copyright (C) 2014-2015, 2017 Gregory Nutt. All rights reserved.
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
/* The Olimex board has four buttons, BUT1-4.  Each is grounded and so should
 * have a weak pull-up so that it will be sensed as "1" when open and "0"
 * when closed.
 *
 * --------------------- ---------------------
 * PIN                   CONNECTIONS
 * --------------------- ---------------------
 * PE0/PCNT0_S0IN/U0_TX  BUT1, EXT-18
 * PE1/PCNT0_S1IN/U0_RX  BUT2, EXT-19
 * PE2/ACMP0_O           BUT3, EXT-20
 * E3/ACMP1_O            BUT4, EXT-21
 * --------------------- ---------------------
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/irq.h>

#include <nuttx/irq.h>
#include <arch/board/board.h>

#include "efm32_gpio.h"
#include "efm32g880f128-stk.h"

#if CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_EFM32_GPIO_IRQ) && defined(CONFIG_ARCH_IRQBUTTONS)
static const uint8_t g_button_irqs[NUM_BUTTONS];
#endif

static const gpio_pinset_t g_button_configs[NUM_BUTTONS] =
{
  GPIO_BUTTON_1, GPIO_BUTTON_2, GPIO_BUTTON_3, GPIO_BUTTON_4
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

void board_button_initialize(void)
{
  int i;

  /* Configure each button */

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      (void)efm32_configgpio(g_button_configs[i]);
    }
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
  uint32_t ret;
  int i;

  /* Check each button */

  for (i = 0, ret = 0; i < NUM_BUTTONS; i++)
    {
      /* The button is closed if a low value is sensed */

      if (!efm32_gpioread(g_button_configs[i]))
        {
          ret |= (1 << i);
        }
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
 *   Configuration CONFIG_EFM32_GPIO_IRQ must be selected to enable the
 *   overall GPIO IRQ feature.
 *
 * Returned Value:
 *   This function should return the old interrupt handler value, but
 *   currently always returns NULL.
 *
 ****************************************************************************/

#if defined(CONFIG_EFM32_GPIO_IRQ) && defined(CONFIG_ARCH_IRQBUTTONS)
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
  if (id >=0 && id < NUM_BUTTONS)
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

          efm32_gpioirq(g_button_configs[id]);

          /* Attach and enable the interrupt */

          (void)irq_attach(g_button_irqs[id], irqhandler, arg);
          efm32_gpioirqenable(g_button_irqs[id]);
        }
      else
        {
          /* Disable and detach the interrupt */

          efm32_gpioirqdisable(g_button_irqs[id]);
          (void)irq_detach(g_button_irqs[id]);
        }

      leave_critical_section(flags);
    }

  /* Return the old button handler (so that it can be restored) */

  return OK;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
