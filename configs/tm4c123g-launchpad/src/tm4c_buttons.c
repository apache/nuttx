/****************************************************************************
 * config/tm4c123g-launchpad/src/tm4c_buttons.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Bradley Noyes <bradley.noyes@trd2inc.com>
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <arch/irq.h>
#include <arch/board/board.h>

#include "tiva_gpio.h"
#include "tm4c123g-launchpad.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
static xcpt_t g_irquser;
#endif

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
 *   After that, board_buttons() may be called to collect the current state of
 *   all buttons or board_button_irq() may be called to register button interrupt
 *   handlers.
 *
 ****************************************************************************/

void board_button_initialize(void)
{
  /* Unlock GPIOF from NMI to use it w/ buttons see Register 19 of GPIOs
   * and see table 10-10 in datasheet for pins with special considerations.
   */

  tiva_gpio_lockport(GPIO_SW2, false);

  /* Configure the GPIO pins as inputs.  NOTE that EXTI interrupts are
   * configured for some pins but NOT used in this file
   */

  tiva_configgpio(GPIO_SW1);
  tiva_configgpio(GPIO_SW2);

  /* Configure GPIO interrupts */

#ifdef CONFIG_ARCH_IRQBUTTONS
  (void)tiva_gpioirqinitialize();
#endif
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   8-bit bit set with each bit associated with a button.  See the BUTTON*
 *   definitions above for the meaning of each bit in the returned value.
 *
 ****************************************************************************/

uint8_t board_buttons(void)
{
  uint8_t ret = 0x0;

  /* Check that state of each key.  A LOW value means that the key is
   * pressed.
   */

  if (!tiva_gpioread(GPIO_SW1))
    {
      ret |= (1 << GPIO_PIN_4);
    }

  if (!tiva_gpioread(GPIO_SW2))
    {
      ret |= (1 << GPIO_PIN_0);
    }

  return ret;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   This function may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is one
 *   of the BUTTON* definitions provided above. The previous interrupt
 *   handler address is returned (so that it may restored, if so desired).
 *
 ****************************************************************************/

#  ifdef CONFIG_ARCH_IRQBUTTONS
xcpt_t board_button_irq(int id, xcpt_t irqhandler)
{
  xcpt_t oldhandler = NULL;

  if (id == BUTTON_SW1 ||
      id == BUTTON_SW2)
    {
      irqstate_t flags;

      /* Disable interrupts until we are done.  This guarantees that the
       * following operations are atomic.
       */

      flags = irqsave();

      /* Get the old button interrupt handler and save the new one */

      oldhandler = g_irquser;
      g_irquser = irqhandler;

      /* Are we attaching or detaching? */

      if (irqhandler != NULL)
        {
          /* Configure the interrupt */

          (void)tiva_gpioirqattach(CONFIG_TIVA_GPIOF_IRQS, irqhandler);
          tiva_gpioirqenable(CONFIG_TIVA_GPIOF_IRQS);
        }
      else
        {
          /* Disable and detach the interrupt */

          tiva_gpioirqdisable(CONFIG_TIVA_GPIOF_IRQS);
          (void)tiva_gpioirqattach(CONFIG_TIVA_GPIOF_IRQS, NULL);
        }
      /* Configure the interrupt */

      irqrestore(flags);
    }

  /* Return the old button handler (so that it can be restored) */

  return oldhandler;
}
#  endif

#endif /* CONFIG_ARCH_BUTTONS */
