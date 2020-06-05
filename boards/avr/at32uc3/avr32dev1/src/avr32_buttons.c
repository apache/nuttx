/****************************************************************************
 * boards/avr/at32uc3/avr32dev1/src/avr32_buttons.c
 *
 *   Copyright (C) 2010-2011, 2014-2015 Gregory Nutt. All rights reserved.
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
