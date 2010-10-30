/****************************************************************************
 * configs/sam3u-ek/src/up_leds.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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
#include "at32uc3_config.h"

#include <sys/types.h>
#include <stdint.h>

#include <nuttx/irq.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "at32uc3_internal.h"
#include "avr32dev1_internal.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
  (void)at32uc3_configgpio(PINMUX_GPIO_BUTTON1);
  (void)at32uc3_configgpio(PINMUX_GPIO_BUTTON2);
}

/****************************************************************************
 * Name: up_buttons
 ****************************************************************************/

uint8_t up_buttons(void)
{
  uint8_t retval;

  retval  = at32uc3_gpioread(PINMUX_GPIO_BUTTON1) ? 0 : BUTTON1;
  retval |= at32uc3_gpioread(PINMUX_GPIO_BUTTON2) ? 0 : BUTTON2;

  return retval;
}

/****************************************************************************
 * Name: up_irqbutton1
 ****************************************************************************/

#ifdef CONFIG_AVR32_GPIOIRQ
xcpt_t up_irqbutton1(xcpt_t irqhandler)
{
#ifdef CONFIG_AVR32DEV_BUTTON1_IRQ
  xcpt_t oldhandler;

  /* Attach the handler */

  gpio_irqattach(GPIO_BUTTON1_IRQ, irqhandler, &oldhandler);

  /* Enable/disable the interrupt */

  if (irqhandler)
    {
	  gpio_irqenable(GPIO_BUTTON1_IRQ);
	}
  else
    {
	  gpio_irqdisable(GPIO_BUTTON1_IRQ);
	}

  /* Return the old button handler (so that it can be restored) */

  return oldhandler;
#else
  return NULL;
#endif
}
#endif

/****************************************************************************
 * Name: up_irqbutton2
 ****************************************************************************/

#ifdef CONFIG_AVR32_GPIOIRQ
xcpt_t up_irqbutton2(xcpt_t irqhandler)
{
#ifdef CONFIG_AVR32DEV_BUTTON2_IRQ
  xcpt_t oldhandler;

  /* Attach the handler */

  gpio_irqattach(GPIO_BUTTON2_IRQ, irqhandler, &oldhandler);

  /* Enable/disable the interrupt */

  if (irqhandler)
    {
	  gpio_irqenable(GPIO_BUTTON2_IRQ);
	}
  else
    {
	  gpio_irqdisable(GPIO_BUTTON2_IRQ);
	}

  /* Return the old button handler (so that it can be restored) */

  return oldhandler;
#else
  return NULL;
#endif
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
