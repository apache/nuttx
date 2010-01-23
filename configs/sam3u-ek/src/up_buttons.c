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

#include <stdint.h>

#include <nuttx/irq.h>

#include <arch/irq.h>
#include <arch/board/board.h>

#include "sam3u_internal.h"
#include "sam3uek_internal.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static xcpt_t g_irqbutton1;
static xcpt_t g_irqbutton2;

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
  (void)sam3u_configgpio(GPIO_BUTTON1);
  (void)sam3u_configgpio(GPIO_BUTTON2);
}

/****************************************************************************
 * Name: up_buttons
 ****************************************************************************/

uint8_t up_buttons(void)
{
  uint8_t retval;

  retval  = sam3u_gpioread(GPIO_BUTTON1) ? 0 : GPIO_BUTTON1;
  retval |= sam3u_gpioread(GPIO_BUTTON2) ? 0 : GPIO_BUTTON2;

  return retval;
}

/****************************************************************************
 * Name: up_irqbutton1
 ****************************************************************************/

#ifdef CONFIG_GPIOA_IRQ
xcpt_t up_irqbutton1(xcpt_t irqhandler)
{
  xcpt_t oldhandler;
  irqstate_t flags;

  /* Disable interrupts until we are done */

  flags        = irqsave();

  /* Get the old button interrupt handler and save the new one */

  oldhandler   = g_irqbutton1;
  g_irqbutton1 = irqhandler;

  /* Configure the interrupt */

  sam3u_gpioirq(IRQ_BUTTON1);
  (void)irq_attach(IRQ_BUTTON1, irqhandler);
  sam3u_gpioirqenable(IRQ_BUTTON1);
  irqrestore(flags);

  /* Return the old button handler (so that it can be restored) */

  return oldhandler;
}
#endif

/****************************************************************************
 * Name: up_irqbutton2
 ****************************************************************************/

#ifdef CONFIG_GPIOA_IRQ
xcpt_t up_irqbutton2(xcpt_t irqhandler)
{
  xcpt_t oldhandler;
  irqstate_t flags;

  /* Disable interrupts until we are done */

  flags        = irqsave();

  /* Get the old button interrupt handler and save the new one */

  oldhandler   = g_irqbutton2;
  g_irqbutton2 = irqhandler;

  /* Configure the interrupt */

  sam3u_gpioirq(IRQ_BUTTON2);
  (void)irq_attach(IRQ_BUTTON2, irqhandler);
  sam3u_gpioirqenable(IRQ_BUTTON2);
  irqrestore(flags);

  /* Return the old button handler (so that it can be restored) */

  return oldhandler;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
