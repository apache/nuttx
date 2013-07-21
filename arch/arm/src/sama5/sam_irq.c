/****************************************************************************
 * arch/arm/src/sama5/sam_irq.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "up_arch.h"
#include "os_internal.h"
#include "up_internal.h"

#ifdef CONFIG_GPIO_IRQ
#  include "sam_gpio.h"
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Enable NVIC debug features that are probably only desireable during
 * bringup
 */

#undef SAM_IRQ_DEBUG

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *current_regs;

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_dumpaic
 *
 * Description:
 *   Dump some interesting AIC registers
 *
 ****************************************************************************/

#if defined(SAM_IRQ_DEBUG) && defined (CONFIG_DEBUG)
static void sam_dumpaic(const char *msg, int irq)
{
  irqstate_t flags;

  flags = irqsave();
#warning Missing logic
  slldbg("AIC (%s, irq=%d):\n", msg, irq);
#warning Missing logic
  irqrestore(flags);
}
#else
#  define sam_dumpaic(msg, irq)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Disable all interrupts  */
#warning Missing logic

  /* Set all interrupts to the default priority */
#warning Missing logic

  /* currents_regs is non-NULL only while processing an interrupt */

  current_regs = NULL;

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* Initialize logic to support a second level of interrupt decoding for
   * GPIO pins.
   */

#ifdef CONFIG_GPIO_IRQ
  sam_gpioirqinitialize();
#endif

  /* And finally, enable interrupts */

  (void)irqenable();
#endif
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  if (irq < SAM_IRQ_NINT)
    {
#warning Missing logic
    }
#ifdef CONFIG_GPIO_IRQ
  else
    {
      /* Maybe it is a (derived) GPIO IRQ */

      sam_gpioirqdisable(irq);
    }
#endif
  sam_dumpaic("disable", irq);
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  if (irq < SAM_IRQ_NINT)
    {
#warning Missing logic
    }
#ifdef CONFIG_GPIO_IRQ
  else
    {
      /* Maybe it is a (derived) GPIO IRQ */

      sam_gpioirqenable(irq);
    }
#endif
  sam_dumpaic("enable", irq);
}

/****************************************************************************
 * Name: up_maskack_irq
 *
 * Description:
 *   Mask the IRQ and acknowledge it
 *
 ****************************************************************************/

void up_maskack_irq(int irq)
{
  up_disable_irq(irq);
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  if (irq < SAM_IRQ_NINT)
    {
#warning Missing logic
    }

  sam_dumpaic("prioritize", irq);
  return OK;
}
#endif
