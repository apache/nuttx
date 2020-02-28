/****************************************************************************
 *  arch/misoc/src/lm32/_irq.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Ramtin Amin <keytwo@gmail.com>
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
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "chip.h"
#include "lm32.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile uint32_t *g_current_regs;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* currents_regs is non-NULL only while processing an interrupt */

  g_current_regs = NULL;

  /* Attach the software interrupt */

  irq_attach(LM32_IRQ_SWINT, lm32_swint, NULL);

  /* Enable interrupts */

  irq_setie(1);
}

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Return the current interrupt enable state and disable all interrupts.
 *
 ****************************************************************************/

irqstate_t up_irq_save(void)
{
  irqstate_t flags;

  /* Get the previous value of IE */

  flags = irq_getie();

  /* Disable interrupts and return the previous interrupt state */

  irq_setie(0);
  return flags;
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore saved interrupt state
 *
 ****************************************************************************/

void up_irq_restore(irqstate_t flags)
{
  /* Restore the interrupt state returned by up_save_irq() */

  irq_setie(flags);
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Return the current interrupt enable state and enable all interrupts
 *
 ****************************************************************************/

irqstate_t up_irq_enable(void)
{
  irqstate_t flags;

  /* Get the previous value of IE */

  flags = irq_getie();

  /* Enable interrupts and return the previous interrupt state */

  irq_setie(1);
  return flags;
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
  irqstate_t flags;

  DEBUGASSERT(irq >= 0 && irq < NR_IRQS);

  /* Ignore any attempt to disable software interrupts */

  if (irq < LM32_NINTERRUPTS)
    {
      /* Disable interrupts by clearing the bit that corresponds to the irq */

      flags  = irq_getmask();
      flags &= ~(1 << irq);
      irq_setmask(flags);
    }
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
  irqstate_t flags;
  DEBUGASSERT(irq >= 0 && irq < NR_IRQS);

  /* Ignore any attempt to enable software interrupts */

  if (irq < LM32_NINTERRUPTS)
    {
      /* Enable interrupts by setting the bit that corresponds to the irq */

      flags  = irq_getmask();
      flags |= (1 << irq);
      irq_setmask(flags);
    }
}
