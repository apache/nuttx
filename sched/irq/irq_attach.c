/****************************************************************************
 * sched/irq/irq_attach.c
 *
 *   Copyright (C) 2007-2008, 2010, 2012, 2017-2018 Gregory Nutt. All rights
 *     reserved.
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

#include <errno.h>

#include <nuttx/irq.h>

#include "irq/irq.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: irq_attach
 *
 * Description:
 *   Configure the IRQ subsystem so that IRQ number 'irq' is dispatched to
 *   'isr'
 *
 ****************************************************************************/

int irq_attach(int irq, xcpt_t isr, FAR void *arg)
{
#if NR_IRQS > 0
  int ret = -EINVAL;

  if ((unsigned)irq < NR_IRQS)
    {
      irqstate_t flags;
      int ndx;

#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
      /* Is there a mapping for this IRQ number? */

      ndx = g_irqmap[irq];
      if ((unsigned)ndx >= CONFIG_ARCH_NUSER_INTERRUPTS)
        {
          /* No.. then return failure. */

          return ret;
        }
#else
      ndx = irq;
#endif

      /* If the new ISR is NULL, then the ISR is being detached.
       * In this case, disable the ISR and direct any interrupts
       * to the unexpected interrupt handler.
       */

      flags = enter_critical_section();
      if (isr == NULL)
        {
          /* Disable the interrupt if we can before detaching it.  We might
           * not be able to do this if:  (1) the device does not have a
           * centralized interrupt controller (so up_disable_irq() is not
           * supported).  Or (2) if the device has different number for vector
           * numbers and IRQ numbers (in that case, we don't know the correct
           * IRQ number to use to disable the interrupt).  In those cases, the
           * code will just need to be careful that it disables all interrupt
           * sources before detaching from the interrupt vector.
           */

#if !defined(CONFIG_ARCH_NOINTC) && !defined(CONFIG_ARCH_VECNOTIRQ)
          up_disable_irq(irq);
#endif
          /* Detaching the ISR really means re-attaching it to the
           * unexpected exception handler.
           */

          isr = irq_unexpected_isr;
          arg = NULL;
        }

#ifdef CONFIG_IRQCHAIN
      /* Save the new ISR and its argument in the table.
       * If there is only one ISR on this irq, then .handler point to the ISR
       * and .arg point to the ISR parameter; Otherwise, .handler point to
       * irqchain_dispatch and .arg point to irqchain_s.
       */

      if (is_irqchain(ndx, isr))
        {
          ret = irqchain_attach(ndx, isr, arg);
          leave_critical_section(flags);
          return ret;
        }
#endif

      /* Save the new ISR and its argument in the table. */

      g_irqvector[ndx].handler = isr;
      g_irqvector[ndx].arg     = arg;
#ifdef CONFIG_SCHED_IRQMONITOR
      g_irqvector[ndx].start   = clock_systimer();
#ifdef CONFIG_HAVE_LONG_LONG
      g_irqvector[ndx].count   = 0;
#else
      g_irqvector[ndx].mscount = 0;
      g_irqvector[ndx].lscount = 0;
#endif
#endif

      leave_critical_section(flags);
      ret = OK;
    }

  return ret;
#else
  return OK;
#endif
}
