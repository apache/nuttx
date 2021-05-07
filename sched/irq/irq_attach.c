/****************************************************************************
 * sched/irq/irq_attach.c
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
           * supported). Or (2) if the device has different number for vector
           * numbers and IRQ numbers (in that case, we don't know the correct
           * IRQ number to use to disable the interrupt). In those cases, the
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
      g_irqvector[ndx].start   = clock_systime_ticks();
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
