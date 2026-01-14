/****************************************************************************
 * sched/irq/irq_attach.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
 * Private Data
 ****************************************************************************/

static spinlock_t g_irqlock = SP_UNLOCKED;
#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE_DYNAMIC
static int g_irqmap_count = 1;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE_DYNAMIC

/* This is the interrupt vector mapping table.  This must be provided by
 * architecture specific logic if CONFIG_ARCH_MINIMAL_VECTORTABLE is define
 * in the configuration.
 *
 * REVISIT: This should be declared in include/nuttx/irq.h.  The declaration
 * at that location, however, introduces a circular include dependency so the
 * declaration is here for the time being.
 */

irq_mapped_t g_irqmap[NR_IRQS];
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE_DYNAMIC
int irq_to_ndx(int irq)
{
  DEBUGASSERT(g_irqmap_count < CONFIG_ARCH_NUSER_INTERRUPTS);

  irqstate_t flags = spin_lock_irqsave(&g_irqlock);
  if (g_irqmap[irq] == 0)
    {
      g_irqmap[irq] = g_irqmap_count++;
    }

  spin_unlock_irqrestore(&g_irqlock, flags);
  return g_irqmap[irq];
}
#endif

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
      int ndx = IRQ_TO_NDX(irq);
      irqstate_t flags;

      if (ndx < 0)
        {
          return ndx;
        }

      /* If the new ISR is NULL, then the ISR is being detached.
       * In this case, disable the ISR and direct any interrupts
       * to the unexpected interrupt handler.
       */

      flags = spin_lock_irqsave(&g_irqlock);
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
          spin_unlock_irqrestore(&g_irqlock, flags);
          return ret;
        }
#endif

      /* Save the new ISR and its argument in the table. */

      g_irqvector[ndx].handler = isr;
      g_irqvector[ndx].arg     = arg;
#ifdef CONFIG_SCHED_IRQMONITOR
      g_irqvector[ndx].start   = clock_systime_ticks();
      g_irqvector[ndx].time    = 0;
      g_irqvector[ndx].count   = 0;
#endif

      spin_unlock_irqrestore(&g_irqlock, flags);
      ret = OK;
    }

  return ret;
#else
  return OK;
#endif
}
