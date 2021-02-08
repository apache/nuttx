/****************************************************************************
 * sched/irq/irq_foreach.c
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

#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>

#include "irq/irq.h"

#ifdef CONFIG_SCHED_IRQMONITOR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* This is the number of entries in the interrupt vector table */

#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
#  define TAB_SIZE CONFIG_ARCH_NUSER_INTERRUPTS
#else
#  define TAB_SIZE NR_IRQS
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: irq_foreach
 *
 * Description:
 *   This function traverses the internal list of interrupts and provides
 *   information about each attached interrupt.
 *
 *   Some caution may be necessary:  If interrupts are disabled then the
 *   counts may change during the traversal.  If pre-emption is enabled, then
 *   the traversed sequence may be widely separated in time.
 *
 * Input Parameters:
 *   callback - This function will be called for each attached interrupt
 *              along with the IRQ number, an instance of struct irq_info_s,
 *              and the caller provided argument
 *   args     - This is an opaque argument provided with each call to the
 *              callback function.
 *
 * Returned Value:
 *   Zero (OK) is returned after callback has been invoked for all of
 *   the attached interrupts.  The callback function may terminate the
 *   traversal at any time by returning a non-zero value.  In that case,
 *   irq_foreach will return that non-zero value.
 *
 ****************************************************************************/

int irq_foreach(irq_foreach_t callback, FAR void *arg)
{
  int irq;
  int ret;

  DEBUGASSERT(callback != NULL);

  /* Visit each interrupt in the interrupt table */

  for (irq = 0; irq < TAB_SIZE; irq++)
    {
      if (g_irqvector[irq].handler != NULL &&
          g_irqvector[irq].handler != irq_unexpected_isr)
        {
          ret = callback(irq, &g_irqvector[irq], arg);
          if (ret != 0)
            {
              return ret;
            }
        }
    }

  return OK;
}

#endif /* CONFIG_SCHED_IRQMONITOR */
