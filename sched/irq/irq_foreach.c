/****************************************************************************
 * sched/irq/irq_foreach.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
