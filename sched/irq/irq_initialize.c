/****************************************************************************
 * sched/irq/irq_initialize.c
 *
 *   Copyright (C) 2007-2008, 2010, 2018 Gregory Nutt. All rights reserved.
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
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "irq/irq.h"

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
 * Public Data
 ****************************************************************************/

/* This is the interrupt vector table */

#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
struct irq_info_s g_irqvector[CONFIG_ARCH_NUSER_INTERRUPTS];
#else
struct irq_info_s g_irqvector[NR_IRQS];
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: irq_initialize
 *
 * Description:
 *   Configure the IRQ subsystem
 *
 ****************************************************************************/

void irq_initialize(void)
{
  int i;

  /* Point all interrupt vectors to the unexpected interrupt */

  for (i = 0; i < TAB_SIZE; i++)
    {
      g_irqvector[i].handler = irq_unexpected_isr;
      g_irqvector[i].arg     = NULL;
#ifdef CONFIG_SCHED_IRQMONITOR
      g_irqvector[i].start   = 0;
#ifdef CONFIG_HAVE_LONG_LONG
      g_irqvector[i].count   = 0;
#else
      g_irqvector[i].mscount = 0;
      g_irqvector[i].lscount = 0;
#endif
      g_irqvector[i].time    = 0;
#endif
    }

#ifdef CONFIG_IRQCHAIN
  /* Initialize IRQ chain support */

  irqchain_initialize();
#endif

  up_irqinitialize();
}
