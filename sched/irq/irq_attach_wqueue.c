/****************************************************************************
 * sched/irq/irq_attach_wqueue.c
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
#include <stdio.h>

#include <nuttx/irq.h>
#include <nuttx/wqueue.h>
#include <debug.h>

#include "irq/irq.h"
#include "sched/sched.h"

/****************************************************************************
 * Privte Types
 ****************************************************************************/

/* This is the type of the list of interrupt handlers, one for each IRQ.
 * This type provided all of the information necessary to irq_dispatch to
 * transfer control to interrupt handlers after the occurrence of an
 * interrupt.
 */

struct irq_work_info_s
{
  xcpt_t handler;     /* Address of the interrupt handler */
  xcpt_t isrwork;     /* Address of the interrupt worked handler */
  FAR void *arg;      /* The argument provided to the interrupt handler. */
  int irq;            /* Irq id */
  struct work_s work; /* Interrupt work to the wq */

  FAR struct kwork_wqueue_s *wqueue;   /* Work queue. */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
static struct irq_work_info_s
g_irq_work_vector[CONFIG_ARCH_NUSER_INTERRUPTS];
#else
static struct irq_work_info_s g_irq_work_vector[NR_IRQS];
#endif

static mutex_t g_irq_wqueue_lock = NXMUTEX_INITIALIZER;
static FAR struct kwork_wqueue_s *g_irq_wqueue[CONFIG_IRQ_NWORKS];

#ifdef IRQ_WORK_SECTION
static uint8_t g_irq_work_stack[CONFIG_IRQ_NWORKS][CONFIG_IRQ_WORK_STACKSIZE]
locate_data(IRQ_WORK_SECTION);
#else
static uint8_t g_irq_work_stack[CONFIG_IRQ_NWORKS]
                               [CONFIG_IRQ_WORK_STACKSIZE];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static
inline_function FAR struct kwork_wqueue_s *irq_get_wqueue(int priority)
{
  FAR struct kwork_wqueue_s *queue;
  int wqueue_priority;
  int i;

  nxmutex_lock(&g_irq_wqueue_lock);
  for (i = 0; g_irq_wqueue[i] != NULL && i < CONFIG_IRQ_NWORKS; i++)
    {
      wqueue_priority = work_queue_priority_wq(g_irq_wqueue[i]);
      DEBUGASSERT(wqueue_priority >= SCHED_PRIORITY_MIN &&
                  wqueue_priority <= SCHED_PRIORITY_MAX);

      if (wqueue_priority == priority)
        {
          nxmutex_unlock(&g_irq_wqueue_lock);
          return g_irq_wqueue[i];
        }
    }

  DEBUGASSERT(i < CONFIG_IRQ_NWORKS);

  queue = work_queue_create("isrwork", priority, g_irq_work_stack[i],
                            CONFIG_IRQ_WORK_STACKSIZE, 1);

  g_irq_wqueue[i] = queue;
  nxmutex_unlock(&g_irq_wqueue_lock);
  return queue;
}

/* Default interrupt handler for worked interrupts.
 * Useful for oneshot interrupts.
 */

static void irq_work_handler(FAR void *arg)
{
  FAR struct irq_work_info_s *info = arg;

  info->isrwork(info->irq, NULL, info->arg);
}

static int irq_default_handler(int irq, FAR void *regs, FAR void *arg)
{
  FAR struct irq_work_info_s *info = arg;
  int ret;

  ret = info->handler(irq, regs, arg);

  if (ret == IRQ_WAKE_THREAD)
    {
      work_queue_wq(info->wqueue, &info->work, irq_work_handler, info, 0);
      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: irq_attach_wqueue
 *
 * Description:
 *   Configure the IRQ subsystem so that IRQ number 'irq' is dispatched to
 *   'wqueue'
 *
 * Input Parameters:
 *   irq - Irq num
 *   isr - Function to be called when the IRQ occurs, called in interrupt
 *   context.
 *   If isr is NULL the default handler is installed(irq_default_handler).
 *   isrwork - called in thread context, If the isrwork is NULL,
 *   then the ISR is being detached.
 *   arg - privdate data
 *   priority - isrwork pri
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int irq_attach_wqueue(int irq, xcpt_t isr, xcpt_t isrwork,
                      FAR void *arg, int priority)
{
  FAR struct irq_work_info_s *info;

#if NR_IRQS > 0
  int ndx;

  if ((unsigned)irq >= NR_IRQS)
    {
      return -EINVAL;
    }

  ndx = IRQ_TO_NDX(irq);
  if (ndx < 0)
    {
      return ndx;
    }

  /* If the isrwork is NULL, then the ISR is being detached. */

  info = &g_irq_work_vector[ndx];

  if (isrwork == NULL)
    {
      irq_detach(irq);
      info->isrwork = NULL;
      info->handler = NULL;
      info->arg     = NULL;
      info->wqueue  = NULL;
      return OK;
    }

  info->isrwork = isrwork;
  info->handler = isr;
  info->arg     = arg;
  info->irq     = irq;
  if (info->wqueue == NULL)
    {
      info->wqueue = irq_get_wqueue(priority);
    }

  irq_attach(irq, irq_default_handler, info);
#endif /* NR_IRQS */

  return OK;
}

