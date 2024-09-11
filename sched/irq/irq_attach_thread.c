/****************************************************************************
 * sched/irq/irq_attach_thread.c
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
#include <nuttx/arch.h>
#include <nuttx/kthread.h>

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

struct irq_thread_info_s
{
  xcpt_t handler;     /* Address of the interrupt handler */
  FAR void *arg;      /* The argument provided to the interrupt handler. */
  FAR sem_t *sem;     /* irq sem used to notify irq thread */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static pid_t g_irq_thread_pid[NR_IRQS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Default interrupt handler for threaded interrupts.
 * Useful for oneshot interrupts.
 */

static int irq_default_handler(int irq, FAR void *regs, FAR void *arg)
{
  FAR struct irq_thread_info_s *info = arg;
  int ret = IRQ_WAKE_THREAD;

  DEBUGASSERT(info->handler != NULL);
  ret = info->handler(irq, regs, info->arg);

  if (ret == IRQ_WAKE_THREAD)
    {
      nxsem_post(info->sem);
      ret = OK;
    }

  return ret;
}

static int isr_thread_main(int argc, FAR char *argv[])
{
  int irq = atoi(argv[1]);
  xcpt_t isr = (xcpt_t)((uintptr_t)strtoul(argv[2], NULL, 16));
  xcpt_t isrthread = (xcpt_t)((uintptr_t)strtoul(argv[3], NULL, 16));
  FAR void *arg = (FAR void *)((uintptr_t)strtoul(argv[4], NULL, 16));
  struct irq_thread_info_s info;
  sem_t sem;

  info.sem = &sem;
  info.arg = arg;
  info.handler = isr;

  nxsem_init(&sem, 0, 0);

  irq_attach(irq, irq_default_handler, &info);

#if !defined(CONFIG_ARCH_NOINTC)
  up_enable_irq(irq);
#endif

  for (; ; )
    {
      if (nxsem_wait_uninterruptible(&sem) < 0)
        {
          continue;
        }

      isrthread(irq, NULL, arg);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: irq_attach_thread
 *
 * Description:
 *   Configure the IRQ subsystem so that IRQ number 'irq' is dispatched to
 *   'isrthread' and up_enable_irq will be invoked after isrthread started.
 *
 * Input Parameters:
 *   irq - Irq num
 *   isr - Function to be called when the IRQ occurs, called in interrupt
 *   context.
 *   If isr is NULL the default handler is installed(irq_default_handler).
 *   isrthread - called in thread context, If the isrthread is NULL,
 *   then the ISR is being detached.
 *   arg - privdate data
 *   priority   - Priority of the new task
 *   stack_size - size (in bytes) of the stack needed
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int irq_attach_thread(int irq, xcpt_t isr, xcpt_t isrthread, FAR void *arg,
                      int priority, int stack_size)
{
#if NR_IRQS > 0
  FAR char *argv[5];
  char arg1[32];  /* irq */
  char arg2[32];  /* isr */
  char arg3[32];  /* isrthread */
  char arg4[32];  /* arg */
  pid_t pid;
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

  /* If the isrthread is NULL, then the ISR is being detached. */

  if (isrthread == NULL)
    {
      irq_detach(irq);
      DEBUGASSERT(g_irq_thread_pid[ndx] != 0);
      kthread_delete(g_irq_thread_pid[ndx]);
      g_irq_thread_pid[ndx] = 0;

      return OK;
    }

  if (g_irq_thread_pid[ndx] != 0)
    {
      return -EINVAL;
    }

  snprintf(arg1, sizeof(arg1), "%d", irq);
  snprintf(arg2, sizeof(arg2), "%p", isr);
  snprintf(arg3, sizeof(arg3), "%p", isrthread);
  snprintf(arg4, sizeof(arg4), "%p", arg);
  argv[0] = arg1;
  argv[1] = arg2;
  argv[2] = arg3;
  argv[3] = arg4;
  argv[4] = NULL;

  pid = kthread_create("isr_thread", priority, stack_size,
                        isr_thread_main, argv);
  if (pid < 0)
    {
      return pid;
    }

  g_irq_thread_pid[ndx] = pid;

#endif /* NR_IRQS */

  return OK;
}
