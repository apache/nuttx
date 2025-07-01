/****************************************************************************
 * sched/sched/sched_process_delivered.c
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

#include <stdbool.h>
#include <assert.h>

#include <nuttx/queue.h>

#include "irq/irq.h"
#include "sched/sched.h"
#include "sched/queue.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsched_process_delivered
 *
 * Description:
 *    This function is used to process the tcb in g_delivertasks.
 * 1 We use direct locking instead of enter_critical_section
 * to save processing time
 * 2 If there is a higher priority task, we will still perform
 * the higher priority task
 * 3 If the schedule lock is on, the task will be placed in g_pendingtasks
 *
 * Input Parameters:
 *   cpu
 *
 * Returned Value:
 *   OK
 *
 * Assumptions:
 * - The caller must be in irq
 * - current cpu must not be locked
 *
 ****************************************************************************/

void nxsched_process_delivered(int cpu)
{
  struct tcb_s *btcb = NULL;

  DEBUGASSERT(g_cpu_nestcount[cpu] == 0);
  DEBUGASSERT(up_interrupt_context());

  if ((g_cpu_irqset & (1 << cpu)) == 0)
    {
      spin_lock_notrace(&g_cpu_irqlock);

      g_cpu_irqset |= (1 << cpu);
    }

  if (g_delivertasks[cpu] != NULL)
    {
      btcb = g_delivertasks[cpu];
      g_delivertasks[cpu] = NULL;
      nxsched_switch_running(btcb, cpu);
    }

  if (current_task(cpu)->irqcount <= 0)
    {
      cpu_irqlock_clear();
    }
}
