/****************************************************************************
 * arch/arm/src/armv7-r/arm_cpupause.c
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

#include <stdint.h>
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>

#include "arm_internal.h"
#include "gic.h"
#include "sched/sched.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_pause_async_handler
 *
 * Description:
 *   This is the handler for async pause.
 *
 *   1. It saves the current task state at the head of the current assigned
 *      task list.
 *   2. It porcess g_delivertasks
 *   3. Returns from interrupt, restoring the state of the new task at the
 *      head of the ready to run list.
 *
 * Input Parameters:
 *   Standard interrupt handling
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int arm_pause_async_handler(int irq, void *context, void *arg)
{
  int cpu = this_cpu();

  nxsched_process_delivered(cpu);
  return OK;
}

/****************************************************************************
 * Name: up_cpu_pause_async
 *
 * Description:
 *   pause task execution on the CPU
 *   check whether there are tasks delivered to specified cpu
 *   and try to run them.
 *
 * Input Parameters:
 *   cpu - The index of the CPU to be paused.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Called from within a critical section;
 *
 ****************************************************************************/

inline_function int up_cpu_pause_async(int cpu)
{
  arm_cpu_sgi(GIC_SMP_CPUPAUSE_ASYNC, (1 << cpu));

  return OK;
}

#endif /* CONFIG_SMP */
