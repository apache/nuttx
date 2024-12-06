/****************************************************************************
 * arch/x86_64/src/intel64/intel64_smpcall.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>
#include <string.h>
#include <stdio.h>

#include <arch/irq.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"

#include "x86_64_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: x86_64_smp_call_handler
 *
 * Description:
 *   This is the handler for SMP_CALL.
 *
 ****************************************************************************/

int x86_64_smp_call_handler(int irq, void *c, void *arg)
{
  nxsched_smp_call_handler(irq, c, arg);

  return OK;
}

/****************************************************************************
 * Name: x86_64_smp_sched_handler
 *
 * Description:
 *   This is the handler for smp.
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

int x86_64_smp_sched_handler(int irq, void *c, void *arg)
{
  int cpu = this_cpu();

  nxsched_process_delivered(cpu);

  return OK;
}

/****************************************************************************
 * Name: up_send_smp_sched
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

int up_send_smp_sched(int cpu)
{
  cpu_set_t cpuset;

  CPU_ZERO(&cpuset);
  CPU_SET(cpu, &cpuset);

  up_trigger_irq(SMP_IPI_SCHED_IRQ, cpuset);

  return OK;
}

/****************************************************************************
 * Name: up_send_smp_call
 *
 * Description:
 *   Send smp call to target cpu.
 *
 * Input Parameters:
 *   cpuset - The set of CPUs to receive the SGI.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void up_send_smp_call(cpu_set_t cpuset)
{
  up_trigger_irq(SMP_IPI_CALL_IRQ, cpuset);
}
