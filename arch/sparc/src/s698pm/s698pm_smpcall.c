/****************************************************************************
 * arch/sparc/src/s698pm/s698pm_smpcall.c
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

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"
#include "sparc_internal.h"
#include "chip.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s698pm_smp_call_handler
 *
 * Description:
 *   This is the handler for SMP_CALL.
 *
 ****************************************************************************/

int s698pm_smp_call_handler(int irq, void *c, void *arg)
{
  struct tcb_s *tcb;
  int cpu = this_cpu();

  /* Clear IPI (Inter-Processor-Interrupt) */

  putreg32(1 << S698PM_IPI_VECTOR, S698PM_IRQREG_ICLEAR);

  tcb = current_task(cpu);
  sparc_savestate(tcb->xcp.regs);
  nxsched_smp_call_handler(irq, c, arg);
  nxsched_process_delivered(cpu);
  tcb = current_task(cpu);
  sparc_restorestate(tcb->xcp.regs);

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
  uintptr_t regaddr;

  /* Execute Pause IRQ to CPU(cpu) */

  regaddr = (uintptr_t)S698PM_IRQREG_P0_FORCE + (4 * cpu);
  putreg32(1 << S698PM_IPI_VECTOR, regaddr);

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
  int cpu;

  for (; cpuset != 0; cpuset &= ~(1 << cpu))
    {
      cpu = ffs(cpuset) - 1;
      up_send_smp_sched(cpu);
    }
}
