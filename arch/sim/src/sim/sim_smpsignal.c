/****************************************************************************
 * arch/sim/src/sim/sim_smpsignal.c
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

#include <assert.h>

#include <nuttx/sched.h>
#include <nuttx/sched_note.h>
#include <nuttx/spinlock.h>

#include "sched/sched.h"
#include "sim_internal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_smp_call_handler
 *
 * Description:
 *   This is the handler for SMP_CALL.
 *
 ****************************************************************************/

static int sim_smp_call_handler(int irq, void *context, void *arg)
{
  struct tcb_s *tcb;
  int cpu = this_cpu();

  tcb = current_task(cpu);
  sim_savestate(tcb->xcp.regs);
  nxsched_smp_call_handler(irq, context, arg);
  tcb = current_task(cpu);
  sim_restorestate(tcb->xcp.regs);

  return OK;
}

/****************************************************************************
 * Name: sim_smp_sched_handler
 *
 * Description:
 *   This is the handler for smp.
 *
 ****************************************************************************/

static int sim_smp_sched_handler(int irq, void *context, void *arg)
{
  struct tcb_s *tcb;
  int cpu = this_cpu();

  tcb = current_task(cpu);
  sim_savestate(tcb->xcp.regs);
  nxsched_process_delivered(cpu);
  tcb = current_task(cpu);
  sim_restorestate(tcb->xcp.regs);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: host_cpu_started
 *
 * Description:
 *   Notify the current cpu start successfully.
 *
 ****************************************************************************/

void host_cpu_started(void)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION_SWITCH
  struct tcb_s *tcb = this_task();

  /* Notify that this CPU has started */

  sched_note_cpu_started(tcb);

  /* Announce that the IDLE task has started */

  sched_note_start(tcb);
#endif
}

/****************************************************************************
 * Name: up_cpu_start
 *
 * Description:
 *   In an SMP configution, only one CPU is initially active (CPU 0). System
 *   initialization occurs on that single thread. At the completion of the
 *   initialization of the OS, just before beginning normal multitasking,
 *   the additional CPUs would be started by calling this function.
 *
 *   Each CPU is provided the entry point to is IDLE task when started.  A
 *   TCB for each CPU's IDLE task has been initialized and placed in the
 *   CPU's g_assignedtasks[cpu] list.  A stack has also been allocateded and
 *   initialized.
 *
 *   The OS initialization logic calls this function repeatedly until each
 *   CPU has been started, 1 through (CONFIG_SMP_NCPUS-1).
 *
 * Input Parameters:
 *   cpu - The index of the CPU being started.  This will be a numeric
 *         value in the range of from one to (CONFIG_SMP_NCPUS-1).  (CPU
 *         0 is already active)
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_cpu_start(int cpu)
{
  struct tcb_s *tcb = current_task(cpu);

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the start event */

  sched_note_cpu_start(this_task(), cpu);
#endif

  return host_cpu_start(cpu, tcb->stack_base_ptr, tcb->adj_stack_size);
}

/****************************************************************************
 * Name: sim_init_ipi
 *
 * Description:
 *   Attach the CPU pause request interrupt to the NuttX logic.
 *
 * Input Parameters:
 *   irq - the SIGUSR1 interrupt number
 *
 * Returned Value:
 *   On success returns OK (0), otherwise a negative value.
 ****************************************************************************/

int sim_init_ipi(int irq)
{
  up_enable_irq(irq);
  return irq_attach(irq, sim_smp_sched_handler, NULL);
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
  /* Generate IRQ for CPU(cpu) */

  host_send_ipi(cpu);

  return OK;
}

/****************************************************************************
 * Name: sim_init_func_call_ipi
 *
 * Description:
 *   Attach the CPU function call request interrupt to the NuttX logic.
 *
 * Input Parameters:
 *   irq - the SIGUSR2 interrupt number
 *
 * Returned Value:
 *   On success returns OK (0), otherwise a negative value.
 ****************************************************************************/

int sim_init_func_call_ipi(int irq)
{
  up_enable_irq(irq);
  return irq_attach(irq, sim_smp_call_handler, NULL);
}

/****************************************************************************
 * Name: up_send_smp_call
 *
 * Description:
 *   Notify the cpuset cpus handler function calls.
 *
 ****************************************************************************/

void up_send_smp_call(cpu_set_t cpuset)
{
  int cpu;

  for (; cpuset != 0; cpuset &= ~(1 << cpu))
    {
      cpu = ffs(cpuset) - 1;
      host_send_func_call_ipi(cpu);
    }
}
