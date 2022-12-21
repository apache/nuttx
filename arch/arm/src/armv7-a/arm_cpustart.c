/****************************************************************************
 * arch/arm/src/armv7-a/arm_cpustart.c
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

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>

#include "arm_internal.h"
#include "cp15_cacheops.h"
#include "gic.h"
#include "sched/sched.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_start_handler
 *
 * Description:
 *   This is the handler for SGI1.  This handler simply returns from the
 *   interrupt, restoring the state of the new task at the head of the ready
 *   to run list.
 *
 * Input Parameters:
 *   Standard interrupt handling
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int arm_start_handler(int irq, void *context, void *arg)
{
  struct tcb_s *tcb = this_task();

  sinfo("CPU%d Started\n", this_cpu());

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify that this CPU has started */

  sched_note_cpu_started(tcb);
#endif

  /* Reset scheduler parameters */

  nxsched_resume_scheduler(tcb);

  /* Dump registers so that we can see what is going to happen on return */

#if 0
  up_dump_register(tcb->xcp.regs);
#endif

  /* Then switch contexts. This instantiates the exception context of the
   * tcb at the head of the assigned task list.  In this case, this should
   * be the CPUs NULL task.
   */

  arm_restorestate(tcb->xcp.regs);
  return OK;
}

/****************************************************************************
 * Name: up_cpu_start
 *
 * Description:
 *   In an SMP configuration, only one CPU is initially active (CPU 0).
 *   System initialization occurs on that single thread. At the completion of
 *   the initialization of the OS, just before beginning normal multitasking,
 *   the additional CPUs would be started by calling this function.
 *
 *   Each CPU is provided the entry point to its IDLE task when started.  A
 *   TCB for each CPU's IDLE task has been initialized and placed in the
 *   CPU's g_assignedtasks[cpu] list.  No stack has been allocated or
 *   initialized.
 *
 *   The OS initialization logic calls this function repeatedly until each
 *   CPU has been started, 1 through (CONFIG_SMP_NCPUS-1).
 *
 * Input Parameters:
 *   cpu - The index of the CPU being started.  This will be a numeric
 *         value in the range of one to (CONFIG_SMP_NCPUS-1).
 *         (CPU 0 is already active)
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_cpu_start(int cpu)
{
  sinfo("Starting CPU%d\n", cpu);

  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_SMP_NCPUS && cpu != this_cpu());

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the start event */

  sched_note_cpu_start(this_task(), cpu);
#endif

  /* Execute SGI1 */

  arm_cpu_sgi(GIC_IRQ_SGI1, (1 << cpu));
  return OK;
}

#endif /* CONFIG_SMP */
