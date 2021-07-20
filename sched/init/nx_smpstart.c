/****************************************************************************
 * sched/init/nx_smpstart.c
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

#include <sys/types.h>
#include <stdio.h>
#include <queue.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>

#include "group/group.h"
#include "sched/sched.h"
#include "init/init.h"

#ifdef CONFIG_SMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_idle_trampoline
 *
 * Description:
 *   This is the common start-up logic for the IDLE task for CPUs 1 through
 *   (CONFIG_SMP_NCPUS-1).  Having a start-up function such as this for the
 *   IDLE is not really an architectural necessity.  It is used only for
 *   symmetry with now other threads are started (see nxtask_start() and
 *   pthread_start()).
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   This function does not return.
 *
 ****************************************************************************/

void nx_idle_trampoline(void)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION
  FAR struct tcb_s *tcb = this_task();

  /* Announce that the IDLE task has started */

  sched_note_start(tcb);
#endif

  /* Enter the IDLE loop */

  sinfo("CPU%d: Beginning Idle Loop\n", this_cpu());

  for (; ; )
    {
      /* Perform any processor-specific idle state operations */

      up_idle();
    }
}

/****************************************************************************
 * Name: nx_smp_start
 *
 * Description:
 *   In an SMP configuration, only one CPU is initially active (CPU 0).
 *   System initialization occurs on that single thread. At the completion
 *   of the initialization of the OS, just before beginning normal
 *   multitasking, the additional CPUs would be started by calling this
 *   function.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 * Assumption:
 *   Runs before the full initialization sequence has completed.  Runs after
 *   all OS facilities are set up, but before multi-tasking has been started.
 *
 ****************************************************************************/

int nx_smp_start(void)
{
  int ret;
  int cpu;

  /* Start all of the other CPUs.  CPU0 is already running. */

  for (cpu = 1; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      /* Start the CPU */

      ret = up_cpu_start(cpu);
      if (ret < 0)
        {
          serr("ERROR: Failed to start CPU%d: %d\n", cpu, ret);
          return ret;
        }
    }

  return OK;
}

#endif /* CONFIG_SMP */
