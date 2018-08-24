/****************************************************************************
 * sched/init/os_smpstart.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>
#include <stdio.h>
#include <queue.h>
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
 * Private Types
 ****************************************************************************/

struct os_tcballoc_s
{
  struct task_tcb_s tcb;  /* IDLE task TCB */
  FAR char *idleargv[2];  /* Argument list */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if CONFIG_TASK_NAME_SIZE < 1
static const char g_idlename[] = "CPUn Idle"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: os_idle_trampoline
 *
 * Description:
 *   This is the common start-up logic for the IDLE task for CPUs 1 through
 *   (CONFIG_SMP_NCPUS-1).  Having a start-up function such as this for the
 *   IDLE is not really an architectural necessity.  It is used only for
 *   symmetry with now other threads are started (see task_start() and
 *   pthread_start()).
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   This function does not return.
 *
 ****************************************************************************/

void os_idle_trampoline(void)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION
  FAR struct tcb_s *tcb = this_task();

  /* Announce that the IDLE task has started */

  sched_note_start(tcb);
#endif

  /* Then transfer control to the IDLE task */

  (void)os_idle_task(0, NULL);

  /* The IDLE task should never return */

  DEBUGPANIC();
}

/****************************************************************************
 * Name: os_idle_task
 *
 * Description:
 *   This is the common IDLE task for CPUs 1 through (CONFIG_SMP_NCPUS-1).
 *   It is equivalent to the CPU 0 IDLE logic in os_start.c
 *
 * Input Parameters:
 *   Standard task arguments.
 *
 * Returned Value:
 *   This function does not return.
 *
 ****************************************************************************/

int os_idle_task(int argc, FAR char *argv[])
{
  /* Enter the IDLE loop */

  sinfo("CPU%d: Beginning Idle Loop\n", this_cpu());

  for (; ; )
    {
      /* Perform garbage collection (if it is not being done by the worker
       * thread).  This cleans-up memory de-allocations that were queued
       * because they could not be freed in that execution context (for
       * example, if the memory was freed from an interrupt handler).
       */

#ifndef CONFIG_SCHED_WORKQUEUE
      /* We must have exclusive access to the memory manager to do this
       * BUT the idle task cannot wait on a semaphore.  So we only do
       * the cleanup now if we can get the semaphore -- this should be
       * possible because if the IDLE thread is running, no other task is!
       *
       * WARNING: This logic could have undesirable side-effects if priority
       * inheritance is enabled.  Imagine the possible issues if the
       * priority of the IDLE thread were to get boosted!  Moral: If you
       * use priority inheritance, then you should also enable the work
       * queue so that is done in a safer context.
       */

      if (sched_have_garbage() && kmm_trysemaphore() == 0)
        {
          sched_garbage_collection();
          kmm_givesemaphore();
        }
#endif

      /* Perform any processor-specific idle state operations */

      up_idle();
    }
}

/****************************************************************************
 * Name: os_smp_start
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
 *   Zero on success; a negater errno value on failure.
 *
 * Assumption:
 *   Runs before the full initialization sequence has completed.  Runs after
 *   all OS facilities are set up, but before multi-tasking has been started.
 *
 ****************************************************************************/

int os_smp_start(void)
{
  int ret;
  int cpu;

  /* Create a stack for all CPU IDLE threads (except CPU0 which already has
   * a stack).
   */

  for (cpu = 1; cpu < CONFIG_SMP_NCPUS; cpu++)
    {
      FAR struct tcb_s *tcb = current_task(cpu);
      DEBUGASSERT(tcb != NULL);

      ret = up_cpu_idlestack(cpu, tcb, CONFIG_SMP_IDLETHREAD_STACKSIZE);
      if (ret < 0)
        {
          serr("ERROR: Failed to allocate stack for CPU%d\n", cpu);
          return ret;
        }

      /* Reinitialize the processor-specific portion of the TCB.  This is
       * the second time this has been called for this CPU, but the stack
       * was not yet initialized on the first call so we need to do it
       * again.
       */

      up_initial_state(tcb);

      /* Set the task flags to indicate that this is a kernel thread and that
       * this task is locked to this CPU.
       */

      tcb->flags = (TCB_FLAG_TTYPE_KERNEL | TCB_FLAG_NONCANCELABLE |
                    TCB_FLAG_CPU_LOCKED);
      tcb->cpu   = cpu;
    }

  /* Then start all of the other CPUs after we have completed the memory
   * allocations.  CPU0 is already running.
   */

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
