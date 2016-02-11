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
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/sched.h>

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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: os_idletask
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

int os_idletask(int argc, FAR char *argv[])
{
  /* Enter the IDLE loop */

  sdbg("CPU%d: Beginning Idle Loop\n");
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
       * inheritance is enabled.  Imaginee the possible issues if the
       * priority of the IDLE thread were to get boosted!  Moral: If you
       * use priority inheritance, then you should also enable the work
       * queue so that is done in a safer context.
       */

      if (kmm_trysemaphore() == 0)
        {
          sched_garbagecollection();
          kmm_givesemaphore();
        }
#endif

      /* Perform any processor-specific idle state operations */

      up_idle();
    }
}

/****************************************************************************
 * Name: os_idletcb_setup
 *
 * Description:
 *   Initialize the IDLE task TCB for this CPU and add it to the correct OS
 *   task list.  The calling sequence here is:
 *
 *   1. os_start() - Initializes the system and calls os_smpstart()
 *   2. os_smpstart() - Calls up_cpustart() for each CPU 1..(CONFIG_SMP_NCPUS-1)
 *   3. up_cpustart() - Calls os_idletcb_setup() when appropriate to configure
 *      the OS data structures.
 *   4. up_cpustart() - may also call os_idletcb_teardown() to recover from
 *      from any errors.
 *
 * Input Parameters:
 *   cpu - The CPU to undel
 *   idle - Memory allocated by os_idletcb_setup
 *   pid - Task ID of the IDLE task
 *
 * Returned Value:
 *   Memory allocated by os_idletcb_setup (for error recovery). NULL is
 *   returned only a a failure to allocate memory.
 *
 ****************************************************************************/

static FAR void *os_idletcb_setup(int cpu, main_t idletask, pid_t pid)
{
  FAR struct os_tcballoc_s *alloc;
  FAR struct task_tcb_s *itcb;
  dq_queue_t *tasklist;

  /* IDLE TCB Initialization ************************************************/
  /* Allocate and clear the IDLE TCB */

  alloc = (FAR struct os_tcballoc_s *)kmm_zalloc(sizeof(struct os_tcballoc_s));
  if (alloc == NULL)
    {
       return NULL;
    }

  /* Initialize the TCB for the IDLE task on the stack.
   * REVISIT: We should be able to use task_schedsetup() to do most of this
   */

  itcb                 = &alloc->tcb;
  itcb->cmn.task_state = TSTATE_TASK_RUNNING;
  itcb->cmn.entry.main = idletask;
  itcb->cmn.flags      = (TCB_FLAG_TTYPE_KERNEL | TCB_FLAG_CPU_ASSIGNED);
  itcb->cmn.cpu        = cpu;

#if CONFIG_TASK_NAME_SIZE > 0
  snprintf(itcb->cmn.name, CONFIG_TASK_NAME_SIZE, "CPU%d IDLE", cpu);
  alloc->idleargv[0]   = itcb->cmn.name;
#else
  alloc->idleargv[0]   = (FAR char *)g_idlename;
#endif
  alloc->idleargv[1]   = NULL;
  itcb->argv           = alloc->idleargv;

  /* Add the IDLE task TCB to the end of the assigned task list */

  tasklist = TLIST_HEAD(TSTATE_TASK_RUNNING, cpu);
  dq_addfirst((FAR dq_entry_t *)itcb, tasklist);

  /* Initialize the processor-specific portion of the TCB */

  up_initial_state(&itcb->cmn);

  /* PID assignment *********************************************************/

  g_pidhash[PIDHASH(pid)].tcb = &itcb->cmn;

  /* IDLE Group Initialization **********************************************/
#ifdef HAVE_TASK_GROUP
  /* Allocate the IDLE group */

  DEBUGVERIFY(group_allocate(itcb, itcb->cmn.flags));
#endif

#if CONFIG_NFILE_DESCRIPTORS > 0 || CONFIG_NSOCKET_DESCRIPTORS > 0
  /* Create stdout, stderr, stdin on the IDLE task.  These will be
   * inherited by all of the threads created by the IDLE task.
   */

  DEBUGVERIFY(group_setupidlefiles(itcb));
#endif

#ifdef HAVE_TASK_GROUP
  /* Complete initialization of the IDLE group.  Suppress retention
   * of child status in the IDLE group.
   */

  DEBUGVERIFY(group_initialize(itcb));
  itcb->cmn.group->tg_flags = GROUP_FLAG_NOCLDWAIT;
#endif

  return alloc;
}

/****************************************************************************
 * Name: os_idletcb_teardown
 *
 * Description:
 *   Undo what os_idletcb_setup() did.  Necessary only for error recovery.
 *
 * Input Parameters:
 *   cpu - The CPU to undel
 *   alloc - Memory allocated by os_idletcb_setup
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void os_idletcb_teardown(int cpu, FAR void *alloc)
{
  /* Undo what os_idletcb_setup() did */

  FAR dq_queue_t *tasklist= TLIST_HEAD(TSTATE_TASK_RUNNING, 0);
  dq_init(tasklist);

  kmm_free(alloc);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: os_smpstart
 *
 * Description:
 *   In an SMP configution, only one CPU is initially active (CPU 0). System
 *   initialization occurs on that single thread. At the completion of the
 *   initialization of the OS, just before beginning normal multitasking,
 *   the additional CPUs would be started by calling this function.
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

int os_smpstart(void)
{
  FAR void *alloc;
  pid_t pid;
  int ret;
  int cpu;

  /* Reserve PIDs for IDLE tasks 1..(CONFIG_SMP_NCPUS-1).  We do not have to
   * be careful here because so far because there is only one CPU running and
   * we have not yet started multi-tasking.
   */

  pid        = g_lastpid + 1;     /* Should be 1 */
  g_lastpid += CONFIG_SMP_NCPUS;  /* Should be CONFIG_SMP_NCPUS */

  /* CPU0 is already running.  Start the remaining CPUs */

  for (cpu = 1; cpu < CONFIG_SMP_NCPUS; cpu++, pid++)
    {
      /* Assign a PID to the IDLE task and set up the IDLE thread TCB */

      alloc = os_idletcb_setup(cpu, os_idletask, pid);
      if (alloc == NULL)
        {
          return -ENOMEM;
        }

      /* And start the CPU.  */

      ret = up_cpustart(cpu, os_idletask, pid);
      if (ret < 0)
        {
          sdbg("ERROR: Failed to start CPU%d: %d\n", cpu, ret);

          /* Undo what os_idletcb_setup() did and return the failure*/

          os_idletcb_teardown(cpu, alloc);
          return ret;
        }
    }

  return OK;
}

#endif /* CONFIG_SMP */
