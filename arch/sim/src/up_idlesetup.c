/****************************************************************************
 * arch/sim/src/up_idlesetup.c
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

#include <stdio.h>
#include <queue.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>

#include "group/group.h"
#include "sched/sched.h"

#include "up_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sim_tcballoc_s
{
  struct task_tcb_s tcb;  /* IDLE task TCB */
  char *idleargv[2];      /* Argument list */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if CONFIG_TASK_NAME_SIZE < 1
static const char g_idlename[] = "Idle Task"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_idletcb_setup
 *
 * Description:
 *   Initialize the IDLE task TCB for this CPU and add it to the
 *
 * Input Parameters:
 *   cpu - The CPU to undel
 *   idle - Memory allocated by sim_idletcb_setup
 *
 * Returned Value:
 *   Memory allocated by sim_idletcb_setup (for error recovery). NULL is
 *   returned only a a failure to allocate memory.
 *
 ****************************************************************************/

static void *sim_idletcb_setup(int cpu, main_t idletask)
{
  struct sim_tcballoc_s *alloc;
  struct task_tcb_s *itcb;
  dq_queue_t *tasklist;

  /* IDLE TCB Initialization ************************************************/
  /* Allocate and clear the IDLE TCB */

  alloc = (struct sim_tcballoc_s *)kmm_zalloc(sizeof(struct sim_tcballoc_s));
  if (alloc == NULL)
    {
       return NULL;
    }

  /* Initialize the TCB for the IDLE task on the stack */

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

#if 0 /* REVISIT */
  g_pidhash[PIDHASH(0)].tcb = &itcb->cmn;
#endif

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
 * Name: sim_idletcb_teardown
 *
 * Description:
 *   Undo what sim_idletcb_setup() did.
 *
 * Input Parameters:
 *   cpu - The CPU to undel
 *   alloc - Memory allocated by sim_idletcb_setup
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sim_idletcb_teardown(int cpu, void *alloc)
{
  /* Undo what sim_idletcb_setup() did */

  dq_queue_t *tasklist= TLIST_HEAD(TSTATE_TASK_RUNNING, 0);
  dq_init(tasklist);

  kmm_free(alloc);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_cpustart
 *
 * Description:
 *   In an SMP configution, only one CPU is initially active (CPU 0). System
 *   initialization occurs on that single thread. At the completion of the
 *   initialization of the OS, just before beginning normal multitasking,
 *   the additional CPUs would be started by calling this function.
 *
 *   Each CPU is provided the entry point to is IDLE task when started.  The
 *   OS initialization logic calls this function repeatedly until each CPU
 *   has been started.
 *
 * Input Parameters:
 *   cpu - The index of the CPU being started.  This will be a numeric
 *         value in the range of from one to (CONFIG_SMP_NCPUS-1).  (CPU
 *         0 is already active)
 *   idletask - The entry point to the IDLE task.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_cpustart(int cpu, main_t idletask)
{
  void *alloc;
  int ret;

  /* Set up the IDLE thread TCB */

  alloc = sim_idletcb_setup(cpu, idletask);
  if (alloc == NULL)
    {
      return -ENOMEM;
    }

  /* Start the CPU emulation thread.  This is analogous to starting the CPU
   * in a multi-CPU hardware model.
   */

  ret = sim_cpustart(cpu, idletask);
  if (ret < 0)
    {
      /* Undo what sim_idletcb_setup() did */

      sim_idletcb_teardown(cpu, alloc);
    }

  return ret;
}
