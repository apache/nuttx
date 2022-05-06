/****************************************************************************
 * sched/group/group_addrenv.c
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
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "group/group.h"

#ifdef CONFIG_ARCH_ADDRENV

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This variable holds the current task group.  This pointer is NULL
 * if the current task is a kernel thread that has no address environment
 * (other than the kernel context).
 *
 * This must only be accessed with interrupts disabled.
 */

FAR struct task_group_s *g_group_current[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_addrenv
 *
 * Description:
 *   Instantiate the group address environment for the current thread at the
 *   the head of the ready to run list.
 *
 *   This function is called from platform-specific code after any context
 *   switch (i.e., after any change in the thread at the head of the
 *   ready-to-run list).  This function will change the address environment
 *   if the new thread is part of a different task group.
 *
 * Input Parameters:
 *   tcb - The TCB of thread that needs an address environment.  This should
 *         be the TCB at the head of the ready-to-run list, but that is not
 *         enough.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   This function should only be called within critical OS sections with
 *   interrupts disabled.  Interrupts are disabled internally just to be
 *   certain, however.
 *
 ****************************************************************************/

int group_addrenv(FAR struct tcb_s *tcb)
{
  FAR struct task_group_s *group;
  FAR struct task_group_s *oldgroup;
  irqstate_t flags;
  int cpu;
  int ret;

  /* NULL for the tcb means to use the TCB of the task at the head of the
   * ready to run list.
   */

  if (!tcb)
    {
      tcb = this_task();
    }

  DEBUGASSERT(tcb && tcb->group);
  group = tcb->group;

  /* Does the group have an address environment? */

  if ((group->tg_flags & GROUP_FLAG_ADDRENV) == 0)
    {
      /* No... just return perhaps leaving a different address environment
       * intact.
       */

      return OK;
    }

  /* Are we going to change address environments? */

  flags = enter_critical_section();

  cpu = this_cpu();
  oldgroup = g_group_current[cpu];
  if (group != oldgroup)
    {
      /* Yes.. Is there a current address environment in place? */

      if (oldgroup)
        {
          /* We need to flush the D-Cache and Invalidate the I-Cache for
           * the group whose environment is disappearing.
           */

          DEBUGASSERT((oldgroup->tg_flags & GROUP_FLAG_ADDRENV) != 0);
          up_addrenv_coherent(&oldgroup->tg_addrenv);
        }

      /* Instantiate the new address environment (removing the old
       * environment in the process).  For the case of kernel threads,
       * the old mappings will be removed and no new mappings will be
       * instantiated.
       */

      ret = up_addrenv_select(&group->tg_addrenv, NULL);
      if (ret < 0)
        {
          berr("ERROR: up_addrenv_select failed: %d\n", ret);
        }

      /* Save the new, current group */

      g_group_current[cpu] = group;
    }

  leave_critical_section(flags);
  return OK;
}

#endif /* CONFIG_ARCH_ADDRENV */
