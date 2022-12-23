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
#include <nuttx/wqueue.h>

#include "sched/sched.h"
#include "group/group.h"

#ifdef CONFIG_ARCH_ADDRENV

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define has_addrenv(grp) (((grp)->tg_flags & GROUP_FLAG_ADDRENV) != 0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: addrenv_dsr
 *
 * Description:
 *   Deferred service routine for destroying an address environment. This is
 *   so that the heavy lifting is not done when the context is switching, or
 *   from ISR.

 * Input Parameters:
 *   arg - Contains pointer to the group structure that is freed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void addrenv_dsr(FAR void *arg)
{
  FAR struct task_group_s *group = (struct task_group_s *)arg;
  save_addrenv_t oldenv;
  irqstate_t flags;

  /* A context switch must not happen here */

  flags = enter_critical_section();

  /* Switch the addrenv and also save the current addrenv */

  up_addrenv_select(&group->tg_addrenv, &oldenv);

  /* Destroy the group address environment */

  up_addrenv_destroy(&group->tg_addrenv);

  /* Restore the previous addrenv */

  up_addrenv_restore(&oldenv);

  /* Can release the scheduler now */

  leave_critical_section(flags);

  /* Address environment can finally be freed */

  group->tg_addrenv_refs = 0;

  /* Finally drop the group itself */

  group_drop(group);
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This variable holds the group of the current address environment, it can
 * belong to another group, if the current task is e.g. a kernel task. These
 * contents are _never_ NULL, besides when the system is started and there
 * are only the initial kernel mappings available.
 *
 * This must only be accessed with interrupts disabled.
 */

static FAR struct task_group_s *g_addrenv_group[CONFIG_SMP_NCPUS];

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
  FAR struct task_group_s *curr;
  FAR struct task_group_s *next;
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

  DEBUGASSERT(tcb);
  next = tcb->mm_group;

  /* Does the group have an address environment? */

  if (!next || !has_addrenv(next))
    {
      /* No... just return perhaps leaving a different address environment
       * intact.
       */

      return OK;
    }

  flags = enter_critical_section();

  cpu = this_cpu();
  curr = g_addrenv_group[cpu];

  /* Are we going to change address environments? */

  if (curr != next)
    {
      /* Yes.. Is there a current address environment in place? */

      if (curr)
        {
          /* We need to flush the D-Cache and Invalidate the I-Cache for
           * the group whose environment is disappearing.
           */

          DEBUGASSERT(has_addrenv(curr));
          up_addrenv_coherent(&curr->tg_addrenv);
        }

      /* While the address environment is instantiated, it cannot be freed */

      group_addrenv_take(next);

      /* Instantiate the new address environment (removing the old
       * environment in the process).  For the case of kernel threads,
       * the old mappings will be removed and no new mappings will be
       * instantiated.
       */

      ret = up_addrenv_select(&next->tg_addrenv, NULL);
      if (ret < 0)
        {
          berr("ERROR: up_addrenv_select failed: %d\n", ret);
        }

      /* This is a safe spot to drop the current address environment */

      if (curr)
        {
          group_addrenv_drop(curr, true);
        }

      /* Save the new, current address environment group */

      g_addrenv_group[cpu] = next;
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: group_addrenv_attach
 *
 * Description:
 *   Attach address environment to a newly created group. Called by exec()
 *   right before injecting the new process into the system.
 *
 * Input Parameters:
 *   tcb     - The tcb of the newly loaded task.
 *   addrenv - The address environment that is attached.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int group_addrenv_attach(FAR struct tcb_s *tcb,
                         FAR const struct group_addrenv_s *addrenv)
{
  int ret;

  /* Clone the address environment for us */

  ret = up_addrenv_clone(addrenv, &tcb->group->tg_addrenv);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_clone failed: %d\n", ret);
      return ret;
    }

  /* Attach the mm group */

  tcb->mm_group = tcb->group;
  tcb->group->tg_flags |= GROUP_FLAG_ADDRENV;
  tcb->group->tg_addrenv_refs = 1;

  return OK;
}

void group_addrenv_take(struct task_group_s *group)
{
  irqstate_t flags = enter_critical_section();
  group->tg_addrenv_refs++;
  leave_critical_section(flags);
}

void group_addrenv_drop(struct task_group_s *group, bool deferred)
{
  irqstate_t flags;
  uint16_t refs;

  flags = enter_critical_section();
  refs = --group->tg_addrenv_refs;
  leave_critical_section(flags);

  /* If no more users, the address environment can be dropped */

  if (refs == 0)
    {
      /* Mark that we no longer have an address environment */

      group->tg_flags &= ~GROUP_FLAG_ADDRENV;

      /* Defer dropping if requested to do so, otherwise drop at once */

      if (deferred)
        {
          /* Need to hold the address environment for a while still */

          group->tg_addrenv_refs = 1;

          /* Let the DSR do the heavy lifting */

          work_queue(LPWORK, &group->tg_addrenv_work, addrenv_dsr, group, 0);
        }
      else
        {
          addrenv_dsr(group);
        }
    }
}

#endif /* CONFIG_ARCH_ADDRENV */
