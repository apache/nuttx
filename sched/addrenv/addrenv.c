/****************************************************************************
 * sched/addrenv/addrenv.c
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

#include <nuttx/addrenv.h>
#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <nuttx/wqueue.h>

#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This variable holds the current address environment. These contents are
 * _never_ NULL, besides when the system is started and there are only the
 * initial kernel mappings available.
 *
 * This must only be accessed with interrupts disabled.
 *
 * REVISIT: Try to get rid of this, global bookkeeping for this is dangerous.
 */

static FAR struct addrenv_s *g_addrenv[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: addrenv_destroy
 *
 * Description:
 *   Deferred service routine for destroying an address environment. This is
 *   so that the heavy lifting is not done when the context is switching, or
 *   from ISR.
 *
 * Input Parameters:
 *   arg - Contains pointer to the address environment that is freed.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void addrenv_destroy(FAR void *arg)
{
  FAR struct addrenv_s *addrenv = (FAR struct addrenv_s *)arg;

  /* Destroy the address environment */

  up_addrenv_destroy(&addrenv->addrenv);

  /* Then finally release the memory */

  kmm_free(addrenv);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: addrenv_switch
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

int addrenv_switch(FAR struct tcb_s *tcb)
{
  FAR struct addrenv_s *curr;
  FAR struct addrenv_s *next;
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
  next = tcb->mm_curr;

  /* Does the group have an address environment? */

  if (!next)
    {
      /* No... just return perhaps leaving a different address environment
       * intact.
       */

      return OK;
    }

  flags = enter_critical_section();

  cpu = this_cpu();
  curr = g_addrenv[cpu];

  /* Are we going to change address environments? */

  if (curr != next)
    {
      /* Yes.. Is there a current address environment in place? */

      if (curr)
        {
          /* We need to flush the D-Cache and Invalidate the I-Cache for
           * the group whose environment is disappearing.
           */

          up_addrenv_coherent(&curr->addrenv);
        }

      /* While the address environment is instantiated, it cannot be freed */

      addrenv_take(next);

      /* Instantiate the new address environment (removing the old
       * environment in the process).  For the case of kernel threads,
       * the old mappings will be removed and no new mappings will be
       * instantiated.
       */

      ret = up_addrenv_select(&next->addrenv, NULL);
      if (ret < 0)
        {
          berr("ERROR: up_addrenv_select failed: %d\n", ret);
        }

      /* This is a safe spot to drop the current address environment */

      if (curr)
        {
          addrenv_drop(curr, true);
        }

      /* Save the new, current address environment group */

      g_addrenv[cpu] = next;
    }

  leave_critical_section(flags);
  return OK;
}

/****************************************************************************
 * Name: addrenv_allocate
 *
 * Description:
 *   Allocate an address environment for a new process.
 *
 * Input Parameters:
 *   tcb   - The tcb of the newly created task.
 *   ttype - The type of the task.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int addrenv_allocate(FAR struct tcb_s *tcb, uint8_t ttype)
{
  int ret = OK;

  if ((ttype & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL)
    {
      tcb->addrenv_own = NULL;
    }
  else
    {
      tcb->addrenv_own = (FAR struct addrenv_s *)
        kmm_zalloc(sizeof(struct addrenv_s));
      if (tcb->addrenv_own == NULL)
        {
          ret = -ENOMEM;
        }
    }

  return ret;
}

/****************************************************************************
 * Name: addrenv_free
 *
 * Description:
 *   Free an address environment for a process.
 *
 * Input Parameters:
 *   tcb - The tcb of the task.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int addrenv_free(FAR struct tcb_s *tcb)
{
  if (tcb->addrenv_own != NULL)
    {
      kmm_free(tcb->addrenv_own);
      tcb->addrenv_own = NULL;
    }

  return OK;
}

/****************************************************************************
 * Name: addrenv_attach
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

int addrenv_attach(FAR struct tcb_s *tcb,
                   FAR const struct arch_addrenv_s *addrenv)
{
  int ret;

  /* Clone the address environment for us */

  ret = up_addrenv_clone(addrenv, &tcb->addrenv_own->addrenv);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_clone failed: %d\n", ret);
      return ret;
    }

  /* Attach the address environment */

  tcb->addrenv_curr = tcb->addrenv_own;
  tcb->addrenv_own->refs = 1;

  return OK;
}

/****************************************************************************
 * Name: addrenv_join
 *
 * Description:
 *   Join the parent process's address environment.
 *
 * Input Parameters:
 *   ptcb - The tcb of the parent process
 *   tcb  - The tcb of the child process
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int addrenv_join(FAR struct tcb_s *ptcb, FAR struct tcb_s *tcb)
{
  int ret;

  ret = up_addrenv_attach(ptcb, tcb);
  if (ret < 0)
    {
      berr("ERROR: up_addrenv_attach failed: %d\n", ret);
      return ret;
    }

  /* Take a reference to the address environment */

  addrenv_take(ptcb->addrenv_own);

  /* Share the parent's address environment */

  tcb->addrenv_own = ptcb->addrenv_own;
  tcb->addrenv_curr = tcb->addrenv_own;

  return OK;
}

/****************************************************************************
 * Name: addrenv_leave
 *
 * Description:
 *   Leave a process's address environment.
 *
 * Input Parameters:
 *   tcb  - The tcb of the process
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int addrenv_leave(FAR struct tcb_s *tcb)
{
  int ret;

  /* Detach from the address environment */

  ret = up_addrenv_detach(tcb);

  /* Then drop the address environment */

  addrenv_drop(tcb->addrenv_own, false);
  tcb->addrenv_own = NULL;

  return ret;
}

/****************************************************************************
 * Name: addrenv_take
 *
 * Description:
 *   Take a reference to an address environment.
 *
 * Input Parameters:
 *   addrenv - The address environment.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

void addrenv_take(FAR struct addrenv_s *addrenv)
{
  irqstate_t flags = enter_critical_section();
  addrenv->refs++;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: addrenv_give
 *
 * Description:
 *   Give back a reference to an address environment.
 *
 * Input Parameters:
 *   addrenv - The address environment.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int addrenv_give(FAR struct addrenv_s *addrenv)
{
  irqstate_t flags;
  int refs;

  flags = enter_critical_section();
  refs = --addrenv->refs;
  leave_critical_section(flags);

  return refs;
}

/****************************************************************************
 * Name: addrenv_drop
 *
 * Description:
 *   Drop an address environment.
 *
 * Input Parameters:
 *   addrenv - The address environment.
 *   deferred - yes: The address environment should be dropped by the worker
 *              no:  The address environment can be dropped at once
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

void addrenv_drop(FAR struct addrenv_s *addrenv, bool deferred)
{
  if (addrenv == NULL)
    {
      /* No address environment, get out */

      return;
    }

  /* If no more users, the address environment can be dropped */

  if (addrenv_give(addrenv) == 0)
    {
      /* Defer dropping if requested to do so, otherwise drop at once */

      if (deferred)
        {
          /* Let the DSR do the heavy lifting */

          work_queue(LPWORK, &addrenv->work, addrenv_destroy, addrenv, 0);
        }
      else
        {
          addrenv_destroy(addrenv);
        }
    }
}
