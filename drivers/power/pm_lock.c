/****************************************************************************
 * drivers/power/pm_lock.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/power/pm.h>
#include <assert.h>
#include <sched.h>
#include "pm.h"

#if defined(CONFIG_PM)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int pm_recursive_lock(struct pm_domain_s *pm)
{
  pid_t me = gettid();
  int ret = OK;

  /* Does this thread already hold the semaphore? */

  if (pm->holder == me)
    {
      /* Yes.. just increment the reference count */

      pm->count++;
    }
  else
    {
      /* No.. take the semaphore (perhaps waiting) */

      ret = nxsem_wait_uninterruptible(&pm->sem);
      if (ret >= 0)
        {
          /* Now this thread holds the semaphore */

          pm->holder = me;
          pm->count  = 1;
        }
    }

  return ret;
}

static void pm_recursive_unlock(struct pm_domain_s *pm)
{
  DEBUGASSERT(pm->holder == getpid() && pm->count > 0);

  /* If the count would go to zero, then release the semaphore */

  if (pm->count == 1)
    {
      /* We no longer hold the semaphore */

      pm->holder = INVALID_PROCESS_ID;
      pm->count  = 0;
      nxsem_post(&pm->sem);
    }
  else
    {
      /* We still hold the semaphore. Just decrement the count */

      pm->count--;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_lock
 *
 * Description:
 *   Lock the power management operation.
 *
 * Input Parameters:
 *   domain - The PM domain to lock
 *
 ****************************************************************************/

irqstate_t pm_lock(int domain)
{
  if (!up_interrupt_context() && !sched_idletask())
    {
      pm_recursive_lock(&g_pmglobals.domain[domain]);
    }

  return enter_critical_section();
}

/****************************************************************************
 * Name: pm_unlock
 *
 * Description:
 *   Unlock the power management operation.
 *
 * Input Parameters:
 *   domain - The PM domain to unlock
 *
 ****************************************************************************/

void pm_unlock(int domain, irqstate_t flags)
{
  leave_critical_section(flags);

  if (!up_interrupt_context() && !sched_idletask())
    {
      pm_recursive_unlock(&g_pmglobals.domain[domain]);
    }
}

#endif /* CONFIG_PM */
