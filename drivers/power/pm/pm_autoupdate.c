/****************************************************************************
 * drivers/power/pm/pm_autoupdate.c
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
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/power/pm.h>
#include "pm.h"

#if defined(CONFIG_PM)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void pm_auto_updatestate_cb(FAR void *arg)
{
  int domain = (uintptr_t)arg;
  enum pm_state_e newstate;
  irqstate_t flags;

  flags = pm_domain_lock(domain);

  newstate = pm_checkstate(domain);
  pm_changestate(domain, newstate);

  pm_domain_unlock(domain, flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_auto_updatestate
 *
 * Description:
 *   This function update the domain state and notify the power system.
 *
 * Input Parameters:
 *   domain - The PM domain to check
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void pm_auto_updatestate(int domain)
{
  FAR struct pm_domain_s *pdom;
  pdom = &g_pmglobals.domain[domain];

  if (pdom->auto_update)
    {
#if defined(CONFIG_SCHED_WORKQUEUE)
      if (up_interrupt_context())
        {
          work_queue(HPWORK, &pdom->update_work,
                     pm_auto_updatestate_cb, (FAR void *)domain, 0);
        }
      else
#endif
        {
          pm_auto_updatestate_cb((FAR void *)domain);
        }
    }
}

/****************************************************************************
 * Name: pm_auto_update
 *
 * Description:
 *   This function set the domain with assign mode.
 *
 * Input Parameters:
 *   domain        - The PM domain to check
 *   auto_update   - The PM domain auto update or not
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void pm_auto_update(int domain, bool auto_update)
{
  FAR struct pm_domain_s *pdom;
  irqstate_t flags;

  DEBUGASSERT(domain >= 0 && domain < CONFIG_PM_NDOMAINS);
  pdom = &g_pmglobals.domain[domain];

  flags = pm_domain_lock(domain);
  pdom->auto_update = auto_update;
  pm_domain_unlock(domain, flags);
}

#endif /* CONFIG_PM */
