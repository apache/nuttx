/****************************************************************************
 * drivers/power/pm/pm_lock.c
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
 * Public Functions
 ****************************************************************************/

irqstate_t pm_lock(FAR rmutex_t *lock)
{
  if (!up_interrupt_context() && !sched_idletask())
    {
      nxrmutex_lock(lock);
    }

  return enter_critical_section();
}

void pm_unlock(FAR rmutex_t *lock, irqstate_t flags)
{
  leave_critical_section(flags);

  if (!up_interrupt_context() && !sched_idletask())
    {
      nxrmutex_unlock(lock);
    }
}

irqstate_t pm_domain_lock(int domain)
{
  return pm_lock(&g_pmglobals.domain[domain].lock);
}

void pm_domain_unlock(int domain, irqstate_t flags)
{
  pm_unlock(&g_pmglobals.domain[domain].lock, flags);
}

#endif /* CONFIG_PM */
