/****************************************************************************
 * drivers/power/pm/pm_lock.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/power/pm.h>
#include "pm.h"

#if defined(CONFIG_PM)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_domain_lock
 *
 * Description:
 *   Lock the power management operation.
 *
 * Input Parameters:
 *   domain - The PM domain to lock
 *
 * Returned Value:
 *   Return current state
 *
 ****************************************************************************/

irqstate_t pm_domain_lock(int domain)
{
  return spin_lock_irqsave(&g_pmdomains[domain].lock);
}

/****************************************************************************
 * Name: pm_domain_unlock
 *
 * Description:
 *   Unlock the power management operation.
 *
 * Input Parameters:
 *   domain - The PM domain to unlock
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pm_domain_unlock(int domain, irqstate_t flags)
{
  spin_unlock_irqrestore(&g_pmdomains[domain].lock, flags);
}

#endif /* CONFIG_PM */
