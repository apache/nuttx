/****************************************************************************
 * drivers/power/pm/pm_checkstate.c
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

#include <nuttx/power/pm.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>

#include "pm.h"

#ifdef CONFIG_PM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_checkstate
 *
 * Description:
 *   This function is called from the MCU-specific IDLE loop to monitor the
 *   the power management conditions.  This function returns the
 *   "recommended" power management state based on the PM policy applied by
 *   the currently chosen governor. The IDLE loop must call pm_changestate()
 *   in order to make the state change, which will interact with all drivers
 *   registered with the PM system.
 *
 *   These two steps are separated because the plaform-specific IDLE loop may
 *   have additional situational information that is not available to the
 *   the PM sub-system.  For example, the IDLE loop may know that the
 *   battery charge level is very low and may force lower power states
 *   even if there is activity.
 *
 *   NOTE: That these two steps are separated in time and, hence, the IDLE
 *   loop could be suspended for a long period of time between calling
 *   pm_checkstate() and pm_changestate().  The IDLE loop may need to make
 *   these calls atomic by either disabling interrupts until the state change
 *   is completed.
 *
 * Input Parameters:
 *   domain - the PM domain to check
 *
 * Returned Value:
 *   The recommended power management state.
 *
 ****************************************************************************/

enum pm_state_e pm_checkstate(int domain)
{
  DEBUGASSERT(domain >= 0 && domain < CONFIG_PM_NDOMAINS);

  if (g_pmglobals.domain[domain].governor->checkstate)
    {
      return g_pmglobals.domain[domain].governor->checkstate(domain);
    }

  return PM_NORMAL;
}

#endif /* CONFIG_PM */
