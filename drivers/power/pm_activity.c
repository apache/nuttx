/****************************************************************************
 * drivers/power/pm_activity.c
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

#include <stdint.h>
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
 * Name: pm_activity
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   performing meaningful activities (non-idle).  This increments an
 *   activity count and/or will restart a idle timer and prevent entering
 *   reduced power states.
 *
 * Input Parameters:
 *   domain - The domain of the PM activity
 *   priority - Activity priority, range 0-9.  Larger values correspond to
 *     higher priorities.  Higher priority activity can prevent the system
 *     from entering reduced power states for a longer period of time.
 *
 *     As an example, a button press might be higher priority activity
 *     because it means that the user is actively interacting with the
 *     device.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler (this is the ONLY
 *   PM function that may be called from an interrupt handler!).
 *
 ****************************************************************************/

void pm_activity(int domain, int priority)
{
  DEBUGASSERT(domain >= 0 && domain < CONFIG_PM_NDOMAINS);

  if (g_pmglobals.domain[domain].governor->activity)
    {
      g_pmglobals.domain[domain].governor->activity(domain, priority);
    }

  pm_auto_updatestate(domain);
}

/****************************************************************************
 * Name: pm_stay
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   performing meaningful activities (non-idle), needs the power at kept
 *   last the specified level.
 *
 * Input Parameters:
 *   domain - The domain of the PM activity
 *   state - The state want to stay.
 *
 *   As an example, media player might stay in normal state during playback.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

void pm_stay(int domain, enum pm_state_e state)
{
  FAR struct pm_domain_s *pdom;
  irqstate_t flags;

  /* Get a convenience pointer to minimize all of the indexing */

  DEBUGASSERT(domain >= 0 && domain < CONFIG_PM_NDOMAINS);
  pdom = &g_pmglobals.domain[domain];

  flags = enter_critical_section();
  DEBUGASSERT(state < PM_COUNT);
  DEBUGASSERT(pdom->stay[state] < UINT16_MAX);
  pdom->stay[state]++;
  leave_critical_section(flags);

  pm_auto_updatestate(domain);
}

/****************************************************************************
 * Name: pm_relax
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   idle now, could relax the previous requested power level.
 *
 * Input Parameters:
 *   domain - The domain of the PM activity
 *   state - The state want to relax.
 *
 *     As an example, media player might relax power level after playback.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

void pm_relax(int domain, enum pm_state_e state)
{
  FAR struct pm_domain_s *pdom;
  irqstate_t flags;

  /* Get a convenience pointer to minimize all of the indexing */

  DEBUGASSERT(domain >= 0 && domain < CONFIG_PM_NDOMAINS);
  pdom = &g_pmglobals.domain[domain];

  flags = enter_critical_section();
  DEBUGASSERT(state < PM_COUNT);
  DEBUGASSERT(pdom->stay[state] > 0);
  pdom->stay[state]--;
  leave_critical_section(flags);

  pm_auto_updatestate(domain);
}

/****************************************************************************
 * Name: pm_staycount
 *
 * Description:
 *   This function is called to get current stay count.
 *
 * Input Parameters:
 *   domain - The domain of the PM activity
 *   state - The state want to relax.
 *
 * Returned Value:
 *   Current pm stay count
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

uint32_t pm_staycount(int domain, enum pm_state_e state)
{
  FAR struct pm_domain_s *pdom;

  /* Get a convenience pointer to minimize all of the indexing */

  DEBUGASSERT(domain >= 0 && domain < CONFIG_PM_NDOMAINS);
  pdom = &g_pmglobals.domain[domain];

  return pdom->stay[state];
}

#endif /* CONFIG_PM */
