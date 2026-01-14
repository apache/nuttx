/****************************************************************************
 * drivers/power/pm/pm_changestate.c
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

#include <assert.h>
#include <stdlib.h>

#include <nuttx/power/pm.h>
#include <nuttx/irq.h>
#include <nuttx/queue.h>

#include "pm.h"

#ifdef CONFIG_PM

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_stats
 *
 * Description:
 *   Statistic when domain on state change events.
 *
 * Input Parameters:
 *   dom      - Identifies the target domain for Statistic
 *   curstate - Identifies the current PM state
 *   newstate - Identifies the new PM state
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/
#ifdef CONFIG_PM_PROCFS
static void pm_stats(FAR struct pm_domain_s *dom, int curstate, int newstate)
{
  struct timespec now;
  struct timespec ts;

  clock_systime_timespec(&now);
  ts = now;
  clock_timespec_subtract(&ts, &dom->start, &ts);

  /* Update start */

  dom->start = now;

  if (newstate == PM_RESTORE)
    {
      /* Wakeup from WFI */

      clock_timespec_add(&ts, &dom->sleep[curstate], &dom->sleep[curstate]);
      dom->in_sleep = false;
    }
  else
    {
      /* Sleep to WFI */

      clock_timespec_add(&ts, &dom->wake[curstate], &dom->wake[curstate]);
      dom->in_sleep = true;
    }
}

/****************************************************************************
 * Name: pm_stats_preparefail
 *
 * Description:
 *   Statistic the domain on drivers prepare failed.
 *
 * Input Parameters:
 *   domain   - Identifies the target domain for Statistic
 *   callback - The prepare failed callback
 *   newstate - The target new state to prepare
 *   ret      - The driver prepare failed returned value
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void pm_stats_preparefail(int domain,
                                 FAR struct pm_callback_s *callback,
                                 int newstate, int ret)
{
  struct timespec ts;
  FAR struct pm_preparefail_s *pf = &callback->preparefail;

  if (pf->state != PM_RESTORE)
    {
      clock_systime_timespec(&ts);
      clock_timespec_subtract(&ts, &pf->start, &ts);
      clock_timespec_add(&ts, &pf->duration[pf->state],
                         &pf->duration[pf->state]);
      pf->state = PM_RESTORE;
    }

  if (ret < 0)
    {
      clock_systime_timespec(&pf->start);
      pf->state = newstate;
    }
}

#else
#  define pm_stats(dom, curstate, newstate)
#  define pm_stats_preparefail(domain, callback, newstate, ret)
#endif

/****************************************************************************
 * Name: pm_prepall
 *
 * Description:
 *   Prepare every driver for the state change.
 *
 * Input Parameters:
 *   domain   - Identifies the domain of the new PM state
 *   newstate - Identifies the new PM state
 *   restore  - Indicate currently in revert the preceding prepare stage.
 *
 * Returned Value:
 *   0 (OK) means that the callback function for all registered drivers
 *   returned OK (meaning that they accept the state change).  Non-zero
 *   means that one of the drivers refused the state change.  In this case,
 *   the system will revert to the preceding state.
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

static int pm_prepall(int domain, enum pm_state_e newstate, bool restore)
{
  FAR struct pm_domain_s *pdom;
  FAR struct pm_callback_s *cb;
  FAR dq_entry_t *entry;
  int ret = OK;

  pdom = &g_pmdomains[domain];
  if (newstate <= pdom->state)
    {
      /* Visit each registered callback structure in normal order. */

      for (entry = dq_peek(&pdom->registry);
           entry && ret == OK;
           entry = dq_next(entry))
        {
          /* Is the prepare callback supported? */

          cb = (FAR struct pm_callback_s *)entry;
          if (cb->prepare)
            {
              /* Yes.. prepare the driver */

              ret = cb->prepare(cb, domain, newstate);
              if (!restore)
                {
                  pm_stats_preparefail(domain, cb, newstate, ret);
                }
            }
        }
    }
  else
    {
      /* Visit each registered callback structure in reverse order. */

      for (entry = dq_tail(&pdom->registry);
           entry && ret == OK;
           entry = dq_prev(entry))
        {
          /* Is the prepare callback supported? */

          cb = (FAR struct pm_callback_s *)entry;
          if (cb->prepare)
            {
              /* Yes.. prepare the driver */

              ret = cb->prepare(cb, domain, newstate);
              if (!restore)
                {
                  pm_stats_preparefail(domain, cb, newstate, ret);
                }
            }
        }
    }

  return ret;
}

/****************************************************************************
 * Name: pm_changeall
 *
 * Description:
 *   domain - Identifies the domain of the new PM state
 *   Inform all drivers of the state change.
 *
 * Input Parameters:
 *   domain   - Identifies the domain of the new PM state
 *   newstate - Identifies the new PM state
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Interrupts are disabled.
 *
 ****************************************************************************/

static inline void pm_changeall(int domain, enum pm_state_e newstate)
{
  FAR struct pm_domain_s *pdom;
  FAR struct pm_callback_s *cb;
  FAR dq_entry_t *entry;

  pdom = &g_pmdomains[domain];
  if (newstate <= pdom->state)
    {
      /* Visit each registered callback structure in normal order. */

      for (entry = dq_peek(&pdom->registry);
           entry; entry = dq_next(entry))
        {
          /* Is the notification callback supported? */

          cb = (FAR struct pm_callback_s *)entry;
          if (cb->notify)
            {
              /* Yes.. notify the driver */

              cb->notify(cb, domain, newstate);
            }
        }
    }
  else
    {
      /* Visit each registered callback structure in reverse order. */

      for (entry = dq_tail(&pdom->registry);
           entry; entry = dq_prev(entry))
        {
          /* Is the notification callback supported? */

          cb = (FAR struct pm_callback_s *)entry;
          if (cb->notify)
            {
              /* Yes.. notify the driver */

              cb->notify(cb, domain, newstate);
            }
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_changestate
 *
 * Description:
 *   This function is used by platform-specific power management logic.  It
 *   will announce the power management power management state change to all
 *   drivers that have registered for power management event callbacks.
 *
 * Input Parameters:
 *   domain   - Identifies the domain of the new PM state
 *   newstate - Identifies the new PM state
 *
 * Returned Value:
 *   0 (OK) means that the callback function for all registered drivers
 *   returned OK (meaning that they accept the state change).  Non-zero
 *   means that one of the drivers refused the state change.  In this case,
 *   the system will revert to the preceding state.
 *
 * Assumptions:
 *   It is assumed that interrupts are disabled when this function is
 *   called.  This function is probably called from the IDLE loop... the
 *   lowest priority task in the system.  Changing driver power management
 *   states may result in renewed system activity and, as a result, can
 *   suspend the IDLE thread before it completes the entire state change
 *   unless interrupts are disabled throughout the state change.
 *
 ****************************************************************************/

int pm_changestate(int domain, enum pm_state_e newstate)
{
  irqstate_t flags;
  int ret = OK;

  DEBUGASSERT(domain >= 0 && domain < CONFIG_PM_NDOMAINS);

  /* Disable interrupts throughout this operation... changing driver states
   * could cause additional driver activity that might interfere with the
   * state change.  When the state change is complete, interrupts will be
   * re-enabled.
   */

  flags = pm_domain_lock(domain);

  if (newstate != PM_RESTORE)
    {
      /* First, prepare the drivers for the state change.  In this phase,
       * drivers may refuse the state state change.
       */

      ret = pm_prepall(domain, newstate, false);
      if (ret != OK)
        {
          /* One or more drivers is not ready for this state change.
           * Revert to the preceding state.
           */

          newstate = g_pmdomains[domain].state;
          pm_prepall(domain, newstate, true);
        }
    }

  /* Statistics */

  pm_stats(&g_pmdomains[domain],
           g_pmdomains[domain].state, newstate);

  /* All drivers have agreed to the state change (or, one or more have
   * disagreed and the state has been reverted).  Set the new state.
   */

  pm_changeall(domain, newstate);

  /* Notify governor of (possible) state change */

  if (g_pmdomains[domain].governor->statechanged)
    {
      g_pmdomains[domain].governor->statechanged(domain, newstate);
    }

  /* Domain state update after statechanged done */

  if (newstate != PM_RESTORE)
    {
      g_pmdomains[domain].state = newstate;
    }

  /* Restore the interrupt state */

  pm_domain_unlock(domain, flags);
  return ret;
}

/****************************************************************************
 * Name: pm_querystate
 *
 * Description:
 *   This function returns the current power management state.
 *
 * Input Parameters:
 *   domain - The PM domain to check
 *
 * Returned Value:
 *   The current power management state.
 *
 ****************************************************************************/

enum pm_state_e pm_querystate(int domain)
{
  DEBUGASSERT(domain >= 0 && domain < CONFIG_PM_NDOMAINS);
  return g_pmdomains[domain].state;
}

#endif /* CONFIG_PM */
