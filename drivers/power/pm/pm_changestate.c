/****************************************************************************
 * drivers/power/pm/pm_changestate.c
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
 * Name: pm_prepall
 *
 * Description:
 *   Prepare every driver for the state change.
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
 *   Interrupts are disabled.
 *
 ****************************************************************************/

static int pm_prepall(int domain, enum pm_state_e newstate)
{
  FAR dq_entry_t *entry;
  int ret = OK;

  if (newstate <= g_pmglobals.domain[domain].state)
    {
      /* Visit each registered callback structure in normal order. */

      for (entry = dq_peek(&g_pmglobals.registry);
           entry && ret == OK;
           entry = dq_next(entry))
        {
          /* Is the prepare callback supported? */

          FAR struct pm_callback_s *cb = (FAR struct pm_callback_s *)entry;
          if (cb->prepare)
            {
              /* Yes.. prepare the driver */

              ret = cb->prepare(cb, domain, newstate);
            }
        }
    }
  else
    {
      /* Visit each registered callback structure in reverse order. */

      for (entry = dq_tail(&g_pmglobals.registry);
           entry && ret == OK;
           entry = dq_prev(entry))
        {
          /* Is the prepare callback supported? */

          FAR struct pm_callback_s *cb = (FAR struct pm_callback_s *)entry;
          if (cb->prepare)
            {
              /* Yes.. prepare the driver */

              ret = cb->prepare(cb, domain, newstate);
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
  FAR dq_entry_t *entry;

  if (newstate <= g_pmglobals.domain[domain].state)
    {
      /* Visit each registered callback structure in normal order. */

      for (entry = dq_peek(&g_pmglobals.registry);
           entry; entry = dq_next(entry))
        {
          /* Is the notification callback supported? */

          FAR struct pm_callback_s *cb = (FAR struct pm_callback_s *)entry;
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

      for (entry = dq_tail(&g_pmglobals.registry);
           entry; entry = dq_prev(entry))
        {
          /* Is the notification callback supported? */

          FAR struct pm_callback_s *cb = (FAR struct pm_callback_s *)entry;
          if (cb->notify)
            {
              /* Yes.. notify the driver */

              cb->notify(cb, domain, newstate);
            }
        }
    }
}

#ifdef CONFIG_PM_PROCFS
static void pm_stats(FAR struct pm_domain_s *dom, int curstate, int newstate)
{
  struct timespec ts;

  clock_systime_timespec(&ts);
  clock_timespec_subtract(&ts, &dom->start, &ts);

  if (newstate == PM_RESTORE)
    {
      /* Wakeup from WFI */

      clock_timespec_add(&ts, &dom->sleep[curstate], &dom->sleep[curstate]);
    }
  else
    {
      /* Sleep to WFI */

      clock_timespec_add(&ts, &dom->wake[curstate], &dom->wake[curstate]);
    }

  /* Update start */

  clock_systime_timespec(&dom->start);
}
#else
#  define pm_stats(dom, curstate, newstate)
#endif

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

  flags = pm_lock(domain);

  if (newstate != PM_RESTORE)
    {
      /* First, prepare the drivers for the state change.  In this phase,
       * drivers may refuse the state state change.
       */

      ret = pm_prepall(domain, newstate);
      if (ret != OK)
        {
          /* One or more drivers is not ready for this state change.
           * Revert to the preceding state.
           */

          newstate = g_pmglobals.domain[domain].state;
          pm_prepall(domain, newstate);
        }
    }

  /* Statistics */

  pm_stats(&g_pmglobals.domain[domain],
           g_pmglobals.domain[domain].state, newstate);

  /* All drivers have agreed to the state change (or, one or more have
   * disagreed and the state has been reverted).  Set the new state.
   */

  pm_changeall(domain, newstate);
  if (newstate != PM_RESTORE)
    {
      g_pmglobals.domain[domain].state = newstate;
    }

  /* Notify governor of (possible) state change */

  if (g_pmglobals.domain[domain].governor->statechanged)
    {
      g_pmglobals.domain[domain].governor->statechanged(domain, newstate);
    }

  /* Restore the interrupt state */

  pm_unlock(domain, flags);
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
  return g_pmglobals.domain[domain].state;
}

#endif /* CONFIG_PM */
