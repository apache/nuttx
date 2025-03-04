/****************************************************************************
 * drivers/power/pm/stability_governor.c
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

#include <nuttx/clock.h>
#include <nuttx/config.h>

#include <nuttx/wdog.h>
#include <sys/types.h>
#include <stdbool.h>

#include <nuttx/power/pm.h>

#include "pm.h"

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

struct pm_stability_governor_domain_s
{
  /* Timer to wakeup system, delay the sleep request */

  struct wdog_s wdog;

  /* The Idle is wakeup from the governor wdog itself */

  bool wdog_wakeup;

  /* This state has not been maintained long enough to meet the threshold. */

  enum pm_state_e state_pending;
};

struct pm_stability_governor_s
{
  struct pm_stability_governor_domain_s domain[CONFIG_PM_NDOMAINS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* PM governor methods */

static void stability_governor_statechanged(int domain,
                                            enum pm_state_e newstate);
static enum pm_state_e stability_governor_checkstate(int domain);
static void stability_governor_activity(int domain, int count);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pm_governor_s g_stability_governor_ops =
{
  NULL,                            /* initialize */
  NULL,                            /* deinitialize */
  stability_governor_statechanged, /* statechanged */
  stability_governor_checkstate,   /* checkstate */
  stability_governor_activity,     /* activity */
  NULL                             /* priv */
};

static const clock_t g_stability_governor_thresh[PM_COUNT] =
{
  0,
  CONFIG_PM_GOVERNOR_STABILITY_IDLE_THRESH,
  CONFIG_PM_GOVERNOR_STABILITY_STANDBY_THRESH,
  CONFIG_PM_GOVERNOR_STABILITY_SLEEP_THRESH,
};

static struct pm_stability_governor_s g_stability_governor;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Timer cb only to make sure system will wake from WFI */

static void stability_governor_timer_cb(wdparm_t arg)
{
}

/****************************************************************************
 * Name: stability_governor_statechanged
 ****************************************************************************/

static void stability_governor_statechanged(int domain,
                                            enum pm_state_e newstate)
{
  if (newstate == PM_RESTORE)
    {
      if (WDOG_ISACTIVE(&g_stability_governor.domain[domain].wdog))
        {
          sclock_t left;

          /* The left tick from wdog, if >0 should be other irq source */

          left = wd_gettime(&g_stability_governor.domain[domain].wdog);
          if (left <= 0)
            {
              g_stability_governor.domain[domain].wdog_wakeup = true;
            }

          /* Don't have to execute callback */

          wd_cancel(&g_stability_governor.domain[domain].wdog);
        }
    }
  else
    {
      enum pm_state_e state;
      clock_t thresh;

      state = g_stability_governor.domain[domain].state_pending;
      thresh = g_stability_governor_thresh[state];

      if (thresh > 0 && state != newstate)
        {
          wd_start(&g_stability_governor.domain[domain].wdog, thresh,
                   stability_governor_timer_cb, 0);
        }
    }
}

/****************************************************************************
 * Name: user_governor_checkstate
 ****************************************************************************/

static enum pm_state_e stability_governor_checkstate(int domain)
{
  FAR struct pm_stability_governor_domain_s *gdom;
  FAR struct pm_domain_s *pdom;
  enum pm_state_e state_pending;
  enum pm_state_e state;
  irqstate_t flags;
  bool wdog_wakeup;

  gdom = &g_stability_governor.domain[domain];
  pdom = &g_pmdomains[domain];
  state = PM_NORMAL;

  /* We disable interrupts since pm_stay()/pm_relax() could be simultaneously
   * invoked, which modifies the stay count which we are about to read
   */

  flags = spin_lock_irqsave(&pdom->lock);

  /* Find the lowest power-level which is not locked. */

  while (dq_empty(&pdom->wakelock[state]) && state < (PM_COUNT - 1))
    {
      state++;
    }

  state_pending = gdom->state_pending;
  wdog_wakeup = gdom->wdog_wakeup;
  gdom->state_pending = state;
  gdom->wdog_wakeup = false;

  /* If pm stability check disabled state or pm stable enough, do nothing */

  if (g_stability_governor_thresh[state] > 0 &&
     (!wdog_wakeup || state_pending != state))
    {
      state = pdom->state;
      if (g_stability_governor_thresh[state] > 0)
        {
          /* The domain last state can not be backward, need to holding
           * to the lowest power-level with stability check disabled
           */

          for (; state > PM_NORMAL; state--)
            {
              if (g_stability_governor_thresh[state] == 0)
                {
                  break;
                }
            }
        }
    }

  spin_unlock_irqrestore(&pdom->lock, flags);

  /* Return the found state */

  return state;
}

/****************************************************************************
 * Name: greedy_activity
 ****************************************************************************/

static void stability_governor_activity(int domain, int count)
{
  pm_staytimeout(domain, PM_NORMAL, (count ? count : 1) * 1000);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_stability_governor_initialize
 *
 * Description:
 *   Register the user_governor driver as the specified device.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

FAR const struct pm_governor_s *pm_stability_governor_initialize(void)
{
  return &g_stability_governor_ops;
}
