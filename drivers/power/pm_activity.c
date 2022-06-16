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

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/power/pm.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>

#include "pm.h"

#ifdef CONFIG_PM

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pm_wakelock_s g_wakelock[CONFIG_PM_NDOMAINS][PM_COUNT];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void pm_waklock_cb(wdparm_t arg)
{
  pm_wakelock_relax((FAR struct pm_wakelock_s *)arg);
}

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
  DEBUGASSERT(state < PM_COUNT);
  DEBUGASSERT(domain >= 0 && domain < CONFIG_PM_NDOMAINS);

  pm_wakelock_stay(&g_wakelock[domain][state]);
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
  DEBUGASSERT(state < PM_COUNT);
  DEBUGASSERT(domain >= 0 && domain < CONFIG_PM_NDOMAINS);

  pm_wakelock_relax(&g_wakelock[domain][state]);
}

/****************************************************************************
 * Name: pm_staytimeout
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   performing meaningful activities (non-idle), needs the power at kept
 *   last the specified level.
 *   And this will be timeout after time (ms), menas auto pm_relax
 *
 * Input Parameters:
 *   domain - The domain of the PM activity
 *   state - The state want to stay.
 *   ms - The timeout value ms
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

void pm_staytimeout(int domain, enum pm_state_e state, int ms)
{
  DEBUGASSERT(state < PM_COUNT);
  DEBUGASSERT(domain >= 0 && domain < CONFIG_PM_NDOMAINS);

  pm_wakelock_staytimeout(&g_wakelock[domain][state], ms);
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
  DEBUGASSERT(state < PM_COUNT);
  DEBUGASSERT(domain >= 0 && domain < CONFIG_PM_NDOMAINS);

  return pm_wakelock_staycount(&g_wakelock[domain][state]);
}

/****************************************************************************
 * Name: pm_wakelock_init
 *
 * Description:
 *   This function is used to init struct pm_wakelock_s
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pm_wakelock_init(FAR struct pm_wakelock_s *wakelock,
                      FAR const char *name, int domain,
                      enum pm_state_e state)
{
  DEBUGASSERT(wakelock);
  DEBUGASSERT(domain >= 0 && domain < CONFIG_PM_NDOMAINS);
  DEBUGASSERT(state < PM_COUNT);

  strlcpy(wakelock->name, name, sizeof(wakelock->name));

  wakelock->domain = domain;
  wakelock->state  = state;
  wakelock->count  = 0;
}

/****************************************************************************
 * Name: pm_wakelock_uninit
 *
 * Description:
 *   Uninit wakelock ID
 *
 * Input Parameters:
 *   wakelock - wakelock ID
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pm_wakelock_uninit(FAR struct pm_wakelock_s *wakelock)
{
  FAR struct pm_domain_s *pdom;
  FAR struct dq_queue_s *dq;
  FAR struct wdog_s *wdog;
  irqstate_t flags;
  int domain;

  DEBUGASSERT(wakelock);

  /* Get a convenience pointer to minimize all of the indexing */

  domain = wakelock->domain;
  pdom   = &g_pmglobals.domain[domain];
  dq     = &pdom->wakelock[wakelock->state];
  wdog   = &pdom->wdog[wakelock->state];

  flags = pm_lock(domain);

  if (wakelock->count > 0)
    {
      dq_rem(&wakelock->node, dq);
    }

  wakelock->count = 0;
  wd_cancel(wdog);

  pm_unlock(domain, flags);
}

/****************************************************************************
 * Name: pm_wakelock_stay
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   performing meaningful activities (non-idle), needs the power kept at
 *   least the specified level.
 *
 * Input Parameters:
 *   wakelock - wakelock ID
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

void pm_wakelock_stay(FAR struct pm_wakelock_s *wakelock)
{
  FAR struct pm_domain_s *pdom;
  FAR struct dq_queue_s *dq;
  irqstate_t flags;
  int domain;

  DEBUGASSERT(wakelock);

  /* Get a convenience pointer to minimize all of the indexing */

  domain = wakelock->domain;
  pdom   = &g_pmglobals.domain[domain];
  dq     = &pdom->wakelock[wakelock->state];

  flags = pm_lock(domain);

  DEBUGASSERT(wakelock->count < UINT32_MAX);
  if (wakelock->count++ == 0)
    {
      dq_addfirst(&wakelock->node, dq);
    }

  pm_unlock(domain, flags);

  pm_auto_updatestate(domain);
}

/****************************************************************************
 * Name: pm_wakelock_relax
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   idle now, could relax the previous requested power level.
 *
 * Input Parameters:
 *   wakelock - wakelock ID
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

void pm_wakelock_relax(FAR struct pm_wakelock_s *wakelock)
{
  FAR struct pm_domain_s *pdom;
  FAR struct dq_queue_s *dq;
  irqstate_t flags;
  int domain;

  DEBUGASSERT(wakelock);

  /* Get a convenience pointer to minimize all of the indexing */

  domain = wakelock->domain;
  pdom   = &g_pmglobals.domain[domain];
  dq     = &pdom->wakelock[wakelock->state];

  flags = pm_lock(domain);

  DEBUGASSERT(wakelock->count > 0);
  if (--wakelock->count == 0)
    {
      dq_rem(&wakelock->node, dq);
    }

  pm_unlock(domain, flags);

  pm_auto_updatestate(domain);
}

/****************************************************************************
 * Name: pm_wakelock_staytimeout
 *
 * Description:
 *   This function is called by a device driver to indicate that it is
 *   performing meaningful activities (non-idle), needs the power at kept
 *   last the specified level.
 *   And this will be timeout after time (ms), menas auto pm_wakelock_relax
 *
 * Input Parameters:
 *   wakelock - wakelock ID
 *   ms       - The timeout value ms
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

void pm_wakelock_staytimeout(FAR struct pm_wakelock_s *wakelock, int ms)
{
  FAR struct pm_domain_s *pdom;
  FAR struct dq_queue_s *dq;
  FAR struct wdog_s *wdog;
  irqstate_t flags;
  int domain;

  DEBUGASSERT(wakelock);

  /* Get a convenience pointer to minimize all of the indexing */

  domain = wakelock->domain;
  pdom   = &g_pmglobals.domain[domain];
  dq     = &pdom->wakelock[wakelock->state];
  wdog   = &pdom->wdog[wakelock->state];

  flags = pm_lock(domain);

  if (!WDOG_ISACTIVE(wdog))
    {
      DEBUGASSERT(wakelock->count < UINT32_MAX);
      if (wakelock->count++ == 0)
        {
          dq_addfirst(&wakelock->node, dq);
        }
    }

  if (TICK2MSEC(wd_gettime(wdog)) < ms)
    {
      wd_start(wdog, MSEC2TICK(ms), pm_waklock_cb, (wdparm_t)wakelock);
    }

  pm_unlock(domain, flags);

  pm_auto_updatestate(domain);
}

/****************************************************************************
 * Name: pm_wakelock_staycount
 *
 * Description:
 *   This function is called to get current stay count in this wakelock ID
 *
 * Input Parameters:
 *   wakelock - wakelock ID
 *
 * Returned Value:
 *   Current pm stay count in this wakelock ID
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

int pm_wakelock_staycount(FAR struct pm_wakelock_s *wakelock)
{
  DEBUGASSERT(wakelock);

  return wakelock->count;
}

/****************************************************************************
 * Name: pm_wakelock_global_init
 *
 * Description:
 *   This function is called to setup global wakelock when system init
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void pm_wakelock_global_init(void)
{
  int i;
  int j;

  for (i = 0; i < CONFIG_PM_NDOMAINS; i++)
    {
      for (j = 0; j < PM_COUNT; j++)
        {
          pm_wakelock_init(&g_wakelock[i][j], "system", i, j);
        }
    }
}

#endif /* CONFIG_PM */
