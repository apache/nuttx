/****************************************************************************
 * drivers/power/greedy_governor.c
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

#include <sys/types.h>
#include <stdbool.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/fs.h>
#include <nuttx/power/pm.h>
#include <nuttx/power/power_ioctl.h>

#include <nuttx/irq.h>

#include "pm.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* PM governor methods */

static void greedy_governor_initialize(void);
static void greedy_governor_statechanged(int domain,
                                         enum pm_state_e newstate);
static enum pm_state_e greedy_governor_checkstate(int domain);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct pm_governor_s g_greedy_governor_ops =
{
  greedy_governor_initialize,   /* initialize */
  NULL,                         /* deinitialize */
  greedy_governor_statechanged, /* statechanged */
  greedy_governor_checkstate,   /* checkstate */
  NULL,                         /* activity */
  NULL                          /* priv */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: greedy_governor_initialize
 ****************************************************************************/

static void greedy_governor_initialize(void)
{
#ifdef CONFIG_PM_GOVERNOR_EXPLICIT_RELAX
  for (int dom = 0; dom < CONFIG_PM_NDOMAINS; dom++)
    {
      for (int state = 0; state < PM_COUNT; state++)
        {
          pm_stay(dom, state);
        }
    }
#endif
}

/****************************************************************************
 * Name: greedy_governor_statechanged
 ****************************************************************************/

static void greedy_governor_statechanged(int domain,
                                         enum pm_state_e newstate)
{
  /* no need to react to state changes */

  UNUSED(domain);
  UNUSED(newstate);
}

/****************************************************************************
 * Name: user_governor_checkstate
 ****************************************************************************/

static enum pm_state_e greedy_governor_checkstate(int domain)
{
  FAR struct pm_domain_s *pdom;
  int state;
  irqstate_t flags;

  pdom = &g_pmglobals.domain[domain];
  state = PM_NORMAL;

  /* We disable interrupts since pm_stay()/pm_relax() could be simultaneously
   * invoked, which modifies the stay count which we are about to read
   */

  flags = pm_lock(domain);

  /* Find the lowest power-level which is not locked. */

  while (!pdom->stay[state] && state < (PM_COUNT - 1))
    {
      state++;
    }

  pm_unlock(domain, flags);

  /* Return the found state */

  return state;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pm_greedy_governor_initialize
 *
 * Description:
 *   Register the user_governor driver as the specified device.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

FAR const struct pm_governor_s *pm_greedy_governor_initialize(void)
{
  return &g_greedy_governor_ops;
}
