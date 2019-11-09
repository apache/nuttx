/****************************************************************************
 * greedy_governor/greedy_governor.c
 *
 *   Copyright (C) 2019 Matias Nitsche. All rights reserved.
 *   Author: Matias Nitsche <mnitsche@dc.uba.ar>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#include "greedy_governor.h"
#include "pm.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* PM governor methods */

static void             greedy_governor_initialize(void);
static void             greedy_governor_statechanged(int domain,
                                                     enum pm_state_e newstate);
static enum pm_state_e  greedy_governor_checkstate(int domain);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct pm_governor_s g_greedy_governor_ops =
{
  .initialize   = greedy_governor_initialize,   /* initialize */
  .statechanged = greedy_governor_statechanged, /* statechanged */
  .checkstate   = greedy_governor_checkstate,   /* checkstate */
  .activity     = NULL,                         /* activity */
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

static void greedy_governor_statechanged(int domain, enum pm_state_e newstate)
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

  flags = enter_critical_section();

  /* Find the lowest power-level which is not locked. */

  while (!pdom->stay[state] && state < (PM_COUNT - 1))
    {
      state++;
    }

  leave_critical_section(flags);

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
