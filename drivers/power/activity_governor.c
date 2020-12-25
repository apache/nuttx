/****************************************************************************
 * activity_governor.c
 *
 *   Copyright (C) 2011-2012, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <stdint.h>
#include <assert.h>
#include <sys/types.h>
#include <stdlib.h>

#include <nuttx/power/pm.h>
#include <nuttx/wqueue.h>
#include <nuttx/irq.h>

#include "pm.h"

#ifdef CONFIG_PM_GOVERNOR_ACTIVITY

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PM_TIMER_GAP        (TIME_SLICE_TICKS * 2)

/* Convert the time slice interval into system clock ticks.
 *
 * CONFIG_PM_SLICEMS provides the duration of one time slice in milliseconds.
 * CLOCKS_PER_SEC provides the number of timer ticks in one second.
 *
 * slice ticks = (CONFIG_PM_SLICEMS msec / 1000 msec/sec) /
 *               (CLOCKS_PER_SEC ticks/sec)
 */

#define TIME_SLICE_TICKS ((CONFIG_PM_GOVERNOR_SLICEMS * CLOCKS_PER_SEC) /  1000)

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

struct pm_domain_state_s
{
  /* recommended - The recommended state based on the governor policy
   * mndex       - The index to the next slot in the memory[] array to use.
   * mcnt        - A tiny counter used only at start up. The actual algorithm
   *               cannot be applied until CONFIG_PM_GOVERNOR_MEMORY
   *               samples have been collected.
   */

  uint8_t recommended;
  uint8_t mndx;
  uint8_t mcnt;

  /* accum - The accumulated counts in this time interval */

  int16_t accum;

#if CONFIG_PM_GOVERNOR_MEMORY > 1
  /* This is the averaging "memory."  The averaging algorithm is simply:
   * Y = (An*X + SUM(Ai*Yi))/SUM(Aj), where i = 1..n-1 and j= 1..n, n is the
   * length of the "memory", Ai is the weight applied to each value, and X is
   * the current activity.
   *
   * CONFIG_PM_GOVERNOR_MEMORY provides the memory for the algorithm.
   *   Default: 2
   * CONFIG_PM_COEFn provides weight for each sample.  Default: 1
   */

  int16_t memory[CONFIG_PM_GOVERNOR_MEMORY - 1];
#endif

  /* stime - The time (in ticks) at the start of the current time slice */

  clock_t stime;

  /* btime - The time (in ticks) at the start of the current state */

  clock_t btime;

  /* Timer to decrease state */

  struct wdog_s wdog;
};

struct pm_activity_governor_s
{
  /* Threshold time slice count to enter the next low power consdumption
   * state. Indexing is next state 0:IDLE, 1: STANDBY, 2: SLEEP.
   */

  const uint32_t pmcount[3];

  /* Threshold activity values to enter into the next lower power consumption
   * state. Indexing is next state 0:IDLE, 1:STANDBY, 2:SLEEP.
   */

  const int32_t pmenterthresh[3];

  /* Threshold activity values to leave the current low power consdumption
   * state. Indexing is current state 0:IDLE, 1: STANDBY, 2: SLEEP.
   */

  const int32_t pmexitthresh[3];

  /* CONFIG_PM_GOVERNOR_MEMORY is the total number of time slices (including
   * the current time slice).  The history of previous values is then
   * CONFIG_PM_GOVERNOR_MEMORY-1.
   */

#if CONFIG_PM_GOVERNOR_MEMORY > 1
  const int16_t pmcoeffs[CONFIG_PM_GOVERNOR_MEMORY - 1];
#endif

  struct pm_domain_state_s domain_states[CONFIG_PM_NDOMAINS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void governor_initialize(void);
static void governor_statechanged(int domain, enum pm_state_e newstate);
static enum pm_state_e governor_checkstate(int domain);
static void governor_activity(int domain, int count);
static void governor_timer(int domain);
static void governor_update(int domain, int16_t accum);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct pm_activity_governor_s g_pm_activity_governor =
{
  .pmcount =
  {
    CONFIG_PM_GOVERNOR_IDLEENTER_COUNT,
    CONFIG_PM_GOVERNOR_STANDBYENTER_COUNT,
    CONFIG_PM_GOVERNOR_SLEEPENTER_COUNT
  },
  .pmenterthresh =
  {
    CONFIG_PM_GOVERNOR_IDLEENTER_THRESH,
    CONFIG_PM_GOVERNOR_STANDBYENTER_THRESH,
    CONFIG_PM_GOVERNOR_SLEEPENTER_THRESH
  },
  .pmexitthresh =
  {
    CONFIG_PM_GOVERNOR_IDLEEXIT_THRESH,
    CONFIG_PM_GOVERNOR_STANDBYEXIT_THRESH,
    CONFIG_PM_GOVERNOR_SLEEPEXIT_THRESH
  },

#if CONFIG_PM_GOVERNOR_MEMORY > 1
  .pmcoeffs =
  {
    CONFIG_PM_GOVERNOR_COEF1
#if CONFIG_PM_GOVERNOR_MEMORY > 2
    , CONFIG_PM_GOVERNOR_COEF2
#endif
#if CONFIG_PM_GOVERNOR_MEMORY > 3
    , CONFIG_PM_GOVERNOR_COEF3
#endif
#if CONFIG_PM_GOVERNOR_MEMORY > 4
    , CONFIG_PM_GOVERNOR_COEF4
#endif
#if CONFIG_PM_GOVERNOR_MEMORY > 5
    , CONFIG_PM_GOVERNOR_COEF5
#endif
#if CONFIG_PM_GOVERNOR_MEMORY > 6
#  warning "This logic needs to be extended"
#endif
  }
#endif
};

struct pm_governor_s g_pmgovernor =
{
  .initialize   = governor_initialize,
  .checkstate   = governor_checkstate,
  .statechanged = governor_statechanged,
  .activity     = governor_activity
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void governor_initialize(void)
{
  FAR struct pm_domain_state_s *pdomstate;
  int i;

  for (i = 0; i < CONFIG_PM_NDOMAINS; i++)
    {
      pdomstate        = &g_pm_activity_governor.domain_states[i];
      pdomstate->stime = clock_systime_ticks();
      pdomstate->btime = clock_systime_ticks();
    }
}

static void governor_activity(int domain, int count)
{
  FAR struct pm_domain_state_s *pdomstate;
  clock_t now, elapsed;
  uint32_t accum;
  irqstate_t flags;

  /* Get a convenience pointer to minimize all of the indexing */

  DEBUGASSERT(domain >= 0 && domain < CONFIG_PM_NDOMAINS);
  pdomstate = &g_pm_activity_governor.domain_states[domain];

  /* Just increment the activity count in the current time slice. The
   * priority is simply the number of counts that are added.
   */

  if (count > 0)
    {
      /* Add the activity count to the accumulated counts. */

      flags = enter_critical_section();
      accum = (uint32_t)pdomstate->accum + count;

      /* Make sure that we do not overflow the underlying representation */

      if (accum > INT16_MAX)
        {
          accum = INT16_MAX;
        }

      /* Save the updated count */

      pdomstate->accum = (int16_t)accum;

      /* Check the elapsed time.  In periods of low activity, time slicing is
       * controlled by IDLE loop polling; in periods of higher activity, time
       * slicing is controlled by driver activity.  In either case, the
       * duration of the time slice is only approximate; during times of
       * heavy activity, time slices may be become longer and the activity
       * level may be over-estimated.
       */

      now     = clock_systime_ticks();
      elapsed = now - pdomstate->stime;
      if (elapsed >= TIME_SLICE_TICKS)
        {
          int16_t tmp;

          /* Sample the count, reset the time and count, and assess the PM
           * state.  This is an atomic operation because interrupts are
           * still disabled.
           */

          tmp              = pdomstate->accum;
          pdomstate->stime = now;
          pdomstate->accum = 0;

          governor_update(domain, tmp);
        }

      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: governor_update
 *
 * Description:
 *   This internal function is called at the end of a time slice in order to
 *   update driver activity metrics and recommended states.
 *
 * Input Parameters:
 *   domain - The PM domain associated with the accumulator
 *   accum  - The value of the activity accumulator at the end of the time
 *            slice.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function may be called from a driver, perhaps even at the interrupt
 *   level.  It may also be called from the IDLE loop at the lowest possible
 *   priority level.
 *
 ****************************************************************************/

static void governor_update(int domain, int16_t accum)
{
  FAR struct pm_domain_state_s *pdomstate;
  uint8_t state;
  int32_t y;
  int index;
#if CONFIG_PM_GOVERNOR_MEMORY > 1
  int32_t denom;
  int i = 0;
  int j;
#endif

  /* Get a convenience pointer to minimize all of the indexing */

  DEBUGASSERT(domain >= 0 && domain < CONFIG_PM_NDOMAINS);
  pdomstate = &g_pm_activity_governor.domain_states[domain];
  state     = g_pmglobals.domain[domain].state;

#if CONFIG_PM_GOVERNOR_MEMORY > 1
  /* We won't bother to do anything until we have accumulated
   * CONFIG_PM_GOVERNOR_MEMORY-1 samples.
   */

  if (pdomstate->mcnt < CONFIG_PM_GOVERNOR_MEMORY - 1)
    {
      index                    = pdomstate->mcnt++;
      pdomstate->memory[index] = accum;
      return;
    }

  /* The averaging algorithm is simply: Y = (An*X + SUM(Ai*Yi))/SUM(Aj),
   * where i = 1..n-1 and j= 1..n, n is the length of the "memory", Ai is
   * the weight applied to each value, and X is the current activity.
   *
   * CONFIG_PM_GOVERNOR_MEMORY:
   *   provides the memory for the algorithm. Default: 2
   * CONFIG_PM_GOVERNOR_COEFn:
   *   provides weight for each sample. Default: 1
   *
   * First, calculate Y = An*X
   */

  y     = CONFIG_PM_GOVERNOR_COEFN * accum;
  denom = CONFIG_PM_GOVERNOR_COEFN;

  /* Then calculate Y +=  SUM(Ai*Yi), i = 1..n-1. The oldest sample will
   * reside at the domain's mndx (and this is the value that we will
   * overwrite with the new value).
   */

  for (j = pdomstate->mndx; i < CONFIG_PM_GOVERNOR_MEMORY - 1; i++, j++)
    {
      if (j >= CONFIG_PM_GOVERNOR_MEMORY - 1)
        {
          j = 0;
        }

      y     += g_pm_activity_governor.pmcoeffs[i] * pdomstate->memory[j];
      denom += g_pm_activity_governor.pmcoeffs[i];
    }

  /* Compute and save the new activity value */

  y /= denom;

  index = pdomstate->mndx++;
  pdomstate->memory[index] = y;
  if (pdomstate->mndx >= CONFIG_PM_GOVERNOR_MEMORY - 1)
    {
      pdomstate->mndx = 0;
    }
#else

  /* No smoothing */

  y = accum;
#endif

  /* First check if increased activity should cause us to return to the
   * normal operating state.  This would be unlikely for the lowest power
   * consumption states because the CPU is probably asleep.  However this
   * probably does apply for the IDLE state.
   */

  if (state > PM_NORMAL)
    {
      /* Get the table index for the current state (which will be the
       * current state minus one)
       */

      index = state - 1;

      /* Has the threshold to return to normal power consumption state been
       * exceeded?
       */

      if (y > g_pm_activity_governor.pmexitthresh[index])
        {
          /* Yes... reset the count and recommend the normal state. */

          pdomstate->btime       = clock_systime_ticks();
          pdomstate->recommended = PM_NORMAL;
          return;
        }
    }

  /* Now, compare this new activity level to the thresholds and counts for
   * the next lower power consumption state. If we are already in the SLEEP
   * state, then there is nothing more to be done (in fact, I would be
   * surprised to be executing!).
   */

  if (state < PM_SLEEP)
    {
      unsigned int nextstate;

      /* Get the next state and the table index for the next state (which
       * will be the current state)
       */

      index     = state;
      nextstate = state + 1;

      /* Has the threshold to enter the next lower power consumption state
       * been exceeded?
       */

      if (y > g_pm_activity_governor.pmenterthresh[index])
        {
          /* No... reset the count and recommend the current state */

          pdomstate->btime       = clock_systime_ticks();
          pdomstate->recommended = state;
        }

      /* Yes.. have we already recommended this state? If so, do nothing */

      else if (pdomstate->recommended < nextstate)
        {
          /* No.. calculate the count.  Has it passed the count required
           * for a state transition?
           */

          if (clock_systime_ticks() - pdomstate->btime >=
                  g_pm_activity_governor.pmcount[index] * TIME_SLICE_TICKS)
            {
              /* Yes, recommend the new state and set up for the next
               * transition.
               */

              pdomstate->btime       = clock_systime_ticks();
              pdomstate->recommended = nextstate;
            }
        }
    }
}

static enum pm_state_e governor_checkstate(int domain)
{
  FAR struct pm_domain_state_s *pdomstate;
  FAR struct pm_domain_s *pdom;
  clock_t now, elapsed;
  irqstate_t flags;
  int index;

  /* Get a convenience pointer to minimize all of the indexing */

  pdomstate = &g_pm_activity_governor.domain_states[domain];
  pdom      = &g_pmglobals.domain[domain];

  /* Check for the end of the current time slice.  This must be performed
   * with interrupts disabled so that it does not conflict with the similar
   * logic in governor_activity().
   */

  flags = enter_critical_section();

  /* Check the elapsed time.  In periods of low activity, time slicing is
   * controlled by IDLE loop polling; in periods of higher activity, time
   * slicing is controlled by driver activity.  In either case, the duration
   * of the time slice is only approximate; during times of heavy activity,
   * time slices may be become longer and the activity level may be over-
   * estimated.
   */

  now     = clock_systime_ticks();
  elapsed = now - pdomstate->stime;
  if (elapsed >= TIME_SLICE_TICKS)
    {
      int16_t accum;

      /* Sample the count, reset the time and count, and assess the PM
       * state.  This is an atomic operation because interrupts are
       * still disabled.
       */

      accum       = pdomstate->accum;
      pdomstate->stime = now;
      pdomstate->accum = 0;

      governor_update(domain, accum);
    }

  /* Consider the possible power state lock here */

  for (index = 0; index < pdomstate->recommended; index++)
    {
      if (pdom->stay[index] != 0)
        {
          pdomstate->recommended = index;
          break;
        }
    }

  leave_critical_section(flags);

  return pdomstate->recommended;
}

static void governor_statechanged(int domain, enum pm_state_e newstate)
{
  if (newstate != PM_RESTORE)
    {
      /* Start PM timer to decrease PM state */

      governor_timer(domain);
    }
}

static void governor_timer_cb(wdparm_t arg)
{
  /* Do nothing here, cause we only need TIMER ISR to wake up PM,
   * for deceasing PM state.
   */

  UNUSED(arg);
}

/****************************************************************************
 * Name: governor_timer
 *
 * Description:
 *   This internal function is called to start one timer to decrease power
 *   state level.
 *
 * Input Parameters:
 *   domain - The PM domain associated with the accumulator
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void governor_timer(int domain)
{
  FAR struct pm_domain_state_s *pdomstate;
  FAR struct pm_domain_s *pdom;
  uint8_t state;

  static const int pmtick[3] =
  {
    TIME_SLICE_TICKS * CONFIG_PM_GOVERNOR_IDLEENTER_COUNT,
    TIME_SLICE_TICKS * CONFIG_PM_GOVERNOR_STANDBYENTER_COUNT,
    TIME_SLICE_TICKS * CONFIG_PM_GOVERNOR_SLEEPENTER_COUNT
  };

  pdom      = &g_pmglobals.domain[domain];
  pdomstate = &g_pm_activity_governor.domain_states[domain];
  state     = pdom->state;

  if (state < PM_SLEEP && !pdom->stay[pdom->state])
    {
      int delay = pmtick[state] + pdomstate->btime - clock_systime_ticks();
      int left  = wd_gettime(&pdomstate->wdog);

      if (delay <= 0)
        {
          delay = 1;
        }

      if (!WDOG_ISACTIVE(&pdomstate->wdog) ||
          abs(delay - left) > PM_TIMER_GAP)
        {
          wd_start(&pdomstate->wdog, delay, governor_timer_cb, 0);
        }
    }
  else
    {
      wd_cancel(&pdomstate->wdog);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct pm_governor_s *pm_activity_governor_initialize(void)
{
  return &g_pmgovernor;
}

#endif /* CONFIG_PM_GOVERNOR_ACTIVITY */
