/****************************************************************************
 * sched/sched/sched_critmonitor.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <sched.h>

#include "sched/sched.h"

#ifdef CONFIG_SCHED_CRITMONITOR

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Start time when pre-emption disabled or critical section entered. */

#ifdef CONFIG_SMP_NCPUS
static uint32_t g_premp_start[CONFIG_SMP_NCPUS];
static uint32_t g_crit_start[CONFIG_SMP_NCPUS];
#else
static uint32_t g_premp_start[1];
static uint32_t g_crit_start[1];
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Maximum time with pre-emption disabled or within critical section. */

#ifdef CONFIG_SMP_NCPUS
uint32_t g_premp_max[CONFIG_SMP_NCPUS];
uint32_t g_crit_max[CONFIG_SMP_NCPUS];
#else
uint32_t g_premp_max[1];
uint32_t g_crit_max[1];
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_critmon_preemption
 *
 * Description:
 *   Called when there is any change in pre-emptible state of a thread.
 *
 * Assumptions:
 *   - Called within a critical section.
 *   - Never called from an interrupt handler
 *
 ****************************************************************************/

void nxsched_critmon_preemption(FAR struct tcb_s *tcb, bool state)
{
  int cpu = this_cpu();

  /* Are we enabling or disabling pre-emption */

  if (state)
    {
      DEBUGASSERT(tcb->premp_start == 0);

      /* Disabling.. Save the thread start time */

      tcb->premp_start = up_critmon_gettime();

      /* Zero means that the timer is not ready */

      if (tcb->premp_start != 0 && g_premp_start[cpu] == 0)
        {
          /* Save the global start time */

          g_premp_start[cpu] = tcb->premp_start;
        }
    }
  else if (tcb->premp_start != 0)
    {
      /* Re-enabling.. Check for the max elapsed time */

      uint32_t now     = up_critmon_gettime();
      uint32_t elapsed = now - tcb->premp_start;

      DEBUGASSERT(now != 0);

      tcb->premp_start = 0;
      if (elapsed > tcb->premp_max)
        {
          tcb->premp_max = elapsed;
        }

      /* Check for the global max elapsed time */

      if (g_premp_start[cpu] != 0)
        {
          elapsed            = now - g_premp_start[cpu];
          g_premp_start[cpu] = 0;

          if (elapsed > g_premp_max[cpu])
            {
              g_premp_max[cpu] = elapsed;
            }
        }
    }
}

/****************************************************************************
 * Name: nxsched_critmon_csection
 *
 * Description:
 *   Called when a thread enters or leaves a critical section.
 *
 * Assumptions:
 *   - Called within a critical section.
 *   - Never called from an interrupt handler
 *
 ****************************************************************************/

void nxsched_critmon_csection(FAR struct tcb_s *tcb, bool state)
{
  int cpu = this_cpu();

  /* Are we entering or leaving the critical section? */

  if (state)
    {
      /* Entering... Save the start time. */

      DEBUGASSERT(tcb->crit_start == 0);
      tcb->crit_start = up_critmon_gettime();

      /* Zero means that the timer is not ready */

      if (tcb->crit_start != 0 && g_crit_start[cpu] == 0)
        {
          /* Set the global start time */

          g_crit_start[cpu] = tcb->crit_start;
        }
    }
  else if (tcb->crit_start != 0)
    {
      /* Leaving .. Check for the max elapsed time */

      uint32_t now     = up_critmon_gettime();
      uint32_t elapsed = now - tcb->crit_start;

      DEBUGASSERT(now != 0);

      tcb->crit_start = 0;
      if (elapsed > tcb->crit_max)
        {
          tcb->crit_max = elapsed;
        }

      /* Check for the global max elapsed time */

      if (g_crit_start[cpu] != 0)
        {
          elapsed           = now - g_crit_start[cpu];
          g_crit_start[cpu] = 0;

          if (elapsed > g_crit_max[cpu])
            {
              g_crit_max[cpu] = elapsed;
            }
        }
    }
}

/****************************************************************************
 * Name: nxsched_resume_critmon
 *
 * Description:
 *   Called when a thread resumes execution, perhaps re-establishing a
 *   critical section or a non-pre-emptible state.
 *
 * Assumptions:
 *   - Called within a critical section.
 *   - Might be called from an interrupt handler
 *
 ****************************************************************************/

void nxsched_resume_critmon(FAR struct tcb_s *tcb)
{
  uint32_t elapsed;
  int cpu = this_cpu();

  DEBUGASSERT(tcb->premp_start == 0 && tcb->crit_start == 0);

  /* Did this task disable pre-emption? */

  if (tcb->lockcount > 0)
    {
      /* Yes.. Save the start time */

      tcb->premp_start = up_critmon_gettime();
      DEBUGASSERT(tcb->premp_start != 0);

      /* Zero means that the timer is not ready */

      if (g_premp_start[cpu] == 0)
        {
          g_premp_start[cpu] = tcb->premp_start;
        }
    }
  else if (g_premp_start[cpu] != 0)
    {
      /* Check for the global max elapsed time */

      elapsed            = up_critmon_gettime() - g_premp_start[cpu];
      g_premp_start[cpu] = 0;

      if (elapsed > g_premp_max[cpu])
        {
          g_premp_max[cpu] = elapsed;
        }
    }

  /* Was this task in a critical section? */

  if (tcb->irqcount > 0)
    {
      /* Yes.. Save the start time */

      tcb->crit_start = up_critmon_gettime();
      DEBUGASSERT(tcb->crit_start != 0);

      if (g_crit_start[cpu] == 0)
        {
          g_crit_start[cpu] = tcb->crit_start;
        }
    }
  else if (g_crit_start[cpu] != 0)
    {
      /* Check for the global max elapsed time */

      elapsed      = up_critmon_gettime() - g_crit_start[cpu];
      g_crit_start[cpu] = 0;

      if (elapsed > g_crit_max[cpu])
        {
          g_crit_max[cpu] = elapsed;
        }
    }
}

/****************************************************************************
 * Name: nxsched_suspend_critmon
 *
 * Description:
 *   Called when a thread suspends execution, perhaps terminating a
 *   critical section or a non-preemptible state.
 *
 * Assumptions:
 *   - Called within a critical section.
 *   - Might be called from an interrupt handler
 *
 ****************************************************************************/

void nxsched_suspend_critmon(FAR struct tcb_s *tcb)
{
  uint32_t elapsed;

  /* Did this task disable preemption? */

  if (tcb->lockcount > 0)
    {
      /* Possibly re-enabling.. Check for the max elapsed time */

      elapsed = up_critmon_gettime() - tcb->premp_start;

      tcb->premp_start = 0;
      if (elapsed > tcb->premp_max)
        {
          tcb->premp_max = elapsed;
        }
    }

  /* Is this task in a critical section? */

  if (tcb->irqcount > 0)
    {
      /* Possibly leaving .. Check for the max elapsed time */

      elapsed = up_critmon_gettime() - tcb->crit_start;

      tcb->crit_start = 0;
      if (elapsed > tcb->crit_max)
        {
          tcb->crit_max = elapsed;
        }
    }
}

#endif
