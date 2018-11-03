/****************************************************************************
 * net/mld/mld_timer.c
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
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of CITEL Technologies Ltd nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY CITEL TECHNOLOGIES AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL CITEL TECHNOLOGIES OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <assert.h>
#include <debug.h>

#include <nuttx/wdog.h>
#include <nuttx/irq.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/mld.h>

#include "devif/devif.h"
#include "mld/mld.h"
#include "utils/utils.h"

#ifdef CONFIG_NET_MLD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#undef MLD_MTMRDEBUG /* Define to enable detailed MLD group debug */

#ifndef CONFIG_NET_MLD
#  undef MLD_MTMRDEBUG
#endif

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef MLD_MTMRDEBUG
#    define mtmrerr(format, ...)    nerr(format, ##__VA_ARGS__)
#    define mtmrinfo(format, ...)   ninfo(format, ##__VA_ARGS__)
#  else
#    define mtmrerr(x...)
#    define mtmrinfo(x...)
#  endif
#else
#  ifdef MLD_MTMRDEBUG
#    define mtmrerr    nerr
#    define mtmrinfo   ninfo
#  else
#    define mtmrerr    (void)
#    define mtmrinfo   (void)
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mld_timeout_work
 *
 * Description:
 *   Timeout watchdog work
 *
 * Assumptions:
 *   This function is called from the wdog timer handler which runs in the
 *   context of the timer interrupt handler.
 *
 ****************************************************************************/

static void mld_timeout_work(FAR void *arg)
{
  FAR struct mld_group_s *group;
  int ret;

  /* Recover the reference to the group */

  group = (FAR struct mld_group_s *)arg;
  DEBUGASSERT(group != NULL);

  /* Check if this a new join to the multicast group. */

  net_lock();
  if (IS_MLD_STARTUP(group->flags))
    {
      /* Schedule (and forget) the Report. */

      MLD_STATINCR(g_netstats.mld.report_sched);
      ret = mld_schedmsg(group, MLD_SEND_REPORT);
      if (ret < 0)
        {
          nerr("ERROR: Failed to schedule message: %d\n", ret);
        }

      /* Send the report until the unsolicited report count goes to zero
       * then terminate the start-up sequence.
       */

      if (group->count > 1)
        {
          /* Decrement the count and restart the timer */

          group->count--;
          mld_starttimer(group, MSEC2TICK(MLD_UNSOLREPORT_MSEC));
        }
      else
        {
          /* Terminate the start-up sequence */

          CLR_MLD_STARTUP(group->flags);

          /* If in Querier mode, start the Querier timer */

         if (IS_MLD_QUERIER(group->flags))
           {
             mld_starttimer(group, MSEC2TICK(MLD_QUERY_MSEC));
           }
        }
    }

  /* Check if this is querier */

  else if (IS_MLD_QUERIER(group->flags))
    {
      /* Schedule (and forget) the general query. */

      MLD_STATINCR(g_netstats.mld.query_sched);
      ret = mld_schedmsg(group, MLD_SEND_GENQUERY);
      if (ret < 0)
        {
          nerr("ERROR: Failed to schedule message: %d\n", ret);
        }

      /* Restart the Querier timer */

      mld_starttimer(group, MSEC2TICK(MLD_QUERY_MSEC));
    }

  net_unlock();
}

/****************************************************************************
 * Name:  mld_timeout
 *
 * Description:
 *   Timeout watchdog handler.
 *
 * Assumptions:
 *   This function is called from the wdog timer handler which runs in the
 *   context of the timer interrupt handler.
 *
 ****************************************************************************/

static void mld_timeout(int argc, uint32_t arg, ...)
{
  FAR struct mld_group_s *group;
  int ret;

  ninfo("Timeout!\n");

  /* Recover the reference to the group */

  group = (FAR struct mld_group_s *)arg;
  DEBUGASSERT(argc == 1 && group != NULL);

  /* Perform the timeout-related operations on (preferably) the low prioirity
   * work queue.
   */

  ret = work_queue(LPWORK, &group->work, mld_timeout_work, group, 0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to queue timeout work: %d\n", ret);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mld_starttimer
 *
 * Description:
 *   Start the MLD timer.
 *
 * Assumptions:
 *   This function may be called from most any context.
 *
 ****************************************************************************/

void mld_starttimer(FAR struct mld_group_s *group, clock_t ticks)
{
  int ret;

  /* Start the timer */

  mtmrinfo("ticks: %ld\n", (unsigned long)ticks);

  ret = wd_start(group->wdog, ticks, mld_timeout, 1, (uint32_t)group);

  DEBUGASSERT(ret == OK);
  UNUSED(ret);
}

/****************************************************************************
 * Name:  mld_cmptimer
 *
 * Description:
 *   Compare the timer remaining on the watching timer to the deci-second
 *   value. If maxticks > ticks-remaining, then (1) cancel the timer (to
 *   avoid race conditions) and return true.
 *
 * Assumptions:
 *   This function may be called from most any context.  If true is returned
 *   then the caller must call mld_starttimer() to restart the timer
 *
 ****************************************************************************/

bool mld_cmptimer(FAR struct mld_group_s *group, int maxticks)
{
  irqstate_t flags;
  int remaining;

  /* Disable interrupts so that there is no race condition with the actual
   * timer expiration.
   */

  flags = enter_critical_section();

  /* Get the timer remaining on the watchdog.  A time of <= zero means that
   * the watchdog was never started.
   */

  remaining = wd_gettime(group->wdog);

  /* A remaining time of zero means that the watchdog was never started
   * or has already expired.  That case should be covered in the following
   * test as well.
   */

  mtmrinfo("maxticks: %d remaining: %d\n", maxticks, remaining);
  if (maxticks > remaining)
    {
      /* Cancel the watchdog timer and return true */

      wd_cancel(group->wdog);
      leave_critical_section(flags);
      return true;
    }

  leave_critical_section(flags);
  return false;
}

#endif /* CONFIG_NET_MLD */
