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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mld_report_msgtype
 *
 * Description:
 *   Determine which type of Report to send, MLDv1 or MLDv2, depending on
 *   current state of compatibility mode flag.
 *
 ****************************************************************************/

static inline uint8_t mld_report_msgtype(FAR struct mld_group_s *group)
{
  if (IS_MLD_V1COMPAT(group->flags))
    {
      return MLD_SEND_V1REPORT;
    }
  else
    {
      return MLD_SEND_V2REPORT;
    }
}

/****************************************************************************
 * Name:  mld_polldog_work
 *
 * Description:
 *   Timeout watchdog work
 *
 * Assumptions:
 *   This function is called from a work queue thread.
 *
 ****************************************************************************/

static void mld_polldog_work(FAR void *arg)
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
      ret = mld_schedmsg(group, mld_report_msgtype(group));
      if (ret < 0)
        {
          mlderr("ERROR: Failed to schedule message: %d\n", ret);
        }

      /* Send the report until the unsolicited report count goes to zero
       * then terminate the start-up sequence.
       */

      if (group->count > 0)
        {
          /* Decrement the count and restart the timer */

          group->count--;
          mld_start_polltimer(group, MSEC2TICK(MLD_UNSOLREPORT_MSEC));
        }
      else
        {
          /* Terminate the start-up sequence */

          CLR_MLD_STARTUP(group->flags);

          /* If in Querier mode, start the Querier timer */

         if (IS_MLD_QUERIER(group->flags))
           {
             mld_start_polltimer(group, MSEC2TICK(MLD_QUERY_MSEC));
           }
        }

      net_unlock();
      return;
    }

#ifdef CONFIG_NET_MLD_ROUTER
  /* This is a Query-related timeout.  Destroy the group if there are
   * no members of the group detected in the last two Query cycles.
   */

  if (group->njoins == 0 group->members == 0 && group->lstmbrs == 0)
    {
      /* Cancel the timers and discard any queued Reports.  Canceling the
       * timer will prevent any new Reports from being sent; clearing the
       * the flags will discard any pending Reports that could interfere
       * with freeing the group.
       */

      wd_cancel(group->polldog);
      wd_cancel(group->v1dog);
      CLR_MLD_SCHEDMSG(group->flags);
      CLR_MLD_WAITMSG(group->flags);

      /* Free the group structure */

      mld_grpfree(dev, group);
      net_unlock();
      return;
    }
#endif

  /* Check for an Other Querier Present Timeout.  This timer is set in non-
   * Querier mode to detect the case where we have lost the Querier.
   */

  if (!IS_MLD_QUERIER(group->flags))
    {
      /* We are not the Querier.  This is an Other Querier Present Timeout.
       * If this timeout expires, it means that there are no Queriers for
       * the group.  Let's revert to Querier mode.
       */

      SET_MLD_QUERIER(group->flags);
    }

  /* Check if this is Querier */

  if (IS_MLD_QUERIER(group->flags))
    {
      /* Schedule (and forget) the general query. */

      MLD_STATINCR(g_netstats.mld.query_sched);
      ret = mld_schedmsg(group, MLD_SEND_GENQUERY);
      if (ret < 0)
        {
          mlderr("ERROR: Failed to schedule message: %d\n", ret);
        }

      /* Restart the Querier timer */

      mld_start_polltimer(group, MSEC2TICK(MLD_QUERY_MSEC));
    }
  else
    {
      /* Not the Querier... Restart the Other Querier Present Timeout */

      mld_start_polltimer(group, MSEC2TICK(MLD_OQUERY_MSEC));
    }

  net_unlock();
}

/****************************************************************************
 * Name:  mld_polldog_timout
 *
 * Description:
 *   Timeout watchdog handler.
 *
 * Assumptions:
 *   This function is called from the polldog timer handler which runs in the
 *   context of the timer interrupt handler.
 *
 ****************************************************************************/

static void mld_polldog_timout(int argc, uint32_t arg, ...)
{
  FAR struct mld_group_s *group;
  int ret;

  mldinfo("Timeout!\n");

  /* Recover the reference to the group */

  group = (FAR struct mld_group_s *)arg;
  DEBUGASSERT(argc == 1 && group != NULL);

  /* Perform the timeout-related operations on (preferably) the low priority
   * work queue.
   */

  ret = work_queue(LPWORK, &group->work, mld_polldog_work, group, 0);
  if (ret < 0)
    {
      mlderr("ERROR: Failed to queue timeout work: %d\n", ret);
    }
}

/****************************************************************************
 * Name:  mld_v1dog_work
 *
 * Description:
 *   Timeout watchdog work
 *
 * Assumptions:
 *   This function is called from a work queue thread.
 *
 ****************************************************************************/

static void mld_v1dog_work(FAR void *arg)
{
  FAR struct mld_group_s *group;

  /* Recover the reference to the group */

  group = (FAR struct mld_group_s *)arg;
  DEBUGASSERT(group != NULL);

  net_lock();
  if (IS_MLD_V1COMPAT(group->flags))
    {
      /* Exit MLDv1 compatibility mode. */

      CLR_MLD_V1COMPAT(group->flags);

      /* Whenever a host changes its compatibility mode, it cancels all its
       * pending responses and retransmission timers.
       */

      wd_cancel(group->polldog);

      /* REVISIT:  We cannot cancel a pending message if there is a waiter.
       * Some additional logic would be required to avoid a hang.
       */

      if (!IS_MLD_WAITMSG(group->flags))
        {
          CLR_MLD_SCHEDMSG(group->flags);
        }
    }

  net_unlock();
}

/****************************************************************************
 * Name:  mld_v1dog_timout
 *
 * Description:
 *   Timeout watchdog handler.
 *
 * Assumptions:
 *   This function is called from the v1dog timer handler which runs in the
 *   context of the timer interrupt handler.
 *
 ****************************************************************************/

static void mld_v1dog_timout(int argc, uint32_t arg, ...)
{
  FAR struct mld_group_s *group;
  int ret;

  mldinfo("Timeout!\n");

  /* Recover the reference to the group */

  group = (FAR struct mld_group_s *)arg;
  DEBUGASSERT(argc == 1 && group != NULL);

  /* Cancels all its pending responses and retransmission timers */

  wd_cancel(group->polldog);

  /* Perform the timeout-related operations on (preferably) the low priority
   * work queue.
   */

  ret = work_queue(LPWORK, &group->work, mld_v1dog_work, group, 0);
  if (ret < 0)
    {
      mlderr("ERROR: Failed to queue timeout work: %d\n", ret);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mld_start_polltimer
 *
 * Description:
 *   Start the MLD poll timer.
 *
 ****************************************************************************/

void mld_start_polltimer(FAR struct mld_group_s *group, clock_t ticks)
{
  int ret;

  /* Start the timer */

  mldinfo("ticks: %lu\n", (unsigned long)ticks);

  ret = wd_start(group->polldog, ticks, mld_polldog_timout, 1, (uint32_t)group);

  DEBUGASSERT(ret == OK);
  UNUSED(ret);
}

/****************************************************************************
 * Name:  mld_start_v1timer
 *
 * Description:
 *   Start the MLDv1 compatibility timer.
 *
 ****************************************************************************/

void mld_start_v1timer(FAR struct mld_group_s *group, clock_t ticks)
{
  int ret;

  /* Start the timer */

  mldinfo("ticks: %lu\n", (unsigned long)ticks);

  ret = wd_start(group->v1dog, ticks, mld_v1dog_timout, 1, (uint32_t)group);

  DEBUGASSERT(ret == OK);
  UNUSED(ret);
}

#endif /* CONFIG_NET_MLD */
