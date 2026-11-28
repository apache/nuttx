/****************************************************************************
 * net/igmp/igmp_timer.c
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
#include <nuttx/compiler.h>

#include <assert.h>
#include <nuttx/debug.h>

#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/igmp.h>

#include "devif/devif.h"
#include "igmp/igmp.h"
#include "utils/utils.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#undef IGMP_GTMRDEBUG /* Define to enable detailed IGMP group debug */

#ifndef CONFIG_NET_IGMP
#  undef IGMP_GTMRDEBUG
#endif

#ifdef IGMP_GTMRDEBUG
#  define gtmrerr    nerr
#  define gtmrinfo   ninfo
#else
#  define gtmrerr    _none
#  define gtmrinfo   _none
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_timeout_work
 *
 * Description:
 *   Timeout watchdog work.
 *
 * Assumptions:
 *   This function is called from a work queue thread.
 *
 ****************************************************************************/

static void igmp_timeout_work(FAR void *arg)
{
  FAR struct igmp_group_s *group;
  int ret;

  /* If the state is DELAYING_MEMBER then we send a report for this group */

  group = (FAR struct igmp_group_s *)arg;
  DEBUGASSERT(group != NULL);

  /* If the group exists and is no an IDLE MEMBER, then it must be a DELAYING
   * member. Race conditions are avoided because (1) the timer is not started
   * until after the first IGMPv2_MEMBERSHIP_REPORT during the join, and (2)
   * the timer is cancelled before sending the IGMP_LEAVE_GROUP during leave.
   */

  if (!IS_IDLEMEMBER(group->flags))
    {
      /* Schedule (and forget) the Membership Report.  NOTE:
       * Since we are executing from a timer interrupt, we cannot wait
       * for the message to be sent.
       */

      IGMP_STATINCR(g_netstats.igmp.report_sched);
      ret = igmp_schedmsg(group, IGMPv2_MEMBERSHIP_REPORT);
      if (ret < 0)
        {
          nerr("ERROR: Failed to schedule message: %d\n", ret);
        }

      /* Also note:  The Membership Report is sent at most two times because
       * the timer is not reset here.  Hmm.. does this mean that the group
       * is stranded if both reports were lost?  This is consistent with the
       * RFC that states: "To cover the possibility of the initial Membership
       * Report being lost or damaged, it is recommended that it be repeated
       * once or twice after short delays [Unsolicited Report Interval]..."
       */
    }
}

/****************************************************************************
 * Name:  igmp_timeout
 *
 * Description:
 *   Timeout watchdog handler.
 *
 * Assumptions:
 *   This function is called from the wdog timer handler which runs in the
 *   context of the timer interrupt handler.
 *
 ****************************************************************************/

static void igmp_timeout(wdparm_t arg)
{
  FAR struct igmp_group_s *group;
  int ret;

  ninfo("Timeout!\n");

  /* Recover the reference to the group */

  group = (FAR struct igmp_group_s *)arg;
  DEBUGASSERT(group != NULL);

  /* Perform the timeout-related operations on (preferably) the low priority
   * work queue.
   */

  ret = work_queue(LPWORK, &group->work, igmp_timeout_work, group, 0);
  if (ret < 0)
    {
      nerr("ERROR: Failed to queue timeout work: %d\n", ret);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_startticks and igmp_starttimer
 *
 * Description:
 *   Start the IGMP timer with differing time units (ticks or deciseconds).
 *
 * Assumptions:
 *   This function may be called from most any context.
 *
 ****************************************************************************/

void igmp_startticks(FAR struct igmp_group_s *group, unsigned int ticks)
{
  int ret;

  /* Start the timer */

  gtmrinfo("ticks: %d\n", ticks);

  ret = wd_start(&group->wdog, ticks, igmp_timeout, (wdparm_t)group);

  DEBUGASSERT(ret == OK);
  UNUSED(ret);
}

void igmp_starttimer(FAR struct igmp_group_s *group, uint8_t decisecs)
{
  /* Convert the decisec value to system clock ticks and start the timer.
   * Important!! this should be a random timer from 0 to decisecs
   */

  gtmrinfo("decisecs: %d\n", decisecs);
  igmp_startticks(group, net_dsec2tick(decisecs));
}

/****************************************************************************
 * Name:  igmp_cmptimer
 *
 * Description:
 *   Compare the timer remaining on the watching timer to the deci-second
 *   value. If maxticks > ticks-remaining, then (1) cancel the timer (to
 *   avoid race conditions) and return true.
 *
 * Assumptions:
 *   This function may be called from most any context.  If true is returned
 *   then the caller must call igmp_startticks() to restart the timer
 *
 ****************************************************************************/

bool igmp_cmptimer(FAR struct igmp_group_s *group, int maxticks)
{
  int remaining;

  /* Get the timer remaining on the watchdog.  A time of <= zero means that
   * the watchdog was never started.
   */

  remaining = wd_gettime(&group->wdog);

  /* A remaining time of zero means that the watchdog was never started
   * or has already expired.  That case should be covered in the following
   * test as well.
   */

  gtmrinfo("maxticks: %d remaining: %d\n", maxticks, remaining);
  if (maxticks > remaining)
    {
      /* Cancel the watchdog timer and return true */

      wd_cancel(&group->wdog);
      return true;
    }

  return false;
}

#endif /* CONFIG_NET_IGMP */
