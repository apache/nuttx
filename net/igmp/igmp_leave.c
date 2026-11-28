/****************************************************************************
 * net/igmp/igmp_leave.c
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

#include <assert.h>
#include <nuttx/debug.h>

#include <netinet/in.h>

#include <nuttx/wdog.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/igmp.h>

#include "devif/devif.h"
#include "igmp/igmp.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_leavegroup
 *
 * Description:
 *   Remove the specified group address to the group.
 *
 * RFC 2236, 3.  Protocol Description:
 *
 *  "When a host leaves a multicast group, if it was the last host to
 *   reply to a Query with a Membership Report for that group, it SHOULD
 *   send a Leave Group message to the all-routers multicast group
 *   (224.0.0.2). If it was not the last host to reply to a Query, it MAY
 *   send nothing as there must be another member on the subnet.  This is
 *   an optimization to reduce traffic; a host without sufficient storage
 *   to remember whether or not it was the last host to reply MAY always
 *   send a Leave Group message when it leaves a group.  Routers SHOULD
 *   accept a Leave Group message addressed to the group being left, in
 *   order to accommodate implementations of an earlier version of this
 *   standard.  Leave Group messages are addressed to the all-routers
 *   group because other group members have no need to know that a host
 *   has left the group, but it does no harm to address the message to the
 *   group."
 *
 *                              ________________
 *                             |                |
 *                             |                |
 *                             |                |
 *                             |                |
 *                  +--------->|   Non-Member   |<---------+
 *                  |          |                |          |
 *                  |          |                |          |
 *                  |          |                |          |
 *                  |          |________________|          |
 *                  |                   |                  |
 *                  | leave group       | join group       | leave group
 *                  | (stop timer,      |(send report,     | (send leave
 *                  |  send leave if    | set flag,        |  if flag set)
 *                  |  flag set)        | start timer)     |
 *          ________|________           |          ________|________
 *         |                 |<---------+         |                 |
 *         |                 |                    |                 |
 *         |                 |<-------------------|                 |
 *         |                 |   query received   |                 |
 *         | Delaying Member |    (start timer)   |   Idle Member   |
 *   +---->|                 |------------------->|                 |
 *   |     |                 |   report received  |                 |
 *   |     |                 |    (stop timer,    |                 |
 *   |     |                 |     clear flag)    |                 |
 *   |     |_________________|------------------->|_________________|
 *   | query received    |        timer expired
 *   | (reset timer if   |        (send report,
 *   |  Max Resp Time    |         set flag)
 *   |  < current timer) |
 *   +-------------------+
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

int igmp_leavegroup(struct net_driver_s *dev,
                    FAR const struct in_addr *grpaddr)
{
  struct igmp_group_s *group;
  int ret;

  DEBUGASSERT(dev && grpaddr);

  /* Find the entry corresponding to the address leaving the group */

  group = igmp_grpfind(dev, &grpaddr->s_addr);
  ninfo("Leaving group: %p\n", group);
  if (group)
    {
      DEBUGASSERT(group->njoins > 0);
      group->njoins--;

      /* Take no further actions if there are other members of this group
       * on this host.
       */

      if (group->njoins > 0)
        {
          return OK;
        }

      /* Cancel the timer and discard any queued Membership Reports.
       * Canceling the timer will prevent any new Membership Reports from
       * being sent; clearing the flags will discard any pending Membership
       * Reports that could interfere with the Leave Group.
       */

      wd_cancel(&group->wdog);
      CLR_SCHEDMSG(group->flags);
      CLR_WAITMSG(group->flags);

      IGMP_STATINCR(g_netstats.igmp.leaves);

      /* Send a leave if the flag is set according to the state diagram */

      if (IFF_IS_RUNNING(dev->d_flags) && IS_LASTREPORT(group->flags))
        {
          ninfo("Schedule Leave Group message\n");
          IGMP_STATINCR(g_netstats.igmp.leave_sched);

          ret = igmp_waitmsg(group, IGMP_LEAVE_GROUP);
          if (ret < 0)
            {
              nerr("ERROR: Failed to schedule message: %d\n", ret);
            }
        }

      /* Free the group structure (state is now Non-Member */

      igmp_grpfree(dev, group);

      /* And remove the group address from the drivers MAC filter set */

      igmp_removemcastmac(dev, (FAR in_addr_t *)&grpaddr->s_addr);
      return OK;
    }

  /* Return ENOENT if the address is not a member of the group */

  ninfo("Return -ENOENT\n");
  return -ENOENT;
}

#endif /* CONFIG_NET_IGMP */
