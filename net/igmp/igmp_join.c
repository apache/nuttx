/****************************************************************************
 * net/igmp/igmp_join.c
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
#include <inttypes.h>
#include <stdint.h>

#include <netinet/in.h>

#include <nuttx/net/netconfig.h>
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
 * Name:  igmp_joingroup
 *
 * Description:
 *   Add the specified group address to the group.
 *
 * RFC 2236, 3.  Protocol Description:
 *
 *  "When a host joins a multicast group, it should immediately transmit
 *   an unsolicited Version 2 Membership Report for that group, in case it
 *   is the first member of that group on the network.  To cover the
 *   possibility of the initial Membership Report being lost or damaged,
 *   it is recommended that it be repeated once or twice after short
 *   delays [Unsolicited Report Interval].  (A simple way to accomplish
 *   this is to send the initial Version 2 Membership Report and then act
 *   as if a Group-Specific Query was received for that group, and set a
 *   timer appropriately)."
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

int igmp_joingroup(struct net_driver_s *dev,
                   FAR const struct in_addr *grpaddr)
{
  struct igmp_group_s *group;
  int ret;

  DEBUGASSERT(dev != NULL && grpaddr != NULL);

  /* Check if a this address is already in the group */

  group = igmp_grpfind(dev, &grpaddr->s_addr);
  if (!group)
    {
      /* No... allocate a new entry */

      ninfo("Join to new group: %08" PRIx32 "\n", (uint32_t)grpaddr->s_addr);
      group = igmp_grpalloc(dev, &grpaddr->s_addr);
      if (group == NULL)
        {
          return -EADDRNOTAVAIL;
        }

      IGMP_STATINCR(g_netstats.igmp.joins);

      /* Send the Membership Report */

      IGMP_STATINCR(g_netstats.igmp.report_sched);
      ret = igmp_waitmsg(group, IGMPv2_MEMBERSHIP_REPORT);
      if (ret < 0)
        {
          nerr("ERROR: Failed to schedule message: %d\n", ret);
          igmp_grpfree(dev, group);
          return ret;
        }

      /* And start the timer at 10*100 msec */

      igmp_starttimer(group, 10);

      /* Add the group (MAC) address to the ether drivers MAC filter list */

      igmp_addmcastmac(dev, (FAR in_addr_t *)&grpaddr->s_addr);
    }

  DEBUGASSERT(group->njoins < UINT8_MAX);
  group->njoins++;

  return OK;
}

#endif /* CONFIG_NET_IGMP */
