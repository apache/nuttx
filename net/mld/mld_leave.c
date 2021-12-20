/****************************************************************************
 * net/mld/mld_leave.c
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
#include <debug.h>

#include <netinet/in.h>

#include <nuttx/wdog.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/net.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/mld.h>

#include "netdev/netdev.h"
#include "mld/mld.h"

#ifdef CONFIG_NET_MLD

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mld_leavegroup
 *
 * Description:
 *   Remove the specified group address to the group.  This function
 *   implements the logic for the IPV6_LEAVE_GROUP socket option.
 *
 *   The IPV6_JOIN_GROUP socket option is used to join a multicast group.
 *   This is accomplished by using the setsockopt() API and specifying the
 *   address of the ipv6_mreq structure containing the IPv6 multicast address
 *   and the local IPv6 multicast interface index.  The stack chooses a
 *   default multicast interface if an interface index of 0 is passed. The
 *   values specified in the IPV6_MREQ structure used by IPV6_JOIN_GROUP
 *   and IPV6_LEAVE_GROUP must be symmetrical. The format of the ipv6_mreq
 *   structure can be found in include/netinet/in.h
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

int mld_leavegroup(FAR const struct ipv6_mreq *mrec)
{
  FAR struct net_driver_s *dev;
  struct mld_group_s *group;
  int ret;

  DEBUGASSERT(mrec != NULL);

  /* Get the device from the interface index.  Use the default network device
   * if an interface index of 0 is provided.
   */

  if (mrec->ipv6mr_interface == 0)
    {
      dev = netdev_default();
    }
  else
    {
      dev = netdev_findbyindex(mrec->ipv6mr_interface);
    }

  if (dev == NULL)
    {
      return -ENODEV;
    }

  mldinfo("Leave group: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
          mrec->ipv6mr_multiaddr.s6_addr16[0],
          mrec->ipv6mr_multiaddr.s6_addr16[1],
          mrec->ipv6mr_multiaddr.s6_addr16[2],
          mrec->ipv6mr_multiaddr.s6_addr16[3],
          mrec->ipv6mr_multiaddr.s6_addr16[4],
          mrec->ipv6mr_multiaddr.s6_addr16[5],
          mrec->ipv6mr_multiaddr.s6_addr16[5],
          mrec->ipv6mr_multiaddr.s6_addr16[7]);

  /* Find the entry corresponding to the address leaving the group */

  group = mld_grpfind(dev, mrec->ipv6mr_multiaddr.s6_addr16);
  if (group != NULL)
    {
      mldinfo("Leaving group: %p\n", group);

      /* Indicate one fewer members of the group from this host */

      DEBUGASSERT(group->njoins > 0);
      group->njoins--;
      MLD_STATINCR(g_netstats.mld.nleaves);

      /* Take no further actions if there are other members of this group
       * on this host.
       */

      if (group->njoins > 0)
        {
          return OK;
        }

      /* Perform actions that would be performed only when the number of
       * njoins from this host goes to zero.
       */

      if (group->njoins == 0)
        {
          /* Send a leave if the LASTREPORT flag is set for the group.  If
           * there are other members of the group, then their reports will
           * clear the LAST REPORT flag.  In this case we know that there are
           * other members of the group and we do not have to send the Done
           * message.
           *
           * The MLDv1 router responds to the Done message with a multicast-
           * address- specific (MAS) Query. If any other node responds to
           * the Query with a Report message the there are still listeners
           * present.
           */

          if (IS_MLD_LASTREPORT(group->flags))
            {
              mldinfo("Schedule Done message\n");

              MLD_STATINCR(g_netstats.mld.done_sched);

              /* REVISIT:  This will interfere if there are any other tasks
               * waiting for a message to be sent.  Can that happen?
               */

              ret = mld_waitmsg(group, MLD_SEND_DONE);
              if (ret < 0)
                {
                  mlderr("ERROR: Failed to schedule message: %d\n", ret);
                }
            }

          /* Remove the group address from the Ethernet drivers MAC filter
           * set
           */

          mld_removemcastmac(dev, mrec->ipv6mr_multiaddr.s6_addr16);

          /* Perform additional the number of members not on this host is
           * also zero.
           */

#ifdef CONFIG_NET_MLD_ROUTER
          if (group->members == 0 && group->lstmbrs == 0)
#endif
            {
              /* Cancel the timers and discard any queued Reports.  Canceling
               * the timer will prevent any new Reports from being sent;
               * clearing the flags will discard any pending Reports that
               * could interfere with freeing the group.
               */

              wd_cancel(&group->polldog);
              CLR_MLD_SCHEDMSG(group->flags);
              CLR_MLD_WAITMSG(group->flags);

              /* Free the group structure */

              mld_grpfree(dev, group);

              /* REVISIT: It is expected that higher level logic will remove
               * the routing table entry for the old multicast address.  That
               * is not done here.
               */
            }
        }

      return OK;
    }

  /* Return ENOENT if the address is not a member of the group */

  mldinfo("Return -ENOENT\n");
  return -ENOENT;
}

#endif /* CONFIG_NET_MLD */
