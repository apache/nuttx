/****************************************************************************
 * net/mld/mld_report.c
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

#include <nuttx/wdog.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/mld.h>

#include "devif/devif.h"
#include "mld/mld.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mld_report
 *
 * Description:
 *   Common report handling.  Since we are not a router, we do very little
 *   on the receipt of a report from another member of the group.
 *
 ****************************************************************************/

int mld_report(FAR struct net_driver_s *dev,
               FAR const net_ipv6addr_t grpaddr)
{
  FAR struct mld_group_s *group;

  /* Reports are send to the group multicast address.  Hence, the IPv6
   * destipaddr idenfies the group.
   */

  mldinfo("grpaddr: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
          grpaddr[0], grpaddr[1], grpaddr[2], grpaddr[3],
          grpaddr[4], grpaddr[5], grpaddr[6], grpaddr[7]);

#ifdef CONFIG_NET_MLD_ROUTER
  /* Assure that the group address is an IPv6 multicast address */

  if (!net_is_addr_mcast(mrec->ipv6mr_multiaddr.s6_addr16))
    {
      return -EINVAL;
    }

  group = mld_grpallocfind(dev, grpaddr);
  if (group == NULL)
    {
      mlderr("ERROR: Failed to allocate group\n");
      return -ENOMEM;
    }

#else
  /* Find the group using the incoming IP address.  If we are not a router,
   * then can ignore reports from groups that we are not a member of.
   */

  group = mld_grpfind(dev, grpaddr);
  if (group == NULL)
    {
      mldinfo("WARNING: Ignoring group.  We are not a member\n");
      return -ENOENT;
    }
#endif

  /* There are certainly other members of this group, we can clear the
   * LASTREPORT flag.
   */

  CLR_MLD_LASTREPORT(group->flags);

#ifdef CONFIG_NET_MLD_ROUTER
  /* Increment the number of members that reported in this query cycle. */

  group->members++;
#endif

  /* Need to set d_len to zero to indication that nothing is being sent */

  dev->d_len = 0;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mld_report_v1
 *
 * Description:
 *   Called from icmpv6_input() when a Version 1 or Version 2 Multicast
 *   Listener Report is received.
 *
 *  If a node receives another node's Report from an interface for a
 *  multicast address while it has a timer running for that same address
 *  on that interface, it stops its timer and does not send a Report for
 *  that address, thus suppressing duplicate reports on the link.
 *
 *  When a router receives a Report from a link, if the reported address
 *  is not already present in the router's list of multicast address
 *  having listeners on that link, the reported address is added to the
 *  list, its timer is set, and its appearance is made known to the router's
 *  multicast routing component.  If a Report is received for a multicast
 *  address that is already present in the router's list, the timer for that
 *  address is reset.  If an address's timer expires, it is assumed that
 *  there are no longer any listeners for that address present on the link,
 *  so it is deleted from the list and its disappearance is made known to
 *  the multicast routing component.
 *
 *  A node MUST accept and process any version 1 Report whose IP
 *  Destination Address field contains *any* of the IPv6 addresses (unicast
 *  or multicast) assigned to the interface on which the Report arrives.
 *
 ****************************************************************************/

int mld_report_v1(FAR struct net_driver_s *dev,
                  FAR const struct mld_mcast_listen_report_v1_s *report)
{
  mldinfo("MLDv1 Multicast Listener Report\n");
  DEBUGASSERT(dev != NULL && report != NULL);

  MLD_STATINCR(g_netstats.mld.v1report_received);
  return mld_report(dev, report->mcastaddr);
}

/****************************************************************************
 * Name: mld_report_v2
 *
 * Description:
 *   Called from icmpv6_input() when a Version 2 Multicast Listener Report is
 *   received.
 *
 *   Upon reception of an MLD message that contains a Report, the router
 *   checks if the source address of the message is a valid link-local
 *   address, if the Hop Limit is set to 1, and if the Router Alert option
 *   is present in the Hop-By-Hop Options header of the IPv6 packet.  If
 *   any of these checks fails, the packet is dropped.  If the validity of
 *   the MLD message is verified, the router starts to process the Report.
 *
 *   Version 2 Multicast Listener Reports are sent with an IP destination
 *   address of FF02:0:0:0:0:0:0:16, to which all MLDv2-capable multicast
 *   routers listen
 *
 ****************************************************************************/

int mld_report_v2(FAR struct net_driver_s *dev,
                  FAR const struct mld_mcast_listen_report_v2_s *report)
{
  uint16_t naddrec;
  int ret = -ENOENT;
  int i;

  mldinfo("Version 2 Multicast Listener Report\n");
  DEBUGASSERT(dev != NULL && report != NULL);

  MLD_STATINCR(g_netstats.mld.v2report_received);

  naddrec = NTOHS(report->naddrec);
  for (i = 0; i < naddrec; i++)
    {
      /* Handle this mcast address in the list */

      int status = mld_report(dev, report->addrec[i].mcast);
      if (status == OK)
        {
          /* Return success if any address in the listed was processed */

          ret = OK;
        }
      else if (status != -ENOENT)
        {
          /* Any result other than -ENOENT would be a problem */

          ret = status;
          break;
        }
    }

  return ret;
}
