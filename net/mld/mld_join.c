/****************************************************************************
 * net/mld/mld_join.c
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

#include <assert.h>
#include <debug.h>

#include <netinet/in.h>

#include <nuttx/net/netconfig.h>
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
 * Name:  mld_joingroup
 *
 * Description:
 *   Add the specified group address to the group.  This function
 *   implements the logic for the IPV6_JOIN_GROUP socket option.
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
 * State transition diagram for a router in Querier state (RFC 2710):
 *                            ________________
 *                           |                |
 *                           |                |timer expired
 *              timer expired|                |(notify routing -,
 *         (notify routing -)|  No Listeners  |clear rxmt tmr)
 *                   ------->|    Present     |<---------
 *                  |        |                |          |
 *                  |        |                |          |
 *                  |        |________________|          |  ---------------
 *                  |                    |               | | rexmt timer   |
 *                  |     report received|               | |  expired      |
 *                  |  (notify routing +,|               | | (send m-a-s   |
 *                  |        start timer)|               | |  query,       |
 *        __________|______              |       ________|_|______ st rxmt |
 *       |                 |<------------       |                 | tmr)   |
 *       |                 |                    |                 |<-------
 *       |                 | report received    |                 |
 *       |                 | (start timer,      |                 |
 *       |                 |  clear rxmt tmr)   |                 |
 *       |    Listeners    |<-------------------|    Checking     |
 *       |     Present     | done received      |    Listeners    |
 *       |                 | (start timer*,     |                 |
 *       |                 |  start rxmt timer, |                 |
 *       |                 |  send m-a-s query) |                 |
 *   --->|                 |------------------->|                 |
 *  |    |_________________|                    |_________________|
 *  | report received |
 *  | (start timer)   |
 *   -----------------
 *
 * State transition diagram for a router in Non-Querier state is
 * similar, but non-Queriers do not send any messages and are only
 * driven by message reception.
 *
 *                             ________________
 *                            |                |
 *                            |                |
 *               timer expired|                |timer expired
 *          (notify routing -)|  No Listeners  |(notify routing -)
 *                  --------->|    Present     |<---------
 *                 |          |                |          |
 *                 |          |                |          |
 *                 |          |                |          |
 *                 |          |________________|          |
 *                 |                   |                  |
 *                 |                   |report received   |
 *                 |                   |(notify routing +,|
 *                 |                   | start timer)     |
 *         ________|________           |          ________|________
 *        |                 |<---------          |                 |
 *        |                 |  report received   |                 |
 *        |                 |  (start timer)     |                 |
 *        |    Listeners    |<-------------------|     Checking    |
 *        |     Present     | m-a-s query rec'd  |    Listeners    |
 *        |                 | (start timer*)     |                 |
 *   ---->|                 |------------------->|                 |
 *  |     |_________________|                    |_________________|
 *  | report received |
 *  | (start timer)   |
 *   -----------------
 *
 ****************************************************************************/

int mld_joingroup(FAR const struct ipv6_mreq *mrec)
{
  FAR struct net_driver_s *dev;
  struct mld_group_s *group;

  DEBUGASSERT(dev != NULL && mrec != NULL);

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

  ninfo("Join group: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        mrec->ipv6mr_multiaddr.s6_addr16[0],
        mrec->ipv6mr_multiaddr.s6_addr16[1],
        mrec->ipv6mr_multiaddr.s6_addr16[2],
        mrec->ipv6mr_multiaddr.s6_addr16[3],
        mrec->ipv6mr_multiaddr.s6_addr16[4],
        mrec->ipv6mr_multiaddr.s6_addr16[5],
        mrec->ipv6mr_multiaddr.s6_addr16[5],
        mrec->ipv6mr_multiaddr.s6_addr16[7);

  /* Check if a this address is already in the group */

  group = mld_grpfind(dev, mrec->ipv6mr_multiaddr.s6_addr16);
  if (group == NULL)
    {
      /* No... allocate a new entry */

      ninfo("Allocate new group entry\n");

      group = mld_grpalloc(dev, mrec->ipv6mr_multiaddr.s6_addr16);
      if (group == NULL)
        {
          return -ENOMEM;
        }

      MLD_STATINCR(g_netstats.mld.joins);

      /* Send the Version 1 Multicast Listener Report */

      MLD_STATINCR(g_netstats.mld.report_sched);
      mld_waitmsg(group, ICMPV6_MCAST_LISTEN_REPORT_V1);

      /* And start the timer at 10*100 msec */

      mld_starttimer(group, 10);

      /* Add the group (MAC) address to the Ethernet drivers MAC filter list */

      mld_addmcastmac(dev, mrec->ipv6mr_multiaddr.s6_addr16);
      return OK;
    }

  /* Return EEXIST if the address is already a member of the group */

  return -EEXIST;
}

#endif /* CONFIG_NET_MLD */
