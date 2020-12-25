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
 * 3. Neither the name of CITEL Technologies Ltd nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY CITEL TECHNOLOGIES AND CONTRIBUTORS ``AS IS''
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL CITEL TECHNOLOGIES OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
 * Name:  mld_group_startup
 *
 * Description:
 *   Set up the startup actions when the first member from this host joins
 *   the group.
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

static int mld_group_startup(FAR const struct ipv6_mreq *mrec,
                             FAR struct net_driver_s *dev,
                             FAR struct mld_group_s *group)
{
  int ret;

  /* Send the Version 2 Multicast Listener Report (Assumes MLDv2 mode
   * initially).
   */

  MLD_STATINCR(g_netstats.mld.report_sched);

  ret = mld_waitmsg(group, MLD_SEND_V2REPORT);
  if (ret < 0)
    {
      mlderr("ERROR: Failed to schedule Report: %d\n", ret);
      return ret;
    }

  /* And start the timer at 1 second */

  SET_MLD_STARTUP(group->flags);
  group->count = MLD_UNSOLREPORT_COUNT - 1;
  mld_start_polltimer(group, MSEC2TICK(MLD_UNSOLREPORT_MSEC));

  /* Add the group (MAC) address to the Ethernet drivers MAC filter list */

  mld_addmcastmac(dev, mrec->ipv6mr_multiaddr.s6_addr16);
  return OK;
}

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
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

int mld_joingroup(FAR const struct ipv6_mreq *mrec)
{
  FAR struct net_driver_s *dev;
  FAR struct mld_group_s *group;
  int ret;

  DEBUGASSERT(mrec != NULL);

  /* Assure that the group address is an IPv6 multicast address */

  if (!net_is_addr_mcast(mrec->ipv6mr_multiaddr.s6_addr16))
    {
      return -EINVAL;
    }

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
      mlderr("ERROR: No device for this interface index: %u\n",
             mrec->ipv6mr_interface);
      return -ENODEV;
    }

  mldinfo("Join group: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
          mrec->ipv6mr_multiaddr.s6_addr16[0],
          mrec->ipv6mr_multiaddr.s6_addr16[1],
          mrec->ipv6mr_multiaddr.s6_addr16[2],
          mrec->ipv6mr_multiaddr.s6_addr16[3],
          mrec->ipv6mr_multiaddr.s6_addr16[4],
          mrec->ipv6mr_multiaddr.s6_addr16[5],
          mrec->ipv6mr_multiaddr.s6_addr16[5],
          mrec->ipv6mr_multiaddr.s6_addr16[7]);

  /* Check if a this address is already in the group */

  group = mld_grpfind(dev, mrec->ipv6mr_multiaddr.s6_addr16);
  if (group == NULL)
    {
      /* No... allocate a new entry */

      mldinfo("Allocate new group entry\n");

      group = mld_grpalloc(dev, mrec->ipv6mr_multiaddr.s6_addr16);
      if (group == NULL)
        {
          return -ENOMEM;
        }

      /* Indicate one request to join the group from this host */

      group->njoins = 1;

      /* Set up the group startup operations */

      ret = mld_group_startup(mrec, dev, group);
      if (ret < 0)
        {
          mlderr("ERROR: Failed to start group: %d\n", ret);
          mld_grpfree(dev, group);
          return ret;
        }

      MLD_STATINCR(g_netstats.mld.njoins);

      /* REVISIT: It is expected that higher level logic will set up
       * the routing table entry for the new multicast address.  That
       * is not done here.
       */
    }
  else
    {
      /* The group already exists; a task from this host is joining an
       * existing group.
       */

      mldinfo("Join existing group\n");

#ifdef CONFIG_NET_MLD_ROUTER
      /* In the Router case this could still be the first join from this
       * host.  If this is the first join from this host, then we need to
       * perform the group startup operations.
       */

      if (group->join == 0)
        {
          /* This is the for join from this host.  Perform out start up
           * operations.
           */

          ret = mld_group_startup(mrec, dev, group);
          if (ret < 0)
            {
              mlderr("ERROR: Failed to start group: %d\n", ret);
              mld_grpfree(dev, group);
              return ret;
            }
        }
#else
      /* Not a router?  Then there must be another join from this host or
       * how could the group have been created?
       */

      DEBUGASSERT(group->njoins > 0);
#endif

      /* Indicate one more request to join the group from this host */

      DEBUGASSERT(group->njoins < UINT8_MAX);
      group->njoins++;
      MLD_STATINCR(g_netstats.mld.njoins);
    }

  return OK;
}

#endif /* CONFIG_NET_MLD */
