/****************************************************************************
 * net/mld/mld_leave.c
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

  ninfo("Leave group: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        mrec->ipv6mr_multiaddr.s6_addr16[0],
        mrec->ipv6mr_multiaddr.s6_addr16[1],
        mrec->ipv6mr_multiaddr.s6_addr16[2],
        mrec->ipv6mr_multiaddr.s6_addr16[3],
        mrec->ipv6mr_multiaddr.s6_addr16[4],
        mrec->ipv6mr_multiaddr.s6_addr16[5],
        mrec->ipv6mr_multiaddr.s6_addr16[5],
        mrec->ipv6mr_multiaddr.s6_addr16[7);

  /* Find the entry corresponding to the address leaving the group */

  group = mld_grpfind(dev, mrec->ipv6mr_multiaddr.s6_addr16);
  if (group != NULL)
    {
      ninfo("Leaving group: %p\n", group);

      /* Cancel the timer and discard any queued Reports.  Canceling the
       * timer will prevent any new Reports from being sent;  clearing the
       * flags will discard any pending Reports that could interfere with
       * leaving the group.
       */

      wd_cancel(group->wdog);
      CLR_MLD_SCHEDMSG(group->flags);
      CLR_MLD_WAITMSG(group->flags);

      MLD_STATINCR(g_netstats.mld.leaves);

      /* Send a leave if the flag is set according to the state diagram */

      if (IS_MLD_LASTREPORT(group->flags))
        {
          ninfo("Schedule Done message\n");

          MLD_STATINCR(g_netstats.mld.done_sched);

          ret = mld_waitmsg(group, MLD_SEND_DONE);
          if (ret < 0)
            {
              nerr("ERROR: Failed to schedule message: %d\n", ret);
            }
        }

      /* Free the group structure */

      mld_grpfree(dev, group);

      /* And remove the group address from the Ethernet drivers MAC filter set */

      mld_removemcastmac(dev, mrec->ipv6mr_multiaddr.s6_addr16);
      return OK;
    }

  /* Return ENOENT if the address is not a member of the group */

  ninfo("Return -ENOENT\n");
  return -ENOENT;
}

#endif /* CONFIG_NET_MLD */
