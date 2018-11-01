/****************************************************************************
 * net/mld/mld_query.c
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

#include <nuttx/wdog.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/mld.h>

#include "devif/devif.h"
#include "inet/inet.h"
#include "mld/mld.h"
#include "utils/utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv6BUF  ((FAR struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mld_query
 *
 * Description:
 *  Called from icmpv6_input() when a Multicast Listener Query is received.
 *
 ****************************************************************************/

int mld_query(FAR struct net_driver_s *dev,
              FAR const struct mld_mcast_listen_query_s *query)
{
  FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
  FAR struct mld_group_s *group;
  unsigned int ticks;
  bool unspec;

  ninfo("Multicast Listener Query\n");
  ninfo("destipaddr: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        ipv6->destipaddr[0], ipv6->destipaddr[1], ipv6->destipaddr[2],
        ipv6->destipaddr[3], ipv6->destipaddr[4], ipv6->destipaddr[5],
        ipv6->destipaddr[6], ipv6->destipaddr[7]);

  /* Find the group (or create a new one) using the incoming IP address */

  group = mld_grpallocfind(dev, ipv6->destipaddr);
  if (group == NULL)
    {
      nerr("ERROR: Failed to allocate/find group\n");
      return -ENOENT;
    }

  /* Max Response Time.  The Max Response Time field is meaningful only in
   * Query  messages, and specifies the maximum allowed time before sending
   * a responding report in units of 1/10 second.  In all other messages,
   * it is set to zero by the sender and ignored by receivers.
   */

  /* Check if the query was sent to all systems */

  unspec = net_ipv6addr_cmp(query->grpaddr, g_ipv6_unspecaddr);

  if (net_ipv6addr_cmp(ipv6->destipaddr, g_ipv6_allnodes))
    {
      /* There are three variants of the Query message (RFC 3810):
       *
       * 1. A "General Query" is sent by the Querier to learn which
       *    multicast addresses have listeners on an attached link.  In a
       *    General Query, both the Multicast Address field and the Number
       *    of Sources (N) field are zero.
       * 2. A "Multicast Address Specific Query" is sent by the Querier to
       *    learn if a particular multicast address has any listeners on an
       *    attached link.  In a Multicast Address Specific Query, the
       *    Multicast Address field contains the multicast address of
       *    interest, while the Number of Sources (N) field is set to zero.
       * 3. A "Multicast Address and Source Specific Query" is sent by the
       *    Querier to learn if any of the sources from the specified list for
       *    the particular multicast address has any listeners on an attached
       *    link or not.  In a Multicast Address and Source Specific Query the
       *    Multicast Address field contains the multicast address of
       *    interest, while the Source Address [i] field(s) contain(s) the
       *    source address(es) of interest.
       */

      if (unspec && query->nsources == 0)
        {
          FAR struct mld_group_s *member;

          /* This is the general query */

          ninfo("General multicast query\n");
          MLD_STATINCR(g_netstats.mld.gmq_query_received);

          for (member = (FAR struct mld_group_s *)dev->grplist.head;
               member;
               member = member->next)
            {
              /* Skip over the all systems group entry */

              if (!net_ipv6addr_cmp(member->grpaddr, g_ipv6_allnodes))
                {
                  ticks = net_dsec2tick((int)query->mrc);
                  if (IS_MLD_IDLEMEMBER(member->flags) ||
                      mld_cmptimer(member, ticks))
                    {
                      mld_startticks(member, ticks);
                      CLR_MLD_IDLEMEMBER(member->flags);
                    }
                }
            }
        }
      else if (!unspec && query->nsources == 0)
        {
          ninfo("Multicast Address Specific Query\n");

          /* We first need to re-lookup the group since we used dest last time.
           * Use the incoming IPaddress!
           */

          MLD_STATINCR(g_netstats.mld.mas_query_received);

          group = mld_grpallocfind(dev, query->grpaddr);
          ticks = net_dsec2tick((int)query->mrc);

          if (IS_MLD_IDLEMEMBER(group->flags) || mld_cmptimer(group, ticks))
            {
              mld_startticks(group, ticks);
              CLR_MLD_IDLEMEMBER(group->flags);
            }
        }
      else
        {
          ninfo("Multicast Address and Source Specific Query\n");
          MLD_STATINCR(g_netstats.mld.massq_query_received);
#warning Missing logic
        }
    }

  /* Not sent to all systems -- Unicast query */

  else if (!unspec)
    {
      ninfo("Unicast query\n");
      MLD_STATINCR(g_netstats.mld.ucast_query_received);

      ninfo("Query to a specific group with the group address as destination\n");

      ticks = net_dsec2tick((int)query->mrc);
      if (IS_MLD_IDLEMEMBER(group->flags) || mld_cmptimer(group, ticks))
        {
          mld_startticks(group, ticks);
          CLR_MLD_IDLEMEMBER(group->flags);
        }
    }

  return OK;
}
