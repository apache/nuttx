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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mld_mrc2mrd
 *
 * Description:
 *  Convert the MLD Maximum Response Code (MRC) to the Maximum Response
 *  Delay (MRD) in units of system clock ticks.
 *
 ****************************************************************************/

static clock_t mld_mrc2mrd(uint16_t mrc)
{
  uint32_t mrd;  /* Units of milliseconds */

  /* If bit 15 is not set (i.e., mrc < 32768), then no conversion is required. */

  if (mrc < 32768)
    {
      mrd = mrc;
    }
  else
    {
      /* Conversion required */

      mrd = MLD_MRD_VALUE(mrc);
    }

  /* Return the MRD in units of clock ticks */

  return MSEC2TICK((clock_t)mrd);
}

/****************************************************************************
 * Name: mld_cmpaddr
 *
 * Description:
 *  Perform a numerical comparison of the IPv6 Source Address and the IPv6
 *  address of the link.  Return true if the source address is less than
 *  the link address.
 *
 ****************************************************************************/

static bool mld_cmpaddr(FAR struct net_driver_s *dev,
                        const net_ipv6addr_t srcaddr)
{
  int i;

  for (i = 0; i < 8; i++)
    {
      if (srcaddr[i] < dev->d_ipv6addr[i])
        {
          return true;
        }
    }

  return false;
}

/****************************************************************************
 * Name: mld_check_querier
 *
 * Description:
 *  Perform a numerical comparison of the IPv6 Source Address and the IPv6
 *  address of the link.  Return true if the source address is less than
 *  the link address.
 *
 ****************************************************************************/

static void mld_check_querier(FAR struct net_driver_s *dev,
                              FAR struct ipv6_hdr_s *ipv6,
                              FAR struct mld_group_s *member,
                              uint16_t mrc)
{
  clock_t ticks;

  /* Check if this member is a Querier */

  if (IS_MLD_QUERIER(member->flags))
    {
      /* This is a querier, check if the IPv6 source address is numerically
       * less than the IPv6 address assigned to this link.
       */

      if (mld_cmpaddr(dev, ipv6->srcipaddr))
        {
          /* This is a querier, then switch to non-querier and set a timeout.
           * If additional queries are received within this timeout period,
           * then we need to revert to Querier.
           */

          ticks =  mld_mrc2mrd(mrc);
          if (mld_cmptimer(member, ticks))
            {
              mld_starttimer(member, ticks);
              CLR_MLD_QUERIER(member->flags);
            }
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mld_query
 *
 * Description:
 *   Called from icmpv6_input() when a Multicast Listener Query is received.
 *
 *   A router may assume one of two roles: Querier or Non-Querier.  There is
 *   normally only one Querier per link.  All routers start up as a Querier
 *   on each of their attached links.  If a router hears a Query message
 *   whose IPv6 Source Address is numerically less than its own selected
 *   address for that link, it MUST become a Non-Querier on that link.  If a
 *   delay passes without receiving, from a particular attached link, any
 *   Queries from a router with an address less than its own, a router
 *   resumes the role of Querier on that link.
 *
 *   A Querier for a link periodically sends a General Query on that link,
 *   to solicit reports of all multicast addresses of interest on that link.
 *   On startup, a router SHOULD send multiple General Queries spaced closely
 *   together Interval] on all attached links in order to quickly and
 *   reliably discover the presence of multicast listeners on those links.
 *
 ****************************************************************************/

int mld_query(FAR struct net_driver_s *dev,
              FAR const struct mld_mcast_listen_query_s *query)
{
  FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
  FAR struct mld_group_s *group;
  uint16_t mrc;
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
  mrc    = NTOHS(query->mrc);

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

          for (member = (FAR struct mld_group_s *)dev->d_mld_grplist.head;
               member;
               member = member->next)
            {
              /* Skip over the all systems group entry */

              if (!net_ipv6addr_cmp(member->grpaddr, g_ipv6_allnodes))
                {
                   /* REVISIT: Missing logic.  No action is taken on the query. */

                   mld_check_querier(dev, ipv6, member, mrc);
                }
            }
        }
      else if (!unspec && query->nsources == 0)
        {
          ninfo("Multicast Address Specific Query\n");
          MLD_STATINCR(g_netstats.mld.mas_query_received);

          /* We first need to re-lookup the group since we used the incoming
           * dest last time.  Use the group address in the query.
           */

          group = mld_grpallocfind(dev, query->grpaddr);

          /* REVISIT: Missing logic.  No action is taken on the query. */

          mld_check_querier(dev, ipv6, group, mrc);
        }
      else
        {
          ninfo("Multicast Address and Source Specific Query\n");
          MLD_STATINCR(g_netstats.mld.massq_query_received);

          /* We first need to re-lookup the group since we used the incoming
           * dest last time.  Use the group address in the query.
           */

          group = mld_grpallocfind(dev, query->grpaddr);

          /* REVISIT: Missing logic.  No action is taken on the query. */

          mld_check_querier(dev, ipv6, group, mrc);
        }
    }

  /* Not sent to all systems -- Unicast query */

  else if (!unspec)
    {
      ninfo("Unicast query\n");
      MLD_STATINCR(g_netstats.mld.ucast_query_received);

      mld_check_querier(dev, ipv6, group, mrc);
    }

  return OK;
}
