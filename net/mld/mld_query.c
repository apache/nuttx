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

#include <stdbool.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/wdog.h>
#include <nuttx/net/netconfig.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/netdev.h>
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
 * Name: mld_setup_v1compat
 *
 * Description:
 *   If this is for MLDv1 query, then select MLDv1 compatibility mode and
 *   start (or re-start) the compatibility timer.  We need to make this
 *   check BEFORE sending the report.
 *
 ****************************************************************************/

static void mld_setup_v1compat(FAR struct mld_group_s *group,
                               FAR const struct mld_mcast_listen_query_s *query,
                               bool mldv1)
{
  unsigned int respmsec;

  if (mldv1)
    {
#if 0 /* REVISIT */
      /* Get the QQI from the query.  Since this is MLDv1, we know that
       * the value is not encoded.
       */

      respmsec = MSEC_PER_SEC * MLD_QQI_VALUE(query->qqic);
#else
      /* REVISIT:  I am confused.  Per RFC 3810:
       * "The Older Version Querier Present Timeout is the time-out for
       *  transitioning a host back to MLDv2 Host Compatibility Mode.  When
       *  an MLDv1 query is received, MLDv2 hosts set their Older Version
       *  Querier Present Timer to [Older Version Querier Present Timeout].
       *
       * "This value MUST be ([Robustness Variable] times (the [Query
       *  Interval] in the last Query received)) plus ([Query Response
       *  Interval])."
       *
       * I am not sure how to do that since the MLDv1 version has no QQI
       * field.  That is an MLDv2 extension.
       */

      respmsec = MLD_QUERY_MSEC;
#endif

      /* Select MLDv1 compatibility mode (might already be selected) */

      SET_MLD_V1COMPAT(group->flags);

      /* Whenever a host changes its compatibility mode, it cancels all its
       * pending responses and retransmission timers.
       */

      wd_cancel(group->polldog);

      /* REVISIT:  We cannot cancel a pending message if there is a waiter.
       * Some additional logic would be required to avoid a hang.
       */

      if (!IS_MLD_WAITMSG(group->flags))
        {
          CLR_MLD_SCHEDMSG(group->flags);
        }

      /* And start the MLDv1 compatibility timer.  If the timer is already
       * running, this will reset the timer.
       */

      mld_start_v1timer(group,
                        MSEC2TICK(MLD_V1PRESENT_MSEC((clock_t)respmsec)));
    }
}

/****************************************************************************
 * Name: mld_report_msgtype
 *
 * Description:
 *   Determine which type of Report to send, MLDv1 or MLDv2, depending on
 *   current state of compatibility mode flag.
 *
 ****************************************************************************/

static inline uint8_t mld_report_msgtype(FAR struct mld_group_s *group)
{
  if (IS_MLD_V1COMPAT(group->flags))
    {
      return MLD_SEND_V1REPORT;
    }
  else
    {
      return MLD_SEND_V2REPORT;
    }
}

/****************************************************************************
 * Name: mld_mrc2mrd
 *
 * Description:
 *  Convert the MLD Maximum Response Code (MRC) to the Maximum Response
 *  Delay (MRD) in units of system clock ticks.
 *
 ****************************************************************************/

#if 0 /* Not used */
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
#endif

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
 *   Check if we are still the querier for this group (assuming that we are
 *   currently the querier).  This compares the IPv6 Source Address of the
 *   query against and the IPv6 address of the link.  Ff the source address
 *   is numerically less than the link address, when we are no longer the
 *   querier.
 *
 ****************************************************************************/

static void mld_check_querier(FAR struct net_driver_s *dev,
                              FAR struct ipv6_hdr_s *ipv6,
                              FAR struct mld_group_s *group)
{
  /* Check if this member is a Querier */

  if (IS_MLD_QUERIER(group->flags))
    {
      /* This is a querier, check if the IPv6 source address is numerically
       * less than the IPv6 address assigned to this link.
       */

      if (mld_cmpaddr(dev, ipv6->srcipaddr))
        {
          /* Switch to non-Querier mode */

           CLR_MLD_QUERIER(group->flags);
        }
    }

  /* Check if the member is a Non-Querier AND that we are past the start up
   * phase (where the timer is used for a different purpose)?
   */

  if (!IS_MLD_QUERIER(group->flags) && !IS_MLD_STARTUP(group->flags))
    {
      /* Yes.. cancel the poll timer and [re-]start the 'Other Querier
       * Present' Timeout.
       */

      mld_start_polltimer(group, MSEC2TICK(MLD_OQUERY_MSEC));
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
  unsigned int mldsize;
  bool mldv1 = false;

  mldinfo("Multicast Listener Query\n");

#if 0 /* Not used */
  /* Max Response Delay.  The Max Response Code field specifies the maximum
   * allowed time before sending a responding report in units of 1/10 second.
   */

  mrc = NTOHS(query->mrc);
#endif

  /* The MLD version of a Multicast Listener Query message is determined
   * as follows:
   *
   *   MLDv1 Query: length = 24 octets
   *   MLDv2 Query: length >= 28 octets
   *
   * Query messages that do not match any of the above conditions (e.g., a
   * Query of length 26 octets) MUST be silently ignored.
   */

  mldsize = (unsigned int)ipv6->len[0] << 8 | ipv6->len[1];
  if (mldsize == sizeof(struct mld_mcast_listen_report_v1_s))
    {
      mldv1 = true;
    }
  else if (mldsize < SIZEOF_MLD_MCAST_LISTEN_QUERY_S(0))
    {
      mldinfo("WARNING:  Invalid size for MLD query: %u\n", mldsize);

      dev->d_len = 0;
      return -EINVAL;
    }

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
   *
   * Another possibility is a Unicast query that is sent specifically
   * to our local IP address.
   */

  /* Check the destination address.  This varies with the type of message
   * being sent:
   *
   *   MESSAGE                 DESTINATION ADDRESS
   *   General Query Message:  The link-local, all nodes multicast address
   *   MAS Query Messages:     The group multicast address
   */

  /* Check for a General Query */

  if (net_ipv6addr_cmp(ipv6->destipaddr, g_ipv6_allnodes) &&
      net_ipv6addr_cmp(query->grpaddr, g_ipv6_unspecaddr) &&
      query->nsources == 0)
    {
      FAR struct mld_group_s *member;
      bool rptsent = false;

      /* This is the general query */

      mldinfo("General multicast query\n");
      MLD_STATINCR(g_netstats.mld.gm_query_received);

      /* Two passes through the member list.  On the first, just check if we
       * are still the querier for the qroup.
       */

      for (member = (FAR struct mld_group_s *)dev->d_mld_grplist.head;
           member;
           member = member->next)
        {
          /* Skip over the all systems group entry */

          if (!net_ipv6addr_cmp(member->grpaddr, g_ipv6_allnodes))
            {
              /* Check if we are still the querier for this group */

               mld_check_querier(dev, ipv6, member);

              /* Warn if we received a MLDv2 query in MLDv1 compatibility
               * mode.
               */

              if (!mldv1 && IS_MLD_V1COMPAT(member->flags))
                {
                  mldinfo("WARNING: MLDv2 query received in MLDv1 "
                          "compatibility mode\n");
                }

#ifdef CONFIG_NET_MLD_ROUTER
              /* Save the number of members that reported in the previous
               * query cycle; reset the number of members that have
               * reported in the new query cycle.
               */

              member->lstmbrs = member->members;
              member->members = 0;
#endif
            }
        }

      /* On the second time through, we send the Report in response to the
       * query.  This has to be done twice because because there is only
       * a single packet buffer that is used for both incoming and outgoing
       * packets.  When the report is sent, it will clobber the incoming
       * query.  Any attempt to send an additional Report would also clobber
       * a preceding report
       *
       * REVISIT:  This is a design flaw:  Only a single report can be sent
       * in this context because there is no mechanism to preserve the
       * incoming request nor to queue multiple outgoing reports.
       */

      for (member = (FAR struct mld_group_s *)dev->d_mld_grplist.head;
           member;
           member = member->next)
        {
          /* Skip over the all systems group entry */

          if (!net_ipv6addr_cmp(member->grpaddr, g_ipv6_allnodes))
            {

              /* Check MLDv1 compatibility mode */

              mld_setup_v1compat(member, query, mldv1);

              /* Send one report and break out of the loop */

              mld_send(dev, member, mld_report_msgtype(member));
              rptsent = true;
              break;
            }
        }

      /* Need to set d_len to zero if nothing is being sent */

      if (!rptsent)
        {
          dev->d_len = 0;
        }

      return OK;
    }

  /* Find the group associated with this group address.  For the purpose of
   * sending reports, we only care about the query if we are a member of the
   * group.
   */

  group = mld_grpfind(dev, query->grpaddr);
  if (group != NULL)
    {
      mldinfo("We are not a member of this group\n");

      dev->d_len = 0;
      return -ENOENT;
    }

  /* Check if we are still the querier for this group */

  mld_check_querier(dev, ipv6, group);

#ifdef CONFIG_NET_MLD_ROUTER
  /* Save the number of members that reported in the previous query cycle;
   * reset the number of members that have reported in the new query cycle.
   */

  group->lstmbrs = group->members;
  group->members = 0;
#endif

  /* Warn if we received a MLDv2 query in MLDv1 compatibility mode. */

  if (!mldv1 && IS_MLD_V1COMPAT(group->flags))
    {
      mldinfo("WARNING: MLDv2 query received in MLDv1 compatibility mode\n");
    }

  /* Check for Multicast Address Specific Query */

  if (net_ipv6addr_cmp(ipv6->destipaddr, g_ipv6_allrouters))
    {
      if (query->nsources == 0)
        {
          mldinfo("Multicast Address Specific Query\n");
          MLD_STATINCR(g_netstats.mld.mas_query_received);
        }
      else
        {
          mldinfo("Multicast Address and Source Specific Query\n");
          MLD_STATINCR(g_netstats.mld.mass_query_received);
        }

      /* Check MLDv1 compatibility mode */

      mld_setup_v1compat(group, query, mldv1);

      /* Send the report */

      mld_send(dev, group,  mld_report_msgtype(group));
    }

  /* Not sent to all systems.  Check for Unicast General Query */

  else if (net_ipv6addr_cmp(ipv6->destipaddr, dev->d_ipv6addr))
    {
      mldinfo("Unicast query\n");
      MLD_STATINCR(g_netstats.mld.ucast_query_received);

      /* Check MLDv1 compatibility mode */

      mld_setup_v1compat(group, query, mldv1);

      /* Send the report */

      mld_send(dev, group,  mld_report_msgtype(group));
    }
  else
    {
      mldinfo("WARNING:  Unhandled query\n");
      MLD_STATINCR(g_netstats.mld.bad_query_received);

      /* Need to set d_len to zero to indication that nothing is being sent */

      dev->d_len = 0;
    }

  return OK;
}
