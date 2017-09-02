/****************************************************************************
 * net/igmp/igmp_input.c
 *
 *   Copyright (C) 2010, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * The NuttX implementation of IGMP was inspired by the IGMP add-on for the
 * lwIP TCP/IP stack by Steve Reynolds:
 *
 *   Copyright (c) 2002 CITEL Technologies Ltd.
 *   All rights reserved.
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
#include <nuttx/net/igmp.h>

#include "devif/devif.h"
#include "igmp/igmp.h"
#include "utils/utils.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IGMPBUF ((struct igmp_iphdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_input
 *
 * Description:
 *   An IGMP packet has been received.
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
 * NOTE: The network must be locked.
 *
 ****************************************************************************/

void igmp_input(struct net_driver_s *dev)
{
  FAR struct igmp_group_s *group;
  in_addr_t destipaddr;
  in_addr_t grpaddr;
  unsigned int ticks;

  ninfo("IGMP message: %04x%04x\n", IGMPBUF->destipaddr[1], IGMPBUF->destipaddr[0]);

  /* Verify the message length */

  if (dev->d_len < NET_LL_HDRLEN(dev) + IPIGMP_HDRLEN)
    {
      IGMP_STATINCR(g_netstats.igmp.length_errors);
      nwarn("WARNING: Length error\n");
      return;
    }

  /* Calculate and check the IGMP checksum */

  if (net_chksum((FAR uint16_t *)&IGMPBUF->type, IGMP_HDRLEN) != 0)
    {
      IGMP_STATINCR(g_netstats.igmp.chksum_errors);
      nwarn("WARNING: Checksum error\n");
      return;
    }

  /* Find the group (or create a new one) using the incoming IP address */

  destipaddr = net_ip4addr_conv32(IGMPBUF->destipaddr);
  group = igmp_grpallocfind(dev, &destipaddr);
  if (!group)
    {
      nerr("ERROR: Failed to allocate/find group: %08x\n", destipaddr);
      return;
    }

  /* Now handle the message based on the IGMP message type */

  switch (IGMPBUF->type)
    {
      case IGMP_MEMBERSHIP_QUERY:
        /* RFC 2236, 2.2.  ax Response Time
         *  "The Max Response Time field is meaningful only in Membership Query
         *   messages, and specifies the maximum allowed time before sending a
         *   responding report in units of 1/10 second.  In all other messages,
         *   it is set to zero by the sender and ignored by receivers.
         */

        /* Check if the query was sent to all systems */

        if (net_ipv4addr_cmp(destipaddr, g_ipv4_allsystems))
          {
            /* Yes... Now check the if this this is a general or a group
             * specific query.
             *
             * RFC 2236, 2.1.  Type
             * There are two sub-types of Membership Query messages:
             * - General Query, used to learn which groups have members on an
             *   attached network.
             * - Group-Specific Query, used to learn if a particular group
             *   has any members on an attached network.
             *
             * RFC 2236, 2.4. Group Address
             *   "In a Membership Query message, the group address field is
             *    set to zero when sending a General Query, and set to the
             *    group address being queried when sending a Group-Specific
             *    Query."
             */

            if (IGMPBUF->grpaddr == 0)
              {
                FAR struct igmp_group_s *member;

                /* This is the general query */

                ninfo("General multicast query\n");
                if (IGMPBUF->maxresp == 0)
                  {
                    IGMP_STATINCR(g_netstats.igmp.v1_received);
                    IGMPBUF->maxresp = 10;

                    nwarn("WARNING: V1 not implemented\n");
                  }

                IGMP_STATINCR(g_netstats.igmp.query_received);
                for (member = (FAR struct igmp_group_s *)dev->grplist.head;
                     member;
                     member = member->next)
                  {
                    /* Skip over the all systems group entry */

                    if (!net_ipv4addr_cmp(member->grpaddr, g_ipv4_allsystems))
                      {
                        ticks = net_dsec2tick((int)IGMPBUF->maxresp);
                        if (IS_IDLEMEMBER(member->flags) ||
                            igmp_cmptimer(member, ticks))
                          {
                            igmp_startticks(member, ticks);
                            CLR_IDLEMEMBER(member->flags);
                          }
                      }
                  }
              }
            else /* if (IGMPBUF->grpaddr != 0) */
              {
                ninfo("Group-specific multicast queury\n");

                /* We first need to re-lookup the group since we used dest last time.
                 * Use the incoming IPaddress!
                 */

                IGMP_STATINCR(g_netstats.igmp.ucast_query);
                grpaddr = net_ip4addr_conv32(IGMPBUF->grpaddr);
                group   = igmp_grpallocfind(dev, &grpaddr);
                ticks   = net_dsec2tick((int)IGMPBUF->maxresp);
                if (IS_IDLEMEMBER(group->flags) || igmp_cmptimer(group, ticks))
                  {
                    igmp_startticks(group, ticks);
                    CLR_IDLEMEMBER(group->flags);
                  }
              }
          }

        /* Not sent to all systems -- Unicast query */

        else if (group->grpaddr != 0)
          {
            ninfo("Unicast query\n");
            IGMP_STATINCR(g_netstats.igmp.ucast_query);

            ninfo("Query to a specific group with the group address as destination\n");

            ticks = net_dsec2tick((int)IGMPBUF->maxresp);
            if (IS_IDLEMEMBER(group->flags) || igmp_cmptimer(group, ticks))
              {
                igmp_startticks(group, ticks);
                CLR_IDLEMEMBER(group->flags);
              }
          }
        break;

      case IGMPv2_MEMBERSHIP_REPORT:
        {
          ninfo("Membership report\n");

          IGMP_STATINCR(g_netstats.igmp.report_received);
          if (!IS_IDLEMEMBER(group->flags))
            {
              /* This is on a specific group we have already looked up */

              wd_cancel(group->wdog);
              SET_IDLEMEMBER(group->flags);
              CLR_LASTREPORT(group->flags);
            }
        }
      break;

      default:
        {
          nwarn("WARNING: Unexpected msg %02x\n", IGMPBUF->type);
        }
      break;
    }
}

#endif /* CONFIG_NET_IGMP */
