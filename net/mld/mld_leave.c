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

#include "devif/devif.h"
#include "mld/mld.h"

#ifdef CONFIG_NET_MLD

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mld_leavegroup
 *
 * Description:
 *   Remove the specified group address to the group.
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

int mld_leavegroup(struct net_driver_s *dev, FAR const struct in6_addr *grpaddr)
{
  struct mld_group_s *group;

  DEBUGASSERT(dev != NULL && grpaddr != NULL);

  /* Find the entry corresponding to the address leaving the group */

  group = mld_grpfind(dev, grpaddr->s6_addr16);

  ninfo("Leaving group: %p\n", group);

  if (group != NULL)
    {
      /* Cancel the timer and discard any queued Reports.  Canceling the
       * timer will prevent any new Reports from being sent;  clearing the
       * flags will discard any pending Reports that could interfere with
       * leaving the group.
       */

      net_lock();
      wd_cancel(group->wdog);
      CLR_MLD_SCHEDMSG(group->flags);
      CLR_MLD_WAITMSG(group->flags);
      net_unlock();

      MLD_STATINCR(g_netstats.mld.leaves);

      /* Send a leave if the flag is set according to the state diagram */

      if (IS_MLD_LASTREPORT(group->flags))
        {
          ninfo("Schedule Leave Group message\n");

          MLD_STATINCR(g_netstats.mld.done_sched);
          mld_waitmsg(group, ICMPV6_MCAST_LISTEN_DONE_V1);
        }

      /* Free the group structure */

      mld_grpfree(dev, group);

      /* And remove the group address from the ethernet drivers MAC filter set */

      mld_removemcastmac(dev, grpaddr->s6_addr16);
      return OK;
    }

  /* Return ENOENT if the address is not a member of the group */

  ninfo("Return -ENOENT\n");
  return -ENOENT;
}

#endif /* CONFIG_NET_MLD */
