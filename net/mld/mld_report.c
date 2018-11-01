/****************************************************************************
 * net/mld/mld_report.c
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
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/mld.h>

#include "devif/devif.h"
#include "mld/mld.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv6BUF  ((FAR struct ipv6_hdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

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
 ****************************************************************************/

int mld_report_v1(FAR struct net_driver_s *dev,
                  FAR const struct mld_mcast_listen_report_v1_s *report)
{
  FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
  FAR struct mld_group_s *group;

  ninfo("Version 1 Multicast Listener Report\n");
  ninfo("destipaddr: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        ipv6->destipaddr[0], ipv6->destipaddr[1], ipv6->destipaddr[2],
        ipv6->destipaddr[3], ipv6->destipaddr[4], ipv6->destipaddr[5],
        ipv6->destipaddr[6], ipv6->destipaddr[7]);

  MLD_STATINCR(g_netstats.mld.v1report_received);

  /* Find the group (or create a new one) using the incoming IP address */

  group = mld_grpallocfind(dev, ipv6->destipaddr);
  if (group == NULL)
    {
      nerr("ERROR: Failed to allocate/find group\n");
      return -ENOENT;
    }

  if (!IS_MLD_IDLEMEMBER(group->flags))
    {
      /* This is on a specific group we have already looked up */

      wd_cancel(group->wdog);
      SET_MLD_IDLEMEMBER(group->flags);
      CLR_MLD_LASTREPORT(group->flags);
    }

   return OK;
}

/****************************************************************************
 * Name: mld_report_v2
 *
 * Description:
 *  Called from icmpv6_input() when a Version 2 Multicast Listener Report is
 *   received.
 *
 ****************************************************************************/

int mld_report_v2(FAR struct net_driver_s *dev,
                  FAR const struct mld_mcast_listen_report_v2_s *report)
{
  FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
  FAR struct mld_group_s *group;

  ninfo("Version 2 Multicast Listener Report\n");
  ninfo("destipaddr: %04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x\n",
        ipv6->destipaddr[0], ipv6->destipaddr[1], ipv6->destipaddr[2],
        ipv6->destipaddr[3], ipv6->destipaddr[4], ipv6->destipaddr[5],
        ipv6->destipaddr[6], ipv6->destipaddr[7]);

  MLD_STATINCR(g_netstats.mld.v2report_received);

   /* Find the group (or create a new one) using the incoming IP address */

  group = mld_grpallocfind(dev, ipv6->destipaddr);
  if (group == NULL)
    {
      nerr("ERROR: Failed to allocate/find group\n");
      return -ENOENT;
    }

  if (!IS_MLD_IDLEMEMBER(group->flags))
    {
      /* This is on a specific group we have already looked up */

      wd_cancel(group->wdog);
      SET_MLD_IDLEMEMBER(group->flags);
      CLR_MLD_LASTREPORT(group->flags);
    }

   return OK;
}
