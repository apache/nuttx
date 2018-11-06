/****************************************************************************
 * net/mld/mld_done.c
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

#include <nuttx/net/netstats.h>
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
 * Name: mld_done_v1
 *
 * Description:
 *   Called from icmpv6_input() when a Version 1 Multicast Listener Done is
 *   received.
 *
 *   When a router in Querier state receives a Done message from a link,
 *   if the Multicast Address identified in the message is present in the
 *   Querier's list of addresses having listeners on that link, the Querier
 *   periodically sends multiple Multicast-Address-Specific Queries to that
 *   multicast address.  If no Reports for the address are received from the
 *   link after the maximum response delay in the Multicast-Address-Specific
 *   Queries of the last query has passed, the routers on the link assume
 *   that the address no longer has any listeners there; the address is
 *   therefore deleted from the list and its disappearance is made known to
 *   the multicast routing component.
 *
 *   Routers in Non-Querier state MUST ignore Done messages.
 *
 ****************************************************************************/

int mld_done_v1(FAR struct net_driver_s *dev,
                FAR const struct mld_mcast_listen_done_v1_s *done)
{
#ifdef CONFIG_MLD_ROUTER
  FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
  FAR struct mld_group_s *group;

  mldinfo("Version 1 Multicast Listener Done\n");
  MLD_STATINCR(g_netstats.mld.done_received);

   /* The done message is sent to the link-local, all routers multicast
    * address. Find the group using the group address in the Done message.
    */

  group = mld_grpfind(dev, done->grpaddr);
  if (group == NULL)
    {
      /* We know nothing of this group */

      return -ENOENT;
    }

  /* Ignore the Done message is this is not a Querier */

  if (IS_MLD_QUERIER(group->flags))
    {
      /* REVISIT:  Here we just remove the group from this list immediately.
       * The RFC requires that we send  Multicast-Address-Specific Queries
       * repeatedly before doing this to assure that the listener is not
       * present.
       */

      mld_grpfree(dev, group);
    }
#else
  /* We are not a router so we can just ignore Done messages */

  mldinfo("Version 1 Multicast Listener Done\n");
  MLD_STATINCR(g_netstats.mld.done_received);
#endif

  /* Need to set d_len to zero to indication that nothing is being sent */

  dev->d_len = 0;
  return OK;
}
