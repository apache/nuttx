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
#include <nuttx/net/netdev.h>
#include <nuttx/net/mld.h>

#include "devif/devif.h"
#include "mld/mld.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mld_done
 *
 * Description:
 *   Called from icmpv6_input() when a Multicast Listener Done is received.
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

int mld_done(FAR struct net_driver_s *dev,
             FAR const struct mld_mcast_listen_done_s *done)
{
  mldinfo("Multicast Listener Done\n");
  MLD_STATINCR(g_netstats.mld.done_received);

  /* The Done message is sent to the link-local, all routers multicast
   * address. We basically ignore the Done message:
   *
   * We cannot free the group if there are other members of the group.  We
   * know how many local tasks have joined the group, but we are less
   * certain of how many non-local members of the group there are.
   *
   * The RFC requires that we send  Multicast-Address-Specific Queries
   * repeatedly before removing the group to assure that the no listeners
   * are present.
   *
   * If we are the Querier, then the Query timer logic will accomplish
   * this requirement for us.  If there is another Querier on the subnet,
   * it will drive the Queries.  No Querier?  We will let the 'Other
   * Querier Present Timeout' handle that case.
   */

  /* Need to set d_len to zero to indication that nothing is being sent */

  dev->d_len = 0;
  return OK;
}
