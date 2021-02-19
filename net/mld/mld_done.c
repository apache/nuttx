/****************************************************************************
 * net/mld/mld_done.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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
