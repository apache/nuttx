/****************************************************************************
 * net/net_delroute.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>

#include <nuttx/net/route.h>

#include "net_internal.h"

#if defined(CONFIG_NET) && defined(CONFIG_NET_ROUTE)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct route_match_s
{
  uip_ipaddr_t target;   /* The target IP address to match */
  uip_ipaddr_t netmask;  /* The network mask to match */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function: net_match
 *
 * Description:
 *   Return 1 if the route is available
 *
 * Parameters:
 *   route - The next route to examine
 *   arg   - The match values (cast to void*)
 *
 * Returned Value:
 *   0 if the entry is not a match; 1 if the entry matched and was cleared.
 *
 ****************************************************************************/

static int net_match(FAR struct net_route_s *route, FAR void *arg)
{
  FAR struct route_match_s *match = ( FAR struct route_match_s *)arg;

  /* To match, the entry has to be in use, the masked target address must
   * be the same, and the masks must be the same.
   */

  if (route->inuse &&
      uip_ipaddr_maskcmp(route->target, match->target, match->netmask) &&
      uip_ipaddr_cmp(route->target, match->netmask))
    {
      /* They match.. clear the route table entry */

      memset(route, 0, sizeof(struct net_route_s));
      return 1;
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function: net_delroute
 *
 * Description:
 *   Remove an existing route from the routing table
 *
 * Parameters:
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int net_delroute(uip_ipaddr_t target, uip_ipaddr_t netmask);
{
  struct route_match_s match;

  /* Set up the comparison structure */

  uip_ipaddr_copy(match.target, target);
  uip_ipaddr_copy(match.netmask, netmask);

  /* Then remove the entry from the routing table */

  return net_foreachroute(net_match, &match) ? OK : -ENOENT;
}

#endif /* CONFIG_NET && CONFIG_NET_SOCKOPTS && !CONFIG_DISABLE_CLOCK */
