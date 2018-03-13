/****************************************************************************
 * net/route/fileroute.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef __NET_ROUTE_FILEROUTE_H
#define __NET_ROUTE_FILEROUTE_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include "route/route.h"

#if defined(CONFIG_ROUTE_IPv4_FILEROUTE) || defined(CONFIG_ROUTE_IPv6_FILEROUTE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IPv4_ROUTE_PATH  CONFIG_ROUTE_FILEDIR "/ipv4"
#define IPv6_ROUTE_PATH  CONFIG_ROUTE_FILEDIR "/ipv6"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct file; /* Forward reference */

/****************************************************************************
 * Name: net_init_fileroute
 *
 * Description:
 *   Initialize the in-memory, RAM routing table
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in initialization so that no special protection is needed.
 *
 ****************************************************************************/

void net_init_fileroute(void);

/****************************************************************************
 * Name: net_openroute_ipv4/net_openroute_ipv6
 *
 * Description:
 *   Open the IPv4/IPv6 routing table with the specified access privileges.
 *
 * Input Parameters:
 *   oflags - Open flags
 *   filep  - Location in which to return the detached file instance.
 *
 * Returned Value:
 *   A non-negative file descriptor is returned on success.  A negated errno
 *   value is returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
int net_openroute_ipv4(int oflags, FAR struct file *filep);
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
int net_openroute_ipv6(int oflags, FAR struct file *filep);
#endif

/****************************************************************************
 * Name: net_readroute_ipv4/net_readroute_ipv6
 *
 * Description:
 *   Read one route entry from the IPv4/IPv6 routing table.
 *
 * Input Parameters:
 *   filep - Detached file instance obtained by net_openroute_ipv{4|6}[_rdonly]
 *   route - Location to return the next route read from the file
 *
 * Returned Value:
 *   The number of bytes read on success.  The special return valud of zero
 *   indiates that the endof of file was encountered (and nothing was read).
 *   A negated errno value is returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
ssize_t net_readroute_ipv4(FAR struct file *filep,
                           FAR struct net_route_ipv4_s *route);
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
ssize_t net_readroute_ipv6(FAR struct file *filep,
                           FAR struct net_route_ipv6_s *route);
#endif

/****************************************************************************
 * Name: net_writeroute_ipv4/net_writeroute_ipv6
 *
 * Description:
 *   Write one route entry to the IPv4/IPv6 routing table.
 *
 * Input Parameters:
 *   filep - Detached file instance obtained by net_openroute_ipv{4|6}[_rdonly]
 *   route - Location to return the next route read from the file
 *
 * Returned Value:
 *   The number of bytes written on success.  A negated errno value is
 *   returned on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
ssize_t net_writeroute_ipv4(FAR struct file *filep,
                            FAR const struct net_route_ipv4_s *route);
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
ssize_t net_writeroute_ipv6(FAR struct file *filep,
                            FAR const struct net_route_ipv6_s *route);
#endif

/****************************************************************************
 * Name: net_seekroute_ipv4/net_seekroute_ipv6
 *
 * Description:
 *   Seek to a specific entry entry to the IPv4/IPv6 routing table.
 *
 * Input Parameters:
 *   filep - Detached file instance obtained by net_openroute_ipv{4|6}[_rdonly]
 *   index - The index of the routing table entry to seek to.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
off_t net_seekroute_ipv4(FAR struct file *filep, unsigned int index);
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
off_t net_seekroute_ipv6(FAR struct file *filep, unsigned int index);
#endif

/****************************************************************************
 * Name: net_routesize_ipv4/net_routesize_ipv6
 *
 * Description:
 *   Return the size of a routing table in terms of the number of entries in
 *   the routing table.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   A non-negative count of the number of entries in the routing table is
 *   returned on success; a negated errno value is returned on and failure.
 *
 * Assumptions:
 *   The size of the routing table may change after this size is returned
 *   unless the routing table is locked to prevent any modification to the
 *   routing table.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
int net_routesize_ipv4(void);
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
int net_routesize_ipv6(void);
#endif

/****************************************************************************
 * Name: net_lockroute_ipv4/net_lockroute_ipv6
 *
 * Description:
 *   Lock access to the routing table.  Necessary when a routing table is
 *   being reorganized due to deletion of a route.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
int net_lockroute_ipv4(void);
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
int net_lockroute_ipv6(void);
#endif

/****************************************************************************
 * Name: net_unlockroute_ipv4/net_unlockroute_ipv6
 *
 * Description:
 *   Release the read lock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
int net_unlockroute_ipv4(void);
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
int net_unlockroute_ipv6(void);
#endif

/****************************************************************************
 * Name: net_closeroute_ipv4/net_closeroute_ipv6
 *
 * Description:
 *   Close the IPv4/IPv6 routing table.
 *
 * Input Parameters:
 *   filep - Detached file instance obtained by net_openroute_ipv{4|6}[_rdonly]
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_FILEROUTE
int net_closeroute_ipv4(FAR struct file *filep);
#endif

#ifdef CONFIG_ROUTE_IPv6_FILEROUTE
int net_closeroute_ipv6(FAR struct file *filep);
#endif

#endif /* CONFIG_ROUTE_IPv4_FILEROUTE || CONFIG_ROUTE_IPv6_FILEROUTE */
#endif /* __NET_ROUTE_FILEROUTE_H */
