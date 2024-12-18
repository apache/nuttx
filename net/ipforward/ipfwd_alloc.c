/****************************************************************************
 * net/ipforward/ipfwd_alloc.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/tcp.h>
#include <nuttx/net/udp.h>
#include <nuttx/net/icmp.h>
#include <nuttx/net/icmpv6.h>

#include "ipforward/ipforward.h"
#include "utils/utils.h"

#ifdef CONFIG_NET_IPFORWARD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
#  define L2_MAXHDRLEN IPv6_HDRLEN
#else
#  define L2_MAXHDRLEN IPv4_HDRLEN
#endif

#if defined(CONFIG_NET_TCP)
#  define L3_MAXHDRLEN TCP_HDRLEN /* Could be up to TCP_MAX_HDRLEN */
#elif defined(CONFIG_NET_UDP)
#  define L3_MAXHDRLEN UDP_HDRLEN
#elif defined(CONFIG_NET_ICMPv6)
#  define L3_MAXHDRLEN ICMPv6_HDRLEN
#elif defined(CONFIG_NET_ICMP)
#  define L3_MAXHDRLEN ICMP_HDRLEN
#endif

#define MAX_HDRLEN (L2_MAXHDRLEN + L3_MAXHDRLEN)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the state of the global forwarding structures */

NET_BUFPOOL_DECLARE(g_fwdpool, sizeof(struct forward_s),
                    CONFIG_NET_IPFORWARD_NSTRUCT,
                    CONFIG_NET_IPFORWARD_ALLOC_STRUCT,
                    CONFIG_IOB_NBUFFERS - CONFIG_IOB_THROTTLE);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipfwd_initialize
 *
 * Description:
 *   Initialize the struct forward_s allocator.
 *
 * Assumptions:
 *   Called early in system initialization.
 *
 ****************************************************************************/

void ipfwd_initialize(void)
{
  /* The IOB size must be such that the maximum L2 and L3 headers fit into
   * the contiguous memory of the first IOB in the IOB chain.
   */

  DEBUGASSERT(MAX_HDRLEN <= CONFIG_IOB_BUFSIZE);

  NET_BUFPOOL_INIT(g_fwdpool);
}

/****************************************************************************
 * Name: ipfwd_alloc
 *
 * Description:
 *   Allocate a forwarding structure by removing a pre-allocated entry from
 *   a free list.
 *
 * Assumptions:
 *   Caller holds the network lock.  Mutually excluvive access to the free
 *   list is assured by this lock.
 *
 ****************************************************************************/

FAR struct forward_s *ipfwd_alloc(void)
{
  return NET_BUFPOOL_TRYALLOC(g_fwdpool);
}

/****************************************************************************
 * Name: ipfwd_free
 *
 * Description:
 *   Free a forwarding structure by adding it to a free list.
 *
 * Assumptions:
 *   Caller holds the network lock.  Mutually excluvive access to the free
 *   list is assured by this lock.
 *
 ****************************************************************************/

void ipfwd_free(FAR struct forward_s *fwd)
{
  NET_BUFPOOL_FREE(g_fwdpool, fwd);
}

#endif /* CONFIG_NET_IPFORWARD */
