/****************************************************************************
 * include/nuttx/net/udp.h
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2007, 2009, 2012, 2014 Gregory Nutt. All rights reserved.
 * SPDX-FileCopyrightText: 2001-2003, Adam Dunkels. All rights reserved.
 * SPDX-FileContributor: Gregory Nutt <gnutt@nuttx.org>
 * SPDX-FileContributor: Adam Dunkels <adam@dunkels.com>
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
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_NET_UDP_H
#define __INCLUDE_NUTTX_NET_UDP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/ip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Header sizes */

#define UDP_HDRLEN       8                           /* Size of UDP header */

#ifdef CONFIG_NET_IPv4
#  define IPv4UDP_HDRLEN (UDP_HDRLEN + IPv4_HDRLEN)  /* Size of IPv4 + UDP headers */
#endif

#ifdef CONFIG_NET_IPv6
#  define IPv6UDP_HDRLEN (UDP_HDRLEN + IPv6_HDRLEN)  /* Size of IPv6 + UDP headers */
#endif

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* The UDP header */

struct udp_hdr_s
{
  uint16_t srcport;
  uint16_t destport;
  uint16_t udplen;
  uint16_t udpchksum;
};

/* The structure holding the UDP statistics that are gathered if
 * CONFIG_NET_STATISTICS is defined.
 */

#ifdef CONFIG_NET_STATISTICS
struct udp_stats_s
{
  net_stats_t drop;         /* Number of dropped UDP segments */
  net_stats_t recv;         /* Number of received UDP segments */
  net_stats_t sent;         /* Number of sent UDP segments */
  net_stats_t chkerr;       /* Number of UDP segments with a bad checksum */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_NET_UDP_H */
