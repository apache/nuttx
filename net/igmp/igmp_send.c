/****************************************************************************
 * net/igmp/igmp_send.c
 *
 *   Copyright (C) 2010, 2015 Gregory Nutt. All rights reserved.
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

#include <debug.h>
#include <arpa/inet.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/netstats.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/ipopt.h>
#include <nuttx/net/tcp.h>
#include <nuttx/net/igmp.h>

#include "devif/devif.h"
#include "inet/inet.h"
#include "igmp/igmp.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug */

#undef IGMP_DUMPPKT       /* Define to enable packet dump */

#ifndef CONFIG_DEBUG_NET
#  undef IGMP_DUMPPKT
#endif

#ifdef IGMP_DUMPPKT
#  define igmp_dumppkt(b,n) lib_dumpbuffer("IGMP", (FAR const uint8_t*)(b), (n))
#else
#  define igmp_dumppkt(b,n)
#endif

/* Buffer layout */

#define RASIZE      (4)
#define IGMPBUF     ((struct igmp_iphdr_s *)&dev->d_buf[NET_LL_HDRLEN(dev)])

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t igmp_chksum(FAR uint8_t *buffer, int buflen)
{
  uint16_t sum = net_chksum((FAR uint16_t *)buffer, buflen);
  return sum ? sum : 0xffff;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: igmp_send
 *
 * Description:
 *   Sends an IGMP IP packet on a network interface. This function constructs
 *   the IP header and calculates the IP header checksum.
 *
 * Input Parameters:
 *   dev        - The device driver structure to use in the send operation.
 *   group      - Describes the multicast group member and identifies the
 *                message to be sent.
 *   destipaddr - The IP address of the recipient of the message
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void igmp_send(FAR struct net_driver_s *dev, FAR struct igmp_group_s *group,
               FAR in_addr_t *destipaddr)
{
  ninfo("msgid: %02x destipaddr: %08x\n", group->msgid, (int)*destipaddr);

  /* The total length to send is the size of the IP and IGMP headers and 4
   * bytes for the ROUTER ALERT (and, eventually, the Ethernet header)
   */

  dev->d_len           = IPIGMP_HDRLEN;

  /* The total size of the data is the size of the IGMP header */

  dev->d_sndlen        = IGMP_HDRLEN;

  /* Add the router alert option */

  IGMPBUF->ra[0]       = HTONS(IPOPT_RA >> 16);
  IGMPBUF->ra[1]       = HTONS(IPOPT_RA & 0xffff);

  /* Initialize the IPv4 header */

  IGMPBUF->vhl         = 0x46;  /* 4->IP; 6->24 bytes */
  IGMPBUF->tos         = 0;
  IGMPBUF->len[0]      = (dev->d_len >> 8);
  IGMPBUF->len[1]      = (dev->d_len & 0xff);
  ++g_ipid;
  IGMPBUF->ipid[0]     = g_ipid >> 8;
  IGMPBUF->ipid[1]     = g_ipid & 0xff;
  IGMPBUF->ipoffset[0] = IP_FLAG_DONTFRAG >> 8;
  IGMPBUF->ipoffset[1] = IP_FLAG_DONTFRAG & 0xff;
  IGMPBUF->ttl         = IGMP_TTL;
  IGMPBUF->proto       = IP_PROTO_IGMP;

  net_ipv4addr_hdrcopy(IGMPBUF->srcipaddr, &dev->d_ipaddr);
  net_ipv4addr_hdrcopy(IGMPBUF->destipaddr, destipaddr);

  /* Calculate IP checksum. */

  IGMPBUF->ipchksum    = 0;
  IGMPBUF->ipchksum    = ~igmp_chksum((FAR uint8_t *)IGMPBUF, IPv4_HDRLEN + RASIZE);

  /* Set up the IGMP message */

  IGMPBUF->type        = group->msgid;
  IGMPBUF->maxresp     = 0;
  net_ipv4addr_hdrcopy(IGMPBUF->grpaddr, &group->grpaddr);

  /* Calculate the IGMP checksum. */

  IGMPBUF->chksum      = 0;
  IGMPBUF->chksum      = ~igmp_chksum(&IGMPBUF->type, IGMP_HDRLEN);

  IGMP_STATINCR(g_netstats.igmp.poll_send);
  IGMP_STATINCR(g_netstats.ipv4.sent);

  ninfo("Outgoing IGMP packet length: %d (%d)\n",
          dev->d_len, (IGMPBUF->len[0] << 8) | IGMPBUF->len[1]);
  igmp_dumppkt(RA, IPIGMP_HDRLEN + RASIZE);
}

#endif /* CONFIG_NET_IGMP */
