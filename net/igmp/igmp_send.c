/****************************************************************************
 * net/igmp/igmp_send.c
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
 *   msgid      - ID of message to send
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void igmp_send(FAR struct net_driver_s *dev, FAR struct igmp_group_s *group,
               FAR const in_addr_t *destipaddr, uint8_t msgid)
{
  FAR struct igmp_hdr_s *igmp;
  uint16_t iphdrlen;
  struct ipv4_opt_s opt;
  uint32_t tmp;

  ninfo("msgid: %02x destipaddr: %08x\n", msgid, (int)*destipaddr);

  /* The IGMP header immediately follows the IP header */

  iphdrlen          = IPv4_HDRLEN + RASIZE;
  igmp              = IPBUF(iphdrlen);

  /* The total length to send is the size of the IP and IGMP headers and 4
   * bytes for the ROUTER ALERT (and, eventually, the Ethernet header)
   */

  dev->d_len        = iphdrlen + IGMP_HDRLEN;

  /* Update device buffer length */

  iob_update_pktlen(dev->d_iob, dev->d_len);

  /* The total size of the data is the size of the IGMP header */

  dev->d_sndlen     = IGMP_HDRLEN;

  /* Add the router alert option to the IPv4 header (RFC 2113) */

  tmp = HTONL(IPOPT_RA);
  memcpy(opt.data, &tmp, sizeof(uint32_t));
  opt.len = sizeof(uint32_t);

  ipv4_build_header(IPv4BUF, dev->d_len, IP_PROTO_IGMP,
                    &dev->d_ipaddr, destipaddr, IGMP_TTL, &opt);

  /* Set up the IGMP message */

  igmp->type        = msgid;
  igmp->maxresp     = 0;
  net_ipv4addr_hdrcopy(igmp->grpaddr, &group->grpaddr);

  /* Calculate the IGMP checksum. */

  igmp->chksum      = 0;
  igmp->chksum      = ~igmp_chksum(&igmp->type, IGMP_HDRLEN);

  IGMP_STATINCR(g_netstats.igmp.poll_send);
  IGMP_STATINCR(g_netstats.ipv4.sent);

  ninfo("Outgoing IGMP packet length: %d\n", dev->d_len);
  igmp_dumppkt(RA, iphdrlen + IGMP_HDRLEN);
}

#endif /* CONFIG_NET_IGMP */
