/****************************************************************************
 * net/icmpv6/icmpv6_reply.c
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

#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>

#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/icmpv6.h>

#include "utils/utils.h"
#include "netdev/netdev.h"
#include "devif/devif.h"
#include "inet/inet.h"
#include "icmpv6/icmpv6.h"

#ifdef CONFIG_NET_ICMPv6

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The latest drafts declared increase in minimal mtu up to 1280. */

#define ICMPv6_MINMTULEN  1280

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmp_reply
 *
 * Description:
 *   Send an ICMPv6 message in response to a situation
 *   RFC 1122: 3.2.2 MUST send at least the IP header and 8 bytes of header.
 *       MAY send more (we do).
 *       MUST NOT change this header information.
 *       MUST NOT reply to a multicast/broadcast IP address.
 *       MUST NOT reply to a multicast/broadcast MAC address.
 *       MUST reply to only the first fragment.
 *
 * Input Parameters:
 *   dev   - The device driver structure containing the received packet
 *   type  - ICMPv6 Message Type, eg. ICMPv6_DEST_UNREACHABLE
 *   code  - ICMPv6 Message Code, eg. ICMPv6_PORT_UNREACH
 *   data  - Additional 32-bit parameter in the ICMPv6 header
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void icmpv6_reply(FAR struct net_driver_s *dev, int type, int code, int data)
{
  int ipicmplen = IPv6_HDRLEN + sizeof(struct icmpv6_hdr_s);
  FAR struct ipv6_hdr_s *ipv6 = IPv6BUF;
  FAR struct icmpv6_hdr_s *icmpv6;
  uint16_t datalen;

  if (net_ipv6addr_cmp(ipv6->destipaddr, g_ipv6_unspecaddr)
#  ifdef CONFIG_NET_BROADCAST
      || net_is_addr_mcast(ipv6->destipaddr)
#  endif /* CONFIG_NET_BROADCAST */
     )
    {
      dev->d_len = 0;
      return;
    }

  /* Get the data (whole original packet) size of the packet. */

  datalen = (ipv6->len[0] << 8) + ipv6->len[1] + IPv6_HDRLEN;

  /* RFC says return as much as we can without exceeding 1280 bytes. */

  if (datalen > ICMPv6_MINMTULEN - ipicmplen)
    {
      datalen = ICMPv6_MINMTULEN - ipicmplen;
      iob_trimtail(dev->d_iob, dev->d_iob->io_pktlen - datalen);
    }

  /* Save the original datagram */

  if (CONFIG_IOB_BUFSIZE >= datalen + ipicmplen +
                            CONFIG_NET_LL_GUARDSIZE)
    {
      /* Reuse current iob */

      memmove((FAR char *)ipv6 + ipicmplen, ipv6, datalen);

      /* Skip icmp header from iob */

      iob_update_pktlen(dev->d_iob, datalen + ipicmplen);
    }
  else
    {
      FAR struct iob_s *iob;

      /* Save the original datagram to iob chain */

      iob = dev->d_iob;
      dev->d_iob = NULL;

      /* Re-prepare device buffer */

      if (netdev_iob_prepare(dev, false, 0) != OK)
        {
          dev->d_len = 0;
          dev->d_iob = iob;
          netdev_iob_release(dev);
          return;
        }

      /* Copy ipv4 header to device buffer */

      if (iob_trycopyin(dev->d_iob, (FAR void *)ipv6,
                        IPv6_HDRLEN, 0, false) != IPv6_HDRLEN)
        {
          dev->d_len = 0;
          netdev_iob_release(dev);
          iob_free_chain(iob);
          return;
        }

      /* Skip icmp header from iob */

      iob_update_pktlen(dev->d_iob, dev->d_iob->io_pktlen +
                                    sizeof(struct icmpv6_hdr_s));

      /* Concat new icmp packet before original datagram */

      iob_concat(dev->d_iob, iob);

      /* IPv6 header to new iob */

      ipv6 = IPBUF(0);
    }

  dev->d_len = ipicmplen + datalen;

  ipv6_build_header(IPv6BUF, dev->d_len - IPv6_HDRLEN, IP_PROTO_ICMP6,
                    dev->d_ipv6addr, ipv6->srcipaddr, 255, 0);

  /* Initialize the ICMPv6 header */

  icmpv6          = (FAR struct icmpv6_hdr_s *)(ipv6 + 1);
  icmpv6->type    = type;
  icmpv6->code    = code;
  icmpv6->data[0] = htons(data >> 16);
  icmpv6->data[1] = htons(data & 0xffff);

  /* Calculate the ICMPv6 checksum over the ICMPv6 header and payload. */

  icmpv6->chksum = 0;
  icmpv6->chksum = ~icmpv6_chksum(dev, IPv6_HDRLEN);
  if (icmpv6->chksum == 0)
    {
      icmpv6->chksum = 0xffff;
    }

  ninfo("Outgoing ICMPv6 packet length: %d\n", dev->d_len);
}

#endif /* CONFIG_NET_ICMPv6 */
