/****************************************************************************
 * net/icmp/icmp_reply.c
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
#include <nuttx/net/icmp.h>

#include "utils/utils.h"
#include "netdev/netdev.h"
#include "devif/devif.h"
#include "inet/inet.h"
#include "icmp/icmp.h"

#ifdef CONFIG_NET_ICMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RFC 1812:
 * 4.3.2.3, Original Message Header
 * ...
 * The ICMP datagram SHOULD contain as much of the original datagram as
 * possible without the length of the ICMP datagram exceeding 576 bytes.
 * ...
 */

#define ICMP_MAXMSGLEN  576

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmp_reply
 *
 * Description:
 *   Send an ICMP message in response to a situation
 *   RFC 1122: 3.2.2 MUST send at least the IP header and 8 bytes of header.
 *       MAY send more (we do).
 *       MUST NOT change this header information.
 *       MUST NOT reply to a multicast/broadcast IP address.
 *       MUST NOT reply to a multicast/broadcast MAC address.
 *       MUST reply to only the first fragment.
 *
 * Input Parameters:
 *   dev   - The device driver structure containing the received packet
 *   type  - ICMP Message Type, eg. ICMP_DEST_UNREACHABLE
 *   code  - ICMP Message Code, eg. ICMP_PORT_UNREACH
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void icmp_reply(FAR struct net_driver_s *dev, int type, int code)
{
  int ipicmplen = IPv4_HDRLEN + sizeof(struct icmp_hdr_s);
  FAR struct ipv4_hdr_s *ipv4 = IPv4BUF;
  FAR struct icmp_hdr_s *icmp;
  uint16_t datalen;
#ifdef CONFIG_NET_BROADCAST
  const in_addr_t bcast = INADDR_BROADCAST;
#endif /* CONFIG_NET_BROADCAST */
  const in_addr_t any = INADDR_ANY;

  if (net_ipv4addr_hdrcmp(ipv4->destipaddr, &any)
#  ifdef CONFIG_NET_BROADCAST
      || net_ipv4addr_hdrcmp(ipv4->destipaddr, &bcast)
      || net_ipv4addr_broadcast(net_ip4addr_conv32(ipv4->destipaddr),
                                dev->d_netmask)
#  endif /* CONFIG_NET_BROADCAST */
     )
    {
      dev->d_len = 0;
      return;
    }

  /* Get the data size of the packet. */

  datalen = (ipv4->len[0] << 8) + ipv4->len[1];

  /* RFC says return as much as we can without exceeding 576 bytes. */

  if (datalen > ICMP_MAXMSGLEN - ipicmplen)
    {
      datalen = ICMP_MAXMSGLEN - ipicmplen;
      iob_trimtail(dev->d_iob, dev->d_iob->io_pktlen - datalen);
    }

  /* Save the original datagram */

  if (CONFIG_IOB_BUFSIZE >= datalen + ipicmplen +
                           CONFIG_NET_LL_GUARDSIZE)
    {
      /* Reuse current iob */

      memmove((FAR char *)ipv4 + ipicmplen, ipv4, datalen);

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

      if (iob_trycopyin(dev->d_iob, (FAR void *)ipv4,
                        IPv4_HDRLEN, 0, false) != IPv4_HDRLEN)
        {
          dev->d_len = 0;
          netdev_iob_release(dev);
          iob_free_chain(iob);
          return;
        }

      /* Skip icmp header from iob */

      iob_update_pktlen(dev->d_iob, dev->d_iob->io_pktlen +
                                    sizeof(struct icmp_hdr_s));

      /* Concat new icmp packet before original datagram */

      iob_concat(dev->d_iob, iob);

      /* IPv4 header to new iob */

      ipv4 = IPBUF(0);
    }

  dev->d_len = ipicmplen + datalen;

  ipv4_build_header(IPv4BUF, dev->d_len, IP_PROTO_ICMP,
                    &dev->d_ipaddr, (FAR in_addr_t *)ipv4->srcipaddr,
                    IP_TTL_DEFAULT, 0, NULL);

  /* Initialize the ICMP header */

  icmp              = (FAR struct icmp_hdr_s *)(ipv4 + 1);
  icmp->type        = type;
  icmp->icode       = code;
  icmp->data[0]     = 0;
  icmp->data[1]     = 0;

  /* Calculate the ICMP checksum. */

  icmp->icmpchksum  = 0;
  icmp->icmpchksum  = ~icmp_chksum_iob(dev->d_iob);
  if (icmp->icmpchksum == 0)
    {
      icmp->icmpchksum = 0xffff;
    }

  ninfo("Outgoing ICMP packet length: %d\n", dev->d_len);
}

#endif /* CONFIG_NET_ICMP */
