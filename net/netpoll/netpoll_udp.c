/****************************************************************************
 * net/netpoll/netpoll_udp.c
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

#include <netinet/ether.h>

#include <nuttx/init.h>
#include <nuttx/mutex.h>
#include <nuttx/net/netpoll.h>
#include <nuttx/net/udp.h>

#include "inet/inet.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netpoll_udp_build_header
 *
 * Description:
 *   Fill packet with UDP header
 *
 * Input Parameters:
 *   np  - pointer to netpoll data
 *   pkt - packet to send
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

static void netpoll_udp_build_header(FAR struct netpoll_s *np,
                                     FAR netpkt_t *pkt)
{
  FAR struct netdev_lowerhalf_s *lower  = np->lower;
  FAR struct eth_hdr_s          *eth;
  FAR struct ipv4_hdr_s         *ipv4;
  FAR struct udp_hdr_s          *udp;
  FAR uint8_t                   *data;
  size_t                         offset = 0;

  /* Get IO data */

  data = netpkt_getdata(lower, pkt);

  /* Fill MAC header */

  eth       = (FAR struct eth_hdr_s *)&data[offset];
  eth->type = htons(ETHTYPE_IP);
  memcpy(eth->dest, np->destmac, sizeof(eth->dest));
  memcpy(eth->src, &lower->netdev.d_mac, sizeof(eth->src));

  /* Fill IPv4 header */

  offset += ETH_HDRLEN;
  ipv4   = (FAR struct ipv4_hdr_s *)&data[offset];
  ipv4_build_header(ipv4, pkt->io_pktlen, IP_PROTO_UDP,
                    &lower->netdev.d_ipaddr, &np->destipaddr,
                    IP_TTL_DEFAULT, 0, NULL);

  /* Fill UDP header */

  offset += IPv4_HDRLEN;
  udp    = (FAR struct udp_hdr_s *)&data[offset];

  udp->srcport   = np->srcport;
  udp->destport  = np->destport;
  udp->udplen    = htons(pkt->io_pktlen - IPv4_HDRLEN);
  udp->udpchksum = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netpoll_udp_init
 *
 * Description:
 *   Initialize netpoll with UDP.
 *
 * Input Parameters:
 *   np         - pointer to netpoll data
 *   lower      - pointer to netdev lower
 *   srcport    - source port number
 *   destipaddr - destination IP address
 *   destport   - destination port number
 *   destmac    - MAC address octets in string format: "ff:ff:ff:ff:ff:ff".
 *                Default:  "ff:ff:ff:ff:ff:ff"
 *
 * Returned Value:
 *   Returns OK on success
 *
 ****************************************************************************/

int netpoll_udp_init(FAR struct netpoll_s *np,
                     FAR struct netdev_lowerhalf_s *lower,
                     uint16_t srcport, in_addr_t destipaddr,
                     uint16_t destport, FAR const char *destmac)
{
  struct ether_addr addr;

  if (lower->ops->txpoll == NULL)
    {
      return -EINVAL;
    }

  memset(np, 0, sizeof(struct netpoll_s));

  if (destmac == NULL)
    {
      destmac = "ff:ff:ff:ff:ff:ff";
    }

  /* Decode MAC */

  if (!ether_aton_r(destmac, &addr))
    {
      return -EINVAL;
    }

  memcpy(np->destmac, addr.ether_addr_octet, ETHER_ADDR_LEN);

  /* Parameters */

  np->srcport    = htons(srcport);
  np->destipaddr = htonl(destipaddr);
  np->destport   = htons(destport);

  /* Store lower half reference */

  np->lower = lower;

  /* Store header length for later */

  np->hdrlen = NET_LL_HDRLEN(&lower->netdev) + IPv4UDP_HDRLEN;

  /* Get maximum UDP packet length for this device */

  np->maxlen = NETDEV_PKTSIZE(&lower->netdev) - np->hdrlen;

  return OK;
}

/****************************************************************************
 * Name: netpoll_udp_pkt_alloc
 *
 * Description:
 *   Allocate UDP packet.
 *
 * Input Parameters:
 *   np - pointer to netpoll data
 *
 * Returned Value:
 *   Pointer to packet on success; NULL on failure.
 *
 ****************************************************************************/

FAR netpkt_t *netpoll_udp_pkt_alloc(FAR struct netpoll_s *np)
{
  FAR struct netdev_lowerhalf_s *lower = np->lower;
  FAR netpkt_t                  *pkt;

  /* Get packet */

  pkt = netpoll_pkt_alloc(lower);
  if (!pkt)
    {
      return NULL;
    }

  /* Initialize packet with header length */

  netpkt_setdatalen(lower, pkt, np->hdrlen);

  return pkt;
}

/****************************************************************************
 * Name: netpoll_udp_pkt_send
 *
 * Description:
 *   Send packet as UDP packet with netpoll.
 *
 * Input Parameters:
 *   np  - pointer to netpoll data
 *   pkt - packet to send
 *
 * Returned Value:
 *   Length of header if success, negative value on failure
 *
 * Assumption:
 *   Packet is ready to send, all packet headers are complete, only UDP
 *   payload len can change.
 *
 ****************************************************************************/

int netpoll_udp_pkt_send(FAR struct netpoll_s *np, FAR netpkt_t *pkt)
{
  FAR struct netdev_lowerhalf_s *lower = np->lower;
  int                            ret;

  /* The upper half calls transmit() with netdev_lock() held, so the
   * caller must hold it too, except on the final panic path where the
   * rest of the system is stopped.
   */

  DEBUGASSERT(OSINIT_IS_PANIC() ||
              nxrmutex_is_hold(&lower->netdev.d_lock));

  /* Don't send data if device is not ready */

  if (!IFF_IS_RUNNING(lower->netdev.d_flags))
    {
      return -ENODEV;
    }

  /* Fill header */

  netpoll_udp_build_header(np, pkt);

  /* Transmit packet */

  ret = lower->ops->transmit(lower, pkt);
  if (ret < 0)
    {
      return ret;
    }

  NETDEV_TXDONE(&lower->netdev);

  return OK;
}

/****************************************************************************
 * Name: netpoll_udp_send
 *
 * Description:
 *   Send UDP packet with netpoll.
 *
 * Input Parameters:
 *   np  - pointer to netpoll data
 *   buf - data to send
 *   len - length of data to send
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int netpoll_udp_send(FAR struct netpoll_s *np, FAR const void *buf,
                     size_t len)
{
  FAR netpkt_t *pkt;
  int           ret;

  /* Get packet */

  pkt = netpoll_udp_pkt_alloc(np);
  if (!pkt)
    {
      return -ENOMEM;
    }

  /* Fill with data */

  netpoll_pkt_copy(np, pkt, buf, len, 0);

  /* Send packet */

  ret = netpoll_udp_pkt_send(np, pkt);
  if (ret < 0)
    {
      netpkt_free(np->lower, pkt, NETPKT_TX);
    }

  return ret;
}

/****************************************************************************
 * Name: netpoll_udp_recv
 *
 * Description:
 *   Receive UDP packet for a given port and store source address.
 *   This function is blocking and returns only when a valid UDP packet
 *   is received so it must be used with caution.
 *
 *   This function doesn't use the NET stack so it can be used in
 *   an interrupt context.
 *
 * Input Parameters:
 *   np   - pointer to netpoll data
 *   buf  - receive buffer
 *   len  - length of receive buffer
 *
 * Returned Value:
 *   Returns the number of characters received.
 *
 ****************************************************************************/

ssize_t netpoll_udp_recv(FAR struct netpoll_s *np, FAR void *buf, size_t len)
{
  FAR struct netdev_lowerhalf_s *lower = np->lower;
  FAR struct eth_hdr_s          *eth;
  FAR struct ipv4_hdr_s         *ipv4;
  FAR struct udp_hdr_s          *udp;
  FAR netpkt_t                  *pkt;
  FAR uint8_t                   *data;
  in_addr_t                      addr;
  size_t                         offset;
  bool                           found = false;

  DEBUGASSERT(lower->ops->receive);

  /* Don't recv data if device is not ready */

  if (!IFF_IS_RUNNING(lower->netdev.d_flags))
    {
      return -ENODEV;
    }

  /* Loop until we get a valid UDP packet which is addressed to us.
   * At this point all other packages are dropped.
   */

  while (!found)
    {
      /* Try to get packet */

      pkt = lower->ops->receive(lower);
      if (pkt == NULL)
        {
          continue;
        }

      /* Get IO data */

      data = netpkt_getdata(lower, pkt);

      /* Get MAC header */

      offset = 0;
      eth    = (FAR struct eth_hdr_s *)&data[offset];

      if (eth->type != htons(ETHTYPE_IP))
        {
          goto errout;
        }

      /* Get IPv4 header */

      offset += ETH_HDRLEN;
      ipv4   = (FAR struct ipv4_hdr_s *)&data[offset];

      /* Check if the packet is destined for our IP address. */

      addr = net_ip4addr_conv32(ipv4->destipaddr);
      if (!net_ipv4addr_cmp(addr, lower->netdev.d_ipaddr))
        {
          goto errout;
        }

      /* Must be UDP */

      if (ipv4->proto != IP_PROTO_UDP)
        {
          goto errout;
        }

      /* Get UDP header */

      offset += IPv4_HDRLEN;
      udp    = (FAR struct udp_hdr_s *)&data[offset];

      if (udp->destport != np->srcport)
        {
          goto errout;
        }

      /* Copy data to buffer */

      len = MIN((ntohs(udp->udplen) - UDP_HDRLEN), len);
      offset += UDP_HDRLEN;
      netpkt_copyout(lower, buf, pkt, len, offset);
      found = true;

      /* Store source info */

      np->destipaddr = net_ip4addr_conv32(ipv4->srcipaddr);
      np->destport   = udp->srcport;

      /* Update destination MAC */

      memcpy(np->destmac, eth->src, sizeof(np->destmac));

errout:
      netpkt_free(lower, pkt, NETPKT_RX);
    }

  return len;
}
