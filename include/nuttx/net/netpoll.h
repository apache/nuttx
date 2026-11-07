/****************************************************************************
 * include/nuttx/net/netpoll.h
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

#ifndef __INCLUDE_NUTTX_NET_NETPOLL_H
#define __INCLUDE_NUTTX_NET_NETPOLL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/netdev_lowerhalf.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct netpoll_s
{
  /* Netdev lower-half */

  FAR struct netdev_lowerhalf_s *lower;

  size_t    maxlen;                  /* Maximum MTU size */
  size_t    hdrlen;                  /* Total header length */
  uint16_t  srcport;                 /* Source port number */
  uint16_t  destport;                /* Destination port number */
  in_addr_t destipaddr;              /* Destination IP address */
  uint8_t   destmac[ETHER_ADDR_LEN]; /* Ethernet destination address */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: netpoll_pkt_alloc
 *
 * Description:
 *   Allocate a TX packet for netpoll.
 *   When there are no free IOBs, this function tries to free one netpkt.
 *
 * Input Parameters:
 *   dev - The lower half device driver structure
 *
 * Returned Value:
 *   Pointer to the packet, NULL on failure
 *
 ****************************************************************************/

FAR netpkt_t *netpoll_pkt_alloc(FAR struct netdev_lowerhalf_s *dev);

/****************************************************************************
 * Name: netpoll_get_maxlen
 *
 * Description:
 *   Get maximum data length for netpoll
 *
 * Input Parameters:
 *   np - pointer to netpoll data
 *
 * Returned Value:
 *   Returns maximum data length.
 *
 ****************************************************************************/

#define netpoll_get_maxlen(np) ((np)->maxlen)

/****************************************************************************
 * Name: netpoll_pkt_copy
 *
 * Description:
 *   Copy 'len' bytes of data from a buffer into the netpkt
 *
 * Input Parameters:
 *   np     - pointer to netpoll data
 *   pkt    - packet to send
 *   src    - The source buffer
 *   len    - How many bytes to copy
 *   offset - The offset of netpkt to put the data
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

static inline_function int netpoll_pkt_copy(FAR struct netpoll_s *np,
                                            FAR netpkt_t *pkt,
                                            FAR const void *src,
                                            size_t len, int offset)
{
  return netpkt_copyin(np->lower, pkt, src, len, np->hdrlen + offset);
}

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
 *   destmac    - MAC address octets in string format: "ff:ff:ff:ff:ff:ff"
 *                Default:  "ff:ff:ff:ff:ff:ff"
 *
 * Returned Value:
 *   Returns OK on success
 *
 ****************************************************************************/

int netpoll_udp_init(FAR struct netpoll_s *np,
                     FAR struct netdev_lowerhalf_s *lower,
                     uint16_t srcport, in_addr_t destipaddr,
                     uint16_t destport, FAR const char *destmac);

/****************************************************************************
 * Name: netpoll_udp_pkt_alloc
 *
 * Description:
 *   Allocate UDP packet and prepare packet header (MAC, IPv4 and UDP).
 *
 * Input Parameters:
 *   np - pointer to netpoll data
 *
 * Returned Value:
 *   Pointer to PACKET on success; NULL on failure.
 *
 ****************************************************************************/

FAR netpkt_t *netpoll_udp_pkt_alloc(FAR struct netpoll_s *np);

/****************************************************************************
 * Name: netpoll_upd_send
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
 * Assumptions:
 *   Same as netpoll_udp_pkt_send().
 *
 ****************************************************************************/

int netpoll_udp_send(FAR struct netpoll_s *np,
                     FAR const void *buf, size_t len);

/****************************************************************************
 * Name: netpoll_udp_recv
 *
 * Description:
 *   Receive UDP packet with netpoll (blocking).
 *
 * Input Parameters:
 *   np  - pointer to netpoll data
 *   buf - receive buffer
 *   len - length of receive buffer
 *
 * Returned Value:
 *   On success, returns the number of characters received.
 *
 ****************************************************************************/

ssize_t netpoll_udp_recv(FAR struct netpoll_s *np, FAR void *buf,
                         size_t len);

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
 *   OK on success; Negated errno on failure.
 *
 * Assumptions:
 *   The caller holds netdev_lock() of the device, so the driver's
 *   transmit() is serialized with the upper half.  The only exception is
 *   the final panic path, where nothing else runs anymore.
 *
 ****************************************************************************/

int netpoll_udp_pkt_send(FAR struct netpoll_s *np, FAR netpkt_t *pkt);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_NETPOLL_H */
