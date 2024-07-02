/****************************************************************************
 * net/inet/inet_offload.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/ethernet.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/tcp.h>
#include <nuttx/net/udp.h>
#include <nuttx/net/offload.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GET_IPv4ID(ipv4) (ipv4->ipid[0] | ipv4->ipid[1] << 8)
#define SET_IPv4ID(ipv4, id)    \
  do {                          \
    ipv4->ipid[0] = id >> 8;    \
    ipv4->ipid[1] = id & 0xff;  \
  } while(0)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_update_iphdr_len
 *
 * Description:
 *    Update the (ipv4/ipv6) ipheader length based on the pkt->io_pktlen.
 *
 * Input Parameters:
 *
 *   pkt  - The data iob structure.
 *   type  - The type of the packet.
 *
 ****************************************************************************/

void inet_update_iphdr_len(FAR struct iob_s *pkt, int type)
{
#ifdef CONFIG_NET_IPv4
  FAR struct ipv4_hdr_s *ip4hdr;
#endif
#ifdef CONFIG_NET_IPv6
  FAR struct ipv6_hdr_s *ip6hdr;
  int payload_len;
#endif

  /* update the payload length in L3 header, used in checksum. */

  switch (type)
  {
#ifdef CONFIG_NET_IPv4
    case ETHTYPE_IP:
      {
        ip4hdr = IPV4HDR(pkt);
        ip4hdr->len[0] = pkt->io_pktlen >> 8;
        ip4hdr->len[1] = pkt->io_pktlen & 0xff;
      }
      break;
#endif
#ifdef CONFIG_NET_IPv6
    case ETHTYPE_IP6:
      {
        payload_len = pkt->io_pktlen - IPv6_HDRLEN;
        ip6hdr = IPV6HDR(pkt);

        /* update payload length */

        ip6hdr->len[0] = payload_len >> 8;
        ip6hdr->len[1] = payload_len & 0xff;
      }
      break;
#endif
    default:
      nerr("Unknown packet type %d", type);
      break;
  }
}

/****************************************************************************
 * Name: inet_gso_segment
 *
 * Description:
 *   Check whether the packet needs to be divided. After the segmentation,
 *   and update the ipv4 header for all packets.
 *
 * Input Parameters:
 *   pkt  - the packet to be sent by the NIC driver.
 *   features  - the features are supported by the target dev of the packet.
 *
 * Returned Value:
 *   The NULL indicates that the devision is failure.
 *   The segs is the head of the multiple packets.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
FAR struct iob_s *inet_gso_segment(FAR struct iob_s *pkt, uint32_t features)
{
  FAR struct iob_s *segs = NULL;
  FAR struct ipv4_hdr_s *ip4hdr = IPV4HDR(pkt);
  uint16_t iphdrlen = (ip4hdr->vhl & IPv4_HLMASK) << 2;
  uint32_t id;
  int tot_len;

  PKT_GSOINFO(pkt)->data_offset += iphdrlen;
  switch (ip4hdr->proto)
    {
#ifdef CONFIG_NET_TCP_OFFLOAD
      case IP_PROTO_TCP:
          segs = tcp4_gso_segment(pkt, features);
          break;
#endif
#ifdef CONFIG_NET_UDP_OFFLOAD
      case IP_PROTO_UDP:
          segs = udp4_gso_segment(pkt, features);
          break;
#endif
      default:
          return NULL;
    }

  if (segs == NULL)
    {
      nerr("inet gso segment failed.");
      return NULL;
    }

  /* loop update ipv4 header for all pkt with ipid, len, chksum */

  ip4hdr = IPV4HDR(segs);
  id = GET_IPv4ID(ip4hdr);
  for (pkt = segs; pkt != NULL; pkt = PKT_GSOINFO(pkt)->seg_list)
    {
      ip4hdr = IPV4HDR(pkt);

      /* update id */

      /* the first pkt doesn't need to update id */

      SET_IPv4ID(ip4hdr, id);

      id += 1; /* PKT_GSOINFO(pkt)->gso_size; */
      tot_len = pkt->io_pktlen;
      ip4hdr->len[0] = tot_len >> 8;
      ip4hdr->len[1] = tot_len & 0xff;

      ip4hdr->ipchksum = 0;
      ip4hdr->ipchksum = ~ipv4_chksum(ip4hdr);
    }

  return segs;
}
#endif

/****************************************************************************
 * Name: ipv6_gso_segment
 *
 * Description:
 *   Check whether the packet needs to be divided. After the segmentation,
 *   and update the ipv6 header for all packets.
 *
 * Input Parameters:
 *   pkt  - the packet to be sent by the NIC driver.
 *   features  - the features are supported by the target dev of the packet.
 *
 * Returned Value:
 *   The NULL indicates that the devision is failure.
 *   The segs is the head of the multiple packets.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
FAR struct iob_s *ipv6_gso_segment(FAR struct iob_s *pkt, uint32_t features)
{
  FAR struct iob_s *segs;
  FAR struct ipv6_hdr_s *ip6hdr = IPV6HDR(pkt);
  uint16_t payload_len;

  PKT_GSOINFO(pkt)->data_offset += IPv6_HDRLEN;
  switch (ip6hdr->proto)
    {
#ifdef CONFIG_NET_TCP_OFFLOAD
      case IP_PROTO_TCP:
          segs = tcp6_gso_segment(pkt, features);
          break;
#endif
#ifdef CONFIG_NET_UDP_OFFLOAD
      case IP_PROTO_UDP:
          segs = udp6_gso_segment(pkt, features);
          break;
#endif
    default:
          return NULL;
    }

  /* update ipv6 header */

  for (pkt = segs; pkt != NULL; pkt = PKT_GSOINFO(pkt)->seg_list)
    {
      ip6hdr = IPV6HDR(pkt);

      /* update payload length */

      payload_len = pkt->io_pktlen - IPv6_HDRLEN;
      ip6hdr->len[0] = payload_len >> 8;
      ip6hdr->len[1] = payload_len & 0xff;
    }

  return segs;
}
#endif

