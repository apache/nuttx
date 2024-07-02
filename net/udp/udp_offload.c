/****************************************************************************
 * net/udp/udp_offload.c
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
#include <assert.h>

#include <nuttx/mm/iob.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ethernet.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/udp.h>
#include <nuttx/net/offload.h>

#include "udp/udp.h"
#include "netdev/netdev.h"
#include "inet/inet.h"
#include "devif/devif.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DIV_ROUND_UP(n,d)           (((n) + (d) - 1) / (d))

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t udp_checksum(FAR struct iob_s *pkt, uint16_t type)
{
  switch (type)
    {
#ifdef CONFIG_NET_IPv4
      case ETHTYPE_IP:
        return ipv4_upperlayer_chksum(pkt, IP_PROTO_UDP);
#endif
#ifdef CONFIG_NET_IPv6
      case ETHTYPE_IP6:
        return ipv6_upperlayer_chksum(pkt, IP_PROTO_UDP, IPv6_HDRLEN);
#endif
      default:
        return 0;
    }
}

/****************************************************************************
 * Name: udp_gso_segment
 *
 * Description:
 *   Segment the UDP packet and update the UDP headers of all segments.
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

static FAR struct iob_s *udp_gso_segment(FAR struct iob_s *pkt,
                                         uint32_t features)
{
  FAR struct iob_s *segs = NULL;
  uint16_t ip_hdrlen = GET_IPHDRLEN(pkt);
  FAR struct udp_hdr_s *uh;
  uint32_t mss = PKT_GSOINFO(pkt)->gso_size;

  /* check gso parameters */

  if (mss > pkt->io_pktlen)
    {
      return pkt;
    }

  PKT_GSOINFO(pkt)->data_offset += UDP_HDRLEN;

  /* 2. segment the packet. */

  segs = devif_pkt_segment(pkt, features);
  if (segs == NULL)
    {
      nerr("ERROR: devif_pkt_segment Failed.");
      return NULL;
    }

  /* 3.update the udp header of segs */

  pkt = segs;

  /* set first seg */

  PKT_GSOINFO(pkt)->is_first = 1;

  while (pkt)
    {
      uh = UDPHDR(pkt, ip_hdrlen);

      /* update len, checksum */

      uh->udplen = HTONS(pkt->io_pktlen - ip_hdrlen);

      /* update the payload length in L3 header, used in checksum. */

      inet_update_iphdr_len(pkt, PKT_GSOINFO(segs)->type);

      uh->udpchksum = 0;
      uh->udpchksum = ~udp_checksum(pkt, PKT_GSOINFO(segs)->type);

      /* next pkt */

      pkt = PKT_GSOINFO(pkt)->seg_list;
    }

  return segs;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: udp_prepare_gso_pkt
 *
 * Description:
 *   pre-allocate the iob buffers for the GSO packets.
 *
 * Input Parameters:
 *
 *   conn    - the UDP connection structure.
 *   sndlen  - the length of data to be sent.
 *
 * Returned Value:
 *   The I/O buffers are allocated for the GSO packets.
 *
 ****************************************************************************/

FAR struct iob_s *udp_prepare_gso_pkt(FAR struct udp_conn_s *conn,
                                      unsigned int sndlen)
{
  FAR struct iob_s *segs = NULL;
  FAR struct iob_s *tmp = NULL;
  uint16_t hdrlen = udpip_hdrsize(conn);

  segs = devif_prepare_gso_segments(conn->gso_size + hdrlen,
                                    DIV_ROUND_UP(sndlen, conn->gso_size));
  if (segs == NULL)
    {
      return segs;
    }

  for (tmp = segs; tmp; tmp = tmp->io_flink)
    {
      tmp->io_offset = hdrlen + CONFIG_NET_LL_GUARDSIZE;
    }

  return segs;
}

/****************************************************************************
 * Name: udp_copy_gso_pkt
 *
 * Description:
 *  Copy data 'len' bytes from a user buffer into the I/O buffer chain,
 *  starting at 'offset', extending the chain as necessary based on gso_size.
 *
 *   segs  - The I/O buffers chain.
 *   src   - The user data.
 *   len   - The length of user data.
 *   hdrlen    - The length of L3 and L4 header.
 *   gso_size  - The size of UDP packet payload.
 *
 * Returned Value:
 *   The length of data that is copied into the segs I/O buffers.
 *
 ****************************************************************************/

int udp_copy_gso_pkt(FAR struct iob_s *segs, FAR const uint8_t *src,
                     unsigned int len, int hdrlen, int gso_size)
{
  FAR struct iob_s *tmp;
  int num = 0;
  int segs_num = DIV_ROUND_UP(len, gso_size);

  for (tmp = segs; tmp; tmp = tmp->io_flink, num++)
    {
      tmp->io_offset = hdrlen + CONFIG_NET_LL_GUARDSIZE;
      tmp->io_len = num < segs_num - 1 ? gso_size : len - num * gso_size;

      memcpy(tmp->io_data + tmp->io_offset, src, tmp->io_len);
      src += tmp->io_len;
    }

  segs->io_pktlen = len;
  segs->io_offset = CONFIG_NET_LL_GUARDSIZE;

  return len;
}

/****************************************************************************
 * Name: udp4_gso_segment
 *
 * Description:
 *   Check whether the IPv4 UDP packet can be segmented.
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
FAR struct iob_s *udp4_gso_segment(FAR struct iob_s *pkt, uint32_t features)
{
  /* check gso_type */

  if (PKT_GSOINFO(pkt)->gso_type != PKT_GSO_UDPV4)
    {
      return NULL;
    }

  return udp_gso_segment(pkt, features);
}
#endif

/****************************************************************************
 * Name: udp6_gso_segment
 *
 * Description:
 *   Check whether the IPv6 UDP packet can be segmented.
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
FAR struct iob_s *udp6_gso_segment(FAR struct iob_s *pkt, uint32_t features)
{
  /* check gso_type */

  if (PKT_GSOINFO(pkt)->gso_type != PKT_GSO_UDPV6)
    {
      return NULL;
    }

  return udp_gso_segment(pkt, features);
}
#endif

