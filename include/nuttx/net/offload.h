/****************************************************************************
 * include/nuttx/net/offload.h
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

#ifndef __INCLUDE_NUTTX_NET_SEG_OFFLOAD_H
#define __INCLUDE_NUTTX_NET_SEG_OFFLOAD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <nuttx/mm/iob.h>

#ifdef CONFIG_NET_SEG_OFFLOAD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TCP_CWR 0x80
#define TCP_ECE 0x40

#define MAX_GSO_SEGS 17

#define IOBDATA_OFFSET(iob, offset) ((FAR void *)(IOB_DATA(iob) + (offset)))
#define ETHHDR(iob)  ((FAR struct eth_hdr_s *) \
                   ((FAR uint8_t *)IOBDATA_OFFSET(iob, 0) - ETH_HDRLEN))

#define IPV4HDR(iob) ((FAR struct ipv4_hdr_s *)IOBDATA_OFFSET(iob, 0))
#define IPV6HDR(iob) ((FAR struct ipv6_hdr_s *)IOBDATA_OFFSET(iob, 0))

/* ip_hdrlen: IPv4_HDRLEN, IPv6_HDRLEN */

#define UDPHDR(iob, ip_hdrlen) ((FAR struct udp_hdr_s *) \
                                 IOBDATA_OFFSET(iob, ip_hdrlen))
#define TCPHDR(iob, ip_hdrlen) ((FAR struct tcp_hdr_s *) \
                                 IOBDATA_OFFSET(iob, ip_hdrlen))

#define GET_IPHDRLEN(pkt) \
  (PKT_GSOINFO(pkt)->type == ETHTYPE_IP ? IPv4_HDRLEN : IPv6_HDRLEN)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct gso_cb
{
  union
  {
    int             mac_offset;
    int             data_offset;
  };
  uint16_t          type;        /* for L3 IPv4/IPv6 */
  uint16_t          protocol;    /* for L4 TCP/UDP */
  uint32_t          csum;
  uint16_t          csum_start;

  uint8_t           is_first:1;
  uint8_t           is_ipv6:1;
  uint8_t           is_fwd:1;
  uint8_t           tx_flags;
  uint16_t          gso_size;
  uint16_t          gso_segs;
  uint32_t          gso_type;
  FAR struct iob_s *seg_list;
  uint8_t           segs_num;
};

#define PKT_GSOINFO(pkt) (&(pkt->gso_info))

enum GSO_TYPE_E
{
    PKT_GSO_TCPV4 = 1 << 0,
    PKT_GSO_TCPV6 = 1 << 1,
    PKT_GSO_UDPV4 = 1 << 2,
    PKT_GSO_UDPV6 = 1 << 3,
};

struct gro_cb
{
  union
  {
    int             mac_offset;
    int             data_offset;
  };

  uint8_t           is_first:1;
  uint8_t           is_ipv6:1;
  uint8_t           rx_flags;
  uint16_t          gro_size;
  uint8_t           segs_count;
  uint16_t          proto;        /* tcp udp */
  FAR struct iob_s *seg_list;     /* maybe use io_flink */
};

#define PKT_GROINFO(pkt) (&(pkt->gro_info))

/****************************************************************************
 * External structure Declaration
 ****************************************************************************/

struct net_driver_s; /* Forward reference */

/****************************************************************************
 * Name: devif_get_max_pktsize
 *
 * Description:
 *   Get the maximum packet size value.
 *
 * Input Parameters:
 *    dev -  The structure of the network driver.
 *
 * Returned Value:
 *   The maximum packet size.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_GSO
uint16_t devif_get_max_pktsize(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: netdev_enable_gso
 *
 * Description:
 *   Enable the GSO function for the dev.
 *   Set the features and GSO-related maximum value.
 *
 * Input Parameters:
 *    dev -  The structure of the network driver that enables the GSO
 *           function.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void netdev_enable_gso(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: devif_needs_gso
 *
 * Description:
 *   Check whether the packet needs to be segmented.
 *
 * Input Parameters:
 *    pkt  - the packet to be sent by the NIC driver.
 *    features  - the features are supported by the target dev of the packet.
 *
 * Returned Value:
 *    The false indicates that the packet does not to be divided.
 *    The true indicates that the packet needs to be divided.
 *
 ****************************************************************************/

bool devif_needs_gso(FAR struct iob_s *pkt, int features);

/****************************************************************************
 * Name: devif_gso_segment
 *
 * Description:
 *   Divide the packet into multiple packets based on the features and
 *   the GSO information of the packet.
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

FAR struct iob_s *devif_gso_segment(FAR struct iob_s *pkt,
                                    uint32_t features);

/****************************************************************************
 * Name: devif_gso_list_free
 *
 * Description:
 *   Free the packet allocated by devif_pkt_segment and
 *   clear the GSO information of the fisrt packet.
 *
 * Input Parameters:
 *    pkt  - The GSO packets includes the segments list.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void devif_gso_list_free(FAR struct iob_s *pkt);

/****************************************************************************
 * Name: devif_pkt_segment
 *
 * Description:
 *   Segment the packet to be divided, and add the header information for all
 *   the segments.
 *
 * Input Parameters:
 *   pkt  - the packet needs to be divided.
 *   features  - the features are supported by the target dev of the packet.
 *
 * Returned Value:
 *   The NULL indicates that the devision is failure.
 *   The segs is the head of the multiple packets.
 *
 ****************************************************************************/

FAR struct iob_s *devif_pkt_segment(FAR struct iob_s *pkt,
                                    uint32_t features);

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

void inet_update_iphdr_len(FAR struct iob_s *pkt, int type);

/****************************************************************************
 * Name: devif_send_gso_pkt
 *
 * Description:
 *   Allocate the iob for the GSO packet, based on the size and count.
 *
 * Input Parameters:
 *   pkt - The packet is to be sent.
 *   sndlen - The length of the packet.
 *   offset - The offset of data in the iob (packet).
 *   hdrlen - The total header length of L3 and L4 in the packet.
 *   gso_size - The size of each pachage when pre-segment.
 *
 * Returned Value:
 *   The NULL indicates that the allocation is failure.
 *   The segs is the head of the multiple iob.
 *
 ****************************************************************************/

FAR struct iob_s *devif_send_gso_pkt(FAR struct iob_s *pkt,
                       unsigned int sndlen, unsigned int offset,
                       unsigned int hdrlen, uint16_t gso_size);

/****************************************************************************
 * Name: devif_prepare_gso_segments
 *
 * Description:
 *   Allocate the iob for the GSO packet, based on the size and count.
 *
 * Input Parameters:
 *   size  - the sum of L3 header length, L4 header length, and gso_size.
 *   count  - the features are supported by the target dev of the packet.
 *
 * Returned Value:
 *   The NULL indicates that the allocation is failure.
 *   The segs is the head of the multiple iob.
 *
 ****************************************************************************/

FAR struct iob_s *devif_prepare_gso_segments(uint16_t size, uint32_t count);

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
FAR struct iob_s *inet_gso_segment(FAR struct iob_s *pkt, uint32_t features);

/****************************************************************************
 * Name: tcp4_gso_segment
 *
 * Description:
 *   Check whether the IPv4 TCP packet can be segmented.
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

FAR struct iob_s *tcp4_gso_segment(FAR struct iob_s *pkt, uint32_t features);
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
FAR struct iob_s *ipv6_gso_segment(FAR struct iob_s *pkt, uint32_t features);

/****************************************************************************
 * Name: tcp6_gso_segment
 *
 * Description:
 *   Check whether the IPv6 TCP packet can be segmented.
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

struct iob_s *tcp6_gso_segment(FAR struct iob_s *pkt, uint32_t features);
#endif

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
FAR struct iob_s *udp4_gso_segment(FAR struct iob_s *pkt, uint32_t features);
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
FAR struct iob_s *udp6_gso_segment(FAR struct iob_s *pkt, uint32_t features);
#endif

#endif /* CONFIG_NET_GSO */

/****************************************************************************
 * Name: devif_init_pkt_groinfo
 *
 * Description:
 *  Initialize the GRO information for the GRO packet.
 *
 * Input Parameters:
 *   dev  -  The structure of the network driver that received the packet.
 *   pkt  - The received packet is GRO packet.
 *   gro_type  - That indicates the GRO type of the received packet.
 *   gro_size  - That indicates the GRO Size of the received packet.
 *   count  - That indicates the number of segments.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NET_GRO
void devif_init_pkt_groinfo(FAR struct net_driver_s *dev,
                            FAR struct iob_s *pkt, int gro_type,
                            uint16_t gro_size, uint8_t count);

/****************************************************************************
 * Name: devif_forward_gro_pkt
 *
 * Description:
 *   Initialize the GRO information for the GRO packet.
 *
 * Input Parameters:
 *   dev - An initialized instance of the common forwarding structure that
 *         includes everything needed to perform the forwarding operation.
 *   pkt - The packets to be forwarded.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The gro packets are list by iob->io_flink.
 *
 ****************************************************************************/

void devif_forward_gro_pkt(FAR struct net_driver_s *dev,
                           FAR struct iob_s *pkt);

/****************************************************************************
 * Name: netdev_enable_lro
 *
 * Description:
 *   Enable the GRO_HW function for the dev.
 *   Set the features and GRO-related maximum value.
 *
 * Input Parameters:
 *   dev -  The structure of the network driver that enables the LRO
 *          function.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void netdev_enable_lro(FAR struct net_driver_s *dev);
#endif

#endif /* CONFIG_NET_SEG_OFFLOAD */

#endif /* __INCLUDE_NUTTX_NET_SEG_OFFLOAD_H */
