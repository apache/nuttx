/****************************************************************************
 * net/devif/devif_offload.c
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

#include "devif/devif.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))

/****************************************************************************
 * External Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
FAR struct iob_s *inet_gso_segment(FAR struct iob_s *pkt, uint32_t features);
#endif
#ifdef CONFIG_NET_IPv6
FAR struct iob_s *ipv6_gso_segment(FAR struct iob_s *pkt, uint32_t features);
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

FAR struct iob_s *devif_pkt_segment(FAR struct iob_s *pkt, uint32_t features)
{
  FAR struct iob_s *segs;
  FAR struct iob_s *tmp = NULL;
  FAR struct iob_s *new_pkt = NULL;
  int data_len;
  bool need_alloc = true;
  uint32_t mss;
  uint16_t payload_len;
  uint16_t size;
  int segs_num = 0;
  uint32_t header_len = PKT_GSOINFO(pkt)->data_offset;

  /* check */

  data_len = pkt->io_pktlen - header_len;

  if (PKT_GSOINFO(pkt)->is_fwd == true)
    {
      /* The MTU is saved during forwarding, and the gso_size can be
       * calculated based on the MTU value.
       */

      PKT_GSOINFO(pkt)->gso_size -= header_len;

      if (PKT_GSOINFO(pkt)->gso_size == PKT_GROINFO(pkt)->gro_size)
        {
          need_alloc = false;
        }
    }

  if (pkt->io_len == PKT_GSOINFO(pkt)->gso_size + header_len)
    {
      need_alloc = false;
    }

  mss = PKT_GSOINFO(pkt)->gso_size;
  size = mss + header_len + CONFIG_NET_LL_GUARDSIZE;

  if (data_len < mss)
    {
      return pkt;
    }

  if (CONFIG_IOB_BUFSIZE < mss)
    {
      nerr("CONFIG_IOB_BUFFERSIZE is smaller than mss.");
      return NULL;
    }

  if (MAX_GSO_SEGS < PKT_GSOINFO(pkt)->gso_segs)
    {
      nerr("MAX_GSO_SEGS is smaller than gso_segs.");
      return NULL;
    }

  /* Recalculate the gso segments. */

  PKT_GSOINFO(pkt)->gso_segs = DIV_ROUND_UP(data_len, mss);

  /* loop segment packet data and copy the header */

  for (segs = NULL; segs_num < PKT_GSOINFO(pkt)->gso_segs;
       data_len -= mss, segs_num++)
    {
      /* 1. create segment */

      if (need_alloc == false)
        {
          /* The first segment. */

          if (tmp == NULL)
            {
              tmp = pkt;
              segs = pkt;
            }
          else
            {
              new_pkt = tmp->io_flink;
            }
        }
      else
        {
          /* 1.1 malloc data as iob, alloc_iob */

          new_pkt = iob_alloc_dynamic(size);
          if (new_pkt == NULL)
            {
              nerr("ERROR: Failed to allocate an I/O buffer.");
              iob_free_chain(segs);
              return NULL;
            }

          new_pkt->io_flink = NULL;
        }

      /* need to init the iob and gso_info */

      if (new_pkt)
        {
          /* 1.2copy header */

          memcpy(new_pkt->io_data, pkt->io_data,
                 header_len + pkt->io_offset);
        }
      else
        {
          new_pkt = segs;
        }

      /* 1.3 copy data */

      payload_len = data_len > mss ? mss : data_len;
      if (need_alloc)
        {
          /* pointer to data */

          new_pkt->io_offset = header_len + pkt->io_offset;

          iob_copyout(new_pkt->io_data + new_pkt->io_offset, pkt,
                      payload_len, segs_num * mss);
        }

      /* 2. update left data_len */

      new_pkt->io_offset = pkt->io_offset;
      new_pkt->io_pktlen = header_len + payload_len;
      new_pkt->io_len = new_pkt->io_pktlen;

      if (need_alloc == true)
        {
          PKT_GSOINFO(new_pkt)->seg_list = NULL;
          new_pkt->io_flink = NULL;

          /* The first segment. */

          if (segs == NULL)
            {
              segs = new_pkt;
              tmp  = new_pkt;
              memcpy(PKT_GSOINFO(tmp), PKT_GSOINFO(pkt),
                     sizeof(struct gso_cb));
            }
        }

      if (tmp != new_pkt)
        {
          PKT_GSOINFO(tmp)->seg_list = new_pkt;
          tmp = new_pkt;
        }
    }

  /* Assumption that a packet uses a single iob structure. */

  if (need_alloc == false)
    {
      tmp = segs;
      while (tmp)
        {
          new_pkt = tmp->io_flink;
          tmp->io_flink = NULL;
          tmp = new_pkt;
        }
    }
  else
    {
      /* free the pkt */

      iob_free_chain(pkt);
    }

  return segs;
}

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

FAR struct iob_s *devif_prepare_gso_segments(uint16_t size, uint32_t count)
{
  FAR struct iob_s *segs = NULL;
  FAR struct iob_s *pkt = NULL;
  FAR struct iob_s *tmp = NULL;

  while (count--)
    {
      pkt = iob_alloc_dynamic(size + CONFIG_NET_LL_GUARDSIZE);
      if (pkt == NULL)
        {
          nerr("ERROR: Failed to allocate an I/O buffer.");
          iob_free_chain(segs);
          return NULL;
        }

      memset(PKT_GSOINFO(pkt), 0, sizeof(struct gso_cb));
      memset(PKT_GROINFO(pkt), 0, sizeof(struct gro_cb));

      if (segs)
        {
          tmp->io_flink = pkt;
          tmp = pkt;
        }
      else
        {
          segs = pkt;
          tmp = segs;
        }
    }

  return segs;
}

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
                       unsigned int hdrlen, uint16_t gso_size)
{
  FAR struct iob_s *segs = NULL;
  FAR struct iob_s *tmp = NULL;
  int size = gso_size + hdrlen;
  int num = 0;
  int segs_num = DIV_ROUND_UP(sndlen, gso_size);
  int payload_len;

  segs = devif_prepare_gso_segments(size, segs_num);

  for (tmp = segs; tmp; tmp = tmp->io_flink, num++)
    {
      tmp->io_offset = hdrlen + CONFIG_NET_LL_GUARDSIZE;
      payload_len = num < segs_num - 1 ? gso_size : sndlen - num * gso_size;

      iob_copyout(tmp->io_data + tmp->io_offset, pkt, payload_len,
                  num * gso_size + offset);

      tmp->io_len = payload_len;
    }

  if (segs)
    {
      segs->io_pktlen = sndlen;
      segs->io_offset = CONFIG_NET_LL_GUARDSIZE;
    }

  return segs;
}

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

FAR struct iob_s *devif_gso_segment(FAR struct iob_s *pkt, uint32_t features)
{
  FAR struct iob_s *segs = NULL;
  FAR struct eth_hdr_s *eth = ETHHDR(pkt);

  /* PKT_GSOINFO(pkt)->data_offset = ETH_HDRLEN; */

  PKT_GSOINFO(pkt)->type = NTOHS(eth->type);
  switch (PKT_GSOINFO(pkt)->type)
    {
#ifdef CONFIG_NET_IPv4
    case ETHTYPE_IP:
          PKT_GSOINFO(pkt)->is_ipv6 = false;
          segs = inet_gso_segment(pkt, features);
          break;
#endif
#ifdef CONFIG_NET_IPv6
    case ETHTYPE_IP6:
          PKT_GSOINFO(pkt)->is_ipv6 = true;
          segs = ipv6_gso_segment(pkt, features);
          break;
#endif
    default:
          return NULL;
    }

  return segs;
}

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

bool devif_needs_gso(FAR struct iob_s *pkt, int features)
{
  if (!(features & NETIF_F_GSO))
    {
      return false;
    }

  if (PKT_GSOINFO(pkt)->gso_size == 0)
    {
      return false;
    }

  if (PKT_GSOINFO(pkt)->gso_size > pkt->io_pktlen)
    {
      return false;
    }

  switch (PKT_GSOINFO(pkt)->gso_type)
    {
      case PKT_GSO_TCPV4:
      case PKT_GSO_TCPV6:
      case PKT_GSO_UDPV4:
      case PKT_GSO_UDPV6:
        return true;
      default:
        return false;
    }
}

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

uint16_t devif_get_max_pktsize(FAR struct net_driver_s *dev)
{
  if ((dev->d_features & NETIF_F_GSO))
    {
      return MAX(dev->d_gso_max_size, dev->d_pktsize);
    }

  return dev->d_pktsize;
}

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

void devif_gso_list_free(FAR struct iob_s *pkt)
{
  FAR struct iob_s *seg = PKT_GSOINFO(pkt)->seg_list;
  FAR struct iob_s *tmp;

  /* Free all packets except the header packet. */

  while (seg)
    {
      tmp = PKT_GSOINFO(seg)->seg_list;
      iob_free_chain(seg);
      seg = tmp;
    }

  PKT_GSOINFO(pkt)->seg_list = NULL;

  /* Clear the GSO information for the first packet. */

  memset(PKT_GSOINFO(pkt), 0, sizeof(struct gso_cb));
}

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

void netdev_enable_gso(FAR struct net_driver_s *dev)
{
  dev->d_features |= NETIF_F_GSO;
  dev->d_gso_max_size = GSO_MAX_SIZE;
  dev->d_gso_max_segs = GSO_MAX_SIZE;
}

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

void netdev_enable_lro(FAR struct net_driver_s *dev)
{
  dev->d_features |= NETIF_F_GRO_HW;
  dev->d_gro_max_size = GRO_MAX_SIZE;
  dev->d_gro_max_segs = GRO_MAX_SEGS;
}

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
 * Assumptions:
 *   The segments of the gro packet are list by iob->io_flink.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_GRO
void devif_init_pkt_groinfo(FAR struct net_driver_s *dev,
                            FAR struct iob_s *pkt, int gro_type,
                            uint16_t gro_size, uint8_t count)
{
  /* 1. pkt is gro pkt ? */

  if (PKT_GROINFO(pkt)->gro_size == 0)
    {
      return;
    }

  /* 2. dev is enable gro ? */

  if (!(dev->d_features & NETIF_F_GRO_HW))
    {
      return;
    }

  /* gso_type gso_size gso_segs */

  PKT_GROINFO(pkt)->proto      = gro_type;
  PKT_GROINFO(pkt)->gro_size   = gro_size;
  PKT_GROINFO(pkt)->segs_count = count;
}
#endif

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

#ifdef CONFIG_NET_GRO
void devif_forward_gro_pkt(FAR struct net_driver_s *dev,
                           FAR struct iob_s *pkt)
{
  /* 1. pkt is gro pkt ? */

#ifdef CONFIG_NET_GSO
  if (PKT_GROINFO(pkt)->gro_size == 0
      && pkt->io_pktlen <= devif_get_max_pktsize(dev))
    {
      return;
    }

  /* 2. dev is enable gso ? */

  if (!(dev->d_features & NETIF_F_GSO))
    {
      return;
    }

  /* 3. set the gso information (gso_type/gso_size/gso_segs). */

  PKT_GSOINFO(pkt)->gso_type = PKT_GROINFO(pkt)->proto;

  /* For the gro pkt, save the MTU and then recalculate the gso_size and
   * gso_segs.
   */

  PKT_GSOINFO(pkt)->gso_size = devif_get_mtu(dev);
  PKT_GSOINFO(pkt)->gso_segs = PKT_GROINFO(pkt)->segs_count;

  /* 4. set the forward flag for the gso seg list. */

  PKT_GSOINFO(pkt)->is_fwd = true;
#endif
}
#endif
