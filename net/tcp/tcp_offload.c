/****************************************************************************
 * net/tcp/tcp_offload.c
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

#include <nuttx/net/netdev.h>
#include <nuttx/net/ethernet.h>
#include <nuttx/mm/iob.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/tcp.h>
#include <nuttx/net/offload.h>

#include "tcp/tcp.h"
#include "netdev/netdev.h"
#include "inet/inet.h"
#include "devif/devif.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint16_t tcp_checksum(FAR struct iob_s *pkt, uint16_t type)
{
  switch (type)
    {
#ifdef CONFIG_NET_IPv4
      case ETHTYPE_IP:
        return ipv4_upperlayer_chksum(pkt, IP_PROTO_TCP);
#endif
#ifdef CONFIG_NET_IPv6
      case ETHTYPE_IP6:
        return ipv6_upperlayer_chksum(pkt, IP_PROTO_TCP, IPv6_HDRLEN);
#endif
      default:
        return 0;
    }
}

/****************************************************************************
 * Name: tcp_gso_segment
 *
 * Description:
 *   Segment the TCP packet and update the TCP headers of all segments.
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

static FAR struct iob_s *tcp_gso_segment(FAR struct iob_s *pkt,
                                         uint32_t features)
{
  FAR struct iob_s *segs = NULL;
  uint16_t ip_hdrlen = GET_IPHDRLEN(pkt);
  FAR struct tcp_hdr_s *th;
  uint32_t mss;
  uint32_t seq;

  /* check gso parameters */

  mss = PKT_GSOINFO(pkt)->gso_size;
  if (mss > pkt->io_pktlen)
    {
      return pkt;
    }

  PKT_GSOINFO(pkt)->data_offset += TCP_HDRLEN;

  /* 2. segment the packet. */

  segs = devif_pkt_segment(pkt, features);
  if (segs == NULL)
    {
      nerr("ERROR: devif_pkt_segment Failed.");
      return NULL;
    }

  /* 3.update the header of segs */

  pkt = segs;
  th = TCPHDR(pkt, ip_hdrlen);
  seq = tcp_getsequence(th->seqno);

  /* set first seg */

  PKT_GSOINFO(pkt)->is_first = 1;

  /* 3.1 pkt->len=mss */

  while (pkt)
    {
      th = TCPHDR(pkt, ip_hdrlen);

      /* fin psh check seq cwr */

      th->flags &= ~(TCP_FIN | TCP_PSH);
      tcp_setsequence(th->seqno, seq);
      if (PKT_GSOINFO(pkt)->is_first == 0)
        {
          th->flags &= ~TCP_CWR;
        }

      /* update the payload length in L3 header, used in checksum. */

      inet_update_iphdr_len(pkt, PKT_GSOINFO(segs)->type);

      th->tcpchksum = 0;
      th->tcpchksum = ~tcp_checksum(pkt, PKT_GSOINFO(segs)->type);

      /* next pkt */

      seq += pkt->io_pktlen - PKT_GSOINFO(segs)->data_offset;
      pkt = PKT_GSOINFO(pkt)->seg_list;
    }

  return segs;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcp_send_mss
 *
 * Description:
 *
 * Input Parameters:
 *   conn - The TCP connection.
 *   *size_goal  - The maximum lenght of the packet that can be sent.
 *
 * Returned Value:
 *   The current mss value of the TCP connection.
 *
 ****************************************************************************/

int tcp_send_mss(FAR struct tcp_conn_s *conn, FAR int *size_goal)
{
  int mss_now;

  mss_now = conn->mss;

  *size_goal = tcp_gso_size_goal(conn);

  return mss_now;
}

/****************************************************************************
 * Name: tcp_gso_size_goal
 *
 * Description:
 *   Calculate the maximum length of the packet that can be sent currently.
 *
 * Input Parameters:
 *   conn  - The TCP connection of interest.
 *
 * Returned Value:
 *   The maximum lenght of the packet that can be sent.
 *
 ****************************************************************************/

int tcp_gso_size_goal(FAR struct tcp_conn_s *conn)
{
  uint32_t new_size_goal;
  uint32_t size_goal;

#ifdef CONFIG_NET_TCP_CC_NEWRENO
  new_size_goal = MIN(conn->snd_wnd, conn->cwnd) >> 1;
#else
  new_size_goal = conn->snd_wnd >> 1;
#endif

  if (new_size_goal > (CONFIG_IOB_NBUFFERS * CONFIG_IOB_BUFSIZE) >> 1)
    {
      new_size_goal = (CONFIG_IOB_NBUFFERS * CONFIG_IOB_BUFSIZE) >> 1;
    }

  size_goal = conn->gso_max_segs * conn->mss;

  if (new_size_goal < size_goal || new_size_goal >= size_goal + conn->mss)
    {
      conn->gso_segs = MIN(new_size_goal / conn->mss,
                           conn->gso_max_segs);
      size_goal = conn->gso_segs * conn->mss;
    }

  return MAX(size_goal, conn->mss);
}

/****************************************************************************
 * Name: tcp_gso_segs
 *
 * Description:
 *   Calculates the number of the GSO packet that can be segmented.
 *
 * Input Parameters:
 *   conn  - The TCP connection of interest.
 *   pktlen  - The length of packet that will be sent.
 *
 * Returned Value:
 *   The number of segments.
 *
 ****************************************************************************/

int tcp_gso_segs(FAR struct tcp_conn_s *conn, int pktlen)
{
  int gso_segs;

  if (pktlen % conn->mss == 0)
    {
      gso_segs = pktlen / conn->mss;
    }
  else
    {
      gso_segs = pktlen / conn->mss + 1;
    }

  return gso_segs;
}

/****************************************************************************
 * Name: tcp_set_gso
 *
 * Description:
 *   Set the gso information of the packet to be sent,
 *   including gso_type, gso_size, gso_segs.
 *
 * Input Parameters:
 *   conn - The TCP connection.
 *   pkt  - Packet to be sent.
 *   len  - The payload length of the packet.
 *
 * Returned Value:
 *   A negative integer (ENOMEM) indicates that no memory available.
 *   A positive integer is the length of the packet to be sent.
 *
 ****************************************************************************/

void tcp_set_gso(FAR struct tcp_conn_s *conn, FAR struct iob_s *pkt,
                 size_t len)
{
  if (len > conn->mss)
    {
      if (conn->domain == PF_INET)
        {
          PKT_GSOINFO(pkt)->gso_type = PKT_GSO_TCPV4;
        }
      else if (conn->domain == PF_INET6)
        {
          PKT_GSOINFO(pkt)->gso_type = PKT_GSO_TCPV6;
        }

      PKT_GSOINFO(pkt)->gso_size = conn->mss;
      PKT_GSOINFO(pkt)->gso_segs = tcp_gso_segs(conn, len);
    }
}

/****************************************************************************
 * Name: tcp_send_gso_pkt
 *
 * Description:
 *   Pre-allocate the iobs for the TCP packets to be sent, and copy the data
 *   into the iobs, in order to reduce the data copy on GSO segmentation.
 *
 * Input Parameters:
 *   conn - The TCP connection.
 *   dev  - The netdevice structure.
 *   pkt  - Packet to be sent.
 *   sndlen  - The length of the packet.
 *   offset  - The offset of the packet in the iob structure.
 *
 * Returned Value:
 *
 *
 ****************************************************************************/

int tcp_send_gso_pkt(FAR struct tcp_conn_s *conn,
                     FAR struct net_driver_s *dev,
                     FAR struct iob_s *pkt, unsigned int sndlen,
                     unsigned int offset)
{
  FAR struct iob_s *segs;

  segs = devif_send_gso_pkt(pkt, sndlen, offset, tcpip_hdrsize(conn),
                            conn->mss);
  if (segs)
    {
      netdev_iob_replace(dev, segs);
    }
  else
    {
      return -ENOMEM;
    }

  dev->d_sndlen = sndlen;

  return dev->d_sndlen;
}

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

FAR struct iob_s *tcp4_gso_segment(FAR struct iob_s *pkt, uint32_t features)
{
  /* check gso_type */

  if (PKT_GSOINFO(pkt)->gso_type != PKT_GSO_TCPV4)
    {
      return NULL;
    }

  return tcp_gso_segment(pkt, features);
}

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

struct iob_s *tcp6_gso_segment(FAR struct iob_s *pkt, uint32_t features)
{
  /* check gso_type */

  if (PKT_GSOINFO(pkt)->gso_type != PKT_GSO_TCPV6)
    {
      return NULL;
    }

  return tcp_gso_segment(pkt, features);
}

