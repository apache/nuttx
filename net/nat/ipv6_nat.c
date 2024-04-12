/****************************************************************************
 * net/nat/ipv6_nat.c
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
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/net/icmpv6.h>
#include <nuttx/net/tcp.h>
#include <nuttx/net/udp.h>

#include "nat/nat.h"
#include "utils/utils.h"

#ifdef CONFIG_NET_NAT66

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR ipv6_nat_entry_t *
ipv6_nat_inbound_internal(FAR struct ipv6_hdr_s *ipv6,
                          enum nat_manip_type_e manip_type);

static FAR ipv6_nat_entry_t *
ipv6_nat_outbound_internal(FAR struct net_driver_s *dev,
                           FAR struct ipv6_hdr_s *ipv6,
                           enum nat_manip_type_e manip_type);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6_nat_ip_adjust
 *
 * Description:
 *   Adjust address and checksum for network packet.
 *
 * Input Parameters:
 *   l4chksum   - Points to the L4 checksum to adjust, NULL for not adjust.
 *   old_ip     - The IP to be set.
 *   new_ip     - The IP to set into header.
 *
 ****************************************************************************/

static void ipv6_nat_ip_adjust(FAR uint16_t *l4chksum, FAR uint16_t *old_ip,
                               net_ipv6addr_t new_ip)
{
  /* TODO: Maybe we can accelerate the checksum adjustment by pre-calculate a
   * difference of checksum, and apply it to each packet, instead of calling
   * chksum_adjust each time.
   */

  if (l4chksum != NULL)
    {
      nat_chksum_adjust(l4chksum, old_ip, new_ip, sizeof(net_ipv6addr_t));
    }

  net_ipv6addr_hdrcopy(old_ip, new_ip);
}

/****************************************************************************
 * Name: ipv6_nat_port_adjust
 *
 * Description:
 *   Adjust port and checksum for network packet.
 *
 * Input Parameters:
 *   l4chksum - Points to the L4 checksum to adjust, NULL for not adjust.
 *   old_port - The port to be set.
 *   new_port - The port to set into header.
 *
 ****************************************************************************/

static void ipv6_nat_port_adjust(FAR uint16_t *l4chksum,
                                 FAR uint16_t *old_port, uint16_t new_port)
{
  if (l4chksum != NULL)
    {
      nat_chksum_adjust(l4chksum, old_port, &new_port, sizeof(new_port));
    }

  *old_port = new_port;
}

/****************************************************************************
 * Name: ipv6_nat_inbound_tcp
 *
 * Description:
 *   Check if a received TCP packet belongs to a NAT entry. If so, translate
 *   the external IP/Port to local IP/Port.
 *
 * Input Parameters:
 *   ipv6       - Points to the IPv6 header to translate.
 *   tcp        - Points to the TCP header to translate.
 *   manip_type - Whether external IP/Port is in source or destination.
 *
 * Returned Value:
 *   The corresponding NAT entry of the packet.
 *
 * Assumptions:
 *   Packet is received on NAT device and is targeting at the address
 *   assigned to the device.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static FAR ipv6_nat_entry_t *
ipv6_nat_inbound_tcp(FAR struct ipv6_hdr_s *ipv6, FAR struct tcp_hdr_s *tcp,
                     enum nat_manip_type_e manip_type)
{
  FAR uint16_t         *external_ip   = MANIP_IPADDR(ipv6, manip_type);
  FAR uint16_t         *external_port = MANIP_PORT(tcp, manip_type);
  FAR uint16_t         *peer_ip       = PEER_IPADDR(ipv6, manip_type);
  FAR uint16_t         *peer_port     = PEER_PORT(tcp, manip_type);
  FAR ipv6_nat_entry_t *entry         =
                 ipv6_nat_inbound_entry_find(IP_PROTO_TCP,
                                             external_ip, *external_port,
                                             peer_ip, *peer_port, true);
  if (!entry)
    {
      return NULL;
    }

  /* Note: Field tcpchksum is not guaranteed exists in TCP header inside
   * ICMPv6 Error MSG, but we manually guarantee that it is inside valid
   * address (IOB >= IP + ICMPv6 + IP + TCP), so we can update it safely.
   */

  ipv6_nat_port_adjust(&tcp->tcpchksum, external_port, entry->local_port);
  ipv6_nat_ip_adjust(&tcp->tcpchksum, external_ip, entry->local_ip);

  return entry;
}
#endif

/****************************************************************************
 * Name: ipv6_nat_inbound_udp
 *
 * Description:
 *   Check if a received UDP packet belongs to a NAT entry. If so, translate
 *   the external IP/Port to local IP/Port.
 *
 * Input Parameters:
 *   ipv6       - Points to the IPv6 header to translate.
 *   udp        - Points to the UDP header to translate.
 *   manip_type - Whether external IP/Port is in source or destination.
 *
 * Returned Value:
 *   The corresponding NAT entry of the packet.
 *
 * Assumptions:
 *   Packet is received on NAT device and is targeting at the address
 *   assigned to the device.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
static FAR ipv6_nat_entry_t *
ipv6_nat_inbound_udp(FAR struct ipv6_hdr_s *ipv6, FAR struct udp_hdr_s *udp,
                     enum nat_manip_type_e manip_type)
{
  FAR uint16_t         *external_ip   = MANIP_IPADDR(ipv6, manip_type);
  FAR uint16_t         *external_port = MANIP_PORT(udp, manip_type);
  FAR uint16_t         *peer_ip       = PEER_IPADDR(ipv6, manip_type);
  FAR uint16_t         *peer_port     = PEER_PORT(udp, manip_type);
  FAR uint16_t         *udpchksum;
  FAR ipv6_nat_entry_t *entry         =
                 ipv6_nat_inbound_entry_find(IP_PROTO_UDP,
                                             external_ip, *external_port,
                                             peer_ip, *peer_port, true);

  if (!entry)
    {
      return NULL;
    }

  /* UDP checksum has special case 0 (no checksum) */

  udpchksum = udp->udpchksum != 0 ? &udp->udpchksum : NULL;

  ipv6_nat_port_adjust(udpchksum, external_port, entry->local_port);
  ipv6_nat_ip_adjust(udpchksum, external_ip, entry->local_ip);

  return entry;
}
#endif

/****************************************************************************
 * Name: ipv6_nat_inbound_icmpv6
 *
 * Description:
 *   Check if a received ICMPv6 packet belongs to a NAT entry. If so,
 *   translate the external IP/ID to local IP/ID.
 *
 * Input Parameters:
 *   ipv6       - Points to the IPv6 header to translate.
 *   icmpv6     - Points to the ICMPv6 header to translate.
 *   manip_type - Whether external IP is in source or destination.
 *
 * Returned Value:
 *   The corresponding NAT entry of the packet.
 *
 * Assumptions:
 *   Packet is received on g_dev and is targeting at the address assigned to
 *   g_dev.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
static FAR ipv6_nat_entry_t *
ipv6_nat_inbound_icmpv6(FAR struct ipv6_hdr_s *ipv6,
                        FAR struct icmpv6_hdr_s *icmpv6,
                        enum nat_manip_type_e manip_type)
{
  FAR uint16_t         *external_ip = MANIP_IPADDR(ipv6, manip_type);
  FAR uint16_t         *peer_ip     = PEER_IPADDR(ipv6, manip_type);
  FAR ipv6_nat_entry_t *entry;

  switch (icmpv6->type)
    {
      case ICMPv6_ECHO_REQUEST:
      case ICMPv6_ECHO_REPLY:
        entry = ipv6_nat_inbound_entry_find(IP_PROTO_ICMP6,
                                            external_ip, icmpv6->data[0],
                                            peer_ip, icmpv6->data[0], true);
        if (!entry)
          {
            return NULL;
          }

        ipv6_nat_port_adjust(&icmpv6->chksum,
                             &icmpv6->data[0], entry->local_port);
        ipv6_nat_ip_adjust(&icmpv6->chksum, external_ip, entry->local_ip);
        return entry;

      case ICMPv6_DEST_UNREACHABLE:
      case ICMPv6_PACKET_TOO_BIG:
      case ICMPv6_PACKET_TIME_EXCEEDED:
      case ICMPv6_PACKET_PARAM_PROBLEM:
        /* ICMPv6 Error MSG inside another ICMPv6 Error MSG is forbidden by
         * RFC4443, Section 2.4, Page 6, so we only process the outermost
         * ICMPv6 Error MSG (manip type is DST).
         */

        if (manip_type == NAT_MANIP_DST)
          {
            /* The payload in the ICMPv6 packet is the origin packet we sent.
             * We don't need to check or backup any inner L4 data, because
             * every ICMPv6 error message (type < 128) MUST include as much
             * of the IPv6 offending (invoking) packet as possible. And the
             * inner packet will be translated by the inbound process
             * without needed to modify any outer packet checksum.
             */

            FAR struct ipv6_hdr_s *inner =
                (FAR struct ipv6_hdr_s *)(icmpv6 + 1);

            /* Find entry and translate inner. */

            entry = ipv6_nat_inbound_internal(inner, NAT_MANIP_SRC);

            if (!entry)
              {
                return NULL;
              }

            /* Adjust outer IP */

            ipv6_nat_ip_adjust(&icmpv6->chksum, external_ip,
                               entry->local_ip);

            return entry;
          }
    }

  return NULL;
}
#endif

/****************************************************************************
 * Name: ipv6_nat_outbound_tcp
 *
 * Description:
 *   Check if we want to perform NAT with this outbound TCP packet before
 *   sending it. If so, translate the local IP/Port to external IP/Port.
 *
 * Input Parameters:
 *   dev        - The device to sent the packet (to get external IP).
 *   ipv6       - Points to the IPv6 header to translate.
 *   tcp        - Points to the TCP header to translate.
 *   manip_type - Whether local IP/Port is in source or destination.
 *
 * Returned Value:
 *   The corresponding NAT entry of the packet.
 *
 * Assumptions:
 *   Packet will be sent on NAT device.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static FAR ipv6_nat_entry_t *
ipv6_nat_outbound_tcp(FAR struct net_driver_s *dev,
                      FAR struct ipv6_hdr_s *ipv6, FAR struct tcp_hdr_s *tcp,
                      enum nat_manip_type_e manip_type)
{
  FAR uint16_t         *local_ip   = MANIP_IPADDR(ipv6, manip_type);
  FAR uint16_t         *local_port = MANIP_PORT(tcp, manip_type);
  FAR uint16_t         *peer_ip    = PEER_IPADDR(ipv6, manip_type);
  FAR uint16_t         *peer_port  = PEER_PORT(tcp, manip_type);
  FAR ipv6_nat_entry_t *entry;

  /* Only create entry when it's the outermost packet (manip type is SRC). */

  entry = ipv6_nat_outbound_entry_find(dev, IP_PROTO_TCP,
              local_ip, *local_port, peer_ip, *peer_port,
              manip_type == NAT_MANIP_SRC);
  if (!entry)
    {
      return NULL;
    }

  /* Note: Field tcpchksum is not guaranteed exists in TCP header inside
   * ICMPv6 Error MSG, but we manually guarantee that it is inside valid
   * address (IOB >= IP + ICMPv6 + IP + TCP), so we can update it safely.
   */

  ipv6_nat_port_adjust(&tcp->tcpchksum, local_port, entry->external_port);
  ipv6_nat_ip_adjust(&tcp->tcpchksum, local_ip, entry->external_ip);

  return entry;
}
#endif

/****************************************************************************
 * Name: ipv6_nat_outbound_udp
 *
 * Description:
 *   Check if we want to perform NAT with this outbound UDP packet before
 *   sending it. If so, translate the local IP/Port to external IP/Port.
 *
 * Input Parameters:
 *   dev        - The device to sent the packet (to get external IP).
 *   ipv6       - Points to the IPv6 header to translate.
 *   udp        - Points to the UDP header to translate.
 *   manip_type - Whether local IP/Port is in source or destination.
 *
 * Returned Value:
 *   The corresponding NAT entry of the packet.
 *
 * Assumptions:
 *   Packet will be sent on NAT device.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
static FAR ipv6_nat_entry_t *
ipv6_nat_outbound_udp(FAR struct net_driver_s *dev,
                      FAR struct ipv6_hdr_s *ipv6, FAR struct udp_hdr_s *udp,
                      enum nat_manip_type_e manip_type)
{
  FAR uint16_t         *local_ip   = MANIP_IPADDR(ipv6, manip_type);
  FAR uint16_t         *local_port = MANIP_PORT(udp, manip_type);
  FAR uint16_t         *peer_ip    = PEER_IPADDR(ipv6, manip_type);
  FAR uint16_t         *peer_port  = PEER_PORT(udp, manip_type);
  FAR uint16_t         *udpchksum;
  FAR ipv6_nat_entry_t *entry;

  /* Only create entry when it's the outermost packet (manip type is SRC). */

  entry = ipv6_nat_outbound_entry_find(dev, IP_PROTO_UDP,
              local_ip, *local_port, peer_ip, *peer_port,
              manip_type == NAT_MANIP_SRC);
  if (!entry)
    {
      return NULL;
    }

  /* UDP checksum has special case 0 (no checksum) */

  udpchksum = udp->udpchksum != 0 ? &udp->udpchksum : NULL;

  ipv6_nat_port_adjust(udpchksum, local_port, entry->external_port);
  ipv6_nat_ip_adjust(udpchksum, local_ip, entry->external_ip);

  return entry;
}
#endif

/****************************************************************************
 * Name: ipv6_nat_outbound_icmpv6
 *
 * Description:
 *   Check if we want to perform NAT with this outbound ICMPv6 packet before
 *   sending it. If so, translate the local IP/ID to external IP/ID.
 *
 * Input Parameters:
 *   dev        - The device to sent the packet (to get external IP).
 *   ipv6       - Points to the IPv6 header to translate.
 *   icmpv6     - Points to the ICMPv6 header to translate.
 *   manip_type - Whether local IP is in source or destination.
 *
 * Returned Value:
 *   The corresponding NAT entry of the packet.
 *
 * Assumptions:
 *   Packet will be sent on NAT device.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMPv6
static FAR ipv6_nat_entry_t *
ipv6_nat_outbound_icmpv6(FAR struct net_driver_s *dev,
                         FAR struct ipv6_hdr_s *ipv6,
                         FAR struct icmpv6_hdr_s *icmpv6,
                         enum nat_manip_type_e manip_type)
{
  FAR uint16_t         *local_ip = MANIP_IPADDR(ipv6, manip_type);
  FAR uint16_t         *peer_ip  = PEER_IPADDR(ipv6, manip_type);
  FAR ipv6_nat_entry_t *entry;

  switch (icmpv6->type)
    {
      case ICMPv6_ECHO_REQUEST:
      case ICMPv6_ECHO_REPLY:

        /* Note: Only create new entry when it's the outermost packet (that
         * is, manip type is SRC).
         */

        entry = ipv6_nat_outbound_entry_find(dev, IP_PROTO_ICMP6,
                    local_ip, icmpv6->data[0], peer_ip, icmpv6->data[0],
                    manip_type == NAT_MANIP_SRC);
        if (!entry)
          {
            return NULL;
          }

        ipv6_nat_port_adjust(&icmpv6->chksum,
                             &icmpv6->data[0], entry->external_port);
        ipv6_nat_ip_adjust(&icmpv6->chksum, local_ip, entry->external_ip);
        return entry;

      case ICMPv6_DEST_UNREACHABLE:
      case ICMPv6_PACKET_TOO_BIG:
      case ICMPv6_PACKET_TIME_EXCEEDED:
      case ICMPv6_PACKET_PARAM_PROBLEM:
        /* ICMPv6 Error MSG inside another ICMPv6 Error MSG is forbidden by
         * RFC4443, Section 2.4, Page 6, so we only process the outermost
         * ICMPv6 Error MSG (manip type is DST).
         */

        if (manip_type == NAT_MANIP_SRC)
          {
            /* The payload in the ICMPv6 packet is the origin packet we got.
             * We don't need to check or backup any inner L4 data, because
             * every ICMPv6 error message (type < 128) MUST include as much
             * of the IPv6 offending (invoking) packet as possible. And the
             * inner packet will be translated by the inbound process
             * without needed to modify any outer packet checksum.
             */

            FAR struct ipv6_hdr_s *inner =
                (FAR struct ipv6_hdr_s *)(icmpv6 + 1);

            /* Find entry and translate inner. */

            entry = ipv6_nat_outbound_internal(dev, inner, NAT_MANIP_DST);

            if (!entry)
              {
                return NULL;
              }

            /* Adjust outer IP */

            ipv6_nat_ip_adjust(&icmpv6->chksum, local_ip,
                               entry->external_ip);

            return entry;
          }
    }

  return NULL;
}
#endif

/****************************************************************************
 * Name: ipv6_nat_inbound_internal
 *
 * Description:
 *   Check if a received packet belongs to a NAT entry. If so, translate
 *   the external IP/Port to local IP/Port.
 *
 * Input Parameters:
 *   ipv6       - Points to the IPv6 header to translate.
 *   manip_type - Whether external IP/Port is in source or destination.
 *
 * Returned Value:
 *   The corresponding NAT entry of the packet.
 *
 * Assumptions:
 *   Packet is received on NAT device and is targeting at the address
 *   assigned to the device.
 *
 ****************************************************************************/

static FAR ipv6_nat_entry_t *
ipv6_nat_inbound_internal(FAR struct ipv6_hdr_s *ipv6,
                          enum nat_manip_type_e manip_type)
{
  uint8_t proto;
  FAR void *l4hdr = net_ipv6_payload(ipv6, &proto);

  switch (ipv6->proto)
    {
#ifdef CONFIG_NET_TCP
      case IP_PROTO_TCP:
        return ipv6_nat_inbound_tcp(ipv6, l4hdr, manip_type);
#endif

#ifdef CONFIG_NET_UDP
      case IP_PROTO_UDP:
        return ipv6_nat_inbound_udp(ipv6, l4hdr, manip_type);
#endif

#ifdef CONFIG_NET_ICMPv6
      case IP_PROTO_ICMP6:
        return ipv6_nat_inbound_icmpv6(ipv6, l4hdr, manip_type);
#endif
    }

  return NULL;
}

/****************************************************************************
 * Name: ipv6_nat_outbound_internal
 *
 * Description:
 *   Check if we want to perform NAT with this outbound packet before
 *   sending it. If so, translate the local IP/Port to external IP/Port.
 *
 * Input Parameters:
 *   dev        - The device to sent the packet (to get external IP).
 *   ipv6       - Points to the IPv6 header to translate.
 *   manip_type - Whether local IP/Port is in source or destination.
 *
 * Returned Value:
 *   The corresponding NAT entry of the packet.
 *
 * Assumptions:
 *   Packet will be sent on NAT device.
 *
 ****************************************************************************/

static FAR ipv6_nat_entry_t *
ipv6_nat_outbound_internal(FAR struct net_driver_s *dev,
                           FAR struct ipv6_hdr_s *ipv6,
                           enum nat_manip_type_e manip_type)
{
  uint8_t proto;
  FAR void *l4hdr = net_ipv6_payload(ipv6, &proto);

  switch (proto)
    {
#ifdef CONFIG_NET_TCP
      case IP_PROTO_TCP:
        return ipv6_nat_outbound_tcp(dev, ipv6, l4hdr, manip_type);
#endif

#ifdef CONFIG_NET_UDP
      case IP_PROTO_UDP:
        return ipv6_nat_outbound_udp(dev, ipv6, l4hdr, manip_type);
#endif

#ifdef CONFIG_NET_ICMPv6
      case IP_PROTO_ICMP6:
        return ipv6_nat_outbound_icmpv6(dev, ipv6, l4hdr, manip_type);
#endif
    }

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv6_nat_inbound
 *
 * Description:
 *   Check if a received packet belongs to a NAT entry. If so, translate it.
 *
 * Input Parameters:
 *   dev   - The device on which the packet is received.
 *   ipv6  - Points to the IPv6 header with dev->d_buf.
 *
 ****************************************************************************/

void ipv6_nat_inbound(FAR struct net_driver_s *dev,
                      FAR struct ipv6_hdr_s *ipv6)
{
  /* We only process packets from NAT device and targeting at the address
   * assigned to the device.
   */

  if (IFF_IS_NAT(dev->d_flags) &&
      NETDEV_IS_MY_V6ADDR(dev, ipv6->destipaddr))
    {
      ipv6_nat_inbound_internal(ipv6, NAT_MANIP_DST);
    }
}

/****************************************************************************
 * Name: ipv6_nat_outbound
 *
 * Description:
 *   Check if we want to perform NAT with this outbound packet before sending
 *   it. If so, translate it.
 *
 * Input Parameters:
 *   dev   - The device on which the packet will be sent.
 *   ipv6  - Points to the IPv6 header to be filled into dev->d_buf later.
 *   manip_type - Whether local IP/Port is in source or destination.
 *
 * Returned Value:
 *   Zero is returned if NAT is successfully applied, or is not enabled for
 *   this packet;
 *   A negated errno value is returned if error occured.
 *
 ****************************************************************************/

int ipv6_nat_outbound(FAR struct net_driver_s *dev,
                      FAR struct ipv6_hdr_s *ipv6,
                      enum nat_manip_type_e manip_type)
{
  /* We only process packets targeting at NAT device but not targeting at the
   * address assigned to the device.
   */

  if (IFF_IS_NAT(dev->d_flags) &&
      !NETDEV_IS_MY_V6ADDR(dev, ipv6->srcipaddr) &&
      !NETDEV_IS_MY_V6ADDR(dev, ipv6->destipaddr))
    {
      FAR ipv6_nat_entry_t *entry =
          ipv6_nat_outbound_internal(dev, ipv6, manip_type);
      if (manip_type == NAT_MANIP_SRC && !entry)
        {
          /* Outbound entry creation failed, should have entry. */

          return -ENOENT;
        }
    }

  return OK;
}

#endif /* CONFIG_NET_NAT66 */
