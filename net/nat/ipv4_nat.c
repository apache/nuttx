/****************************************************************************
 * net/nat/ipv4_nat.c
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
#include <errno.h>
#include <stdint.h>
#include <sys/types.h>

#include <nuttx/net/icmp.h>
#include <nuttx/net/tcp.h>
#include <nuttx/net/udp.h>

#include "nat/nat.h"
#include "utils/utils.h"

#if defined(CONFIG_NET_NAT) && defined(CONFIG_NET_IPv4)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Adjust checksums in headers. */

#define chksum_adjust(chksum,old_data,new_data) \
  net_chksum_adjust((FAR uint16_t *)&(chksum), \
                    (FAR uint16_t *)&(old_data), sizeof(old_data), \
                    (FAR uint16_t *)&(new_data), sizeof(new_data))

/* Getting IP & Port to manipulate from L3/L4 header. */

#define MANIP_IPADDR(iphdr,manip_type) \
  ((manip_type) == NAT_MANIP_SRC ? &(iphdr)->srcipaddr : &(iphdr)->destipaddr)

#define MANIP_PORT(l4hdr,manip_type) \
  ((manip_type) == NAT_MANIP_SRC ? &(l4hdr)->srcport : &(l4hdr)->destport)

/* Getting L4 header from IPv4 header. */

#define L4_HDR(ipv4) \
  (FAR void *)((FAR uint8_t *)(ipv4) + (((ipv4)->vhl & IPv4_HLMASK) << 2))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* NAT IP/Port manipulate type, to indicate whether to manipulate source or
 * destination IP/Port in a packet.
 */

enum nat_manip_type_e
{
  NAT_MANIP_SRC,
  NAT_MANIP_DST
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv4_nat_ip_adjust
 *
 * Description:
 *   Adjust address and checksum for network packet.
 *
 * Input Parameters:
 *   ipv4       - Points to the IPv4 header to adjust.
 *   l4chksum   - Points to the L4 checksum to adjust, NULL for not adjust.
 *   new_ip     - The IP to set into header.
 *   manip_type - Whether manipulate source ip or destination ip.
 *
 ****************************************************************************/

static void ipv4_nat_ip_adjust(FAR struct ipv4_hdr_s *ipv4,
                               FAR uint16_t *l4chksum, in_addr_t new_ip,
                               enum nat_manip_type_e manip_type)
{
  FAR uint16_t (*old_ip)[2] = MANIP_IPADDR(ipv4, manip_type);

  if (l4chksum != NULL)
    {
      chksum_adjust(*l4chksum, *old_ip, new_ip);
    }

  chksum_adjust(ipv4->ipchksum, *old_ip, new_ip);
  net_ipv4addr_hdrcopy(*old_ip, &new_ip);
}

/****************************************************************************
 * Name: ipv4_nat_port_adjust
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

static void ipv4_nat_port_adjust(FAR uint16_t *l4chksum,
                                 FAR uint16_t *old_port, uint16_t new_port)
{
  if (l4chksum != NULL)
    {
      chksum_adjust(*l4chksum, *old_port, new_port);
    }

  *old_port = new_port;
}

/****************************************************************************
 * Name: ipv4_nat_inbound_tcp
 *
 * Description:
 *   Check if a received TCP packet belongs to a NAT entry. If so, translate
 *   the external IP/Port to local IP/Port.
 *
 * Input Parameters:
 *   ipv4       - Points to the IPv4 header to translate.
 *   manip_type - Whether external IP/Port is in source or destination.
 *
 * Returned Value:
 *   Zero is returned if NAT is successfully applied, or is not enabled for
 *   this packet;
 *   A negated errno value is returned if error occured.
 *
 * Assumptions:
 *   Packet is received on NAT device and is targeting at the address
 *   assigned to the device.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static int ipv4_nat_inbound_tcp(FAR struct ipv4_hdr_s *ipv4,
                                enum nat_manip_type_e manip_type)
{
  FAR struct tcp_hdr_s *tcp = L4_HDR(ipv4);
  FAR uint16_t *external_port = MANIP_PORT(tcp, manip_type);
  FAR struct ipv4_nat_entry *entry =
      ipv4_nat_inbound_entry_find(IP_PROTO_TCP, *external_port, true);
  if (!entry)
    {
      /* Inbound without entry is OK (e.g. towards NuttX itself), skip NAT. */

      return OK;
    }

  ipv4_nat_port_adjust(&tcp->tcpchksum, external_port, entry->local_port);
  ipv4_nat_ip_adjust(ipv4, &tcp->tcpchksum, entry->local_ip, manip_type);

  return OK;
}
#endif

/****************************************************************************
 * Name: ipv4_nat_inbound_udp
 *
 * Description:
 *   Check if a received UDP packet belongs to a NAT entry. If so, translate
 *   the external IP/Port to local IP/Port.
 *
 * Input Parameters:
 *   ipv4       - Points to the IPv4 header to translate.
 *   manip_type - Whether external IP/Port is in source or destination.
 *
 * Returned Value:
 *   Zero is returned if NAT is successfully applied, or is not enabled for
 *   this packet;
 *   A negated errno value is returned if error occured.
 *
 * Assumptions:
 *   Packet is received on NAT device and is targeting at the address
 *   assigned to the device.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
static int ipv4_nat_inbound_udp(FAR struct ipv4_hdr_s *ipv4,
                                enum nat_manip_type_e manip_type)
{
  FAR struct udp_hdr_s *udp = L4_HDR(ipv4);
  FAR uint16_t *external_port = MANIP_PORT(udp, manip_type);
  FAR uint16_t *udpchksum;
  FAR struct ipv4_nat_entry *entry =
      ipv4_nat_inbound_entry_find(IP_PROTO_UDP, *external_port, true);
  if (!entry)
    {
      /* Inbound without entry is OK (e.g. towards NuttX itself), skip NAT. */

      return OK;
    }

  /* UDP checksum has special case 0 (no checksum) */

  udpchksum = udp->udpchksum != 0 ? &udp->udpchksum : NULL;

  ipv4_nat_port_adjust(udpchksum, external_port, entry->local_port);
  ipv4_nat_ip_adjust(ipv4, udpchksum, entry->local_ip, manip_type);

  return OK;
}
#endif

/****************************************************************************
 * Name: ipv4_nat_inbound_icmp
 *
 * Description:
 *   Check if a received ICMP packet belongs to a NAT entry. If so, translate
 *   the external IP/ID to local IP/ID.
 *
 * Input Parameters:
 *   ipv4       - Points to the IPv4 header to translate.
 *   manip_type - Whether external IP is in source or destination.
 *
 * Returned Value:
 *   Zero is returned if NAT is successfully applied, or is not enabled for
 *   this packet;
 *   A negated errno value is returned if error occured.
 *
 * Assumptions:
 *   Packet is received on g_dev and is targeting at the address assigned to
 *   g_dev.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP
static int ipv4_nat_inbound_icmp(FAR struct ipv4_hdr_s *ipv4,
                                 enum nat_manip_type_e manip_type)
{
  FAR struct icmp_hdr_s *icmp = L4_HDR(ipv4);
  FAR struct ipv4_nat_entry *entry;

  switch (icmp->type)
    {
      /* TODO: Support other ICMP types. */

      case ICMP_ECHO_REQUEST:
      case ICMP_ECHO_REPLY:
        entry = ipv4_nat_inbound_entry_find(IP_PROTO_ICMP, icmp->id, true);
        if (!entry)
          {
            /* Inbound without entry is OK, skip NAT. */

            return OK;
          }

        ipv4_nat_port_adjust(&icmp->icmpchksum,
                             &icmp->id, entry->local_port);
        ipv4_nat_ip_adjust(ipv4, NULL, entry->local_ip, manip_type);
        break;
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: ipv4_nat_outbound_tcp
 *
 * Description:
 *   Check if we want to perform NAT with this outbound TCP packet before
 *   sending it. If so, translate the local IP/Port to external IP/Port.
 *
 * Input Parameters:
 *   dev        - The device to sent the packet (to get external IP).
 *   ipv4       - Points to the IPv4 header to translate.
 *   manip_type - Whether local IP/Port is in source or destination.
 *
 * Returned Value:
 *   Zero is returned if NAT is successfully applied, or is not enabled for
 *   this packet;
 *   A negated errno value is returned if error occured.
 *
 * Assumptions:
 *   Packet will be sent on NAT device.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_TCP
static int ipv4_nat_outbound_tcp(FAR struct net_driver_s *dev,
                                 FAR struct ipv4_hdr_s *ipv4,
                                 enum nat_manip_type_e manip_type)
{
  FAR struct tcp_hdr_s *tcp = L4_HDR(ipv4);
  FAR uint16_t (*local_ip)[2] = MANIP_IPADDR(ipv4, manip_type);
  FAR uint16_t *local_port = MANIP_PORT(tcp, manip_type);
  FAR struct ipv4_nat_entry *entry = ipv4_nat_outbound_entry_find(
      dev, IP_PROTO_TCP, net_ip4addr_conv32(*local_ip), *local_port);
  if (!entry)
    {
      /* Outbound entry creation failed, should have corresponding entry. */

      return -ENOMEM;
    }

  ipv4_nat_port_adjust(&tcp->tcpchksum, local_port, entry->external_port);
  ipv4_nat_ip_adjust(ipv4, &tcp->tcpchksum, dev->d_ipaddr, manip_type);

  return OK;
}
#endif

/****************************************************************************
 * Name: ipv4_nat_outbound_udp
 *
 * Description:
 *   Check if we want to perform NAT with this outbound UDP packet before
 *   sending it. If so, translate the local IP/Port to external IP/Port.
 *
 * Input Parameters:
 *   dev        - The device to sent the packet (to get external IP).
 *   ipv4       - Points to the IPv4 header to translate.
 *   manip_type - Whether local IP/Port is in source or destination.
 *
 * Returned Value:
 *   Zero is returned if NAT is successfully applied, or is not enabled for
 *   this packet;
 *   A negated errno value is returned if error occured.
 *
 * Assumptions:
 *   Packet will be sent on NAT device.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_UDP
static int ipv4_nat_outbound_udp(FAR struct net_driver_s *dev,
                                 FAR struct ipv4_hdr_s *ipv4,
                                 enum nat_manip_type_e manip_type)
{
  FAR struct udp_hdr_s *udp = L4_HDR(ipv4);
  FAR uint16_t (*local_ip)[2] = MANIP_IPADDR(ipv4, manip_type);
  FAR uint16_t *local_port = MANIP_PORT(udp, manip_type);
  FAR uint16_t *udpchksum;
  FAR struct ipv4_nat_entry *entry = ipv4_nat_outbound_entry_find(
      dev, IP_PROTO_UDP, net_ip4addr_conv32(*local_ip), *local_port);
  if (!entry)
    {
      /* Outbound entry creation failed, should have corresponding entry. */

      return -ENOMEM;
    }

  /* UDP checksum has special case 0 (no checksum) */

  udpchksum = udp->udpchksum != 0 ? &udp->udpchksum : NULL;

  ipv4_nat_port_adjust(udpchksum, local_port, entry->external_port);
  ipv4_nat_ip_adjust(ipv4, udpchksum, dev->d_ipaddr, manip_type);

  return OK;
}
#endif

/****************************************************************************
 * Name: ipv4_nat_outbound_icmp
 *
 * Description:
 *   Check if we want to perform NAT with this outbound ICMP packet before
 *   sending it. If so, translate the local IP/ID to external IP/ID.
 *
 * Input Parameters:
 *   dev        - The device to sent the packet (to get external IP).
 *   ipv4       - Points to the IPv4 header to translate.
 *   manip_type - Whether local IP is in source or destination.
 *
 * Returned Value:
 *   Zero is returned if NAT is successfully applied, or is not enabled for
 *   this packet;
 *   A negated errno value is returned if error occured.
 *
 * Assumptions:
 *   Packet will be sent on NAT device.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_ICMP
static int ipv4_nat_outbound_icmp(FAR struct net_driver_s *dev,
                                  FAR struct ipv4_hdr_s *ipv4,
                                  enum nat_manip_type_e manip_type)
{
  FAR struct icmp_hdr_s *icmp = L4_HDR(ipv4);
  FAR uint16_t (*local_ip)[2] = MANIP_IPADDR(ipv4, manip_type);
  FAR struct ipv4_nat_entry *entry;

  switch (icmp->type)
    {
      /* TODO: Support other ICMP types. */

      case ICMP_ECHO_REQUEST:
      case ICMP_ECHO_REPLY:
        entry = ipv4_nat_outbound_entry_find(
            dev, IP_PROTO_ICMP, net_ip4addr_conv32(*local_ip),
            icmp->id);
        if (!entry)
          {
            /* Outbound entry creation failed. */

            return -ENOMEM;
          }

        ipv4_nat_port_adjust(&icmp->icmpchksum,
                             &icmp->id, entry->external_port);
        ipv4_nat_ip_adjust(ipv4, NULL, dev->d_ipaddr, manip_type);
        break;
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv4_nat_enable
 *
 * Description:
 *   Enable NAT function on a network device.
 *
 * Input Parameters:
 *   dev   - The device on which the outbound packets will be masqueraded.
 *
 * Returned Value:
 *   Zero is returned if NAT function is successfully enabled on the device;
 *   A negated errno value is returned if failed.
 *
 * Assumptions:
 *   NAT will only be enabled on at most one device.
 *
 * Limitations:
 *   External ports are not isolated between devices yet, so if NAT is
 *   enabled on more than one device, an external port used on one device
 *   will also be used by same local ip:port on another device.
 *
 * TODO:
 *   Support multiple NAT devices with isolated external port mapping.
 ****************************************************************************/

int ipv4_nat_enable(FAR struct net_driver_s *dev)
{
  if (IFF_IS_NAT(dev->d_flags))
    {
      nwarn("WARNING: NAT was already enabled for %s!\n", dev->d_ifname);
      return -EEXIST;
    }

  IFF_SET_NAT(dev->d_flags);
  return OK;
}

/****************************************************************************
 * Name: ipv4_nat_disable
 *
 * Description:
 *   Disable NAT function on a network device.
 *
 * Input Parameters:
 *   dev   - The device on which the NAT function will be disabled.
 *
 * Returned Value:
 *   Zero is returned if NAT function is successfully disabled on the device;
 *   A negated errno value is returned if failed.
 *
 ****************************************************************************/

int ipv4_nat_disable(FAR struct net_driver_s *dev)
{
  if (!IFF_IS_NAT(dev->d_flags))
    {
      nwarn("WARNING: NAT was not enabled for %s!\n", dev->d_ifname);
      return -ENODEV;
    }

  /* TODO: Clear entries related to dev. */

  IFF_CLR_NAT(dev->d_flags);
  return OK;
}

/****************************************************************************
 * Name: ipv4_nat_inbound
 *
 * Description:
 *   Check if a received packet belongs to a NAT entry. If so, translate it.
 *
 * Input Parameters:
 *   dev   - The device on which the packet is received.
 *   ipv4  - Points to the IPv4 header with dev->d_buf.
 *
 * Returned Value:
 *   Zero is returned if NAT is successfully applied, or is not enabled for
 *   this packet;
 *   A negated errno value is returned if error occured.
 *
 ****************************************************************************/

int ipv4_nat_inbound(FAR struct net_driver_s *dev,
                     FAR struct ipv4_hdr_s *ipv4)
{
  /* We only process packets from NAT device and targeting at the address
   * assigned to the device.
   */

  if (IFF_IS_NAT(dev->d_flags) &&
      net_ipv4addr_hdrcmp(ipv4->destipaddr, &dev->d_ipaddr))
    {
      switch (ipv4->proto)
        {
#ifdef CONFIG_NET_TCP
          case IP_PROTO_TCP:
            return ipv4_nat_inbound_tcp(ipv4, NAT_MANIP_DST);
#endif

#ifdef CONFIG_NET_UDP
          case IP_PROTO_UDP:
            return ipv4_nat_inbound_udp(ipv4, NAT_MANIP_DST);
#endif

#ifdef CONFIG_NET_ICMP
          case IP_PROTO_ICMP:
            return ipv4_nat_inbound_icmp(ipv4, NAT_MANIP_DST);
#endif
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ipv4_nat_outbound
 *
 * Description:
 *   Check if we want to perform NAT with this outbound packet before sending
 *   it. If so, translate it.
 *
 * Input Parameters:
 *   dev   - The device on which the packet will be sent.
 *   ipv4  - Points to the IPv4 header to be filled into dev->d_buf later.
 *
 * Returned Value:
 *   Zero is returned if NAT is successfully applied, or is not enabled for
 *   this packet;
 *   A negated errno value is returned if error occured.
 *
 ****************************************************************************/

int ipv4_nat_outbound(FAR struct net_driver_s *dev,
                      FAR struct ipv4_hdr_s *ipv4)
{
  /* We only process packets targeting at NAT device but not targeting at the
   * address assigned to the device.
   */

  if (IFF_IS_NAT(dev->d_flags) &&
      !net_ipv4addr_hdrcmp(ipv4->destipaddr, &dev->d_ipaddr))
    {
      /* TODO: Skip broadcast? */

      switch (ipv4->proto)
        {
#ifdef CONFIG_NET_TCP
          case IP_PROTO_TCP:
            return ipv4_nat_outbound_tcp(dev, ipv4, NAT_MANIP_SRC);
#endif

#ifdef CONFIG_NET_UDP
          case IP_PROTO_UDP:
            return ipv4_nat_outbound_udp(dev, ipv4, NAT_MANIP_SRC);
#endif

#ifdef CONFIG_NET_ICMP
          case IP_PROTO_ICMP:
            return ipv4_nat_outbound_icmp(dev, ipv4, NAT_MANIP_SRC);
#endif
        }
    }

  return OK;
}

/****************************************************************************
 * Name: ipv4_nat_port_inuse
 *
 * Description:
 *   Check whether a port is currently used by NAT.
 *
 * Input Parameters:
 *   protocol      - The L4 protocol of the packet.
 *   ip            - The IP bind with the port (in network byte order).
 *   port          - The port number to check (in network byte order).
 *
 * Returned Value:
 *   True if the port is already used by NAT, otherwise false.
 *
 ****************************************************************************/

bool ipv4_nat_port_inuse(uint8_t protocol, in_addr_t ip, uint16_t port)
{
  FAR struct ipv4_nat_entry *entry =
      ipv4_nat_inbound_entry_find(protocol, port, false);

  /* Not checking ip is enough for single NAT device, may save external_ip in
   * entry for multiple device support in future.
   */

  UNUSED(ip);

  return entry != NULL;
}

#endif /* CONFIG_NET_NAT && CONFIG_NET_IPv4 */
