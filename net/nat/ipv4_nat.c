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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv4_nat_inbound_tcp
 *
 * Description:
 *   Check if a received TCP packet belongs to a NAT entry. If so, translate
 *   it.
 *
 * Input Parameters:
 *   ipv4  - Points to the IPv4 header with dev->d_buf.
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
static int ipv4_nat_inbound_tcp(FAR struct ipv4_hdr_s *ipv4)
{
  uint16_t iphdrlen = (ipv4->vhl & IPv4_HLMASK) << 2;
  FAR struct tcp_hdr_s *tcp =
      (FAR struct tcp_hdr_s *)((FAR uint8_t *)ipv4 + iphdrlen);
  FAR struct ipv4_nat_entry *entry =
      ipv4_nat_inbound_entry_find(IP_PROTO_TCP, tcp->destport, true);
  if (!entry)
    {
      /* Inbound without entry is OK (e.g. towards NuttX itself), skip NAT. */

      return OK;
    }

  /* Modify port and checksum. */

  chksum_adjust(tcp->tcpchksum, tcp->destport, entry->local_port);
  tcp->destport = entry->local_port;

  /* Modify address and checksum. */

  chksum_adjust(tcp->tcpchksum, ipv4->destipaddr, entry->local_ip);
  chksum_adjust(ipv4->ipchksum, ipv4->destipaddr, entry->local_ip);
  net_ipv4addr_hdrcopy(ipv4->destipaddr, &entry->local_ip);

  return OK;
}
#endif

/****************************************************************************
 * Name: ipv4_nat_inbound_udp
 *
 * Description:
 *   Check if a received UDP packet belongs to a NAT entry. If so, translate
 *   it.
 *
 * Input Parameters:
 *   ipv4  - Points to the IPv4 header with dev->d_buf.
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
static int ipv4_nat_inbound_udp(FAR struct ipv4_hdr_s *ipv4)
{
  uint16_t iphdrlen = (ipv4->vhl & IPv4_HLMASK) << 2;
  FAR struct udp_hdr_s *udp =
      (FAR struct udp_hdr_s *)((FAR uint8_t *)ipv4 + iphdrlen);
  FAR struct ipv4_nat_entry *entry =
      ipv4_nat_inbound_entry_find(IP_PROTO_UDP, udp->destport, true);
  if (!entry)
    {
      /* Inbound without entry is OK (e.g. towards NuttX itself), skip NAT. */

      return OK;
    }

  /* Modify port and checksum. */

  if (udp->udpchksum != 0) /* UDP checksum has special case 0 (no checksum) */
    {
      chksum_adjust(udp->udpchksum, udp->destport, entry->local_port);
    }

  udp->destport = entry->local_port;

  /* Modify address and checksum. */

  if (udp->udpchksum != 0) /* UDP checksum has special case 0 (no checksum) */
    {
      chksum_adjust(udp->udpchksum, ipv4->destipaddr, entry->local_ip);
    }

  chksum_adjust(ipv4->ipchksum, ipv4->destipaddr, entry->local_ip);
  net_ipv4addr_hdrcopy(ipv4->destipaddr, &entry->local_ip);

  return OK;
}
#endif

/****************************************************************************
 * Name: ipv4_nat_inbound_icmp
 *
 * Description:
 *   Check if a received ICMP packet belongs to a NAT entry. If so, translate
 *   it.
 *
 * Input Parameters:
 *   ipv4  - Points to the IPv4 header with dev->d_buf.
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
static int ipv4_nat_inbound_icmp(FAR struct ipv4_hdr_s *ipv4)
{
  uint16_t iphdrlen = (ipv4->vhl & IPv4_HLMASK) << 2;
  FAR struct icmp_hdr_s *icmp =
      (FAR struct icmp_hdr_s *)((FAR uint8_t *)ipv4 + iphdrlen);
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

        /* Modify id and checksum. */

        chksum_adjust(icmp->icmpchksum, icmp->id, entry->local_port);
        icmp->id = entry->local_port;

        /* Modify address and checksum. */

        chksum_adjust(ipv4->ipchksum, ipv4->destipaddr, entry->local_ip);
        net_ipv4addr_hdrcopy(ipv4->destipaddr, &entry->local_ip);
    }

  return OK;
}
#endif

/****************************************************************************
 * Name: ipv4_nat_outbound_tcp
 *
 * Description:
 *   Check if we want to perform NAT with this outbound TCP packet before
 *   sending it. If so, translate it.
 *
 * Input Parameters:
 *   dev   - The device to sent the packet.
 *   ipv4  - Points to the IPv4 header to be filled into dev->d_buf later.
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
                                 FAR struct ipv4_hdr_s *ipv4)
{
  uint16_t iphdrlen = (ipv4->vhl & IPv4_HLMASK) << 2;
  FAR struct tcp_hdr_s *tcp =
      (FAR struct tcp_hdr_s *)((FAR uint8_t *)ipv4 + iphdrlen);
  FAR struct ipv4_nat_entry *entry = ipv4_nat_outbound_entry_find(
      dev, IP_PROTO_TCP, net_ip4addr_conv32(ipv4->srcipaddr), tcp->srcport);
  if (!entry)
    {
      /* Outbound entry creation failed, should have corresponding entry. */

      return -ENOMEM;
    }

  /* Modify port and checksum. */

  chksum_adjust(tcp->tcpchksum, tcp->srcport, entry->external_port);
  tcp->srcport = entry->external_port;

  /* Modify address and checksum. */

  chksum_adjust(tcp->tcpchksum, ipv4->srcipaddr, dev->d_ipaddr);
  chksum_adjust(ipv4->ipchksum, ipv4->srcipaddr, dev->d_ipaddr);
  net_ipv4addr_hdrcopy(ipv4->srcipaddr, &dev->d_ipaddr);

  return OK;
}
#endif

/****************************************************************************
 * Name: ipv4_nat_outbound_udp
 *
 * Description:
 *   Check if we want to perform NAT with this outbound UDP packet before
 *   sending it. If so, translate it.
 *
 * Input Parameters:
 *   dev   - The device to sent the packet.
 *   ipv4  - Points to the IPv4 header to be filled into dev->d_buf later.
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
                                 FAR struct ipv4_hdr_s *ipv4)
{
  uint16_t iphdrlen = (ipv4->vhl & IPv4_HLMASK) << 2;
  FAR struct udp_hdr_s *udp =
      (FAR struct udp_hdr_s *)((FAR uint8_t *)ipv4 + iphdrlen);
  FAR struct ipv4_nat_entry *entry = ipv4_nat_outbound_entry_find(
      dev, IP_PROTO_UDP, net_ip4addr_conv32(ipv4->srcipaddr), udp->srcport);
  if (!entry)
    {
      /* Outbound entry creation failed, should have corresponding entry. */

      return -ENOMEM;
    }

  /* Modify port and checksum. */

  if (udp->udpchksum != 0) /* UDP checksum has special case 0 (no checksum) */
    {
      chksum_adjust(udp->udpchksum, udp->srcport, entry->external_port);
    }

  udp->srcport = entry->external_port;

  /* Modify address and checksum. */

  if (udp->udpchksum != 0) /* UDP checksum has special case 0 (no checksum) */
    {
      chksum_adjust(udp->udpchksum, ipv4->srcipaddr, dev->d_ipaddr);
    }

  chksum_adjust(ipv4->ipchksum, ipv4->srcipaddr, dev->d_ipaddr);
  net_ipv4addr_hdrcopy(ipv4->srcipaddr, &dev->d_ipaddr);

  return OK;
}
#endif

/****************************************************************************
 * Name: ipv4_nat_outbound_icmp
 *
 * Description:
 *   Check if we want to perform NAT with this outbound ICMP packet before
 *   sending it. If so, translate it.
 *
 * Input Parameters:
 *   dev   - The device to sent the packet.
 *   ipv4  - Points to the IPv4 header to be filled into dev->d_buf later.
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
                                  FAR struct ipv4_hdr_s *ipv4)
{
  uint16_t iphdrlen = (ipv4->vhl & IPv4_HLMASK) << 2;
  FAR struct icmp_hdr_s *icmp =
      (FAR struct icmp_hdr_s *)((FAR uint8_t *)ipv4 + iphdrlen);
  FAR struct ipv4_nat_entry *entry;

  switch (icmp->type)
    {
      /* TODO: Support other ICMP types. */

      case ICMP_ECHO_REQUEST:
      case ICMP_ECHO_REPLY:
        entry = ipv4_nat_outbound_entry_find(
            dev, IP_PROTO_ICMP, net_ip4addr_conv32(ipv4->srcipaddr),
            icmp->id);
        if (!entry)
          {
            /* Outbound entry creation failed. */

            return -ENOMEM;
          }

        /* Modify id and checksum. */

        chksum_adjust(icmp->icmpchksum, icmp->id, entry->external_port);
        icmp->id = entry->external_port;

        /* Modify address and checksum. */

        chksum_adjust(ipv4->ipchksum, ipv4->srcipaddr, dev->d_ipaddr);
        net_ipv4addr_hdrcopy(ipv4->srcipaddr, &dev->d_ipaddr);
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
            return ipv4_nat_inbound_tcp(ipv4);
#endif

#ifdef CONFIG_NET_UDP
          case IP_PROTO_UDP:
            return ipv4_nat_inbound_udp(ipv4);
#endif

#ifdef CONFIG_NET_ICMP
          case IP_PROTO_ICMP:
            return ipv4_nat_inbound_icmp(ipv4);
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
            return ipv4_nat_outbound_tcp(dev, ipv4);
#endif

#ifdef CONFIG_NET_UDP
          case IP_PROTO_UDP:
            return ipv4_nat_outbound_udp(dev, ipv4);
#endif

#ifdef CONFIG_NET_ICMP
          case IP_PROTO_ICMP:
            return ipv4_nat_outbound_icmp(dev, ipv4);
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
