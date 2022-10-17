/****************************************************************************
 * net/nat/nat.h
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

#ifndef __NET_NAT_NAT_H
#define __NET_NAT_NAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <netinet/in.h>

#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>

#if defined(CONFIG_NET_NAT) && defined(CONFIG_NET_IPv4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ipv4_nat_entry
{
  /* Support for single-linked lists.
   *
   * TODO: Implement a general hash table, and use it to optimize performance
   * here.
   */

  FAR struct ipv4_nat_entry *flink;

  /*  Local Network                             External Network
   *                |----------------|
   *   <local IP,   |                | <external IP,             <peer IP,
   *     -----------|                |-----------------------------
   *    local port> |                |  external port>            peer port>
   *                |----------------|
   *
   * Full cone NAT on single WAN only need to save local ip:port and external
   * port. For ICMP, save id in port field.
   */

  in_addr_t  local_ip;       /* IP address of the local (private) host. */
  uint16_t   local_port;     /* Port of the local (private) host. */
  uint16_t   external_port;  /* The external port of local (private) host. */
  uint8_t    protocol;       /* L4 protocol (TCP, UDP etc). */

  /* TODO: Timeout check and remove outdated entry. */
};

/****************************************************************************
 * Public Function Prototypes
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
 ****************************************************************************/

int ipv4_nat_enable(FAR struct net_driver_s *dev);

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

int ipv4_nat_disable(FAR struct net_driver_s *dev);

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
                     FAR struct ipv4_hdr_s *ipv4);

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
                      FAR struct ipv4_hdr_s *ipv4);

/****************************************************************************
 * Name: ipv4_nat_inbound_entry_find
 *
 * Description:
 *   Find the inbound entry in NAT entry list.
 *
 * Input Parameters:
 *   protocol      - The L4 protocol of the packet.
 *   external_port - The external port of the packet.
 *
 * Returned Value:
 *   Pointer to entry on success; null on failure
 *
 ****************************************************************************/

FAR struct ipv4_nat_entry *
ipv4_nat_inbound_entry_find(uint8_t protocol, uint16_t external_port);

/****************************************************************************
 * Name: ipv4_nat_outbound_entry_find
 *
 * Description:
 *   Find the outbound entry in NAT entry list. Create one if corresponding
 *   entry does not exist.
 *
 * Input Parameters:
 *   protocol   - The L4 protocol of the packet.
 *   local_ip   - The local ip of the packet.
 *   local_port - The local port of the packet.
 *
 * Returned Value:
 *   Pointer to entry on success; null on failure
 *
 ****************************************************************************/

FAR struct ipv4_nat_entry *
ipv4_nat_outbound_entry_find(uint8_t protocol, in_addr_t local_ip,
                             uint16_t local_port);

#endif /* CONFIG_NET_NAT && CONFIG_NET_IPv4 */
#endif /* __NET_NAT_NAT_H */
