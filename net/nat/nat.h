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

#include <stdbool.h>
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
  /* Support for doubly-linked lists.
   *
   * TODO: Implement a general hash table, and use it to optimize performance
   * here.
   */

  FAR struct ipv4_nat_entry *flink;
  FAR struct ipv4_nat_entry *blink;

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

  uint32_t   expire_time;    /* The expiration time of this entry. */
};

/* NAT IP/Port manipulate type, to indicate whether to manipulate source or
 * destination IP/Port in a packet.
 */

enum nat_manip_type_e
{
  NAT_MANIP_SRC,
  NAT_MANIP_DST
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
 *   manip_type - Whether local IP/Port is in source or destination.
 *
 * Returned Value:
 *   Zero is returned if NAT is successfully applied, or is not enabled for
 *   this packet;
 *   A negated errno value is returned if error occured.
 *
 ****************************************************************************/

int ipv4_nat_outbound(FAR struct net_driver_s *dev,
                      FAR struct ipv4_hdr_s *ipv4,
                      enum nat_manip_type_e manip_type);

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

bool ipv4_nat_port_inuse(uint8_t protocol, in_addr_t ip, uint16_t port);

/****************************************************************************
 * Name: ipv4_nat_inbound_entry_find
 *
 * Description:
 *   Find the inbound entry in NAT entry list.
 *
 * Input Parameters:
 *   protocol      - The L4 protocol of the packet.
 *   external_port - The external port of the packet.
 *   refresh       - Whether to refresh the selected entry.
 *
 * Returned Value:
 *   Pointer to entry on success; null on failure
 *
 ****************************************************************************/

FAR struct ipv4_nat_entry *
ipv4_nat_inbound_entry_find(uint8_t protocol, uint16_t external_port,
                            bool refresh);

/****************************************************************************
 * Name: ipv4_nat_outbound_entry_find
 *
 * Description:
 *   Find the outbound entry in NAT entry list. Create one if corresponding
 *   entry does not exist.
 *
 * Input Parameters:
 *   dev        - The device on which the packet will be sent.
 *   protocol   - The L4 protocol of the packet.
 *   local_ip   - The local ip of the packet.
 *   local_port - The local port of the packet.
 *   try_create - Try create the entry if no entry found.
 *
 * Returned Value:
 *   Pointer to entry on success; null on failure
 *
 ****************************************************************************/

FAR struct ipv4_nat_entry *
ipv4_nat_outbound_entry_find(FAR struct net_driver_s *dev, uint8_t protocol,
                             in_addr_t local_ip, uint16_t local_port,
                             bool try_create);

#endif /* CONFIG_NET_NAT && CONFIG_NET_IPv4 */
#endif /* __NET_NAT_NAT_H */
