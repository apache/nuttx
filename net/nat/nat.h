/****************************************************************************
 * net/nat/nat.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/hashtable.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/netdev.h>

#ifdef CONFIG_NET_NAT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Adjust checksums in headers. */

#define nat_chksum_adjust(chksum,optr,nptr,len) \
  net_chksum_adjust((FAR uint16_t *)(chksum), (FAR uint16_t *)(optr), len, \
                    (FAR uint16_t *)(nptr), len)

/* Getting IP & Port to manipulate from L3/L4 header. */

#define MANIP_IPADDR(iphdr,manip_type) \
  ((manip_type) == NAT_MANIP_SRC ? (iphdr)->srcipaddr : (iphdr)->destipaddr)

#define MANIP_PORT(l4hdr,manip_type) \
  ((manip_type) == NAT_MANIP_SRC ? &(l4hdr)->srcport : &(l4hdr)->destport)

/* Getting peer IP & Port (just other than MANIP) */

#define PEER_IPADDR(iphdr,manip_type) \
  ((manip_type) != NAT_MANIP_SRC ? (iphdr)->srcipaddr : (iphdr)->destipaddr)

#define PEER_PORT(l4hdr,manip_type) \
  ((manip_type) != NAT_MANIP_SRC ? &(l4hdr)->srcport : &(l4hdr)->destport)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct ipv4_nat_entry_s
{
  hash_node_t hash_inbound;
  hash_node_t hash_outbound;

  /*  Local Network                             External Network
   *                |----------------|
   *   <local IP,   |                | <external IP,             <peer IP,
   *     -----------|                |-----------------------------
   *    local port> |                |  external port>            peer port>
   *                |----------------|
   *
   * Full cone NAT only need to save local ip:port and external ip:port.
   * Symmetric NAT need to save peer ip:port as well.
   * For ICMP, save id in port field.
   */

  in_addr_t  local_ip;       /* IP address of the local (private) host. */
  in_addr_t  external_ip;    /* External IP address. */
#ifdef CONFIG_NET_NAT44_SYMMETRIC
  in_addr_t  peer_ip;        /* Peer IP address. */
#endif
  uint16_t   local_port;     /* Port of the local (private) host. */
  uint16_t   external_port;  /* The external port of local (private) host. */
#ifdef CONFIG_NET_NAT44_SYMMETRIC
  uint16_t   peer_port;      /* Peer port. */
#endif
  uint8_t    protocol;       /* L4 protocol (TCP, UDP etc). */

  int32_t    expire_time;    /* The expiration time of this entry. */
};

struct ipv6_nat_entry_s
{
  hash_node_t    hash_inbound;
  hash_node_t    hash_outbound;

  net_ipv6addr_t local_ip;      /* IP address of the local host. */
  net_ipv6addr_t external_ip;   /* External IP address. */
#ifdef CONFIG_NET_NAT66_SYMMETRIC
  net_ipv6addr_t peer_ip;       /* Peer IP address. */
#endif
  uint16_t       local_port;    /* Port of the local host. */
  uint16_t       external_port; /* The external port of local host. */
#ifdef CONFIG_NET_NAT66_SYMMETRIC
  uint16_t       peer_port;     /* Peer port. */
#endif
  uint8_t        protocol;      /* L4 protocol (TCP, UDP etc). */

  int32_t        expire_time;   /* The expiration time of this entry. */
};

typedef struct ipv4_nat_entry_s ipv4_nat_entry_t;
typedef struct ipv6_nat_entry_s ipv6_nat_entry_t;

typedef CODE void (*ipv4_nat_entry_cb_t)(FAR ipv4_nat_entry_t *entry,
                                         FAR void *arg);
typedef CODE void (*ipv6_nat_entry_cb_t)(FAR ipv6_nat_entry_t *entry,
                                         FAR void *arg);

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
 * Name: nat_enable
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
 ****************************************************************************/

int nat_enable(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: nat_disable
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

int nat_disable(FAR struct net_driver_s *dev);

/****************************************************************************
 * Name: ipv4/ipv6_nat_inbound
 *
 * Description:
 *   Check if a received packet belongs to a NAT entry. If so, translate it.
 *
 * Input Parameters:
 *   dev       - The device on which the packet is received.
 *   ipv4/ipv6 - Points to the IP header with dev->d_buf.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_NAT44
void ipv4_nat_inbound(FAR struct net_driver_s *dev,
                      FAR struct ipv4_hdr_s *ipv4);
#endif
#ifdef CONFIG_NET_NAT66
void ipv6_nat_inbound(FAR struct net_driver_s *dev,
                      FAR struct ipv6_hdr_s *ipv6);
#endif

/****************************************************************************
 * Name: ipv4/ipv6_nat_outbound
 *
 * Description:
 *   Check if we want to perform NAT with this outbound packet before sending
 *   it. If so, translate it.
 *
 * Input Parameters:
 *   dev        - The device on which the packet will be sent.
 *   ipv4/ipv6  - Points to the IP header to be filled into dev->d_buf later.
 *   manip_type - Whether local IP/Port is in source or destination.
 *
 * Returned Value:
 *   Zero is returned if NAT is successfully applied, or is not enabled for
 *   this packet;
 *   A negated errno value is returned if error occured.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_NAT44
int ipv4_nat_outbound(FAR struct net_driver_s *dev,
                      FAR struct ipv4_hdr_s *ipv4,
                      enum nat_manip_type_e manip_type);
#endif
#ifdef CONFIG_NET_NAT66
int ipv6_nat_outbound(FAR struct net_driver_s *dev,
                      FAR struct ipv6_hdr_s *ipv6,
                      enum nat_manip_type_e manip_type);
#endif

/****************************************************************************
 * Name: nat_port_inuse
 *
 * Description:
 *   Check whether a port is currently used by NAT.
 *
 * Input Parameters:
 *   domain        - The domain of the packet.
 *   protocol      - The L4 protocol of the packet.
 *   ip            - The IP bind with the port (in network byte order).
 *   port          - The port number to check (in network byte order).
 *
 * Returned Value:
 *   True if the port is already used by NAT, otherwise false.
 *
 ****************************************************************************/

bool nat_port_inuse(uint8_t domain, uint8_t protocol,
                    FAR const union ip_addr_u *ip, uint16_t port);

/****************************************************************************
 * Name: nat_port_select
 *
 * Description:
 *   Select an available port number for TCP/UDP protocol, or id for ICMP.
 *
 * Input Parameters:
 *   dev         - The device on which the packet will be sent.
 *   domain      - The domain of the packet.
 *   protocol    - The L4 protocol of the packet.
 *   external_ip - The external IP bind with the port.
 *   local_port  - The local port of the packet, as reference.
 *
 * Returned Value:
 *   External port number on success; 0 on failure
 *
 ****************************************************************************/

uint16_t nat_port_select(FAR struct net_driver_s *dev,
                         uint8_t domain, uint8_t protocol,
                         FAR const union ip_addr_u *external_ip,
                         uint16_t local_port);

/****************************************************************************
 * Name: nat_expire_time
 *
 * Description:
 *   Get the expiration time of a specific protocol.
 *
 * Input Parameters:
 *   protocol - The L4 protocol of the packet.
 *
 * Returned Value:
 *   The expiration time of the protocol.
 *
 ****************************************************************************/

uint32_t nat_expire_time(uint8_t protocol);

/****************************************************************************
 * Name: ipv4/ipv6_nat_entry_foreach
 *
 * Description:
 *   Call the callback function for each NAT entry.
 *
 * Input Parameters:
 *   cb  - The callback function.
 *   arg - The argument to pass to the callback function.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_NAT44
void ipv4_nat_entry_foreach(ipv4_nat_entry_cb_t cb, FAR void *arg);
#endif
#ifdef CONFIG_NET_NAT66
void ipv6_nat_entry_foreach(ipv6_nat_entry_cb_t cb, FAR void *arg);
#endif

/****************************************************************************
 * Name: ipv4/ipv6_nat_entry_clear
 *
 * Description:
 *   Clear all entries related to dev. Called when NAT will be disabled on
 *   any device.
 *
 * Input Parameters:
 *   dev - The device on which NAT entries will be cleared.
 *
 * Assumptions:
 *   NAT is initialized.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_NAT44
void ipv4_nat_entry_clear(FAR struct net_driver_s *dev);
#endif
#ifdef CONFIG_NET_NAT66
void ipv6_nat_entry_clear(FAR struct net_driver_s *dev);
#endif

/****************************************************************************
 * Name: ipv4/ipv6_nat_inbound_entry_find
 *
 * Description:
 *   Find the inbound entry in NAT entry list.
 *
 * Input Parameters:
 *   protocol      - The L4 protocol of the packet.
 *   external_ip   - The external ip of the packet, supports INADDR_ANY.
 *   external_port - The external port of the packet.
 *   peer_ip       - The peer ip of the packet.
 *   peer_port     - The peer port of the packet.
 *   refresh       - Whether to refresh the selected entry.
 *
 * Returned Value:
 *   Pointer to entry on success; null on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_NAT44
FAR ipv4_nat_entry_t *
ipv4_nat_inbound_entry_find(uint8_t protocol, in_addr_t external_ip,
                            uint16_t external_port, in_addr_t peer_ip,
                            uint16_t peer_port, bool refresh);
#endif
#ifdef CONFIG_NET_NAT66
FAR ipv6_nat_entry_t *
ipv6_nat_inbound_entry_find(uint8_t protocol,
                            const net_ipv6addr_t external_ip,
                            uint16_t external_port,
                            const net_ipv6addr_t peer_ip,
                            uint16_t peer_port, bool refresh);
#endif

/****************************************************************************
 * Name: ipv4/ipv6_nat_outbound_entry_find
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
 *   peer_ip    - The peer ip of the packet.
 *   peer_port  - The peer port of the packet.
 *   try_create - Try create the entry if no entry found.
 *
 * Returned Value:
 *   Pointer to entry on success; null on failure
 *
 ****************************************************************************/

#ifdef CONFIG_NET_NAT44
FAR ipv4_nat_entry_t *
ipv4_nat_outbound_entry_find(FAR struct net_driver_s *dev, uint8_t protocol,
                             in_addr_t local_ip, uint16_t local_port,
                             in_addr_t peer_ip, uint16_t peer_port,
                             bool try_create);
#endif
#ifdef CONFIG_NET_NAT66
FAR ipv6_nat_entry_t *
ipv6_nat_outbound_entry_find(FAR struct net_driver_s *dev, uint8_t protocol,
                             const net_ipv6addr_t local_ip,
                             uint16_t local_port,
                             const net_ipv6addr_t peer_ip,
                             uint16_t peer_port, bool try_create);
#endif

#endif /* CONFIG_NET_NAT */
#endif /* __NET_NAT_NAT_H */
