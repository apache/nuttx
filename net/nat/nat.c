/****************************************************************************
 * net/nat/nat.c
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

#include "icmp/icmp.h"
#include "icmpv6/icmpv6.h"
#include "inet/inet.h"
#include "nat/nat.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "utils/utils.h"

#ifdef CONFIG_NET_NAT

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nat_port_select_without_stack
 *
 * Description:
 *   Select an available port number for TCP/UDP protocol, or id for ICMP.
 *   Used when corresponding stack is disabled.
 *
 * Input Parameters:
 *   domain     - The domain of the packet.
 *   protocol   - The L4 protocol of the packet.
 *   ip         - The IP bind with the port (in network byte order).
 *   local_port - The local port (in network byte order), as reference.
 *
 * Returned Value:
 *   port number on success; 0 on failure
 *
 ****************************************************************************/

#if (defined(CONFIG_NET_TCP) && defined(CONFIG_NET_TCP_NO_STACK))  || \
    (defined(CONFIG_NET_UDP) && defined(CONFIG_NET_UDP_NO_STACK))  || \
    (defined(CONFIG_NET_ICMP) && !defined(CONFIG_NET_ICMP_SOCKET)) || \
    (defined(CONFIG_NET_ICMPv6) && !defined(CONFIG_NET_ICMPv6_SOCKET))

static uint16_t nat_port_select_without_stack(
    uint8_t domain, uint8_t protocol, FAR const union ip_addr_u *ip,
    uint16_t local_port)
{
  uint16_t portno = local_port;
  uint16_t hport = NTOHS(portno);
  while (nat_port_inuse(domain, protocol, ip, portno))
    {
      NET_PORT_NEXT_NH(portno, hport);
      if (portno == local_port)
        {
          /* We have looped back, failed. */

          return 0;
        }
    }

  return portno;
}

#endif

/****************************************************************************
 * Public Functions
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

int nat_enable(FAR struct net_driver_s *dev)
{
  net_lock();

  if (IFF_IS_NAT(dev->d_flags))
    {
      nwarn("WARNING: NAT was already enabled for %s!\n", dev->d_ifname);
      net_unlock();
      return -EEXIST;
    }

  IFF_SET_NAT(dev->d_flags);

  net_unlock();
  return OK;
}

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

int nat_disable(FAR struct net_driver_s *dev)
{
  net_lock();

  if (!IFF_IS_NAT(dev->d_flags))
    {
      nwarn("WARNING: NAT was not enabled for %s!\n", dev->d_ifname);
      net_unlock();
      return -ENODEV;
    }

  /* Clear entries related to dev. */

#ifdef CONFIG_NET_NAT44
  ipv4_nat_entry_clear(dev);
#endif
#ifdef CONFIG_NET_NAT66
  ipv6_nat_entry_clear(dev);
#endif

  IFF_CLR_NAT(dev->d_flags);

  net_unlock();
  return OK;
}

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
                    FAR const union ip_addr_u *ip, uint16_t port)
{
#ifdef CONFIG_NET_NAT44
  if (domain == PF_INET)
    {
      return !!ipv4_nat_inbound_entry_find(protocol, ip->ipv4, port,
                                           INADDR_ANY, 0, false);
    }
#endif

#ifdef CONFIG_NET_NAT66
  if (domain == PF_INET6)
    {
      return !!ipv6_nat_inbound_entry_find(protocol, ip->ipv6, port,
                                           g_ipv6_unspecaddr, 0, false);
    }
#endif

  return false;
}

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
                         uint16_t local_port)
{
  switch (protocol)
    {
#ifdef CONFIG_NET_TCP
      case IP_PROTO_TCP:
        {
#ifndef CONFIG_NET_TCP_NO_STACK
          /* Try to select local_port first. */

          int ret = tcp_selectport(domain, external_ip, local_port);

          /* If failed, try select another unused port. */

          if (ret < 0)
            {
              ret = tcp_selectport(domain, external_ip, 0);
            }

          return ret > 0 ? ret : 0;
#else
          return nat_port_select_without_stack(domain, IP_PROTO_TCP,
                                               external_ip, local_port);
#endif
        }
#endif

#ifdef CONFIG_NET_UDP
      case IP_PROTO_UDP:
        {
#ifndef CONFIG_NET_UDP_NO_STACK
          union ip_binding_u u;

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
          if (domain == PF_INET)
#endif
            {
              u.ipv4.laddr = external_ip->ipv4;
              u.ipv4.raddr = INADDR_ANY;
            }
#endif
#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
          else
#endif
            {
              net_ipv6addr_copy(u.ipv6.laddr, external_ip->ipv6);
              net_ipv6addr_copy(u.ipv6.raddr, g_ipv6_unspecaddr);
            }
#endif

          /* TODO: Try keep origin port as possible. */

          return HTONS(udp_select_port(domain, &u));
#else
          return nat_port_select_without_stack(domain, IP_PROTO_UDP,
                                               external_ip, local_port);
#endif
        }
#endif

#ifdef CONFIG_NET_ICMP
      case IP_PROTO_ICMP:
        {
#ifdef CONFIG_NET_ICMP_SOCKET
          uint16_t id = local_port;
          uint16_t hid = NTOHS(id);
          while (icmp_findconn(dev, id) ||
                 nat_port_inuse(domain, IP_PROTO_ICMP, external_ip, id))
            {
              NET_PORT_NEXT_NH(id, hid);
              if (id == local_port)
                {
                  /* We have looped back, failed. */

                  return 0;
                }
            }

          return id;
#else
          return nat_port_select_without_stack(domain, IP_PROTO_ICMP,
                                               external_ip, local_port);
#endif
        }
#endif

#ifdef CONFIG_NET_ICMPv6
      case IP_PROTO_ICMP6:
        {
#ifdef CONFIG_NET_ICMPv6_SOCKET
          uint16_t id = local_port;
          uint16_t hid = NTOHS(id);
          while (icmpv6_active(id) ||
                 nat_port_inuse(domain, IP_PROTO_ICMP6, external_ip, id))
            {
              NET_PORT_NEXT_NH(id, hid);
              if (id == local_port)
                {
                  /* We have looped back, failed. */

                  return 0;
                }
            }

          return id;
#else
          return nat_port_select_without_stack(domain, IP_PROTO_ICMP6,
                                               external_ip, local_port);
#endif
        }
#endif
    }

  /* Select original port for unsupported protocol. */

  return local_port;
}

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

uint32_t nat_expire_time(uint8_t protocol)
{
  /* Note: May add logic here to move recent node to head side if each chain
   * in hashtable is still too long (with long expire time).
   */

  switch (protocol)
    {
#ifdef CONFIG_NET_TCP
      case IP_PROTO_TCP:
        /* NOTE: According to RFC2663, Section 2.6, Page 5, we can reduce the
         * time to 4min if we have received FINs from both side of one
         * connection, and keep 24h for other TCP connections. However, full
         * cone NAT may have multiple connections on one entry, so this
         * optimization may not work and we only use one expiration time.
         */

        return TICK2SEC(clock_systime_ticks()) +
               CONFIG_NET_NAT_TCP_EXPIRE_SEC;
#endif

#ifdef CONFIG_NET_UDP
      case IP_PROTO_UDP:
        return TICK2SEC(clock_systime_ticks()) +
               CONFIG_NET_NAT_UDP_EXPIRE_SEC;
#endif

#ifdef CONFIG_NET_ICMP
      case IP_PROTO_ICMP:
        return TICK2SEC(clock_systime_ticks()) +
               CONFIG_NET_NAT_ICMP_EXPIRE_SEC;
#endif

#ifdef CONFIG_NET_ICMPv6
      case IP_PROTO_ICMP6:
        return TICK2SEC(clock_systime_ticks()) +
               CONFIG_NET_NAT_ICMPv6_EXPIRE_SEC;
#endif

      default:
        nwarn("WARNING: Unsupported protocol %" PRIu8 "\n", protocol);
        return 0;
  }
}

#endif /* CONFIG_NET_NAT */
