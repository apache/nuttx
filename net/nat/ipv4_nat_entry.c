/****************************************************************************
 * net/nat/ipv4_nat_entry.c
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

#include <nuttx/clock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>

#include "icmp/icmp.h"
#include "nat/nat.h"
#include "tcp/tcp.h"
#include "udp/udp.h"

#if defined(CONFIG_NET_NAT) && defined(CONFIG_NET_IPv4)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TODO: Why we limit to 32000 in net stack? */

#define NAT_PORT_REASSIGN_MAX 32000
#define NAT_PORT_REASSIGN_MIN 4096

/****************************************************************************
 * Private Data
 ****************************************************************************/

static dq_queue_t g_entries;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv4_nat_select_port_without_stack
 *
 * Description:
 *   Select an available port number for TCP/UDP protocol, or id for ICMP.
 *   Used when corresponding stack is disabled.
 *
 * Input Parameters:
 *   protocol   - The L4 protocol of the packet.
 *   ip         - The IP bind with the port (in network byte order).
 *   portno     - The local port (in network byte order), as reference.
 *
 * Returned Value:
 *   port number on success; 0 on failure
 *
 ****************************************************************************/

#if (defined(CONFIG_NET_TCP) && defined(CONFIG_NET_TCP_NO_STACK)) || \
    (defined(CONFIG_NET_UDP) && defined(CONFIG_NET_UDP_NO_STACK)) || \
    (defined(CONFIG_NET_ICMP) && !defined(CONFIG_NET_ICMP_SOCKET))

static uint16_t ipv4_nat_select_port_without_stack(
    uint8_t protocol, in_addr_t ip, uint16_t portno)
{
  uint16_t hport = NTOHS(portno);
  while (ipv4_nat_port_inuse(protocol, ip, portno))
    {
      if (++hport >= NAT_PORT_REASSIGN_MAX)
        {
          hport = NAT_PORT_REASSIGN_MIN;
        }

      portno = HTONS(hport);
    }

  return portno;
}

#endif

/****************************************************************************
 * Name: ipv4_nat_select_port
 *
 * Description:
 *   Select an available port number for TCP/UDP protocol, or id for ICMP.
 *
 * Input Parameters:
 *   dev        - The device on which the packet will be sent.
 *   protocol   - The L4 protocol of the packet.
 *   local_port - The local port of the packet, as reference.
 *
 * Returned Value:
 *   port number on success; 0 on failure
 *
 ****************************************************************************/

static uint16_t ipv4_nat_select_port(FAR struct net_driver_s *dev,
                                     uint8_t protocol,
                                     uint16_t local_port)
{
  switch (protocol)
    {
#ifdef CONFIG_NET_TCP
      case IP_PROTO_TCP:
        {
#ifndef CONFIG_NET_TCP_NO_STACK
          /* Try to select local_port first. */

          int ret = tcp_selectport(PF_INET,
                        (FAR const union ip_addr_u *)&dev->d_draddr,
                        local_port);

          /* If failed, try select another unused port. */

          if (ret < 0)
            {
              ret = tcp_selectport(PF_INET,
                        (FAR const union ip_addr_u *)&dev->d_draddr, 0);
            }

          return ret > 0 ? ret : 0;
#else
          return ipv4_nat_select_port_without_stack(IP_PROTO_TCP,
                                                    dev->d_draddr,
                                                    local_port);
#endif
        }
#endif

#ifdef CONFIG_NET_UDP
      case IP_PROTO_UDP:
        {
#ifndef CONFIG_NET_UDP_NO_STACK
          union ip_binding_u u;
          u.ipv4.laddr = dev->d_draddr;
          u.ipv4.raddr = INADDR_ANY;

          /* TODO: Try keep origin port as possible. */

          return HTONS(udp_select_port(PF_INET, &u));
#else
          return ipv4_nat_select_port_without_stack(IP_PROTO_UDP,
                                                    dev->d_draddr,
                                                    local_port);
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
                 ipv4_nat_port_inuse(IP_PROTO_ICMP, dev->d_draddr, id))
            {
              if (++hid >= NAT_PORT_REASSIGN_MAX)
                {
                  hid = NAT_PORT_REASSIGN_MIN;
                }

              id = HTONS(hid);
            }

          return id;
#else
          return ipv4_nat_select_port_without_stack(IP_PROTO_ICMP,
                                                    dev->d_draddr,
                                                    local_port);
#endif
        }
#endif
    }

  /* TODO: Currently select original port for unsupported protocol, maybe
   * return zero to indicate failure.
   */

  return local_port;
}

/****************************************************************************
 * Name: ipv4_nat_entry_refresh
 *
 * Description:
 *   Refresh a NAT entry, update its expiration time.
 *
 * Input Parameters:
 *   entry      - The entry to refresh.
 *
 ****************************************************************************/

static void ipv4_nat_entry_refresh(FAR struct ipv4_nat_entry *entry)
{
  switch (entry->protocol)
    {
#ifdef CONFIG_NET_TCP
      case IP_PROTO_TCP:
        /* NOTE: According to RFC2663, Section 2.6, Page 5, we can reduce the
         * time to 4min if we have received FINs from both side of one
         * connection, and keep 24h for other TCP connections. However, full
         * cone NAT may have multiple connections on one entry, so this
         * optimization may not work and we only use one expiration time.
         */

        entry->expire_time = TICK2SEC(clock_systime_ticks()) +
                             CONFIG_NET_NAT_TCP_EXPIRE_SEC;
        break;
#endif

#ifdef CONFIG_NET_UDP
      case IP_PROTO_UDP:
        entry->expire_time = TICK2SEC(clock_systime_ticks()) +
                             CONFIG_NET_NAT_UDP_EXPIRE_SEC;
        break;
#endif

#ifdef CONFIG_NET_ICMP
      case IP_PROTO_ICMP:
        entry->expire_time = TICK2SEC(clock_systime_ticks()) +
                             CONFIG_NET_NAT_ICMP_EXPIRE_SEC;
        break;
#endif
  }
}

/****************************************************************************
 * Name: ipv4_nat_entry_create
 *
 * Description:
 *   Create a NAT entry and insert into entry list.
 *
 * Input Parameters:
 *   protocol      - The L4 protocol of the packet.
 *   external_port - The external port of the packet.
 *   local_ip      - The local ip of the packet.
 *   local_port    - The local port of the packet.
 *
 * Returned Value:
 *   Pointer to entry on success; null on failure
 *
 ****************************************************************************/

static FAR struct ipv4_nat_entry *
ipv4_nat_entry_create(uint8_t protocol, uint16_t external_port,
                      in_addr_t local_ip, uint16_t local_port)
{
  FAR struct ipv4_nat_entry *entry =
      (FAR struct ipv4_nat_entry *)kmm_malloc(sizeof(struct ipv4_nat_entry));
  if (entry == NULL)
    {
      nwarn("WARNING: Failed to allocate IPv4 NAT entry\n");
      return NULL;
    }

  entry->protocol      = protocol;
  entry->external_port = external_port;
  entry->local_ip      = local_ip;
  entry->local_port    = local_port;

  ipv4_nat_entry_refresh(entry);

  dq_addfirst((FAR dq_entry_t *)entry, &g_entries);
  return entry;
}

/****************************************************************************
 * Name: ipv4_nat_entry_delete
 *
 * Description:
 *   Delete a NAT entry and remove from entry list.
 *
 * Input Parameters:
 *   entry      - The entry to remove.
 *
 ****************************************************************************/

void ipv4_nat_entry_delete(FAR struct ipv4_nat_entry *entry)
{
  ninfo("INFO: Removing NAT entry proto=%d, local=%x:%d, external=:%d\n",
        entry->protocol, entry->local_ip, entry->local_port,
        entry->external_port);

  dq_rem((FAR dq_entry_t *)entry, &g_entries);
  kmm_free(entry);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
                            bool refresh)
{
  FAR sq_entry_t *p;
  FAR sq_entry_t *tmp;
  uint32_t current_time = TICK2SEC(clock_systime_ticks());

  sq_for_every_safe((FAR sq_queue_t *)&g_entries, p, tmp)
    {
      FAR struct ipv4_nat_entry *entry = (FAR struct ipv4_nat_entry *)p;

      /* Remove expired entries. */

      if (entry->expire_time < current_time)
        {
          ipv4_nat_entry_delete(entry);
          continue;
        }

      if (entry->protocol == protocol &&
          entry->external_port == external_port)
        {
          /* TODO: Use hash table, or move recent node to head. */

          if (refresh)
            {
              ipv4_nat_entry_refresh(entry);
            }

          return entry;
        }
    }

  nwarn("WARNING: Failed to find IPv4 inbound NAT entry for "
        "proto=%d, external_port=%d\n", protocol, external_port);
  return NULL;
}

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
                             bool try_create)
{
  FAR sq_entry_t *p;
  FAR sq_entry_t *tmp;
  uint32_t current_time = TICK2SEC(clock_systime_ticks());

  sq_for_every_safe((FAR sq_queue_t *)&g_entries, p, tmp)
    {
      FAR struct ipv4_nat_entry *entry = (FAR struct ipv4_nat_entry *)p;

      /* Remove expired entries. */

      if (entry->expire_time < current_time)
        {
          ipv4_nat_entry_delete(entry);
          continue;
        }

      if (entry->protocol == protocol &&
          net_ipv4addr_cmp(entry->local_ip, local_ip) &&
          entry->local_port == local_port)
        {
          /* TODO: Use hash table, or move recent node to head. */

          ipv4_nat_entry_refresh(entry);
          return entry;
        }
    }

  if (!try_create)
    {
      return NULL;
    }

  /* Failed to find the entry, create one. */

  ninfo("INFO: Failed to find IPv4 outbound NAT entry for "
        "proto=%d, local=%x:%d, try creating one.\n",
        protocol, local_ip, local_port);

  uint16_t external_port = ipv4_nat_select_port(dev, protocol, local_port);
  if (!external_port)
    {
      nwarn("WARNING: Failed to find an available port!\n");
      return NULL;
    }

  return ipv4_nat_entry_create(protocol, external_port, local_ip,
                               local_port);
}

#endif /* CONFIG_NET_NAT && CONFIG_NET_IPv4 */
