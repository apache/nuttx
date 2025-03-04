/****************************************************************************
 * net/nat/ipv4_nat_entry.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <stdint.h>

#include <nuttx/clock.h>
#include <nuttx/hashtable.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>

#include "nat/nat.h"
#include "netlink/netlink.h"

#ifdef CONFIG_NET_NAT44

/****************************************************************************
 * Private Data
 ****************************************************************************/

static DECLARE_HASHTABLE(g_nat44_inbound, CONFIG_NET_NAT_HASH_BITS);
static DECLARE_HASHTABLE(g_nat44_outbound, CONFIG_NET_NAT_HASH_BITS);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv4_nat_inbound_key
 *
 * Description:
 *   Create an inbound hash key for NAT.
 *
 ****************************************************************************/

static inline uint32_t ipv4_nat_inbound_key(in_addr_t external_ip,
                                            uint16_t external_port,
                                            uint8_t protocol)
{
  return NTOHL(external_ip) ^ /* external ip may different in higher bits. */
         ((uint32_t)protocol << 16) ^ external_port;
}

/****************************************************************************
 * Name: ipv4_nat_outbound_key
 *
 * Description:
 *   Create an outbound hash key for NAT.
 *
 ****************************************************************************/

static inline uint32_t ipv4_nat_outbound_key(in_addr_t local_ip,
                                             uint16_t local_port,
                                             uint8_t protocol)
{
  return NTOHL(local_ip) ^ /* NTOHL makes sure difference is in lower bits. */
         ((uint32_t)protocol << 8) ^ ((uint32_t)local_port << 16);
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

static void ipv4_nat_entry_refresh(FAR ipv4_nat_entry_t *entry)
{
  entry->expire_time = nat_expire_time(entry->protocol);
}

/****************************************************************************
 * Name: ipv4_nat_entry_create
 *
 * Description:
 *   Create a NAT entry and insert into entry list.
 *
 * Input Parameters:
 *   protocol      - The L4 protocol of the packet.
 *   external_ip   - The external ip of the packet.
 *   external_port - The external port of the packet.
 *   local_ip      - The local ip of the packet.
 *   local_port    - The local port of the packet.
 *   peer_ip       - The peer ip of the packet.
 *   peer_port     - The peer port of the packet.
 *
 * Returned Value:
 *   Pointer to entry on success; null on failure
 *
 ****************************************************************************/

static FAR ipv4_nat_entry_t *
ipv4_nat_entry_create(uint8_t protocol,
                      in_addr_t external_ip, uint16_t external_port,
                      in_addr_t local_ip, uint16_t local_port,
                      in_addr_t peer_ip, uint16_t peer_port)
{
  FAR ipv4_nat_entry_t *entry = kmm_malloc(sizeof(ipv4_nat_entry_t));
  if (entry == NULL)
    {
      nwarn("WARNING: Failed to allocate IPv4 NAT entry\n");
      return NULL;
    }

  entry->protocol      = protocol;
  entry->external_ip   = external_ip;
  entry->external_port = external_port;
  entry->local_ip      = local_ip;
  entry->local_port    = local_port;
#ifdef CONFIG_NET_NAT44_SYMMETRIC
  entry->peer_ip       = peer_ip;
  entry->peer_port     = peer_port;
#endif

  ipv4_nat_entry_refresh(entry);

  hashtable_add(g_nat44_inbound, &entry->hash_inbound,
                ipv4_nat_inbound_key(external_ip, external_port, protocol));
  hashtable_add(g_nat44_outbound, &entry->hash_outbound,
                ipv4_nat_outbound_key(local_ip, local_port, protocol));

#ifdef CONFIG_NETLINK_NETFILTER
  netlink_conntrack_notify(IPCTNL_MSG_CT_NEW, PF_INET, entry);
#endif

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

static void ipv4_nat_entry_delete(FAR ipv4_nat_entry_t *entry)
{
  ninfo("INFO: Removing NAT44 entry proto=%" PRIu8
        ", local=%" PRIx32 ":%" PRIu16 ", external=:%" PRIu16 "\n",
        entry->protocol, entry->local_ip, entry->local_port,
        entry->external_port);

  hashtable_delete(g_nat44_inbound, &entry->hash_inbound,
                   ipv4_nat_inbound_key(entry->external_ip,
                                        entry->external_port,
                                        entry->protocol));
  hashtable_delete(g_nat44_outbound, &entry->hash_outbound,
                   ipv4_nat_outbound_key(entry->local_ip,
                                         entry->local_port,
                                         entry->protocol));

#ifdef CONFIG_NETLINK_NETFILTER
  netlink_conntrack_notify(IPCTNL_MSG_CT_DELETE, PF_INET, entry);
#endif

  kmm_free(entry);
}

/****************************************************************************
 * Name: ipv4_nat_reclaim_entry
 *
 * Description:
 *   Try reclaim all expired NAT entries.
 *   Only works after every CONFIG_NET_NAT_ENTRY_RECLAIM_SEC (low frequency).
 *
 *   Although expired entries will be automatically reclaimed when matching
 *   inbound/outbound entries, there might be some situations that entries
 *   will be kept in memory, e.g. big hashtable with only a few connections.
 *
 * Assumptions:
 *   NAT is initialized.
 *
 ****************************************************************************/

#if CONFIG_NET_NAT_ENTRY_RECLAIM_SEC > 0
static void ipv4_nat_reclaim_entry_cb(FAR ipv4_nat_entry_t *entry,
                                      FAR void *arg)
{
  int32_t current_time = *(FAR int32_t *)arg;

  if (entry->expire_time - current_time <= 0)
    {
      ipv4_nat_entry_delete(entry);
    }
}

static void ipv4_nat_reclaim_entry(int32_t current_time)
{
  static int32_t next_reclaim_time = CONFIG_NET_NAT_ENTRY_RECLAIM_SEC;

  if (next_reclaim_time - current_time <= 0)
    {
      ninfo("INFO: Reclaiming all expired NAT44 entries.\n");
      ipv4_nat_entry_foreach(ipv4_nat_reclaim_entry_cb, &current_time);
      next_reclaim_time = current_time + CONFIG_NET_NAT_ENTRY_RECLAIM_SEC;
    }
}
#else
#  define ipv4_nat_reclaim_entry(t)
#endif

/****************************************************************************
 * Name: ipv4_nat_entry_clear_cb
 *
 * Description:
 *   Clear an entry related to dev. Called when NAT will be disabled on
 *   any device.
 *
 ****************************************************************************/

static void ipv4_nat_entry_clear_cb(FAR ipv4_nat_entry_t *entry,
                                    FAR void *arg)
{
  FAR struct net_driver_s *dev = arg;

  if (net_ipv4addr_cmp(entry->external_ip, dev->d_ipaddr))
    {
      ipv4_nat_entry_delete(entry);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv4_nat_entry_clear
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

void ipv4_nat_entry_clear(FAR struct net_driver_s *dev)
{
  ninfo("INFO: Clearing all NAT44 entries for %s\n", dev->d_ifname);
  ipv4_nat_entry_foreach(ipv4_nat_entry_clear_cb, dev);
}

/****************************************************************************
 * Name: ipv4_nat_entry_foreach
 *
 * Description:
 *   Call the callback function for each NAT entry.
 *
 * Input Parameters:
 *   cb  - The callback function.
 *   arg - The argument to pass to the callback function.
 *
 ****************************************************************************/

void ipv4_nat_entry_foreach(ipv4_nat_entry_cb_t cb, FAR void *arg)
{
  FAR hash_node_t *p;
  FAR hash_node_t *tmp;
  int i;

  hashtable_for_every_safe(g_nat44_inbound, p, tmp, i)
    {
      FAR ipv4_nat_entry_t *entry =
        container_of(p, ipv4_nat_entry_t, hash_inbound);

      cb(entry, arg);
    }
}

/****************************************************************************
 * Name: ipv4_nat_inbound_entry_find
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

FAR ipv4_nat_entry_t *
ipv4_nat_inbound_entry_find(uint8_t protocol, in_addr_t external_ip,
                            uint16_t external_port, in_addr_t peer_ip,
                            uint16_t peer_port, bool refresh)
{
  FAR hash_node_t *p;
  FAR hash_node_t *tmp;
  bool skip_ip = net_ipv4addr_cmp(external_ip, INADDR_ANY);
#ifdef CONFIG_NET_NAT44_SYMMETRIC
  bool skip_peer = net_ipv4addr_cmp(peer_ip, INADDR_ANY);
#endif
  int32_t current_time = TICK2SEC(clock_systime_ticks());

  ipv4_nat_reclaim_entry(current_time);

  hashtable_for_every_possible_safe(g_nat44_inbound, p, tmp,
                  ipv4_nat_inbound_key(external_ip, external_port, protocol))
    {
      FAR ipv4_nat_entry_t *entry =
        container_of(p, ipv4_nat_entry_t, hash_inbound);

      /* Remove expired entries. */

      if (entry->expire_time - current_time <= 0)
        {
          ipv4_nat_entry_delete(entry);
          continue;
        }

      if (entry->protocol == protocol &&
          (skip_ip || net_ipv4addr_cmp(entry->external_ip, external_ip)) &&
          entry->external_port == external_port
#ifdef CONFIG_NET_NAT44_SYMMETRIC
          && (skip_peer || (net_ipv4addr_cmp(entry->peer_ip, peer_ip) &&
                            entry->peer_port == peer_port))
#endif
          )
        {
          if (refresh)
            {
              ipv4_nat_entry_refresh(entry);
            }

          return entry;
        }
    }

  if (refresh) /* false = a test of whether entry exists, no need to warn */
    {
      nwarn("WARNING: Failed to find IPv4 inbound NAT entry for "
            "proto=%" PRIu8 ", external=%" PRIx32 ":%" PRIu16 "\n",
            protocol, external_ip, external_port);
    }

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
 *   peer_ip    - The peer ip of the packet.
 *   peer_port  - The peer port of the packet.
 *   try_create - Try create the entry if no entry found.
 *
 * Returned Value:
 *   Pointer to entry on success; null on failure
 *
 ****************************************************************************/

FAR ipv4_nat_entry_t *
ipv4_nat_outbound_entry_find(FAR struct net_driver_s *dev, uint8_t protocol,
                             in_addr_t local_ip, uint16_t local_port,
                             in_addr_t peer_ip, uint16_t peer_port,
                             bool try_create)
{
  FAR hash_node_t *p;
  FAR hash_node_t *tmp;
  uint16_t external_port;
  int32_t current_time = TICK2SEC(clock_systime_ticks());

  ipv4_nat_reclaim_entry(current_time);

  hashtable_for_every_possible_safe(g_nat44_outbound, p, tmp,
                      ipv4_nat_outbound_key(local_ip, local_port, protocol))
    {
      FAR ipv4_nat_entry_t *entry =
        container_of(p, ipv4_nat_entry_t, hash_outbound);

      /* Remove expired entries. */

      if (entry->expire_time - current_time <= 0)
        {
          ipv4_nat_entry_delete(entry);
          continue;
        }

      if (entry->protocol == protocol &&
          net_ipv4addr_cmp(entry->external_ip, dev->d_ipaddr) &&
          net_ipv4addr_cmp(entry->local_ip, local_ip) &&
          entry->local_port == local_port
#ifdef CONFIG_NET_NAT44_SYMMETRIC
          && net_ipv4addr_cmp(entry->peer_ip, peer_ip) &&
          entry->peer_port == peer_port
#endif
          )
        {
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
        "proto=%" PRIu8 ", local=%" PRIx32 ":%" PRIu16 ", try create one.\n",
        protocol, local_ip, local_port);

  external_port = nat_port_select(dev, PF_INET, protocol,
                          (FAR union ip_addr_u *)&dev->d_ipaddr, local_port);
  if (!external_port)
    {
      nwarn("WARNING: Failed to find an available port!\n");
      return NULL;
    }

  return ipv4_nat_entry_create(protocol, dev->d_ipaddr, external_port,
                               local_ip, local_port, peer_ip, peer_port);
}

#endif /* CONFIG_NET_NAT44 */
