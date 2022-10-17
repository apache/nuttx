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

#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>

#include "nat/nat.h"

#if defined(CONFIG_NET_NAT) && defined(CONFIG_NET_IPv4)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sq_queue_t g_entries;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ipv4_nat_select_port
 *
 * Description:
 *   Select an available port number for TCP/UDP protocol, or id for ICMP.
 *
 * Input Parameters:
 *   protocol   - The L4 protocol of the packet.
 *   local_port - The local port of the packet, as reference.
 *
 * Returned Value:
 *   port number on success; 0 on failure
 *
 ****************************************************************************/

static uint16_t ipv4_nat_select_port(uint8_t protocol, uint16_t local_port)
{
  /* TODO: Implement this, need to consider local ports and nat ports.
   * TODO: Shall we let the chosen port same as local_port if possible?
   * TODO: Also need to modify local port number assignment.
   */

# warning Missing logic

  return local_port;
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

  sq_addfirst((FAR sq_entry_t *)entry, &g_entries);
  return entry;
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
 *
 * Returned Value:
 *   Pointer to entry on success; null on failure
 *
 ****************************************************************************/

FAR struct ipv4_nat_entry *
ipv4_nat_inbound_entry_find(uint8_t protocol, uint16_t external_port)
{
  FAR sq_entry_t *p;
  sq_for_every(&g_entries, p)
    {
      FAR struct ipv4_nat_entry *entry = (FAR struct ipv4_nat_entry *)p;
      if (entry->protocol == protocol &&
          entry->external_port == external_port)
        {
          /* TODO: Use hash table, or move recent node to head. */

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
                             uint16_t local_port)
{
  FAR sq_entry_t *p;
  sq_for_every(&g_entries, p)
    {
      FAR struct ipv4_nat_entry *entry = (FAR struct ipv4_nat_entry *)p;
      if (entry->protocol == protocol &&
          net_ipv4addr_cmp(entry->local_ip, local_ip) &&
          entry->local_port == local_port)
        {
          /* TODO: Use hash table, or move recent node to head. */

          return entry;
        }
    }

  /* Failed to find the entry, create one. */

  ninfo("INFO: Failed to find IPv4 outbound NAT entry for "
        "proto=%d, local=%x:%d, try creating one.\n",
        protocol, local_ip, local_port);

  uint16_t external_port = ipv4_nat_select_port(protocol, local_port);
  if (!external_port)
    {
      nwarn("WARNING: Failed to find an available port!\n");
      return NULL;
    }

  return ipv4_nat_entry_create(protocol, external_port, local_ip,
                               local_port);
}

#endif /* CONFIG_NET_NAT && CONFIG_NET_IPv4 */
