/****************************************************************************
 * net/route/ramroute.h
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

#ifndef __NET_ROUTE_RAMROUTE_H
#define __NET_ROUTE_RAMROUTE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "route/route.h"

#if defined(CONFIG_ROUTE_IPv4_RAMROUTE) || defined(CONFIG_ROUTE_IPv6_RAMROUTE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_ROUTE_MAX_IPv4_RAMROUTES
#  define CONFIG_ROUTE_MAX_IPv4_RAMROUTES 4
#endif

#ifndef CONFIG_ROUTE_MAX_IPv6_RAMROUTES
#  define CONFIG_ROUTE_MAX_IPv6_RAMROUTES 4
#endif

/* Routing table initializer */

#define ramroute_init(rr) \
  do \
    { \
      (rr)->head = NULL; \
      (rr)->tail = NULL; \
    } \
  while (0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
/* This structure describes one entry in the routing table */

struct net_route_ipv4_entry_s
{
  struct net_route_ipv4_s entry;
  FAR struct net_route_ipv4_entry_s *flink;
};

/* This structure describes the head of a routing table list */

struct net_route_ipv4_queue_s
{
  FAR struct net_route_ipv4_entry_s *head;
  FAR struct net_route_ipv4_entry_s *tail;
};
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
/* This structure describes one entry in the routing table */

struct net_route_ipv6_entry_s
{
  struct net_route_ipv6_s entry;
  FAR struct net_route_ipv6_entry_s *flink;
};

/* This structure describes the head of a routing table list */

struct net_route_ipv6_queue_s
{
  FAR struct net_route_ipv6_entry_s *head;
  FAR struct net_route_ipv6_entry_s *tail;
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* These are the routing tables */

#if defined(CONFIG_ROUTE_IPv4_RAMROUTE)
/* The in-memory routing tables are represented as singly linked lists. */

extern struct net_route_ipv4_queue_s g_ipv4_routes;
#endif

#if defined(CONFIG_ROUTE_IPv6_RAMROUTE)
/* The in-memory routing tables are represented as singly linked lists. */

extern struct net_route_ipv6_queue_s g_ipv6_routes;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: net_init_ramroute
 *
 * Description:
 *   Initialize the in-memory, RAM routing table
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in initialization so that no special protection is needed.
 *
 ****************************************************************************/

void net_init_ramroute(void);

/****************************************************************************
 * Name: net_allocroute_ipv4 and net_allocroute_ipv6
 *
 * Description:
 *   Allocate one route by removing it from the free list
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a pointer to the newly allocated routing table entry is
 *   returned; NULL is returned on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
FAR struct net_route_ipv4_s *net_allocroute_ipv4(void);
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
FAR struct net_route_ipv6_s *net_allocroute_ipv6(void);
#endif

/****************************************************************************
 * Name: net_freeroute_ipv4 and net_freeroute_ipv6
 *
 * Description:
 *   Free one route by adding it from the free list
 *
 * Input Parameters:
 *   route - The route to be freed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
void net_freeroute_ipv4(FAR struct net_route_ipv4_s *route);
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
void net_freeroute_ipv6(FAR struct net_route_ipv6_s *route);
#endif

/****************************************************************************
 * Name: (various low-level list operations)
 *
 * Description:
 *   Perform operations on in-memory routing table lists
 *
 * Input Parameters:
 *   entry - A pointer to the new entry to add to the list
 *   list - The list to be used.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
void ramroute_ipv4_addlast(FAR struct net_route_ipv4_entry_s *entry,
                           FAR struct net_route_ipv4_queue_s *list);
FAR struct net_route_ipv4_entry_s *
ramroute_ipv4_remfirst(FAR struct net_route_ipv4_queue_s *list);
FAR struct net_route_ipv4_entry_s *
ramroute_ipv4_remafter(FAR struct net_route_ipv4_entry_s *entry,
                       FAR struct net_route_ipv4_queue_s *list);
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
void ramroute_ipv6_addlast(FAR struct net_route_ipv6_entry_s *entry,
                           FAR struct net_route_ipv6_queue_s *list);
FAR struct net_route_ipv6_entry_s *
ramroute_ipv6_remfirst(FAR struct net_route_ipv6_queue_s *list);
FAR struct net_route_ipv6_entry_s *
ramroute_ipv6_remafter(FAR struct net_route_ipv6_entry_s *entry,
                       FAR struct net_route_ipv6_queue_s *list);
#endif

#endif /* CONFIG_ROUTE_IPv4_RAMROUTE || CONFIG_ROUTE_IPv6_RAMROUTE */
#endif /* __NET_ROUTE_RAMROUTE_H */
