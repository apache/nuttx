/****************************************************************************
 * net/route/net_queue_ramroute.c
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

#include "route/ramroute.h"
#include "route/route.h"

#if defined(CONFIG_ROUTE_IPv4_RAMROUTE) || defined(CONFIG_ROUTE_IPv6_RAMROUTE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ramroute_ipv4_addlast/ramroute_ipv6_addlast
 *
 * Description:
 *   Remove the entry at the end of an IPv4/IPv6 routing table list
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
                           FAR struct net_route_ipv4_queue_s *list)
{
  entry->flink = NULL;
  if (!list->head)
    {
      list->head = entry;
      list->tail = entry;
    }
  else
    {
      list->tail->flink = entry;
      list->tail        = entry;
    }
}
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
void ramroute_ipv6_addlast(FAR struct net_route_ipv6_entry_s *entry,
                           FAR struct net_route_ipv6_queue_s *list)
{
  entry->flink = NULL;
  if (!list->head)
    {
      list->head = entry;
      list->tail = entry;
    }
  else
    {
      list->tail->flink = entry;
      list->tail        = entry;
    }
}
#endif

/****************************************************************************
 * Name: ramroute_ipv4_remfirst/ramroute_ipv6_remfirst
 *
 * Description:
 *   Add an entry to the end of an IPv4/IPv6 routing table list
 *
 * Input Parameters:
 *   entry - A pointer to the new entry to add to the list
 *   list - The list to be used.
 *
 * Returned Value:
 *   A pointer to the entry removed from the head of the list or NULL if the
 *   list was empty.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
FAR struct net_route_ipv4_entry_s *
ramroute_ipv4_remfirst(FAR struct net_route_ipv4_queue_s *list)
{
  FAR struct net_route_ipv4_entry_s *ret = list->head;

  if (ret)
    {
      list->head = ret->flink;
      if (!list->head)
        {
          list->tail = NULL;
        }

      ret->flink = NULL;
    }

  return ret;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
FAR struct net_route_ipv6_entry_s *
ramroute_ipv6_remfirst(FAR struct net_route_ipv6_queue_s *list)
{
  FAR struct net_route_ipv6_entry_s *ret = list->head;

  if (ret)
    {
      list->head = ret->flink;
      if (!list->head)
        {
          list->tail = NULL;
        }

      ret->flink = NULL;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: ramroute_ipv4_remafter/ramroute_ipv6_remafter
 *
 * Description:
 *   Remove the entry in the IPv4/IPv6 routing table list after the specified
 *   entry.
 *
 * Input Parameters:
 *   entry - A pointer to the new entry to add to the list
 *   list - The list to be used.
 *
 * Returned Value:
 *   A pointer to the entry removed from the head of the list or NULL if the
 *   list was empty.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_RAMROUTE
FAR struct net_route_ipv4_entry_s *
ramroute_ipv4_remafter(FAR struct net_route_ipv4_entry_s *entry,
                       FAR struct net_route_ipv4_queue_s *list)
{
  FAR struct net_route_ipv4_entry_s *ret = entry->flink;

  if (list->head && ret)
    {
      if (list->tail == ret)
        {
          list->tail = entry;
          entry->flink = NULL;
        }
      else
        {
          entry->flink = ret->flink;
        }

      ret->flink = NULL;
    }

  return ret;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_RAMROUTE
FAR struct net_route_ipv6_entry_s *
ramroute_ipv6_remafter(FAR struct net_route_ipv6_entry_s *entry,
                       FAR struct net_route_ipv6_queue_s *list)
{
  FAR struct net_route_ipv6_entry_s *ret = entry->flink;

  if (list->head && ret)
    {
      if (list->tail == ret)
        {
          list->tail = entry;
          entry->flink = NULL;
        }
      else
        {
          entry->flink = ret->flink;
        }

      ret->flink = NULL;
    }

  return ret;
}
#endif

#endif /* CONFIG_ROUTE_IPv4_RAMROUTE || CONFIG_ROUTE_IPv4_RAMROUTE */
