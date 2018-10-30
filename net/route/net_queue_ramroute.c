/****************************************************************************
 * net/route/net_queue_ramroute.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
  ramroute_ipv4_remfirst(struct net_route_ipv4_queue_s *list)
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
  ramroute_ipv6_remfirst(struct net_route_ipv6_queue_s *list)
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
  struct net_route_ipv6_queue_s *list)
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
