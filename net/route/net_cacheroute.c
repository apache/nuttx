/****************************************************************************
 * net/route/net_cacheroute.c
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

#include <string.h>
#include <semaphore.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/semaphore.h>

#include "route/cacheroute.h"
#include "route/route.h"

#if defined(CONFIG_ROUTE_IPv4_CACHEROUTE) || defined(CONFIG_ROUTE_IPv6_CACHEROUTE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_ROUTE_MAX_IPv4_CACHEROUTES
#  define CONFIG_ROUTE_MAX_IPv4_CACHEROUTES 4
#endif

#ifndef CONFIG_ROUTE_MAX_IPv6_CACHEROUTES
#  define CONFIG_ROUTE_MAX_IPv6_CACHEROUTES 4
#endif

/* Routing table initializer */

#define cacheroute_init(rr) \
  do \
    { \
      (rr)->head = NULL; \
      (rr)->tail = NULL; \
    } \
  while (0) \

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
/* This structure describes one entry in the routing table cache */

struct net_cache_ipv4_entry_s
{
  struct net_route_ipv4_s entry;
  FAR struct net_cache_ipv4_entry_s *flink;
};

/* This structure describes the head of a routing table cache list */

struct net_cache_ipv4_queue_s
{
  FAR struct net_cache_ipv4_entry_s *head;
  FAR struct net_cache_ipv4_entry_s *tail;
};
#endif

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
/* This structure describes one entry in the routing table cache */

struct net_cache_ipv6_entry_s
{
  struct net_route_ipv6_s entry;
  FAR struct net_cache_ipv6_entry_s *flink;
};

/* This structure describes the head of a routing table cache list */

struct net_cache_ipv6_queue_s
{
  FAR struct net_cache_ipv6_entry_s *head;
  FAR struct net_cache_ipv6_entry_s *tail;
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These are the routing tables */

#if defined(CONFIG_ROUTE_IPv4_CACHEROUTE)
/* The in-memory cache as a singly linked list. */

static struct net_cache_ipv4_queue_s g_ipv4_cache;

/* List of free routing table cache entries */

static struct net_cache_ipv4_queue_s g_free_ipv4cache;

/* Pre-allocated routing table cache entries */

static struct net_cache_ipv4_entry_s
  g_prealloc_ipv4cache[CONFIG_ROUTE_MAX_IPv4_CACHEROUTES];

/* Serializes access to the routing table cache */

static sem_t g_ipv4_cachelock;
#endif

#if defined(CONFIG_ROUTE_IPv6_CACHEROUTE)
/* The in-memory routing tables are represented as singly linked lists. */

static struct net_cache_ipv6_queue_s g_ipv6_cache;

/* List of free routing table cache entries */

static struct net_cache_ipv6_queue_s g_free_ipv6cache;

/* Pre-allocated routing table cache entries */

static struct net_cache_ipv6_entry_s
  g_prealloc_ipv6cache[CONFIG_ROUTE_MAX_IPv6_CACHEROUTES];

/* Serializes access to the routing table cache */

static sem_t g_ipv6_cachelock;
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: net_lock_ipv4_cache and net_lock_ipv6_cache
 *
 * Description:
 *   Lock the routing table cache list.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success;  A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

#define net_lock_ipv4_cache() nxsem_wait(&g_ipv4_cachelock);
#define net_lock_ipv6_cache() nxsem_wait(&g_ipv6_cachelock);

/****************************************************************************
 * Name: net_unlock_ipv4_cache and net_unlock_ipv6_cache
 *
 * Description:
 *   Unlock the routing table cache list.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#define net_unlock_ipv4_cache() (void)nxsem_post(&g_ipv4_cachelock)
#define net_unlock_ipv6_cache() (void)nxsem_post(&g_ipv6_cachelock)

/****************************************************************************
 * Name: net_add_newest_ipv4 and net_add_newest_ipv6
 *
 * Description:
 *   Add a new entry to the routing table cache list.  The list is ordered
 *   by "new-ness" so this would be the entry at the head of the list.
 *
 * Input Parameters:
 *   cache - The cache entry to add to the head of the routing table cache
 *           list.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Caller has the routing table cache locked.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
static void net_add_newest_ipv4(FAR struct net_cache_ipv4_entry_s *cache)
{
  cache->flink = g_ipv4_cache.head;
  if (!g_ipv4_cache.head)
    {
      g_ipv4_cache.tail = cache;
    }

  g_ipv4_cache.head = cache;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
static void net_add_newest_ipv6(FAR struct net_cache_ipv6_entry_s *cache)
{
  cache->flink = g_ipv6_cache.head;
  if (!g_ipv6_cache.head)
    {
      g_ipv6_cache.tail = cache;
    }

  g_ipv6_cache.head = cache;
}
#endif

/****************************************************************************
 * Name: net_remove_oldest_ipv4 and net_remove_oldest_ipv6
 *
 * Description:
 *   Remove the oldest entry from the routing table cache list.  The list is
 *   ordered
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a pointer to the oldest routing table cache entry is
 *   returned.  NULL would be returned if the routing table cache is empty.
 *
 * Assumptions:
 *   Caller has the routing table cache locked.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
FAR struct net_cache_ipv4_entry_s *net_remove_oldest_ipv4(void)
{
  FAR struct net_cache_ipv4_entry_s *cache;
  FAR struct net_cache_ipv4_entry_s *prev;

  cache = g_ipv4_cache.tail;
  if (cache != NULL)
    {
      if (g_ipv4_cache.head == g_ipv4_cache.tail)
        {
          g_ipv4_cache.head = NULL;
          g_ipv4_cache.tail = NULL;
        }
      else
        {
          for (prev  = g_ipv4_cache.head;
               prev != NULL && prev->flink != cache;
               prev  = prev->flink);

          if (prev != NULL)
            {
              prev->flink       = NULL;
              g_ipv4_cache.tail = prev;
            }
        }

      cache->flink = NULL;
    }

  return cache;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
FAR struct net_cache_ipv6_entry_s *net_remove_oldest_ipv6(void)
{
  FAR struct net_cache_ipv6_entry_s *cache;
  FAR struct net_cache_ipv6_entry_s *prev;

  cache = g_ipv6_cache.tail;
  if (cache != NULL)
    {
      if (g_ipv6_cache.head == g_ipv6_cache.tail)
        {
          g_ipv6_cache.head = NULL;
          g_ipv6_cache.tail = NULL;
        }
      else
        {
          for (prev  = g_ipv6_cache.head;
               prev != NULL && prev->flink != cache;
               prev  = prev->flink);

          if (prev != NULL)
            {
              prev->flink       = NULL;
              g_ipv6_cache.tail = prev;
            }
        }

      cache->flink = NULL;
    }

  return cache;
}
#endif

/****************************************************************************
 * Name: net_alloccache_ipv4 and net_alloccache_ipv6
 *
 * Description:
 *   Allocate one routing table cache entry by removing it from the free
 *   list.  If the free list is empty, then remove the entry at the tail
 *   of the current routing table cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, a pointer to the newly allocated routing table cache entry
 *   is returned.  Should never fail
 *
 * Assumptions:
 *   Caller has the routing table cache locked.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
FAR struct net_cache_ipv4_entry_s *net_alloccache_ipv4(void)
{
  FAR struct net_cache_ipv4_entry_s *cache;

  /* Remove the first entry from the free list */

  cache = g_free_ipv4cache.head;
  if (cache != NULL)
    {
      g_free_ipv4cache.head = cache->flink;
      if (g_free_ipv4cache.head == NULL)
        {
          g_free_ipv4cache.tail = NULL;
        }

      cache->flink = NULL;
    }

  /* If the free list is empty, then remove the oldest entry at the tail of
   * the routing table cache.
   */

  else
    {
      /* If the free list is empty, then the cache list cannot be empty */

      cache = net_remove_oldest_ipv4();
      DEBUGASSERT(cache != NULL);
    }

  return cache;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
FAR struct net_cache_ipv6_entry_s *net_alloccache_ipv6(void)
{
  FAR struct net_cache_ipv6_entry_s *cache;

  /* Remove the first entry from the free list */

  cache = g_free_ipv6cache.head;
  if (cache != NULL)
    {
      g_free_ipv6cache.head = cache->flink;
      if (g_free_ipv6cache.head == NULL)
        {
          g_free_ipv6cache.tail = NULL;
        }

      cache->flink = NULL;
    }

  /* If the free list is empty, then remove the oldest entry at the tail of
   * the routing table cache.
   */

  else
    {
      /* If the free list is empty, then the cache list cannot be empty */

      cache = net_remove_oldest_ipv6();
      DEBUGASSERT(cache != NULL);
    }

  return cache;
}
#endif

/****************************************************************************
 * Name: net_reset_ipv4_cache and net_reset_ipv6_cache
 *
 * Description:
 *   Clear the routing table cache and return the entries to the free list.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
static void net_reset_ipv4_cache(void)
{
  int i;

  cacheroute_init(&g_ipv4_cache);
  cacheroute_init(&g_free_ipv4cache);

  /* Add all of the pre-allocated routing table cache entries to a free list */

  for (i = 0; i < CONFIG_ROUTE_MAX_IPv4_CACHEROUTES; i++)
    {
      net_add_newest_ipv4(&g_prealloc_ipv4cache[i]);
    }
}
#endif

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
static void net_reset_ipv6_cache(void)
{
  int i;

  cacheroute_init(&g_ipv6_cache);
  cacheroute_init(&g_free_ipv6cache);

  /* Add all of the pre-allocated routing table entries to a free list */

  for (i = 0; i < CONFIG_ROUTE_MAX_IPv6_CACHEROUTES; i++)
    {
      net_add_newest_ipv6(&g_prealloc_ipv6cache[i]);
    }

}
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: net_init_cacheroute
 *
 * Description:
 *   Initialize the in-memory, routing table cache
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

void net_init_cacheroute(void)
{
  /* Initialize the routing table cash and the free list */

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
  nxsem_init(&g_ipv4_cachelock, 0, 1);
  net_reset_ipv4_cache();
#endif

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
  nxsem_init(&g_ipv6_cachelock, 0, 1);
  net_reset_ipv6_cache();
#endif
}

/****************************************************************************
 * Name: net_addcache_ipv4 and net_addcache_ipv6
 *
 * Description:
 *   Add one route to the routing table cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
int net_addcache_ipv4(FAR struct net_route_ipv4_s *route)
{
  FAR struct net_cache_ipv4_entry_s *cache;
  FAR struct net_cache_ipv4_entry_s *prev;
  int ret;

  DEBUGASSERT(route != NULL);

  /* Get exclusive access to the cache */

  ret = net_lock_ipv4_cache();
  if (ret < 0)
    {
      return ret;
    }

  /* First, check if the route already exists in the cache */
  /* Visit each entry in the cache */

  for (prev = NULL, cache = g_ipv4_cache.head;
       cache != NULL;
       prev = cache, cache = cache->flink)
    {
      /* To match, the masked target addresses and netmasks must be the
       * same.
       */

      if (net_ipv4addr_cmp(route->target, cache->entry.target) &&
          net_ipv4addr_cmp(route->netmask, cache->entry.netmask))
        {
          /* If the route already exists and is already at the head of the
           * list, then do nothing.  It is already the most recently used.
           */

          if (prev == NULL)
            {
              net_unlock_ipv4_cache();
              return OK;
            }

          /* Otherwise, remove the cache entry from the middle or end of
           * the list.
           */

          prev->flink = cache->flink;
          if (g_ipv4_cache.tail == cache)
            {
              g_ipv4_cache.tail = prev;
            }

          cache->flink = NULL;
          break;
        }
    }

  /* If we did not find the entry in the list, then we will have to
   * allocate a new entry.
   */

  if (cache == NULL)
    {
      /* Allocate a new cache entry (should never fail) */

      cache = net_alloccache_ipv4();
      DEBUGASSERT(cache != NULL);

      /* Copy the routing table entry into the allocated cache entry. */

      memcpy(&cache->entry, route, sizeof(struct net_route_ipv4_s));
    }

  /* Then add the new cache entry as the newest entry in the table */

  net_add_newest_ipv4(cache);
  net_unlock_ipv4_cache();
  return OK;
}
#endif

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
int net_addcache_ipv6(FAR struct net_route_ipv6_s *route)
{
  FAR struct net_cache_ipv6_entry_s *cache;
  FAR struct net_cache_ipv6_entry_s *prev;
  int ret;

  DEBUGASSERT(route != NULL);

  /* Get exclusive access to the cache */

  ret = net_lock_ipv6_cache();
  if (ret < 0)
    {
      return ret;
    }

  /* First, check if the route already exists in the cache */
  /* Visit each entry in the cache */

  for (prev = NULL, cache = g_ipv6_cache.head;
       cache != NULL;
       prev = cache, cache = cache->flink)
    {
      /* To match, the masked target addresses and netmasks must be the
       * same.
       */

      if (net_ipv6addr_cmp(route->target, cache->entry.target) &&
          net_ipv6addr_cmp(route->netmask, cache->entry.netmask))
        {
          /* If the route already exists and is already at the head of the
           * list, then do nothing.  It is already the most recently used.
           */

          if (prev == NULL)
            {
              net_unlock_ipv6_cache();
              return OK;
            }

          /* Otherwise, remove the cache entry from the middle or end of
           * the list.
           */

          prev->flink = cache->flink;
          if (g_ipv6_cache.tail == cache)
            {
              g_ipv6_cache.tail = prev;
            }

          cache->flink = NULL;
          break;
        }
    }

  /* If we did not find the entry in the list, then we will have to
   * allocate a new entry.
   */

  if (cache == NULL)
    {
      /* Allocate a new cache entry (should never fail) */

      cache = net_alloccache_ipv6();
      DEBUGASSERT(cache != NULL);

      /* Copy the routing table entry into the allocated cache entry */

      memcpy(&cache->entry, route, sizeof(struct net_route_ipv6_s));
    }

  /* Then add the new cache entry as the newest entry in the table */

  net_add_newest_ipv6(cache);
  net_unlock_ipv6_cache();
  return OK;
}
#endif

/****************************************************************************
 * Name: net_foreachcache_ipv4/net_foreachcache_ipv6
 *
 * Description:
 *   Traverse the routing table cahce
 *
 * Input Parameters:
 *   handler - Will be called for each route in the routing table cache.
 *   arg     - An arbitrary value that will be passed tot he handler.
 *
 * Returned Value:
 *   Zero (OK) returned if the entire table was searched.  A negated errno
 *   value will be returned in the event of a failure.  Handlers may also
 *   terminate the search early with any non-zero value.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_IPv4
int net_foreachcache_ipv4(route_handler_ipv4_t handler, FAR void *arg)
{
  FAR struct net_cache_ipv4_entry_s *cache;
  FAR struct net_cache_ipv4_entry_s *next;
  int ret = 0;

  /* Get exclusive access to the cache */

  ret = net_lock_ipv4_cache();
  if (ret < 0)
    {
      return ret;
    }

  /* Visit each entry in the routing table */

  for (cache = g_ipv4_cache.head; ret == 0 && cache != NULL; cache = next)
    {
      /* Get the next entry in the to visit.  We do this BEFORE calling the
       * handler because the hanlder may delete this entry.
       */

      next = cache->flink;
      ret  = handler(&cache->entry, arg);
    }

  /* Unlock the cache */

  net_unlock_ipv4_cache();
  return ret;
}
#endif

#ifdef CONFIG_NET_IPv6
int net_foreachcache_ipv6(route_handler_ipv6_t handler, FAR void *arg)
{
  FAR struct net_cache_ipv6_entry_s *cache;
  FAR struct net_cache_ipv6_entry_s *next;
  int ret = 0;

  /* Get exclusive access to the cache */

  ret = net_lock_ipv6_cache();
  if (ret < 0)
    {
      return ret;
    }

  /* Visit each entry in the routing table */

  for (cache = g_ipv6_cache.head; ret == 0 && cache != NULL; cache = next)
    {
      /* Get the next entry in the to visit.  We do this BEFORE calling the
       * handler because the hanlder may delete this entry.
       */

      next = cache->flink;
      ret  = handler(&cache->entry, arg);
    }

  /* Unlock the cache */

  net_unlock_ipv6_cache();
  return ret;
}
#endif

/****************************************************************************
 * Name: net_flushcache_ipv4 and net_flushcache_ipv6
 *
 * Description:
 *   Flush the content of the routing table cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ROUTE_IPv4_CACHEROUTE
void net_flushcache_ipv4(void)
{
  int ret;

  /* Get exclusive access to the cache */

  do
    {
      ret = net_lock_ipv4_cache();
    }
  while (ret == -EINTR);
  DEBUGASSERT(ret == OK);

  /* Reset the cache */

  net_reset_ipv4_cache();

  /* Unlock the cache */

  net_unlock_ipv4_cache();
}
#endif

#ifdef CONFIG_ROUTE_IPv6_CACHEROUTE
void net_flushcache_ipv6(void)
{
  int ret;

  /* Get exclusive access to the cache */

  do
    {
      ret = net_lock_ipv6_cache();
    }
  while (ret == -EINTR);
  DEBUGASSERT(ret == OK);

  /* Reset the cache */

  net_reset_ipv6_cache();

  /* Unlock the cache */

  net_unlock_ipv6_cache();
}
#endif

#endif /* CONFIG_ROUTE_IPv4_CACHEROUTE || CONFIG_ROUTE_IPv6_CACHEROUTE */
