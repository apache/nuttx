/****************************************************************************
 * libs/libc/netdb/lib_dnscache.c
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

#include <sys/time.h>
#include <string.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include "netdb/lib_dns.h"

#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This described one entry in the cache of resolved hostnames.
 *
 * REVISIT: this consumes extra space, especially when multiple
 * addresses per name are stored.
 */

struct dns_cache_s
{
#if CONFIG_NETDB_DNSCLIENT_LIFESEC > 0
  time_t            ctime;      /* Creation time */
#endif
  char              name[CONFIG_NETDB_DNSCLIENT_NAMESIZE];
  uint8_t           naddr;      /* How many addresses per name */
  union dns_addr_u  addr[CONFIG_NETDB_MAX_IPADDR];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_dns_head;        /* Head of the circular, DNS resolver cache */
static uint8_t g_dns_tail;        /* Tail of the circular, DNS resolver cache */

/* This is the DNS resolver cache */

static struct dns_cache_s g_dns_cache[CONFIG_NETDB_DNSCLIENT_ENTRIES];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_save_answer
 *
 * Description:
 *   Save the last resolved hostname in the DNS cache
 *
 * Input Parameters:
 *   hostname - The hostname string to be cached.
 *   addr     - The IP addresses associated with the hostname.
 *   naddr    - The count of the IP addresses.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void dns_save_answer(FAR const char *hostname,
                     FAR const union dns_addr_u *addr, int naddr)
{
  FAR struct dns_cache_s *entry;
#if CONFIG_NETDB_DNSCLIENT_LIFESEC > 0
  struct timespec now;
#endif
  int next;
  int ndx;

  naddr = MIN(naddr, CONFIG_NETDB_MAX_IPADDR);
  DEBUGASSERT(naddr >= 1 && naddr <= UCHAR_MAX);

  /* Get exclusive access to the DNS cache */

  dns_semtake();

  /* Get the index to the new head of the list */

  ndx  = g_dns_head;
  next = ndx + 1;
  if (next >= CONFIG_NETDB_DNSCLIENT_ENTRIES)
    {
      next = 0;
    }

  /* If the next head pointer would match the tail index, then increment
   * the tail index, discarding the oldest mapping in the cache.
   */

  if (next == g_dns_tail)
    {
      int tmp = g_dns_tail + 1;
      if (tmp >= CONFIG_NETDB_DNSCLIENT_ENTRIES)
        {
          tmp = 0;
        }

      g_dns_tail = tmp;
    }

  /* Save the answer in the cache */

  entry = &g_dns_cache[ndx];

#if CONFIG_NETDB_DNSCLIENT_LIFESEC > 0
  /* Get the current time */

  clock_gettime(CLOCK_MONOTONIC, &now);
  entry->ctime = (time_t)now.tv_sec;
#endif

  strncpy(entry->name, hostname, CONFIG_NETDB_DNSCLIENT_NAMESIZE);
  memcpy(&entry->addr, addr, naddr * sizeof(*addr));
  entry->naddr = naddr;

  /* Save the updated head index */

  g_dns_head = next;
  dns_semgive();
}

/****************************************************************************
 * Name: dns_clear_answer
 *
 * Description:
 *   Clear the resolved hostname in the DNS cache
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void dns_clear_answer(void)
{
  /* Get exclusive access to the DNS cache */

  dns_semtake();

  /* Reset the circular of DNS cache */

  g_dns_head = 0;
  g_dns_tail = 0;

  dns_semgive();
}

/****************************************************************************
 * Name: dns_find_answer
 *
 * Description:
 *   Check if we already have the resolved hostname address in the cache.
 *
 * Input Parameters:
 *   hostname - The hostname string to be resolved.
 *   addr     - The location to return the IP addresses associated with the
 *     hostname.
 *   naddr    - On entry, the count of addresses backing up the 'addr'
 *     pointer.  On return, this location will hold the actual count of
 *     the returned addresses.
 *
 * Returned Value:
 *   If the host name was successfully found in the DNS name resolution
 *   cache, zero (OK) will be returned.  Otherwise, some negated errno
 *   value will be returned, typically -ENOENT meaning that the hostname
 *   was not found in the cache.
 *
 ****************************************************************************/

int dns_find_answer(FAR const char *hostname, FAR union dns_addr_u *addr,
                    FAR int *naddr)
{
  FAR struct dns_cache_s *entry;
#if CONFIG_NETDB_DNSCLIENT_LIFESEC > 0
  struct timespec now;
  uint32_t elapsed;
  int ret;
#endif
  int next;
  int ndx;

  /* Get exclusive access to the DNS cache */

  dns_semtake();

#if CONFIG_NETDB_DNSCLIENT_LIFESEC > 0
  /* Get the current time */

  ret = clock_gettime(CLOCK_MONOTONIC, &now);
#endif

  for (ndx = g_dns_tail; ndx != g_dns_head; ndx = next)
    {
      entry = &g_dns_cache[ndx];

      /* Advance the index for the next time through the loop, handling
       * wrapping to the beginning of the circular buffer.
       */

      next = ndx + 1;
      if (next >= CONFIG_NETDB_DNSCLIENT_ENTRIES)
        {
          next = 0;
        }

#if CONFIG_NETDB_DNSCLIENT_LIFESEC > 0
      /* Check if this entry has expired
       * REVISIT: Does not this calculation assume that the sizeof(time_t)
       * is equal to the sizeof(uint32_t)?
       */

      elapsed = (uint32_t)now.tv_sec - (uint32_t)entry->ctime;
      if (ret >= 0 && elapsed > CONFIG_NETDB_DNSCLIENT_LIFESEC)
        {
          /* This entry has expired.  Increment the tail index to exclude
           * this entry on future traversals.
           */

          g_dns_tail = next;
        }
      else
#endif
        {
          /* The entry has not expired, check for a name match. Because
           * the names are truncated to CONFIG_NETDB_DNSCLIENT_NAMESIZE,
           * this has the possibility of aliasing two names and returning
           * the wrong entry from the cache.
           */

          if (strncmp(hostname, entry->name,
                      CONFIG_NETDB_DNSCLIENT_NAMESIZE) == 0)
            {
              /* We have a match.  Return the resolved host address */

              /* Make sure that the address will fit in the caller-provided
               * buffer.
               */

              *naddr = MIN(*naddr, entry->naddr);

              /* Return the address information */

              memcpy(addr, &entry->addr, *naddr * sizeof(*addr));

              dns_semgive();
              return OK;
            }
        }
    }

  ret = -ENOENT;

  dns_semgive();
  return ret;
}

#endif /* CONFIG_NETDB_DNSCLIENT_ENTRIES > 0 */
