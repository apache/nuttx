/****************************************************************************
 * libs/libc/netdb/lib_dnscache.c
 *
 *   Copyright (C) 2007, 2009, 2012, 2014-2016 Gregory Nutt. All rights reserved.
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

#include <sys/time.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include "netdb/lib_dns.h"

#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Use clock monotonic, if possible */

#ifdef CONFIG_CLOCK_MONOTONIC
#  define DNS_CLOCK CLOCK_MONOTONIC
#else
#  define DNS_CLOCK CLOCK_REALTIME
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This described one entry in the cache of resolved hostnames */

struct dns_cache_s
{
#if CONFIG_NETDB_DNSCLIENT_LIFESEC > 0
  time_t              ctime;      /* Creation time */
#endif
  char                name[CONFIG_NETDB_DNSCLIENT_NAMESIZE];
  union dns_server_u  addr;       /* Resolved address */
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
 *   Same the last resolved hostname in the DNS cache
 *
 * Input Parameters:
 *   hostname - The hostname string to be cached.
 *   addr     - The IP address associated with the hostname
 *   addrlen  - The size of the of the IP address.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void dns_save_answer(FAR const char *hostname,
                     FAR const struct sockaddr *addr, socklen_t addrlen)
{
  FAR struct dns_cache_s *entry;
#if CONFIG_NETDB_DNSCLIENT_LIFESEC > 0
  struct timespec now;
#endif
  int next;
  int ndx;

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
  /* Get the current time, using CLOCK_MONOTONIC if possible */

  (void)clock_gettime(DNS_CLOCK, &now);
  entry->ctime = (time_t)now.tv_sec;
#endif

  strncpy(entry->name, hostname, CONFIG_NETDB_DNSCLIENT_NAMESIZE);
  memcpy(&entry->addr.addr, addr, addrlen);

  /* Save the updated head index */

  g_dns_head = next;
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
 *   addr     - The location to return the IP address associated with the
 *     hostname
 *   addrlen  - On entry, the size of the buffer backing up the 'addr'
 *     pointer.  On return, this location will hold the actual size of
 *     the returned address.
 *
 * Returned Value:
 *   If the host name was successfully found in the DNS name resolution
 *   cache, zero (OK) will be returned.  Otherwise, some negated errno
 *   value will be returned, typically -ENOENT meaning that the hostname
 *   was not found in the cache.
 *
 ****************************************************************************/

int dns_find_answer(FAR const char *hostname, FAR struct sockaddr *addr,
                    FAR socklen_t *addrlen)
{
  FAR struct dns_cache_s *entry;
#if CONFIG_NETDB_DNSCLIENT_LIFESEC > 0
  struct timespec now;
  uint32_t elapsed;
  int ret;
#endif
  int next;
  int ndx;

  /* If DNS not initialized, no need to proceed */

  if (!dns_initialize())
    {
      nerr("ERROR: DNS failed to initialize\n");
      return -EAGAIN;
    }

  /* Get exclusive access to the DNS cache */

  dns_semtake();

#if CONFIG_NETDB_DNSCLIENT_LIFESEC > 0
  /* Get the current time, using CLOCK_MONOTONIC if possible */

  ret = clock_gettime(DNS_CLOCK, &now);
#endif

  /* REVISIT: This is not thread safe */

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
          /* The entry has not expired, check for a name match.  Notice that
           * because the names are truncated to CONFIG_NETDB_DNSCLIENT_NAMESIZE,
           * this has the possibility of aliasing two names and returning
           * the wrong entry from the cache.
           */

          if (strncmp(hostname, entry->name, CONFIG_NETDB_DNSCLIENT_NAMESIZE) == 0)
            {
              socklen_t inlen;

              /* We have a match.  Return the resolved host address */

#ifdef CONFIG_NET_IPv4
#ifdef CONFIG_NET_IPv6
              if (entry->addr.addr.sa_family == AF_INET)
#endif
                {
                   inlen = sizeof(struct sockaddr_in);
                }
#endif

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_IPv4
              else
#endif
                {
                   inlen = sizeof(struct sockaddr_in6);
                }
#endif
              /* Make sure that the address will fit in the caller-provided
               * buffer.
               */

              if (*addrlen < inlen)
                {
                  ret = -ERANGE;
                  goto errout_with_sem;
                }

              /* Return the address information */

              memcpy(addr, &entry->addr.addr, inlen);
              *addrlen = inlen;

              dns_semgive();
              return OK;
            }
        }
    }

  ret = -ENOENT;

errout_with_sem:
  dns_semgive();
  return ret;
}

#endif /* CONFIG_NETDB_DNSCLIENT_ENTRIES > 0 */

