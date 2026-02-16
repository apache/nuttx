/****************************************************************************
 * libs/libc/netdb/lib_dnsdelserver.c
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

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/ip.h>
#include <nuttx/net/dns.h>

#include "netdb/lib_dns.h"

#ifdef CONFIG_NETDB_DNSCLIENT

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_NETDB_RESOLVCONF

static int dns_find_nameserver_index(FAR const struct sockaddr *addr,
                                     socklen_t addrlen)
{
  size_t cmplen = 0;
  int i;

#ifdef CONFIG_NET_IPv4
  if (addr->sa_family == AF_INET)
    {
      cmplen = sizeof(struct sockaddr_in);
    }
#endif

#ifdef CONFIG_NET_IPv6
  if (addr->sa_family == AF_INET6)
    {
      cmplen = sizeof(struct sockaddr_in6);
    }
#endif

  if (cmplen == 0)
    {
      /* Unsupported address family */

      return -ENOSYS;
    }

  if (addrlen < cmplen)
    {
      return -EINVAL;
    }

  dns_lock();

  /* Search for the matching nameserver in the array */

  for (i = 0; i < g_dns_nservers; i++)
    {
      if (g_dns_servers[i].addr.sa_family == addr->sa_family)
        {
#ifdef CONFIG_NET_IPv4
          if (addr->sa_family == AF_INET)
            {
              FAR struct sockaddr_in *in1 =
                  (FAR struct sockaddr_in *)&g_dns_servers[i].addr;
              FAR struct sockaddr_in *in2 =
                  (FAR struct sockaddr_in *)addr;

              /* Compare only the IP address part, ignore port and padding */

              if (net_ipv4addr_cmp(in1->sin_addr.s_addr,
                                   in2->sin_addr.s_addr))
                {
                  dns_unlock();
                  return i;
                }
            }
#endif

#ifdef CONFIG_NET_IPv6
          if (addr->sa_family == AF_INET6)
            {
              FAR struct sockaddr_in6 *in6_1 =
                  (FAR struct sockaddr_in6 *)&g_dns_servers[i].addr;
              FAR struct sockaddr_in6 *in6_2 =
                  (FAR struct sockaddr_in6 *)addr;

              /* Compare only the IPv6 address part */

              if (net_ipv6addr_cmp(in6_1->sin6_addr.s6_addr,
                                   in6_2->sin6_addr.s6_addr))
                {
                  dns_unlock();
                  return i;
                }
            }
#endif
        }
    }

  dns_unlock();
  return -ENOENT;
}

#endif /* !CONFIG_NETDB_RESOLVCONF */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_del_nameserver
 *
 * Description:
 *   Remove a DNS server from the list by matching the address
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_RESOLVCONF

int dns_del_nameserver(FAR const struct sockaddr *addr, socklen_t addrlen)
{
  /* Not implemented for CONFIG_NETDB_RESOLVCONF */

  return -ENOSYS;
}

int dns_del_nameserver_by_index(int index)
{
  /* For resolv.conf mode, removing requires rewriting the file */

  return -ENOSYS;
}

#else /* !CONFIG_NETDB_RESOLVCONF */

int dns_del_nameserver(FAR const struct sockaddr *addr, socklen_t addrlen)
{
  int index;

  if (addr == NULL)
    {
      return -EINVAL;
    }

  /* Find the nameserver in the array */

  index = dns_find_nameserver_index(addr, addrlen);
  if (index < 0)
    {
      return index;  /* Return the error code */
    }

  /* Remove the server by index */

  return dns_del_nameserver_by_index(index);
}

int dns_del_nameserver_by_index(int index)
{
  int i;

  if (index < 0 || index >= CONFIG_NETDB_DNSSERVER_NAMESERVERS)
    {
      return -EINVAL;
    }

  dns_lock();

  if (index >= g_dns_nservers)
    {
      dns_unlock();
      return -ENOENT;
    }

  /* Shift all subsequent entries down by one position */

  for (i = index; i < g_dns_nservers - 1; i++)
    {
      memcpy(&g_dns_servers[i], &g_dns_servers[i + 1],
             sizeof(union dns_addr_u));
    }

  memset(&g_dns_servers[g_dns_nservers - 1], 0, sizeof(union dns_addr_u));

  /* Decrement the server count */

  g_dns_nservers--;

  dns_unlock();

#if CONFIG_NETDB_DNSCLIENT_ENTRIES > 0
  /* Clear the DNS cache after removing a server */

  dns_clear_answer();
#endif

  return OK;
}

#endif /* CONFIG_NETDB_RESOLVCONF */
#endif /* CONFIG_NETDB_DNSCLIENT */
