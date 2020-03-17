/****************************************************************************
 * libs/libc/netdb/lib_dnsaddserver.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/ip.h>
#include <nuttx/net/dns.h>

#include "netdb/lib_dns.h"

#ifdef CONFIG_NETDB_DNSCLIENT

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef CONFIG_NETDB_RESOLVCONF
/* The DNS server address */

union dns_addr_u g_dns_server;
bool g_dns_address;     /* true: We have the address of the DNS server */
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dns_add_nameserver
 *
 * Description:
 *   Configure which DNS server to use for queries
 *
 ****************************************************************************/

#ifdef CONFIG_NETDB_RESOLVCONF
int dns_add_nameserver(FAR const struct sockaddr *addr, socklen_t addrlen)
{
  FAR FILE *stream;
  char addrstr[DNS_MAX_ADDRSTR];
#ifdef CONFIG_NETDB_RESOLVCONF_NONSTDPORT
  uint16_t port;
#endif
  int status;
  int ret;

  stream = fopen(CONFIG_NETDB_RESOLVCONF_PATH, "a+");
  if (stream == NULL)
    {
      int errcode = get_errno();
      nerr("ERROR: Failed to open %s: %d\n",
           CONFIG_NETDB_RESOLVCONF_PATH, errcode);
      DEBUGASSERT(errcode > 0);
      return -errcode;
    }

#ifdef CONFIG_NET_IPv4
  /* Check for an IPv4 address */

  if (addr->sa_family == AF_INET)
    {
      if (addrlen < sizeof(struct sockaddr_in))
        {
          ret = -EINVAL;
          goto errout;
        }
      else
        {
          FAR struct sockaddr_in *in4 = (FAR struct sockaddr_in *)addr;

          if (inet_ntop(AF_INET, &in4->sin_addr, addrstr,
                        DNS_MAX_ADDRSTR) == NULL)
            {
              ret = -errno;
              nerr("ERROR: inet_ntop failed: %d\n", errcode);
              DEBUGASSERT(errcode < 0);
              goto errout;
            }

#ifdef CONFIG_NETDB_RESOLVCONF_NONSTDPORT
          /* Get the port number */

          port = ntohs(in4->sin_port);
#endif
        }
    }
  else
#endif

#ifdef CONFIG_NET_IPv6
  /* Check for an IPv6 address */

  if (addr->sa_family == AF_INET6)
    {
      if (addrlen < sizeof(struct sockaddr_in6))
        {
          ret = -EINVAL;
          goto errout;
        }
      else
        {
          FAR struct sockaddr_in6 *in6 = (FAR struct sockaddr_in6 *)addr;

          if (inet_ntop(AF_INET6, &in6->sin6_addr, addrstr,
                        DNS_MAX_ADDRSTR) == NULL)
            {
              ret = -errno;
              nerr("ERROR: inet_ntop failed: %d\n", errcode);
              DEBUGASSERT(errcode < 0);
              goto errout;
            }

#ifdef CONFIG_NETDB_RESOLVCONF_NONSTDPORT
          /* Get the port number */

          port = ntohs(in6->sin6_port);
#endif
        }
    }
  else
#endif
    {
      nerr("ERROR: Unsupported family: %d\n",
            g_dns_server.addr.sa_family);
      ret = -ENOSYS;
      goto errout;
    }

  /* Write the new record to the end of the resolv.conf file. */

#ifdef CONFIG_NETDB_RESOLVCONF_NONSTDPORT
  /* The OpenBSD version supports a [host]:port syntax.  When a non-standard
   * port is specified the host address must be enclosed in square brackets.
   * For example:
   *
   *   nameserver [10.0.0.1]:5353
   *   nameserver [::1]:5353
   */

  if (port != 0 && port != DNS_DEFAULT_PORT)
    {
      status = fprintf(stream, "%s [%s]:%u\n",
                       NETDB_DNS_KEYWORD, addrstr, port);
    }
  else
#endif
    {
      status = fprintf(stream, "%s %s\n",
                       NETDB_DNS_KEYWORD, addrstr);
    }

  if (status < 0)
    {
      ret = -errno;
      nerr("ERROR: fprintf failed: %d\n", errcode);
      DEBUGASSERT(errcode < 0);
      goto errout;
    }

  dns_notify_nameserver(addr, addrlen);
  ret = OK;

errout:
  fclose(stream);
  return ret;
}

#else /* CONFIG_NETDB_RESOLVCONF */

int dns_add_nameserver(FAR const struct sockaddr *addr, socklen_t addrlen)
{
  FAR uint16_t *pport;
  size_t copylen;

  DEBUGASSERT(addr != NULL);

  /* Copy the new server IP address into our private global data structure */

#ifdef CONFIG_NET_IPv4
  /* Check for an IPv4 address */

  if (addr->sa_family == AF_INET)
    {
      /* Set up for the IPv4 address copy */

      copylen = sizeof(struct sockaddr_in);
      pport   = &g_dns_server.ipv4.sin_port;
    }
  else
#endif

#ifdef CONFIG_NET_IPv6
  /* Check for an IPv6 address */

  if (addr->sa_family == AF_INET6)
    {
      /* Set up for the IPv6 address copy */

      copylen = sizeof(struct sockaddr_in6);
      pport   = &g_dns_server.ipv6.sin6_port;
    }
  else
#endif
    {
      nerr("ERROR: Unsupported family: %d\n", addr->sa_family);
      return -ENOSYS;
    }

  /* Copy the IP address */

  if (addrlen < copylen)
    {
      nerr("ERROR: Invalid addrlen %ld for family %d\n",
            (long)addrlen, addr->sa_family);
      return -EINVAL;
    }

  memcpy(&g_dns_server.addr, addr, copylen);

  /* A port number of zero means to use the default DNS server port number */

  if (*pport == 0)
    {
      *pport = HTONS(DNS_DEFAULT_PORT);
    }

  /* We now have a valid DNS address */

  g_dns_address = true;
  dns_notify_nameserver(addr, addrlen);
  return OK;
}

#endif /* CONFIG_NETDB_RESOLVCONF */
#endif /* CONFIG_NETDB_DNSCLIENT */
