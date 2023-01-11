/****************************************************************************
 * libs/libc/netdb/lib_getaddrinfo.c
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

#include <string.h>

#include <arpa/inet.h>
#include <nuttx/net/loopback.h>
#include <netpacket/rpmsg.h>
#include <netdb.h>
#include <sys/un.h>

#include "libc.h"
#include "lib_netdb.h"

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct ai_s
{
  struct addrinfo ai;
  union
  {
    struct sockaddr_un sun;
    struct sockaddr_in sin;
    struct sockaddr_in6 sin6;
    struct sockaddr_rpmsg srp;
  } sa;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

FAR static struct ai_s *alloc_ai(int family, int socktype, int protocol,
                                 int port, FAR const void *addr)
{
  FAR struct ai_s *ai;

  ai = lib_zalloc(sizeof(struct ai_s));
  if (ai == NULL)
    {
      return ai;
    }

  ai->ai.ai_addr     = (FAR struct sockaddr *)&ai->sa;
  ai->ai.ai_family   = family;
  ai->ai.ai_socktype = socktype;
  ai->ai.ai_protocol = protocol;

  switch (family)
    {
#ifdef CONFIG_NET_LOCAL
      case AF_LOCAL:
        ai->ai.ai_addrlen       = sizeof(struct sockaddr_un);
        ai->sa.sun.sun_family   = AF_LOCAL;
        strlcpy(ai->sa.sun.sun_path, addr, sizeof(ai->sa.sun.sun_path));
        break;
#endif
#ifdef CONFIG_NET_IPv4
      case AF_INET:
        ai->ai.ai_addrlen       = sizeof(struct sockaddr_in);
        ai->sa.sin.sin_family   = AF_INET;
        ai->sa.sin.sin_port     = port;  /* Already network order */
        memcpy(&ai->sa.sin.sin_addr, addr, sizeof(ai->sa.sin.sin_addr));
        break;
#endif
#ifdef CONFIG_NET_IPv6
      case AF_INET6:
        ai->ai.ai_addrlen       = sizeof(struct sockaddr_in6);
        ai->sa.sin6.sin6_family = AF_INET6;
        ai->sa.sin6.sin6_port   = port;  /* Already network order */
        memcpy(&ai->sa.sin6.sin6_addr, addr, sizeof(ai->sa.sin6.sin6_addr));
        break;
#endif
#ifdef CONFIG_NET_RPMSG
      case AF_RPMSG:
        ai->ai.ai_addrlen       = sizeof(struct sockaddr_rpmsg);
        ai->sa.srp.rp_family    = AF_RPMSG;
        strlcpy(ai->sa.srp.rp_cpu, addr, sizeof(ai->sa.srp.rp_cpu));
        snprintf(ai->sa.srp.rp_name, sizeof(ai->sa.srp.rp_name), "%d", port);
        break;
#endif
    }

  return ai;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getaddrinfo
 ****************************************************************************/

int getaddrinfo(FAR const char *hostname, FAR const char *servname,
                FAR const struct addrinfo *hint, FAR struct addrinfo **res)
{
  int family = AF_UNSPEC;
  int port = 0;
  int flags = 0;
  int proto = 0;
  int socktype = 0;
  FAR char *hostbuffer;
  FAR struct hostent_s host;
  FAR struct ai_s *ai;
  FAR struct ai_s *prev_ai = NULL;
  const int valid_flags = AI_PASSIVE | AI_CANONNAME | AI_NUMERICHOST |
                          AI_NUMERICSERV | AI_V4MAPPED | AI_ALL |
                          AI_ADDRCONFIG;
  int ret = OK;
  int i;

  if (hostname == NULL && servname == NULL)
    {
      return EAI_NONAME;
    }

  if (hint)
    {
      family   = hint->ai_family;
      flags    = hint->ai_flags;
      proto    = hint->ai_protocol;
      socktype = hint->ai_socktype;

      if ((flags & valid_flags) != flags)
        {
          return EAI_BADFLAGS;
        }

      if (family != AF_INET &&
          family != AF_INET6 &&
          family != AF_LOCAL &&
          family != AF_RPMSG &&
          family != AF_UNSPEC)
        {
          return EAI_FAMILY;
        }
    }

  if (servname != NULL)
    {
      struct servent ent;
      FAR struct servent *sp;
      FAR char *endp;

      port = strtol(servname, &endp, 10);
      if (port > 0 && port <= 65535 && *endp == '\0')
        {
          /* Force network byte order */

          port = HTONS(port);
        }
      else if ((flags & AI_NUMERICSERV) != 0)
        {
          return EAI_NONAME;
        }
      else if (getservbyname_r(servname, NULL, &ent, NULL, 0, &sp) == OK)
        {
          /* The s_port field of struct servent is required to
           * be in network byte order (per OpenGroup.org)
           */

          port = sp->s_port;
        }
      else
        {
          return EAI_SERVICE;
        }
    }

  *res = NULL;

  /* If hostname is not NULL, then the AI_PASSIVE flag is ignored. */

  if ((flags & AI_PASSIVE) != 0 && hostname == NULL)
    {
      struct in6_addr addr;

      memset(&addr, 0, sizeof(struct in6_addr));

#ifdef CONFIG_NET_IPv4
      if (family == AF_INET || family == AF_UNSPEC)
        {
          ai = alloc_ai(AF_INET, socktype, proto, port, &addr);
          if (ai != NULL)
            {
              *res = (FAR struct addrinfo *)ai;
            }
        }
#endif

#ifdef CONFIG_NET_IPv6
      if (family == AF_INET6 || family == AF_UNSPEC)
        {
          ai = alloc_ai(AF_INET6, socktype, proto, port, &addr);
          if (ai != NULL)
            {
              /* Can return both IPv4 and IPv6 loopback. */

              if (*res != NULL)
                {
                  (*res)->ai_next = (FAR struct addrinfo *)ai;
                }
              else
                {
                  *res = (FAR struct addrinfo *)ai;
                }
            }
        }
#endif

      return (*res != NULL) ? OK : EAI_MEMORY;
    }

  if (hostname == NULL)
    {
#ifdef CONFIG_NET_LOOPBACK
      /* Local service. */

#ifdef CONFIG_NET_IPv4
      if (family == AF_INET || family == AF_UNSPEC)
        {
          ai = alloc_ai(AF_INET, socktype, proto, port,
                        &g_lo_ipv4addr);
          if (ai != NULL)
            {
              *res = (FAR struct addrinfo *)ai;
            }
        }
#endif

#ifdef CONFIG_NET_IPv6
      if (family == AF_INET6 || family == AF_UNSPEC)
        {
          ai = alloc_ai(AF_INET6, socktype, proto, port,
                        &g_lo_ipv6addr);
          if (ai != NULL)
            {
              /* Can return both IPv4 and IPv6 loopback. */

              if (*res != NULL)
                {
                  (*res)->ai_next = (FAR struct addrinfo *)ai;
                }
              else
                {
                  *res = (FAR struct addrinfo *)ai;
                }
            }
        }
#endif

      return (*res != NULL) ? OK : EAI_MEMORY;
#else
      /* Local service, but no loopback so cannot succeed. */

      return EAI_FAIL;
#endif /* CONFIG_NET_LOOPBACK */
    }

#if defined(CONFIG_NET_LOCAL) || defined(CONFIG_NET_RPMSG)
  if (family == AF_LOCAL || family == AF_RPMSG)
    {
      ai = alloc_ai(family, socktype, proto, port, hostname);
      if (ai != NULL)
        {
          *res = (FAR struct addrinfo *)ai;
          if (flags & AI_CANONNAME)
            {
              ai->ai.ai_canonname = (FAR char *)hostname;
            }
        }

      return (*res != NULL) ? OK : EAI_MEMORY;
    }
#endif

  hostbuffer = lib_malloc(CONFIG_NETDB_BUFSIZE);
  if (hostbuffer == NULL)
    {
      return EAI_MEMORY;
    }

  gethostentbyname_r(hostname, &host,
                     hostbuffer, CONFIG_NETDB_BUFSIZE, &ret, flags);
  if (ret != OK)
    {
      lib_free(hostbuffer);
      return ret;
    }

  for (i = 0; host.h_addr_list[i]; i++)
    {
      if (family != AF_UNSPEC && host.h_addrtypes[i] != family)
        {
          /* Filter by protocol family. */

          continue;
        }

      /* REVISIT: filter by socktype and protocol not implemented. */

      ai = alloc_ai(host.h_addrtypes[i], socktype, proto, port,
                    host.h_addr_list[i]);
      if (ai == NULL)
        {
          if (*res)
            {
              freeaddrinfo(*res);
            }

          lib_free(hostbuffer);
          return EAI_MEMORY;
        }

      /* REVISIT: grok canonical name.
       *
       * OpenGroup: "if the canonical name is not available, then
       * ai_canonname shall refer to the hostname argument or a string
       * with the same contents."
       */

      ai->ai.ai_canonname = (FAR char *)hostname;

      /* Add result to linked list.
       * TODO: RFC 3484/6724 destination address sort not implemented.
       */

      if (prev_ai != NULL)
        {
          prev_ai->ai.ai_next = (FAR struct addrinfo *)ai;
        }
      else
        {
          *res = (FAR struct addrinfo *)ai;
        }

      prev_ai = ai;
    }

  lib_free(hostbuffer);
  return (*res != NULL) ? OK : EAI_FAMILY;
}
