/****************************************************************************
 * libs/libc/netdb/lib_getaddrinfo.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Juha Niskanen <juha.niskanen@haltian.com>
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

#include <arpa/inet.h>
#include <nuttx/net/loopback.h>
#include <netdb.h>

#include "libc.h"

/****************************************************************************
 * Private Data Types
 ****************************************************************************/

struct ai_s
{
  struct addrinfo ai;
  union
  {
    struct sockaddr_in sin;
    struct sockaddr_in6 sin6;
  } sa;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

FAR static struct ai_s *alloc_ai(int family, int port, FAR void *addr)
{
  struct ai_s *ai;
  socklen_t addrlen;

  addrlen = (family == AF_INET) ? sizeof(struct sockaddr_in)
                                : sizeof(struct sockaddr_in6);

  ai = lib_zalloc(sizeof(struct ai_s));
  if (ai == NULL)
    {
      return ai;
    }

  ai->ai.ai_addr            = (struct sockaddr *)&ai->sa;
  ai->ai.ai_addrlen         = addrlen;
  ai->ai.ai_addr->sa_family = ai->ai.ai_family = family;

  switch (family)
    {
#ifdef CONFIG_NET_IPv4
      case AF_INET:
        ai->sa.sin.sin_family = AF_INET;
        ai->sa.sin.sin_port   = port;  /* Already network order */
        memcpy(&ai->sa.sin.sin_addr, addr, sizeof(ai->sa.sin.sin_addr));
        break;
#endif
#ifdef CONFIG_NET_IPv6
      case AF_INET6:
        ai->sa.sin6.sin6_family = AF_INET6;
        ai->sa.sin6.sin6_port   = port;  /* Already network order */
        memcpy(&ai->sa.sin6.sin6_addr, addr, sizeof(ai->sa.sin6.sin6_addr));
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
  struct hostent *hp;
  struct ai_s *ai;
  struct ai_s *prev_ai = NULL;
  const int valid_flags = AI_PASSIVE | AI_CANONNAME | AI_NUMERICHOST |
                          AI_NUMERICSERV | AI_V4MAPPED | AI_ALL |
                          AI_ADDRCONFIG;
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
          family != AF_UNSPEC)
        {
            return EAI_FAMILY;
        }
    }

  if (servname != NULL)
    {
      char *endp;
      struct servent *sp;

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
      else if ((sp = getservbyname(servname, NULL)) != NULL)
        {
          /* The sp_port field of struct servent is required to
           * be in network byte order (per OpenGroup.org)
           */

          port = sp->s_port;
        }
      else
        {
          return EAI_SERVICE;
        }
    }

  if ((flags & AI_PASSIVE) != 0)
    {
      in_addr_t addr;

      addr = hostname ? inet_addr(hostname) : HTONL(0x00000000);

      /* REVISIT: IPv6? */

      ai = alloc_ai(AF_INET, port, &addr);
      if (ai == NULL)
        {
          return EAI_MEMORY;
        }

      *res = (struct addrinfo *)ai;
      return OK;
   }

  *res = NULL;

  if (hostname == NULL)
    {
#ifdef CONFIG_NET_LOOPBACK
      /* Local service. */

#ifdef CONFIG_NET_IPv4
      if (family == AF_INET || family == AF_UNSPEC)
        {
          ai = alloc_ai(AF_INET, port, (void *)&g_lo_ipv4addr);
          if (ai == NULL)
            {
              return EAI_MEMORY;
            }

          *res = (struct addrinfo *)ai;
        }
#endif
#ifdef CONFIG_NET_IPv6
      if (family == AF_INET6 || family == AF_UNSPEC)
        {
          ai = alloc_ai(AF_INET6, port, (void *)&g_lo_ipv6addr);
          if (ai == NULL)
            {
              return (*res != NULL) ? OK : EAI_MEMORY;
            }

          /* Can return both IPv4 and IPv6 loopback. */

          if (*res != NULL)
            {
              (*res)->ai_next = (struct addrinfo *)ai;
            }
          else
            {
              *res = (struct addrinfo *)ai;
            }
        }
#endif
      return (*res != NULL) ? OK : EAI_FAMILY;
#else
      /* Local service, but no loopback so cannot succeed. */

      return EAI_FAIL;
#endif /* CONFIG_NET_LOOPBACK */
    }

  /* REVISIT: no check for AI_NUMERICHOST flag. */

  /* REVISIT: use gethostbyname_r with own buffer of refactor all
   * public APIs to use internal lookup function.
   */

  hp = gethostbyname(hostname);
  if (hp && hp->h_name && hp->h_name[0] && hp->h_addr_list[0])
    {
      for (i = 0; hp->h_addr_list[i]; i++)
        {
          if (family != AF_UNSPEC && hp->h_addrtype != family)
            {
              /* Filter by protocol family. */

              continue;
            }

          /* REVISIT: filter by socktype and protocol not implemented. */

          UNUSED(proto);
          UNUSED(socktype);

          ai = alloc_ai(hp->h_addrtype, port, hp->h_addr_list[i]);
          if (ai == NULL)
            {
              if (*res)
                {
                  freeaddrinfo(*res);
                }

              return EAI_MEMORY;
            }

          /* REVISIT: grok canonical name.
           *
           * OpenGroup: "if the canonical name is not available, then ai_canonname shall
           * refer to the hostname argument or a string with the same contents."
           */

          ai->ai.ai_canonname = (char *)hostname;

          /* Add result to linked list.
           * TODO: RFC 3484/6724 destination address sort not implemented.
           */

          if (prev_ai != NULL)
            {
              prev_ai->ai.ai_next = (struct addrinfo *)ai;
            }
          else
            {
              *res = (struct addrinfo *)ai;
            }

          prev_ai = ai;
        }

      return OK;
    }

  return EAI_AGAIN;
}
