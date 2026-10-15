/****************************************************************************
 * libs/libc/netdb/lib_getaddrinfo.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdlib.h>
#include <string.h>

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <nuttx/net/loopback.h>
#include <netpacket/rpmsg.h>
#include <netpacket/vm_sockets.h>
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
    struct sockaddr_vm svm;
  } sa;
};

struct transport_s
{
  int socktype;
  int proto;
  int port;
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
#ifdef CONFIG_NET_VSOCK
      case AF_VSOCK:
        ai->ai.ai_addrlen     = sizeof(struct sockaddr_vm);
        ai->sa.svm.svm_family = AF_VSOCK;
        ai->sa.svm.svm_cid    = strtoul(addr, NULL, 0);
        ai->sa.svm.svm_port   = ntohl(port);
        break;
#endif
    }

  return ai;
}

static FAR struct addrinfo **add_ai(FAR struct addrinfo **prev_ai,
                                    int family, int socktype, int proto,
                                    int port, FAR const void *addr,
                                    FAR const char *canonname)
{
  FAR struct ai_s *ai;

  ai = alloc_ai(family, socktype, proto, port, addr);
  if (ai == NULL)
    {
      return NULL;
    }

  if (canonname != NULL)
    {
      ai->ai.ai_canonname = (FAR char *)canonname;
    }

  *prev_ai = (FAR struct addrinfo *)ai;
  return &ai->ai.ai_next;
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
  FAR char *hostbuffer = NULL;
  struct hostent_s host;
  FAR struct addrinfo **prev_ai = res;
  struct transport_s transports[2];
  int ntransports = 0;
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
          family != AF_VSOCK &&
          family != AF_UNSPEC)
        {
          return EAI_FAMILY;
        }

      if ((family == AF_INET || family == AF_INET6 ||
           family == AF_UNSPEC) &&
          socktype != 0 &&
          socktype != SOCK_STREAM &&
          socktype != SOCK_DGRAM &&
          socktype != SOCK_RAW)
        {
          return EAI_SOCKTYPE;
        }
    }

  if (family == AF_INET || family == AF_INET6 || family == AF_UNSPEC)
    {
      if (socktype == SOCK_STREAM)
        {
          if (proto == 0)
            {
              proto = IPPROTO_TCP;
            }
          else if (proto != IPPROTO_TCP)
            {
              return EAI_SOCKTYPE;
            }
        }
      else if (socktype == SOCK_DGRAM)
        {
          if (proto == 0)
            {
              proto = IPPROTO_UDP;
            }
          else if (proto != IPPROTO_UDP)
            {
              return EAI_SOCKTYPE;
            }
        }
      else if (socktype == 0)
        {
          if (proto == IPPROTO_TCP)
            {
              socktype = SOCK_STREAM;
            }
          else if (proto == IPPROTO_UDP)
            {
              socktype = SOCK_DGRAM;
            }
        }
    }

  if (servname != NULL)
    {
      FAR char *endp;

      port = strtol(servname, &endp, 10);
      if (port >= 0 && port <= 65535 && *endp == '\0')
        {
          /* Force network byte order */

          port = HTONS(port);

          if (socktype == SOCK_STREAM)
            {
              transports[0].socktype = SOCK_STREAM;
              transports[0].proto    = IPPROTO_TCP;
              transports[0].port     = port;
              ntransports            = 1;
            }
          else if (socktype == SOCK_DGRAM)
            {
              transports[0].socktype = SOCK_DGRAM;
              transports[0].proto    = IPPROTO_UDP;
              transports[0].port     = port;
              ntransports            = 1;
            }
          else if (socktype == SOCK_RAW)
            {
              transports[0].socktype = SOCK_RAW;
              transports[0].proto    = proto;
              transports[0].port     = port;
              ntransports            = 1;
            }
          else if (proto == IPPROTO_TCP)
            {
              transports[0].socktype = SOCK_STREAM;
              transports[0].proto    = IPPROTO_TCP;
              transports[0].port     = port;
              ntransports            = 1;
            }
          else if (proto == IPPROTO_UDP)
            {
              transports[0].socktype = SOCK_DGRAM;
              transports[0].proto    = IPPROTO_UDP;
              transports[0].port     = port;
              ntransports            = 1;
            }
          else if (proto != 0)
            {
              transports[0].socktype = SOCK_RAW;
              transports[0].proto    = proto;
              transports[0].port     = port;
              ntransports            = 1;
            }
          else
            {
              /* Both socktype and proto are unspecified: return both
               * TCP and UDP transports per POSIX.
               */

              transports[0].socktype = SOCK_STREAM;
              transports[0].proto    = IPPROTO_TCP;
              transports[0].port     = port;

              transports[1].socktype = SOCK_DGRAM;
              transports[1].proto    = IPPROTO_UDP;
              transports[1].port     = port;

              ntransports            = 2;
            }
        }
      else if ((flags & AI_NUMERICSERV) != 0)
        {
          return EAI_NONAME;
        }
      else
        {
          struct servent ent;
          FAR struct servent *sp;

          if (socktype == SOCK_RAW)
            {
              return EAI_SERVICE;
            }
          else if (socktype == SOCK_STREAM || proto == IPPROTO_TCP)
            {
              if (getservbyname_r(servname, "tcp", &ent, NULL, 0, &sp) == OK)
                {
                  transports[0].socktype = SOCK_STREAM;
                  transports[0].proto    = IPPROTO_TCP;
                  transports[0].port     = sp->s_port;
                  ntransports            = 1;
                }
              else
                {
                  return EAI_SERVICE;
                }
            }
          else if (socktype == SOCK_DGRAM || proto == IPPROTO_UDP)
            {
              if (getservbyname_r(servname, "udp", &ent, NULL, 0, &sp) == OK)
                {
                  transports[0].socktype = SOCK_DGRAM;
                  transports[0].proto    = IPPROTO_UDP;
                  transports[0].port     = sp->s_port;
                  ntransports            = 1;
                }
              else
                {
                  return EAI_SERVICE;
                }
            }
          else if (proto != 0)
            {
              /* Explicit non-TCP/UDP protocol is incompatible with
               * named TCP/UDP services. Do not infer over caller's proto.
               */

              return EAI_SERVICE;
            }
          else
            {
              /* Both socktype and proto are unspecified: check if the
               * service is defined for TCP, UDP, or both.
               */

              if (getservbyname_r(servname, "tcp", &ent, NULL, 0, &sp) == OK)
                {
                  transports[ntransports].socktype = SOCK_STREAM;
                  transports[ntransports].proto    = IPPROTO_TCP;
                  transports[ntransports].port     = sp->s_port;
                  ntransports++;
                }

              if (getservbyname_r(servname, "udp", &ent, NULL, 0, &sp) == OK)
                {
                  transports[ntransports].socktype = SOCK_DGRAM;
                  transports[ntransports].proto    = IPPROTO_UDP;
                  transports[ntransports].port     = sp->s_port;
                  ntransports++;
                }

              if (ntransports == 0)
                {
                  return EAI_SERVICE;
                }
            }

          port = transports[0].port;
        }
    }
  else
    {
      /* servname is NULL */

      if (socktype == SOCK_STREAM)
        {
          transports[0].socktype = SOCK_STREAM;
          transports[0].proto    = IPPROTO_TCP;
          transports[0].port     = 0;
          ntransports            = 1;
        }
      else if (socktype == SOCK_DGRAM)
        {
          transports[0].socktype = SOCK_DGRAM;
          transports[0].proto    = IPPROTO_UDP;
          transports[0].port     = 0;
          ntransports            = 1;
        }
      else if (socktype == SOCK_RAW)
        {
          transports[0].socktype = SOCK_RAW;
          transports[0].proto    = proto;
          transports[0].port     = 0;
          ntransports            = 1;
        }
      else if (proto == IPPROTO_TCP)
        {
          transports[0].socktype = SOCK_STREAM;
          transports[0].proto    = IPPROTO_TCP;
          transports[0].port     = 0;
          ntransports            = 1;
        }
      else if (proto == IPPROTO_UDP)
        {
          transports[0].socktype = SOCK_DGRAM;
          transports[0].proto    = IPPROTO_UDP;
          transports[0].port     = 0;
          ntransports            = 1;
        }
      else if (proto != 0)
        {
          transports[0].socktype = SOCK_RAW;
          transports[0].proto    = proto;
          transports[0].port     = 0;
          ntransports            = 1;
        }
      else
        {
          /* Both socktype and proto are unspecified: return both
           * TCP and UDP transports per POSIX.
           */

          transports[0].socktype = SOCK_STREAM;
          transports[0].proto    = IPPROTO_TCP;
          transports[0].port     = 0;

          transports[1].socktype = SOCK_DGRAM;
          transports[1].proto    = IPPROTO_UDP;
          transports[1].port     = 0;

          ntransports            = 2;
        }
    }

  /* Override transports for local socket families */

#if defined(CONFIG_NET_LOCAL) || defined(CONFIG_NET_RPMSG) || \
    defined(CONFIG_NET_VSOCK)
  if (family == AF_LOCAL || family == AF_RPMSG || family == AF_VSOCK)
    {
      transports[0].socktype = socktype;
      transports[0].proto    = proto;
      transports[0].port     = port;
      ntransports            = 1;
    }
#endif

  *res = NULL;

  /* If hostname is not NULL, then the AI_PASSIVE flag is ignored. */

  if ((flags & AI_PASSIVE) != 0 && hostname == NULL)
    {
#ifdef CONFIG_NET_IPv4
      if (family == AF_INET || family == AF_UNSPEC)
        {
          struct in_addr addr;
          int t;

          memset(&addr, 0, sizeof(struct in_addr));
          for (t = 0; t < ntransports; t++)
            {
              prev_ai = add_ai(prev_ai, AF_INET,
                               transports[t].socktype,
                               transports[t].proto,
                               transports[t].port, &addr, NULL);
              if (prev_ai == NULL)
                {
                  goto errout_nomem;
                }
            }
        }
#endif

#ifdef CONFIG_NET_IPv6
      if (family == AF_INET6 || family == AF_UNSPEC)
        {
          struct in6_addr addr;
          int t;

          memset(&addr, 0, sizeof(struct in6_addr));
          for (t = 0; t < ntransports; t++)
            {
              prev_ai = add_ai(prev_ai, AF_INET6,
                               transports[t].socktype,
                               transports[t].proto,
                               transports[t].port, &addr, NULL);
              if (prev_ai == NULL)
                {
                  goto errout_nomem;
                }
            }
        }
#endif

#if defined(CONFIG_NET_RPMSG)
      if (family == AF_RPMSG || family == AF_UNSPEC)
        {
          prev_ai = add_ai(prev_ai, AF_RPMSG, socktype, proto,
                           port, "", NULL);
          if (prev_ai == NULL)
            {
              goto errout_nomem;
            }
        }
#endif

#if defined(CONFIG_NET_VSOCK)
      if (family == AF_VSOCK || family == AF_UNSPEC)
        {
          /* "-1" <--> VMADDR_CID_ANY */

          prev_ai = add_ai(prev_ai, AF_VSOCK, socktype, proto,
                           port, "-1", NULL);
          if (prev_ai == NULL)
            {
              goto errout_nomem;
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
          int t;

          for (t = 0; t < ntransports; t++)
            {
              prev_ai = add_ai(prev_ai, AF_INET,
                               transports[t].socktype,
                               transports[t].proto,
                               transports[t].port, &g_lo_ipv4addr, NULL);
              if (prev_ai == NULL)
                {
                  goto errout_nomem;
                }
            }
        }
#endif

#ifdef CONFIG_NET_IPv6
      if (family == AF_INET6 || family == AF_UNSPEC)
        {
          int t;

          for (t = 0; t < ntransports; t++)
            {
              prev_ai = add_ai(prev_ai, AF_INET6,
                               transports[t].socktype,
                               transports[t].proto,
                               transports[t].port, &g_lo_ipv6addr, NULL);
              if (prev_ai == NULL)
                {
                  goto errout_nomem;
                }
            }
        }
#endif
#endif

#if defined(CONFIG_NET_VSOCK)
      if (family == AF_VSOCK || family == AF_UNSPEC)
        {
          /* "1" <--> VMADDR_CID_LOCAL */

          prev_ai = add_ai(prev_ai, AF_VSOCK, socktype, proto,
                           port, "1", NULL);
          if (prev_ai == NULL)
            {
              goto errout_nomem;
            }
        }
#endif

#if defined(CONFIG_NET_LOOPBACK) || defined(CONFIG_NET_VSOCK)
      return (*res != NULL) ? OK : EAI_MEMORY;
#else
      /* Local service, but no loopback so cannot succeed. */

      return EAI_FAIL;
#endif
    }

#if defined(CONFIG_NET_LOCAL) || defined(CONFIG_NET_RPMSG) || \
    defined(CONFIG_NET_VSOCK)
  if (family == AF_LOCAL || family == AF_RPMSG || family == AF_VSOCK)
    {
      FAR const char *canon = (flags & AI_CANONNAME) ? hostname : NULL;

      prev_ai = add_ai(prev_ai, family, socktype, proto,
                       port, hostname, canon);
      if (prev_ai == NULL)
        {
          goto errout_nomem;
        }

      return OK;
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
      int t;

      if (family != AF_UNSPEC && host.h_addrtypes[i] != family)
        {
          /* Filter by protocol family. */

          continue;
        }

      for (t = 0; t < ntransports; t++)
        {
          FAR const char *canon = (flags & AI_CANONNAME) ? hostname : NULL;

          prev_ai = add_ai(prev_ai, host.h_addrtypes[i],
                           transports[t].socktype,
                           transports[t].proto,
                           transports[t].port, host.h_addr_list[i],
                           canon);
          if (prev_ai == NULL)
            {
              goto errout_nomem;
            }
        }
    }

  lib_free(hostbuffer);
  return (*res != NULL) ? OK : EAI_FAMILY;

errout_nomem:
  if (*res != NULL)
    {
      freeaddrinfo(*res);
      *res = NULL;
    }

  if (hostbuffer != NULL)
    {
      lib_free(hostbuffer);
    }

  return EAI_MEMORY;
}
