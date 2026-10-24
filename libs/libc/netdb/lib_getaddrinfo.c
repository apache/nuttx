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
 * Pre-processor Definitions
 ****************************************************************************/

#define AI_VALID_FLAGS (AI_PASSIVE | AI_CANONNAME | AI_NUMERICHOST | \
                        AI_NUMERICSERV | AI_V4MAPPED | AI_ALL | \
                        AI_ADDRCONFIG)

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

/****************************************************************************
 * Name: add_transports
 ****************************************************************************/

static FAR struct addrinfo **
add_transports(FAR struct addrinfo **prev_ai, int family,
               FAR const struct transport_s *transports,
               int ntransports, FAR const void *addr,
               FAR const char *canonname)
{
  FAR struct ai_s *ai;
  int t;

  for (t = 0; t < ntransports; t++)
    {
      ai = lib_zalloc(sizeof(struct ai_s));
      if (ai == NULL)
        {
          return NULL;
        }

      ai->ai.ai_addr      = (FAR struct sockaddr *)&ai->sa;
      ai->ai.ai_family    = family;
      ai->ai.ai_socktype  = transports[t].socktype;
      ai->ai.ai_protocol  = transports[t].proto;
      ai->ai.ai_canonname = (FAR char *)canonname;

      switch (family)
        {
#ifdef CONFIG_NET_LOCAL
          case AF_LOCAL:
            ai->ai.ai_addrlen     = sizeof(struct sockaddr_un);
            ai->sa.sun.sun_family = AF_LOCAL;
            strlcpy(ai->sa.sun.sun_path, addr, sizeof(ai->sa.sun.sun_path));
            break;
#endif
#ifdef CONFIG_NET_IPv4
          case AF_INET:
            ai->ai.ai_addrlen     = sizeof(struct sockaddr_in);
            ai->sa.sin.sin_family = AF_INET;
            ai->sa.sin.sin_port   = transports[t].port;
            if (addr != NULL)
              {
                memcpy(&ai->sa.sin.sin_addr, addr, sizeof(struct in_addr));
              }
            break;
#endif
#ifdef CONFIG_NET_IPv6
          case AF_INET6:
            ai->ai.ai_addrlen       = sizeof(struct sockaddr_in6);
            ai->sa.sin6.sin6_family = AF_INET6;
            ai->sa.sin6.sin6_port   = transports[t].port;
            if (addr != NULL)
              {
                memcpy(&ai->sa.sin6.sin6_addr, addr,
                       sizeof(struct in6_addr));
              }
            break;
#endif
#ifdef CONFIG_NET_RPMSG
          case AF_RPMSG:
            ai->ai.ai_addrlen     = sizeof(struct sockaddr_rpmsg);
            ai->sa.srp.rp_family  = AF_RPMSG;
            strlcpy(ai->sa.srp.rp_cpu, addr, sizeof(ai->sa.srp.rp_cpu));
            snprintf(ai->sa.srp.rp_name, sizeof(ai->sa.srp.rp_name), "%d",
                     transports[t].port);
            break;
#endif
#ifdef CONFIG_NET_VSOCK
          case AF_VSOCK:
            ai->ai.ai_addrlen     = sizeof(struct sockaddr_vm);
            ai->sa.svm.svm_family = AF_VSOCK;
            ai->sa.svm.svm_cid    = strtoul(addr, NULL, 0);
            ai->sa.svm.svm_port   = ntohl(transports[t].port);
            break;
#endif
        }

      *prev_ai = (FAR struct addrinfo *)ai;
      prev_ai  = &ai->ai.ai_next;
    }

  return prev_ai;
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
  FAR const char *canon = NULL;
  FAR char *hostbuffer = NULL;
  FAR struct addrinfo **prev_ai = res;
  struct hostent_s host;
  struct transport_s transports[2];
  int ntransports = 0;
  int family = AF_UNSPEC;
  int flags = 0;
  int proto = 0;
  int socktype = 0;
  int port = 0;
  int ret = OK;
  int i;

  if (hostname == NULL && servname == NULL)
    {
      return EAI_NONAME;
    }

  if (hint != NULL)
    {
      family   = hint->ai_family;
      flags    = hint->ai_flags;
      proto    = hint->ai_protocol;
      socktype = hint->ai_socktype;

      if ((flags & AI_VALID_FLAGS) != flags)
        {
          return EAI_BADFLAGS;
        }

      switch (family)
        {
#ifdef CONFIG_NET_IPv4
          case AF_INET:
#endif
#ifdef CONFIG_NET_IPv6
          case AF_INET6:
#endif
#ifdef CONFIG_NET_LOCAL
          case AF_LOCAL:
#endif
#ifdef CONFIG_NET_RPMSG
          case AF_RPMSG:
#endif
#ifdef CONFIG_NET_VSOCK
          case AF_VSOCK:
#endif
          case AF_UNSPEC:
            break;
          default:
            return EAI_FAMILY;
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
          else if (socktype != SOCK_RAW)
            {
              return EAI_SOCKTYPE;
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
        }
      else if ((flags & AI_NUMERICSERV) != 0)
        {
          return EAI_NONAME;
        }
      else
        {
          struct servent ent;
          FAR struct servent *sp;

          if (socktype == SOCK_RAW ||
              (proto != 0 && proto != IPPROTO_TCP && proto != IPPROTO_UDP))
            {
              return EAI_SERVICE;
            }

          if (socktype != SOCK_DGRAM && proto != IPPROTO_UDP &&
              getservbyname_r(servname, "tcp", &ent, NULL, 0, &sp) == OK)
            {
              transports[ntransports].socktype = SOCK_STREAM;
              transports[ntransports].proto    = IPPROTO_TCP;
              transports[ntransports++].port   = sp->s_port;
            }

          if (socktype != SOCK_STREAM && proto != IPPROTO_TCP &&
              getservbyname_r(servname, "udp", &ent, NULL, 0, &sp) == OK)
            {
              transports[ntransports].socktype = SOCK_DGRAM;
              transports[ntransports].proto    = IPPROTO_UDP;
              transports[ntransports++].port   = sp->s_port;
            }

          if (ntransports == 0)
            {
              return EAI_SERVICE;
            }

          port = transports[0].port;
          if (ntransports == 1)
            {
              socktype = transports[0].socktype;
              proto    = transports[0].proto;
            }
        }
    }

  if (ntransports == 0)
    {
      if (socktype == 0 && proto == 0)
        {
          transports[0].socktype = SOCK_STREAM;
          transports[0].proto    = IPPROTO_TCP;
          transports[0].port     = port;
          transports[1].socktype = SOCK_DGRAM;
          transports[1].proto    = IPPROTO_UDP;
          transports[1].port     = port;
          ntransports            = 2;
        }
      else
        {
          transports[0].socktype = (socktype != 0) ? socktype : SOCK_RAW;
          transports[0].proto    = (proto != 0) ? proto :
                                   (transports[0].socktype == SOCK_STREAM ?
                                    IPPROTO_TCP : 0);
          transports[0].port     = port;
          ntransports            = 1;
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

  if (hostname == NULL)
    {
#if !defined(CONFIG_NET_LOOPBACK) && !defined(CONFIG_NET_VSOCK)
      if ((flags & AI_PASSIVE) == 0)
        {
          return EAI_FAIL;
        }
#endif

#ifdef CONFIG_NET_IPv4
      if (family == AF_INET || family == AF_UNSPEC)
        {
          FAR const void *addr = NULL;

#ifdef CONFIG_NET_LOOPBACK
          if ((flags & AI_PASSIVE) == 0)
            {
              addr = &g_lo_ipv4addr;
            }
#endif

          if ((flags & AI_PASSIVE) != 0 || addr != NULL)
            {
              prev_ai = add_transports(prev_ai, AF_INET, transports,
                                       ntransports, addr, NULL);
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
          FAR const void *addr = NULL;

#ifdef CONFIG_NET_LOOPBACK
          if ((flags & AI_PASSIVE) == 0)
            {
              addr = &g_lo_ipv6addr;
            }
#endif

          if ((flags & AI_PASSIVE) != 0 || addr != NULL)
            {
              prev_ai = add_transports(prev_ai, AF_INET6, transports,
                                       ntransports, addr, NULL);
              if (prev_ai == NULL)
                {
                  goto errout_nomem;
                }
            }
        }
#endif

#if defined(CONFIG_NET_RPMSG)
      if (((flags & AI_PASSIVE) != 0) &&
          (family == AF_RPMSG || family == AF_UNSPEC))
        {
          prev_ai = add_transports(prev_ai, AF_RPMSG, transports,
                                   ntransports, "", NULL);
          if (prev_ai == NULL)
            {
              goto errout_nomem;
            }
        }
#endif

#if defined(CONFIG_NET_VSOCK)
      if (family == AF_VSOCK || family == AF_UNSPEC)
        {
          FAR const char *cid = ((flags & AI_PASSIVE) != 0) ? "-1" : "1";

          prev_ai = add_transports(prev_ai, AF_VSOCK, transports,
                                   ntransports, cid, NULL);
          if (prev_ai == NULL)
            {
              goto errout_nomem;
            }
        }
#endif

      return (*res != NULL) ? OK : EAI_NONAME;
    }

  if ((flags & AI_CANONNAME) != 0)
    {
      canon = hostname;
    }

#if defined(CONFIG_NET_LOCAL) || defined(CONFIG_NET_RPMSG) || \
    defined(CONFIG_NET_VSOCK)
  if (family == AF_LOCAL || family == AF_RPMSG || family == AF_VSOCK)
    {
      prev_ai = add_transports(prev_ai, family, transports,
                               ntransports, hostname, canon);
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
      if (family != AF_UNSPEC && host.h_addrtypes[i] != family)
        {
          /* Filter by protocol family. */

          continue;
        }

      prev_ai = add_transports(prev_ai, host.h_addrtypes[i],
                               transports, ntransports,
                               host.h_addr_list[i], canon);
      if (prev_ai == NULL)
        {
          goto errout_nomem;
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

  lib_free(hostbuffer);

  return EAI_MEMORY;
}
