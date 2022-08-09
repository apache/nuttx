/****************************************************************************
 * libs/libc/netdb/lib_getnameinfo.c
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

#include <netdb.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <arpa/inet.h>
#include <assert.h>

#include "netdb/lib_netdb.h"

#ifdef CONFIG_LIBC_NETDB

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getnameinfo
 ****************************************************************************/

int getnameinfo(FAR const struct sockaddr *addr, socklen_t addrlen,
                FAR char *host, socklen_t hostlen,
                FAR char *serv, socklen_t servlen, int flags)
{
  FAR const void *saddr;
  socklen_t saddr_len;
  int port;
  int ret;

  if (addr && addr->sa_family == AF_INET &&
      addrlen == sizeof(struct sockaddr_in))
    {
      FAR const struct sockaddr_in *sa_in;

      sa_in = (FAR const struct sockaddr_in *)addr;
      port = NTOHS(sa_in->sin_port);
      saddr = &sa_in->sin_addr;
      saddr_len = sizeof(sa_in->sin_addr);
    }
#ifdef CONFIG_NET_IPv6
  else if (addr && addr->sa_family == AF_INET6 &&
           addrlen == sizeof(struct sockaddr_in6))
    {
      FAR const struct sockaddr_in6 *sa_in6;

      sa_in6 = (FAR const struct sockaddr_in6 *)addr;
      port = NTOHS(sa_in6->sin6_port);
      saddr = &sa_in6->sin6_addr;
      saddr_len = sizeof(sa_in6->sin6_addr);
    }
#endif
  else
    {
      return EAI_FAMILY;
    }

  if (host && !(flags & NI_NUMERICHOST))
    {
      struct hostent hostent;
      FAR struct hostent *res;
      int error;

      ret = gethostbyaddr_r(saddr, saddr_len, addr->sa_family, &hostent,
                            host, hostlen, &res, &error);

      if (ret == OK)
        {
          size_t sz = strlen(res->h_name) + 1;

          if (sz <= hostlen)
            {
              memmove(host, res->h_name, sz);
            }
          else
            {
              memmove(host, res->h_name, hostlen);
              host[hostlen - 1] = '\0';
            }
        }
      else
        {
          switch (error)
            {
              case HOST_NOT_FOUND:
                {
                  if (flags & NI_NAMEREQD)
                    {
                      return EAI_NONAME;
                    }
                }
                break;

              case NO_RECOVERY:
                {
                  if (flags & NI_NAMEREQD)
                    {
                      return EAI_FAIL;
                    }
                }
                break;

              case NO_DATA:
              case TRY_AGAIN:
                {
                  if (flags & NI_NAMEREQD)
                    {
                      return EAI_AGAIN;
                    }
                }
                break;

              default:
                DEBUGPANIC();
            }

          /* Fall-back to numeric for the host name. */

          flags |= NI_NUMERICHOST;
        }
    }

  if (host && (flags & NI_NUMERICHOST))
    {
      if (!inet_ntop(addr->sa_family, saddr, host, hostlen))
        {
          switch (get_errno())
            {
              case ENOSPC:
                return EAI_OVERFLOW;

              default:
                DEBUGPANIC();
            }
        }
    }

  if (serv && !(flags & NI_NUMERICSERV))
    {
      struct servent servent;
      FAR struct servent *result;

      ret = getservbyport_r(port, flags & NI_DGRAM ? "udp" : "tcp",
                            &servent, serv, servlen, &result);

      if (ret == OK)
        {
          size_t sz = strlen(servent.s_name) + 1;

          if (sz <= servlen)
            {
              memmove(serv, servent.s_name, sz);
            }
          else
            {
              memmove(serv, servent.s_name, servlen);
              serv[servlen - 1] = '\0';
            }
        }
      else
        {
          /* Fall-back to numeric for the host name. */

          flags |= NI_NUMERICSERV;
        }
    }

  if (serv && (flags & NI_NUMERICSERV))
    {
      snprintf(serv, servlen, "%d", port);
    }

  return OK;
}

#endif /* CONFIG_LIBC_NETDB */
