/****************************************************************************
 * libc/netdb/lib_getnameinfo.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Michael Jung <mijung@gmx.net>
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
      port = ntohs(sa_in->sin_port);
      saddr = &sa_in->sin_addr;
      saddr_len = sizeof(sa_in->sin_addr);
    }
#ifdef CONFIG_NET_IPv6
  else if (addr && addr->sa_family == AF_INET6 &&
           addrlen == sizeof(struct sockaddr_in6))
    {
      FAR const struct sockaddr_in6 *sa_in6;

      sa_in6 = (FAR const struct sockaddr_in6 *)addr;
      port = ntohs(sa_in6->sin6_port);
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
                DEBUGASSERT(0);
            }

          /* Fall-back to numeric for the host name. */

          flags |= NI_NUMERICHOST;
        }
    }

  if (host && (flags & NI_NUMERICHOST))
    {
      if (!inet_ntop(addr->sa_family, saddr, host, hostlen))
        {
          switch (errno)
            {
              case ENOSPC:
                return EAI_OVERFLOW;

              default:
                DEBUGASSERT(0);
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
