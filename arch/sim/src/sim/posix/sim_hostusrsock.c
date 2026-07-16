/****************************************************************************
 * arch/sim/src/sim/posix/sim_hostusrsock.c
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

#include <sys/types.h>
#include <sys/uio.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/param.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <syslog.h>

#include <netinet/in.h>
#include <sys/un.h>

#include "sim_internal.h"
#include "sim_hostusrsock.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int    g_active_maxfd = -1;
static fd_set g_active_read_fds;
static fd_set g_active_write_fds;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void host_usrsock_clear_fd(int fd, fd_set *fds)
{
  if (FD_ISSET(fd, fds))
    {
      FD_CLR(fd, fds);

      if (fd == g_active_maxfd)
        {
          while (fd--)
            {
              if (FD_ISSET(fd, &g_active_read_fds))
                {
                  break;
                }
              else if (FD_ISSET(fd, &g_active_write_fds))
                {
                  break;
                }
            }

          g_active_maxfd = fd;
        }
    }
}

static void host_usrsock_set_fd(int fd, fd_set *fds)
{
  if (!FD_ISSET(fd, fds))
    {
      FD_SET(fd, fds);
      if (fd > g_active_maxfd)
        {
          g_active_maxfd = fd;
        }
    }
}

static int sockaddr_to_native(const struct nuttx_sockaddr *addr,
                              nuttx_socklen_t addrlen,
                              struct sockaddr_storage *naddr,
                              socklen_t *naddrlen)
{
  if (addr == NULL || naddr == NULL || naddrlen == NULL)
    {
      return -EINVAL;
    }

  memset(naddr, 0, sizeof(*naddr));

  if (addr->sa_family == NUTTX_AF_LOCAL)
    {
      const struct nuttx_sockaddr_un *un =
        (const struct nuttx_sockaddr_un *)addr;
      struct sockaddr_un *native = (struct sockaddr_un *)naddr;
      size_t pathlen;

      if (addrlen < offsetof(struct nuttx_sockaddr_un, sun_path) + 1)
        {
          return -EINVAL;
        }

      pathlen = strnlen(un->sun_path, sizeof(un->sun_path));
      if (pathlen >= sizeof(native->sun_path))
        {
          return -ENAMETOOLONG;
        }

      native->sun_family = AF_UNIX;
      memcpy(native->sun_path, un->sun_path, pathlen + 1);
      *naddrlen = offsetof(struct sockaddr_un, sun_path) + pathlen + 1;

      return 0;
    }

  if (addrlen > sizeof(*naddr))
    {
      return -ENOSPC;
    }

  memcpy(naddr, addr, addrlen);
  *naddrlen = addrlen;

  return 0;
}

static int sockaddr_to_nuttx(const struct sockaddr_storage *naddr,
                             socklen_t naddrlen,
                             struct nuttx_sockaddr *addr,
                             nuttx_socklen_t *addrlen)
{
  if (naddr == NULL || addr == NULL || addrlen == NULL)
    {
      return -EINVAL;
    }

  if (naddr->ss_family == AF_UNIX)
    {
      const struct sockaddr_un *native = (const struct sockaddr_un *)naddr;
      struct nuttx_sockaddr_un *un = (struct nuttx_sockaddr_un *)addr;
      size_t pathlen;

      if (*addrlen < sizeof(*un))
        {
          return -ENOSPC;
        }

      memset(un, 0, sizeof(*un));
      pathlen = strnlen(native->sun_path, sizeof(native->sun_path));
      un->sun_family = NUTTX_AF_LOCAL;
      memcpy(un->sun_path, native->sun_path, pathlen);
      *addrlen = offsetof(struct nuttx_sockaddr_un, sun_path) + pathlen + 1;

      return 0;
    }

  if (*addrlen < naddrlen)
    {
      return -ENOSPC;
    }

  memcpy(addr, naddr, naddrlen);
  *addrlen = naddrlen;

  return 0;
}

static void sock_nonblock(int socket, int enable)
{
  if (enable)
    {
      fcntl(socket, F_SETFL, fcntl(socket, F_GETFL) | O_NONBLOCK);
    }
  else
    {
      fcntl(socket, F_SETFL, fcntl(socket, F_GETFL) & ~O_NONBLOCK);
    }
}

static int optname_to_native(int optname)
{
  switch (optname)
    {
      case NUTTX_SO_ACCEPTCONN:
        return SO_ACCEPTCONN;

      case NUTTX_SO_BROADCAST:
        return SO_BROADCAST;

      case NUTTX_SO_DEBUG:
        return SO_DEBUG;

      case NUTTX_SO_DONTROUTE:
        return SO_DONTROUTE;

      case NUTTX_SO_ERROR:
        return SO_ERROR;

      case NUTTX_SO_KEEPALIVE:
        return SO_KEEPALIVE;

      case NUTTX_SO_LINGER:
        return SO_LINGER;

      case NUTTX_SO_OOBINLINE:
        return SO_OOBINLINE;

      case NUTTX_SO_RCVBUF:
        return SO_RCVBUF;

      case NUTTX_SO_RCVLOWAT:
        return SO_RCVLOWAT;

      case NUTTX_SO_RCVTIMEO:
        return SO_RCVTIMEO;

      case NUTTX_SO_REUSEADDR:
        return SO_REUSEADDR;

      case NUTTX_SO_SNDBUF:
        return SO_SNDBUF;

      case NUTTX_SO_SNDLOWAT:
        return SO_SNDLOWAT;

      case NUTTX_SO_SNDTIMEO:
        return SO_SNDTIMEO;

      case NUTTX_SO_TYPE:
        return SO_TYPE;

      case NUTTX_SO_TIMESTAMP:
        return SO_TIMESTAMP;

#ifdef CONFIG_HOST_LINUX
      case NUTTX_SO_BINDTODEVICE:
        return SO_BINDTODEVICE;
#endif

      default:
        syslog(LOG_ERR, "Invalid optname: %x\n", optname);
        return -ENOPROTOOPT;
    }
}

static int host_usrsock_sockopt(int sockfd, int level, int optname,
                                const void *optval, nuttx_socklen_t *optlen,
                                bool set)
{
  int ret = -EINVAL;

  /* For the parameters that nuttx does not support,
   * return the ENOPROTOOPT.
   */

  if (level == NUTTX_SOL_SOCKET)
    {
      level = SOL_SOCKET;
    }
  else if (level == NUTTX_IPPROTO_TCP)
    {
      level = IPPROTO_TCP;
    }
  else if (level == NUTTX_IPPROTO_UDP)
    {
      level = IPPROTO_UDP;
    }
  else if (level == NUTTX_IPPROTO_IP)
    {
      level = IPPROTO_IP;
    }
  else
    {
      return -ENOPROTOOPT;
    }

  optname = optname_to_native(optname);
  if (optname < 0)
    {
      return optname;
    }

  if (set)
    {
      ret = setsockopt(sockfd, level, optname, optval, *optlen);
    }
  else
    {
      ret = getsockopt(sockfd, level, optname, (void *)optval, optlen);
    }

  return ret < 0 ? -errno : 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_usrsock_socket(int domain, int type, int protocol)
{
  int opt = 1;
  int sockflags = 0;
  int ret;

  switch (domain)
    {
      case NUTTX_PF_INET:
        domain = PF_INET;
        break;

#ifdef CONFIG_NET_IPv6
      case NUTTX_PF_INET6:
        domain = PF_INET6;
        break;
#endif

      case NUTTX_PF_LOCAL:
        domain = PF_UNIX;
        break;

      default:
        return -EINVAL;
    }

  switch (type & NUTTX_SOCK_TYPE_MASK)
    {
      case NUTTX_SOCK_STREAM:
        sockflags = SOCK_STREAM;
        break;

      case NUTTX_SOCK_DGRAM:
        sockflags = SOCK_DGRAM;
        break;

      case NUTTX_SOCK_RAW:
        sockflags = SOCK_RAW;
        break;

      case NUTTX_SOCK_SEQPACKET:
#ifdef SOCK_SEQPACKET
        sockflags = SOCK_SEQPACKET;
        break;
#else
        return -EPROTONOSUPPORT;
#endif

      default:
        return -EINVAL;
    }

#ifdef SOCK_CLOEXEC
  if ((type & NUTTX_SOCK_CLOEXEC) != 0)
    {
      sockflags |= SOCK_CLOEXEC;
    }
#endif

#ifdef SOCK_NONBLOCK
  if ((type & NUTTX_SOCK_NONBLOCK) != 0)
    {
      sockflags |= SOCK_NONBLOCK;
    }
#endif

  if (protocol == NUTTX_IPPROTO_IP)
    {
      protocol = IPPROTO_IP;
    }
  else if (protocol == NUTTX_IPPROTO_ICMP)
    {
      protocol = IPPROTO_ICMP;
    }
  else if (protocol == NUTTX_IPPROTO_TCP)
    {
      protocol = IPPROTO_TCP;
    }
  else if (protocol == NUTTX_IPPROTO_UDP)
    {
      protocol = IPPROTO_UDP;
    }
  else
    {
      return -EINVAL;
    }

  ret = socket(domain, sockflags, protocol);
  if (ret < 0)
    {
      return -errno;
    }

  /* Reuse all addresses to avoid bind fail if the
   * nuttx exits unexpectedly.
   */

  setsockopt(ret, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sock_nonblock(ret, true);
  host_usrsock_set_fd(ret, &g_active_read_fds);

  return ret;
}

int host_usrsock_close(int sockfd)
{
  host_usrsock_clear_fd(sockfd, &g_active_read_fds);
  host_usrsock_clear_fd(sockfd, &g_active_write_fds);

  return close(sockfd);
}

int host_usrsock_connect(int sockfd,
                         const struct nuttx_sockaddr *addr,
                         nuttx_socklen_t addrlen)
{
  struct sockaddr_storage naddr;
  socklen_t naddrlen;
  int ret;

  ret = sockaddr_to_native(addr, addrlen, &naddr, &naddrlen);
  if (ret < 0)
    {
      return ret;
    }

  ret = connect(sockfd, (struct sockaddr *)&naddr, naddrlen);
  if (ret < 0)
    {
      if (errno == EINPROGRESS || errno == EALREADY || errno == EWOULDBLOCK)
        {
          host_usrsock_set_fd(sockfd, &g_active_write_fds);
          host_usrsock_set_fd(sockfd, &g_active_read_fds);
          return -EINPROGRESS;
        }

      return -errno;
    }

  host_usrsock_set_fd(sockfd, &g_active_read_fds);

  return ret;
}

ssize_t host_usrsock_sendto(int sockfd, const void *buf,
                            size_t len, int flags,
                            const struct nuttx_sockaddr *dest_addr,
                            nuttx_socklen_t addrlen)
{
  struct sockaddr_storage naddr;
  socklen_t naddrlen;
  int ret;

  if (dest_addr && addrlen >= sizeof(*dest_addr))
    {
      ret = sockaddr_to_native(dest_addr, addrlen, &naddr, &naddrlen);
      if (ret < 0)
        {
          return ret;
        }

      ret = sendto(sockfd, buf, len, flags,
                   (struct sockaddr *)&naddr, naddrlen);
    }
  else
    {
      ret = sendto(sockfd, buf, len, flags, NULL, 0);
    }

  if (ret < 0)
    {
      if (errno == EAGAIN)
        {
          host_usrsock_set_fd(sockfd, &g_active_write_fds);
        }
      else
        {
          usrsock_event_callback(sockfd, NUTTX_USRSOCK_EVENT_REMOTE_CLOSED);
        }
    }

  return ret >= 0 ? ret : -errno;
}

ssize_t host_usrsock_recvfrom(int sockfd, void *buf, size_t len, int flags,
                              struct nuttx_sockaddr *src_addr,
                              nuttx_socklen_t *addrlen)
{
  struct sockaddr_storage naddr;
  socklen_t naddrlen = sizeof(naddr);
  int ret;

  if (src_addr && addrlen)
    {
      ret = recvfrom(sockfd, buf, len, flags,
                     (struct sockaddr *)&naddr, &naddrlen);
    }
  else
    {
      ret = recvfrom(sockfd, buf, len, flags, NULL, NULL);
    }

  if (ret <= 0)
    {
      if (ret == 0 || errno != EAGAIN)
        {
          usrsock_event_callback(sockfd, NUTTX_USRSOCK_EVENT_REMOTE_CLOSED);
        }

      return -errno;
    }

  if (src_addr && addrlen)
    {
      sockaddr_to_nuttx(&naddr, naddrlen, src_addr, addrlen);
    }

  host_usrsock_set_fd(sockfd, &g_active_read_fds);

  return ret;
}

int host_usrsock_setsockopt(int sockfd, int level, int optname,
                            const void *optval, nuttx_socklen_t optlen)
{
  return host_usrsock_sockopt(sockfd, level, optname,
                              optval, &optlen, true);
}

int host_usrsock_getsockopt(int sockfd, int level, int optname,
                            void *optval, nuttx_socklen_t *optlen)
{
  return host_usrsock_sockopt(sockfd, level, optname,
                              optval, optlen, false);
}

int host_usrsock_getsockname(int sockfd,
                             struct nuttx_sockaddr *addr,
                             nuttx_socklen_t *addrlen)
{
  socklen_t naddrlen = sizeof(struct sockaddr_storage);
  struct sockaddr_storage naddr;
  int ret;

  ret = getsockname(sockfd, (struct sockaddr *)&naddr, &naddrlen);
  if (ret < 0)
    {
      return -errno;
    }

  if (addr && addrlen)
    {
      ret = sockaddr_to_nuttx(&naddr, naddrlen, addr, addrlen);
    }

  return ret;
}

int host_usrsock_getpeername(int sockfd,
                             struct nuttx_sockaddr *addr,
                             nuttx_socklen_t *addrlen)
{
  socklen_t naddrlen = sizeof(struct sockaddr_storage);
  struct sockaddr_storage naddr;
  int ret;

  ret = getpeername(sockfd, (struct sockaddr *)&naddr, &naddrlen);
  if (ret < 0)
    {
      return -errno;
    }

  if (addr && addrlen)
    {
      ret = sockaddr_to_nuttx(&naddr, naddrlen, addr, addrlen);
    }

  return ret;
}

int host_usrsock_bind(int sockfd,
                      const struct nuttx_sockaddr *addr,
                      nuttx_socklen_t addrlen)
{
  struct sockaddr_storage naddr;
  socklen_t naddrlen;
  int ret;

  ret = sockaddr_to_native(addr, addrlen, &naddr, &naddrlen);
  if (ret < 0)
    {
      return ret;
    }

  return bind(sockfd, (struct sockaddr *)&naddr, naddrlen) < 0 ? -errno : 0;
}

int host_usrsock_listen(int sockfd, int backlog)
{
  int ret;

  ret = listen(sockfd, backlog);
  if (ret < 0)
    {
      return -errno;
    }

  host_usrsock_set_fd(sockfd, &g_active_read_fds);

  return ret;
}

int host_usrsock_accept(int sockfd, struct nuttx_sockaddr *addr,
                        nuttx_socklen_t *addrlen)
{
  socklen_t naddrlen = sizeof(struct sockaddr_storage);
  struct sockaddr_storage naddr;
  int ret;

  ret = accept(sockfd, (struct sockaddr *)&naddr, &naddrlen);
  if (ret < 0)
    {
      return -errno;
    }

  if (addr && addrlen)
    {
      sockaddr_to_nuttx(&naddr, naddrlen, addr, addrlen);
    }

  sock_nonblock(ret, true);
  host_usrsock_set_fd(ret, &g_active_read_fds);
  host_usrsock_set_fd(sockfd, &g_active_read_fds);

  return ret;
}

int host_usrsock_ioctl(int fd, unsigned long request, ...)
{
  return 0;
}

int host_usrsock_shutdown(int sockfd, int how)
{
  switch (how)
    {
      case NUTTX_SHUT_RD:
        how = SHUT_RD;
        break;
      case NUTTX_SHUT_WR:
        how = SHUT_WR;
        break;
      case NUTTX_SHUT_RDWR:
        how = SHUT_RDWR;
        break;
      default:
        return -EINVAL;
    }

  return shutdown(sockfd, how) < 0 ? -errno : 0;
}

void host_usrsock_loop(void)
{
  struct timeval timeout;
  fd_set write_fds;
  fd_set read_fds;
  uint16_t events;
  int ret;
  int i;

  if (g_active_maxfd < 0)
    {
      return;
    }

  memset(&timeout, 0x0, sizeof(timeout));
  memcpy(&read_fds,  &g_active_read_fds,  sizeof(read_fds));
  memcpy(&write_fds, &g_active_write_fds, sizeof(write_fds));

  ret = select(g_active_maxfd + 1, &read_fds, &write_fds, NULL, &timeout);
  if (ret == 0)
    {
      return;
    }

  for (i = 0; i <= g_active_maxfd; i++)
    {
      events = 0;

      if (FD_ISSET(i, &read_fds))
        {
          host_usrsock_clear_fd(i, &g_active_read_fds);
          events |= NUTTX_USRSOCK_EVENT_RECVFROM_AVAIL;
        }

      if (FD_ISSET(i, &write_fds))
        {
          host_usrsock_clear_fd(i, &g_active_write_fds);
          events |= NUTTX_USRSOCK_EVENT_SENDTO_READY;
        }

      if (events)
        {
          usrsock_event_callback(i, events);

          if (--ret == 0)
            {
              break;
            }
        }
    }
}
