/****************************************************************************
 * arch/sim/src/sim/posix/sim_hostusrsock.c
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
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <syslog.h>

#include <netinet/in.h>

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

static void sockaddr_to_native(const struct nuttx_sockaddr *addr,
                               const nuttx_socklen_t addrlen,
                               struct sockaddr *naddr,
                               socklen_t *naddrlen)
{
  naddr->sa_family = addr->sa_family;
  memcpy(naddr->sa_data, addr->sa_data, sizeof(naddr->sa_data));

  *naddrlen = addrlen;
}

static void sockaddr_to_nuttx(const struct sockaddr *naddr,
                              const socklen_t naddrlen,
                              struct nuttx_sockaddr *addr,
                              nuttx_socklen_t *addrlen)
{
  addr->sa_family = naddr->sa_family;
  memcpy(addr->sa_data, naddr->sa_data, sizeof(addr->sa_data));

  *addrlen = naddrlen;
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

static int host_usrsock_sockopt(int sockfd, int level, int optname,
                                const void *optval, nuttx_socklen_t *optlen,
                                bool set)
{
  int ret = -EINVAL;

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
      return ret;
    }

  if (optname == NUTTX_SO_REUSEADDR)
    {
      optname = SO_REUSEADDR;
    }
  else if (optname == NUTTX_SO_ERROR)
    {
      optname = SO_ERROR;
    }
  else
    {
      syslog(LOG_ERR, "Invalid optname: %x\n", optname);
      return ret;
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
  int ret;

  if (domain == NUTTX_PF_INET)
    {
      domain = PF_INET;
    }
  else
    {
      return -EINVAL;
    }

  if (type == NUTTX_SOCK_STREAM)
    {
      type = SOCK_STREAM;
    }
  else if (type == NUTTX_SOCK_DGRAM)
    {
      type = SOCK_DGRAM;
    }
  else
    {
      return -EINVAL;
    }

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

  ret = socket(domain, type, protocol);
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
  struct sockaddr naddr;
  socklen_t naddrlen;
  int ret;

  sockaddr_to_native(addr, addrlen, &naddr, &naddrlen);

  sock_nonblock(sockfd, false);
  ret = connect(sockfd, &naddr, naddrlen);
  sock_nonblock(sockfd, true);
  if (ret < 0)
    {
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
  struct sockaddr naddr;
  socklen_t naddrlen;
  int ret;

  if (dest_addr && addrlen >= sizeof(*dest_addr))
    {
      sockaddr_to_native(dest_addr, addrlen, &naddr, &naddrlen);
      ret = sendto(sockfd, buf, len, flags, &naddr, naddrlen);
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
  struct sockaddr naddr;
  socklen_t naddrlen;
  int ret;

  if (src_addr && addrlen && *addrlen >= sizeof(*src_addr))
    {
      sockaddr_to_native(src_addr, *addrlen, &naddr, &naddrlen);
      ret = recvfrom(sockfd, buf, len, flags, &naddr, &naddrlen);
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

  if (src_addr && addrlen && *addrlen >= sizeof(*src_addr))
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
  socklen_t naddrlen = sizeof(struct sockaddr);
  struct sockaddr naddr;
  int ret;

  ret = getsockname(sockfd, &naddr, &naddrlen);
  if (ret < 0)
    {
      return -errno;
    }

  if (addr && addrlen && *addrlen >= sizeof(*addr))
    {
      sockaddr_to_nuttx(&naddr, naddrlen, addr, addrlen);
    }

  return ret;
}

int host_usrsock_getpeername(int sockfd,
                             struct nuttx_sockaddr *addr,
                             nuttx_socklen_t *addrlen)
{
  socklen_t naddrlen = sizeof(struct sockaddr);
  struct sockaddr naddr;
  int ret;

  ret = getpeername(sockfd, &naddr, &naddrlen);
  if (ret < 0)
    {
      return -errno;
    }

  if (addr && addrlen && *addrlen >= sizeof(*addr))
    {
      sockaddr_to_nuttx(&naddr, naddrlen, addr, addrlen);
    }

  return ret;
}

int host_usrsock_bind(int sockfd,
                      const struct nuttx_sockaddr *addr,
                      nuttx_socklen_t addrlen)
{
  struct sockaddr naddr;
  socklen_t naddrlen;

  sockaddr_to_native(addr, addrlen, &naddr, &naddrlen);

  return bind(sockfd, &naddr, naddrlen) < 0 ? -errno : 0;
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
  socklen_t naddrlen = sizeof(socklen_t);
  struct sockaddr naddr;
  int ret;

  ret = accept(sockfd, &naddr, &naddrlen);
  if (ret <= 0)
    {
      return -errno;
    }

  if (addr && addrlen && *addrlen >= sizeof(*addr))
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

  if (g_active_maxfd <= 0)
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
