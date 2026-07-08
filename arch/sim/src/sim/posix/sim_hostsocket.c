/****************************************************************************
 * arch/sim/src/sim/posix/sim_hostsocket.c
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
#include <sys/socket.h>
#include <sys/un.h>

#include "sim_internal.h"

#ifdef CONFIG_HOST_LINUX
#  include <linux/netlink.h>
#endif

#include <errno.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "sim_hostsocket.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef PF_BLUETOOTH
#  define PF_BLUETOOTH 31
#endif

#ifndef AF_BLUETOOTH
#  define AF_BLUETOOTH PF_BLUETOOTH
#endif

#ifndef BTPROTO_HCI
#  define BTPROTO_HCI 1
#endif

#ifndef AF_NETLINK
#  define AF_NETLINK 16
#endif

#ifndef PF_NETLINK
#  define PF_NETLINK AF_NETLINK
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sockaddr_hci
{
  sa_family_t     hci_family;
  unsigned short  hci_dev;
  unsigned short  hci_channel;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int host_socket_domain_convert(int domain)
{
  switch (domain)
    {
      case NUTTX_PF_UNIX:
        return PF_UNIX;

      case NUTTX_PF_INET:
        return PF_INET;

      case NUTTX_PF_INET6:
        return PF_INET6;

      case NUTTX_PF_NETLINK:
        return PF_NETLINK;

      case NUTTX_PF_BLUETOOTH:
        return PF_BLUETOOTH;

      default:
        return -EAFNOSUPPORT;
    }
}

static int host_socket_type_convert(int type)
{
  int hosttype;

  switch (type & NUTTX_SOCK_TYPE_MASK)
    {
      case NUTTX_SOCK_STREAM:
        hosttype = SOCK_STREAM;
        break;

      case NUTTX_SOCK_DGRAM:
        hosttype = SOCK_DGRAM;
        break;

      case NUTTX_SOCK_RAW:
        hosttype = SOCK_RAW;
        break;

      case NUTTX_SOCK_SEQPACKET:
        hosttype = SOCK_SEQPACKET;
        break;

      default:
        return -EPROTONOSUPPORT;
    }

  if (type & NUTTX_SOCK_CLOEXEC)
    {
      hosttype |= SOCK_CLOEXEC;
    }

  if (type & NUTTX_SOCK_NONBLOCK)
    {
      hosttype |= SOCK_NONBLOCK;
    }

  return hosttype;
}

static int host_socket_protocol_convert(int domain, int protocol)
{
  if (domain == NUTTX_PF_BLUETOOTH && protocol == BTPROTO_HCI)
    {
      return BTPROTO_HCI;
    }

  return protocol;
}

static int host_socket_level_convert(int level)
{
  switch (level)
    {
      case NUTTX_SOL_SOCKET:
        return SOL_SOCKET;

      case NUTTX_IPPROTO_TCP:
        return IPPROTO_TCP;

      default:
        return -ENOPROTOOPT;
    }
}

static int host_socket_option_convert(int level, int option)
{
  if (level == NUTTX_SOL_SOCKET)
    {
      switch (option)
        {
          case NUTTX_SO_ERROR:
            return SO_ERROR;

          case NUTTX_SO_RCVBUF:
            return SO_RCVBUF;

          case NUTTX_SO_RCVTIMEO:
            return SO_RCVTIMEO;

          case NUTTX_SO_REUSEADDR:
            return SO_REUSEADDR;

          case NUTTX_SO_SNDBUF:
            return SO_SNDBUF;

          case NUTTX_SO_SNDTIMEO:
            return SO_SNDTIMEO;

          default:
            return -ENOPROTOOPT;
        }
    }

  if (level == NUTTX_IPPROTO_TCP)
    {
      switch (option)
        {
          case NUTTX_TCP_NODELAY:
            return TCP_NODELAY;

          default:
            return -ENOPROTOOPT;
        }
    }

  return -ENOPROTOOPT;
}

static int host_socket_addr_from_native(const struct sockaddr *addr,
                                        socklen_t addrlen,
                                        struct host_socket_addr *hostaddr)
{
  memset(hostaddr, 0, sizeof(*hostaddr));

  if (addr->sa_family == AF_INET)
    {
      const struct sockaddr_in *in = (const struct sockaddr_in *)addr;

      if (addrlen < sizeof(*in))
        {
          return -EINVAL;
        }

      hostaddr->type = HOST_SOCKET_ADDR_IPV4;
      memcpy(hostaddr->ipv4.addr, &in->sin_addr.s_addr,
             sizeof(hostaddr->ipv4.addr));
      hostaddr->ipv4.port = ntohs(in->sin_port);
      return 0;
    }

  if (addr->sa_family == AF_INET6)
    {
      const struct sockaddr_in6 *in6 = (const struct sockaddr_in6 *)addr;

      if (addrlen < sizeof(*in6))
        {
          return -EINVAL;
        }

      hostaddr->type = HOST_SOCKET_ADDR_IPV6;
      memcpy(hostaddr->ipv6.addr, &in6->sin6_addr.s6_addr,
             sizeof(hostaddr->ipv6.addr));
      hostaddr->ipv6.port = ntohs(in6->sin6_port);
      return 0;
    }

  return -EAFNOSUPPORT;
}

static int host_socket_addr_to_native(const struct host_socket_addr *addr,
                                      struct sockaddr_storage *storage,
                                      socklen_t *addrlen)
{
  memset(storage, 0, sizeof(*storage));

  switch (addr->type)
    {
      case HOST_SOCKET_ADDR_HCI:
        {
          struct sockaddr_hci hci;

          memset(&hci, 0, sizeof(hci));
          hci.hci_family = AF_BLUETOOTH;
          hci.hci_dev = addr->hci.dev;
          hci.hci_channel = addr->hci.channel;
          memcpy(storage, &hci, sizeof(hci));
          *addrlen = sizeof(hci);
          return 0;
        }

      case HOST_SOCKET_ADDR_UNIX:
        {
          struct sockaddr_un un;

          memset(&un, 0, sizeof(un));
          un.sun_family = AF_UNIX;
          snprintf(un.sun_path, sizeof(un.sun_path), "%s", addr->un.path);
          memcpy(storage, &un, sizeof(un));
          *addrlen = sizeof(un);
          return 0;
        }

      case HOST_SOCKET_ADDR_IPV4:
        {
          struct sockaddr_in in;

          memset(&in, 0, sizeof(in));
          in.sin_family = AF_INET;
          in.sin_port = htons(addr->ipv4.port);
          memcpy(&in.sin_addr.s_addr, addr->ipv4.addr,
                 sizeof(addr->ipv4.addr));
          memcpy(storage, &in, sizeof(in));
          *addrlen = sizeof(in);
          return 0;
        }

      case HOST_SOCKET_ADDR_IPV6:
        {
          struct sockaddr_in6 in6;

          memset(&in6, 0, sizeof(in6));
          in6.sin6_family = AF_INET6;
          in6.sin6_port = htons(addr->ipv6.port);
          memcpy(&in6.sin6_addr.s6_addr, addr->ipv6.addr,
                 sizeof(addr->ipv6.addr));
          memcpy(storage, &in6, sizeof(in6));
          *addrlen = sizeof(in6);
          return 0;
        }

      case HOST_SOCKET_ADDR_NETLINK:
        {
#ifdef CONFIG_HOST_LINUX
          struct sockaddr_nl nl;

          memset(&nl, 0, sizeof(nl));
          nl.nl_family = AF_NETLINK;
          nl.nl_pid = addr->netlink.pid;
          nl.nl_groups = addr->netlink.groups;
          memcpy(storage, &nl, sizeof(nl));
          *addrlen = sizeof(nl);
          return 0;
#else
          return -EAFNOSUPPORT;
#endif
        }

      default:
        return -EAFNOSUPPORT;
    }
}

static int host_socket_gai_error_convert(int ret)
{
  if (ret == 0)
    {
      return 0;
    }

  switch (ret)
    {
      case EAI_AGAIN:
        return -EAGAIN;

      case EAI_BADFLAGS:
      case EAI_SERVICE:
        return -EINVAL;

      case EAI_FAMILY:
        return -EAFNOSUPPORT;

      case EAI_MEMORY:
        return -ENOMEM;

      case EAI_NONAME:
        return -ENOENT;

      default:
        return -EIO;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int host_socket(int domain, int type, int protocol)
{
  int hostdomain = host_socket_domain_convert(domain);
  int hosttype = host_socket_type_convert(type);
  int hostprotocol = host_socket_protocol_convert(domain, protocol);
  int ret;

  if (hostdomain < 0)
    {
      return hostdomain;
    }

  if (hosttype < 0)
    {
      return hosttype;
    }

  ret = socket(hostdomain, hosttype, hostprotocol);
  return ret < 0 ? host_errno_convert(-errno) : ret;
}

int host_bind(int fd, const struct host_socket_addr *addr)
{
  struct sockaddr_storage storage;
  socklen_t addrlen;
  int ret;

  ret = host_socket_addr_to_native(addr, &storage, &addrlen);
  if (ret < 0)
    {
      return ret;
    }

  ret = bind(fd, (struct sockaddr *)&storage, addrlen);
  return ret < 0 ? host_errno_convert(-errno) : ret;
}

int host_connect(int fd, const struct host_socket_addr *addr)
{
  struct sockaddr_storage storage;
  socklen_t addrlen;
  int ret;

  ret = host_socket_addr_to_native(addr, &storage, &addrlen);
  if (ret < 0)
    {
      return ret;
    }

  ret = connect(fd, (struct sockaddr *)&storage, addrlen);
  return ret < 0 ? host_errno_convert(-errno) : ret;
}

int host_listen(int fd, int backlog)
{
  int ret = listen(fd, backlog);
  return ret < 0 ? host_errno_convert(-errno) : ret;
}

int host_accept(int fd)
{
  int ret = accept(fd, NULL, NULL);
  return ret < 0 ? host_errno_convert(-errno) : ret;
}

int host_setsockopt(int fd, int level, int option, const void *value,
                    uint32_t value_len)
{
  int hostlevel = host_socket_level_convert(level);
  int hostoption = host_socket_option_convert(level, option);
  int ret;

  if (hostlevel < 0)
    {
      return hostlevel;
    }

  if (hostoption < 0)
    {
      return hostoption;
    }

  ret = setsockopt(fd, hostlevel, hostoption, value, (socklen_t)value_len);
  return ret < 0 ? host_errno_convert(-errno) : ret;
}

int host_poll(int fd, int events, int *revents, int timeout_ms)
{
  struct pollfd pfd;
  int ret;

  memset(&pfd, 0, sizeof(pfd));
  pfd.fd = fd;
  pfd.events = events;

  ret = poll(&pfd, 1, timeout_ms);
  if (ret < 0)
    {
      return host_errno_convert(-errno);
    }

  if (revents != NULL)
    {
      *revents = pfd.revents;
    }

  return ret;
}

int host_getsockopt_error(int fd)
{
  socklen_t len = sizeof(int);
  int err = 0;
  int ret;

  ret = getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len);
  if (ret < 0)
    {
      return host_errno_convert(-errno);
    }

  return -err;
}

int host_getaddrinfo(const char *node, const char *service,
                     const struct host_addrinfo *hints,
                     struct host_addrinfo **res)
{
  struct addrinfo native_hints;
  struct addrinfo *result;
  struct addrinfo *cur;
  struct host_addrinfo **next;
  int family;
  int socktype;
  int protocol;
  int hostfamily;
  int hosttype;
  int hostprotocol;
  int ret;

  if (res == NULL)
    {
      return -EINVAL;
    }

  *res = NULL;
  family = hints == NULL ? NUTTX_PF_UNSPEC : hints->family;
  socktype = hints == NULL ? 0 : hints->socktype;
  protocol = hints == NULL ? 0 : hints->protocol;

  hostfamily = family == NUTTX_PF_UNSPEC ? AF_UNSPEC :
               host_socket_domain_convert(family);
  if (hostfamily < 0)
    {
      return hostfamily;
    }

  hosttype = socktype == 0 ? 0 : host_socket_type_convert(socktype);
  if (hosttype < 0)
    {
      return hosttype;
    }

  hostprotocol = host_socket_protocol_convert(family, protocol);

  memset(&native_hints, 0, sizeof(native_hints));
  native_hints.ai_flags = hints == NULL ? 0 : hints->flags;
  native_hints.ai_family = hostfamily;
  native_hints.ai_socktype = hosttype;
  native_hints.ai_protocol = hostprotocol;

  ret = getaddrinfo(node, service, &native_hints, &result);
  if (ret != 0)
    {
      return host_socket_gai_error_convert(ret);
    }

  ret = -EAFNOSUPPORT;
  next = res;
  for (cur = result; cur != NULL; cur = cur->ai_next)
    {
      struct host_addrinfo *entry;

      entry = calloc(1, sizeof(*entry));
      if (entry == NULL)
        {
          host_freeaddrinfo(*res);
          *res = NULL;
          ret = -ENOMEM;
          break;
        }

      ret = host_socket_addr_from_native(cur->ai_addr, cur->ai_addrlen,
                                         &entry->addr);
      if (ret == 0)
        {
          entry->flags = cur->ai_flags;
          entry->family = cur->ai_family;
          entry->socktype = cur->ai_socktype;
          entry->protocol = cur->ai_protocol;
          *next = entry;
          next = &entry->next;
          continue;
        }

      free(entry);
    }

  if (*res != NULL)
    {
      ret = 0;
    }

  freeaddrinfo(result);
  return ret;
}

void host_freeaddrinfo(struct host_addrinfo *res)
{
  while (res != NULL)
    {
      struct host_addrinfo *next = res->next;

      free(res);
      res = next;
    }
}

int host_getifaddrs(struct host_ifaddrs **ifap)
{
  struct ifaddrs *native;
  struct ifaddrs *cur;
  struct host_ifaddrs **next;
  int ret;

  if (ifap == NULL)
    {
      return -EINVAL;
    }

  *ifap = NULL;
  ret = getifaddrs(&native);
  if (ret < 0)
    {
      return host_errno_convert(-errno);
    }

  next = ifap;
  for (cur = native; cur != NULL; cur = cur->ifa_next)
    {
      struct host_ifaddrs *entry;

      if (cur->ifa_addr == NULL)
        {
          continue;
        }

      entry = calloc(1, sizeof(*entry));
      if (entry == NULL)
        {
          host_freeifaddrs(*ifap);
          *ifap = NULL;
          freeifaddrs(native);
          return -ENOMEM;
        }

      snprintf(entry->name, sizeof(entry->name), "%s",
               cur->ifa_name == NULL ? "" : cur->ifa_name);
      entry->flags = cur->ifa_flags;

      ret = host_socket_addr_from_native(cur->ifa_addr,
                                         sizeof(struct sockaddr_storage),
                                         &entry->addr);
      if (ret == 0)
        {
          entry->has_addr = 1;
          *next = entry;
          next = &entry->next;
          continue;
        }

      free(entry);
    }

  freeifaddrs(native);
  return 0;
}

void host_freeifaddrs(struct host_ifaddrs *ifa)
{
  while (ifa != NULL)
    {
      struct host_ifaddrs *next = ifa->next;

      free(ifa);
      ifa = next;
    }
}

ssize_t host_recv(int fd, void *data, size_t len, int flags)
{
  ssize_t ret;

  do
    {
      ret = recv(fd, data, len, flags);
    }
  while (ret < 0 && errno == EINTR);

  return ret < 0 ? host_errno_convert(-errno) : ret;
}

ssize_t host_send(int fd, const void *data, size_t len, int flags)
{
  ssize_t ret;

  do
    {
      ret = send(fd, data, len, flags);
    }
  while (ret < 0 && errno == EINTR);

  return ret < 0 ? host_errno_convert(-errno) : ret;
}
