/****************************************************************************
 * libs/libc/net/lib_socketpair.c
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

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

union sockaddr_u
{
  struct sockaddr addr;
  struct sockaddr_in inaddr;
  struct sockaddr_in6 in6addr;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void init_loop_addr(int domain, FAR union sockaddr_u *addr,
                                  FAR socklen_t *len)
{
  if (domain == AF_INET6)
    {
      struct in6_addr init_sin6_addr = IN6ADDR_LOOPBACK_INIT;

      memset(&addr->in6addr, 0, sizeof(addr->in6addr));
      addr->in6addr.sin6_family = domain;
      addr->in6addr.sin6_addr = init_sin6_addr;
      *len = sizeof(addr->in6addr);
    }
  else
    {
      memset(&addr->inaddr, 0, sizeof(addr->inaddr));
      addr->inaddr.sin_family = domain;
      addr->inaddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
      *len = sizeof(addr->inaddr);
    }
}

static int create_socket(int domain, int type, int protocol,
                         FAR union sockaddr_u *addr, FAR socklen_t *len)
{
  int socketfd;

  socketfd = socket(domain, type, protocol);
  if (socketfd < 0)
    {
      return socketfd;
    }

  init_loop_addr(domain, addr, len);
  if (bind(socketfd, &addr->addr, *len) == 0)
    {
      if (getsockname(socketfd, &addr->addr, len) == 0)
        {
          return socketfd;
        }
    }

  close(socketfd);
  return -1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: socketpair
 *
 * Description:
 * Create an unbound pair of connected sockets in a specified domain, of a
 * specified type, under the protocol optionally specified by the protocol
 * argument. The two sockets shall be identical. The file descriptors used
 * in referencing the created sockets shall be returned in
 * sv[0] and sv[1].
 *
 ****************************************************************************/

int socketpair(int domain, int type, int protocol, int sv[2])
{
  union sockaddr_u addr[2];
  socklen_t        len;

  if (domain != AF_UNIX && domain != AF_INET && domain != AF_INET6)
    {
      set_errno(EAFNOSUPPORT);
      return -1;
    }

  if (sv == NULL)
    {
      set_errno(EINVAL);
      return -1;
    }

  if (domain == AF_UNIX)
    {
      domain = AF_INET;
    }

  sv[0] = create_socket(domain, type, protocol, &addr[0], &len);
  sv[1] = create_socket(domain, type, protocol, &addr[1], &len);
  if (sv[0] < 0 || sv[1] < 0)
    {
      goto err;
    }

  if (type == SOCK_DGRAM)
    {
      if (connect(sv[0], &addr[1].addr, len) < 0)
        {
          goto err;
        }

      if (connect(sv[1], &addr[0].addr, len) < 0)
        {
          goto err;
        }
    }
  else
    {
      int listener = sv[0];

      if (listen(listener, 2) < 0)
        {
          goto err;
        }

      if (connect(sv[1], &addr[0].addr, len) < 0)
        {
          goto err;
        }

      sv[0] = accept(listener, &addr[0].addr, &len);
      close(listener);
      if (sv[0] < 0)
        {
          goto err;
        }
    }

  return 0;

err:
  if (sv[0] != -1)
    {
      close(sv[0]);
    }

  if (sv[1] != -1)
    {
      close(sv[1]);
    }

  return -1;
}
