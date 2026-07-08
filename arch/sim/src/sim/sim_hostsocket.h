/****************************************************************************
 * arch/sim/src/sim/sim_hostsocket.h
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

#ifndef __ARCH_SIM_SRC_SIM_HOSTSOCKET_H
#define __ARCH_SIM_SRC_SIM_HOSTSOCKET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <sys/socket.h>

#include <stdint.h>

#include "sim_hostsock.h"

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

enum host_socket_addr_type
{
  HOST_SOCKET_ADDR_HCI,
  HOST_SOCKET_ADDR_UNIX,
  HOST_SOCKET_ADDR_IPV4,
  HOST_SOCKET_ADDR_IPV6,
  HOST_SOCKET_ADDR_NETLINK,
};

struct host_socket_addr
{
  enum host_socket_addr_type type;
  union
  {
    struct
    {
      uint16_t dev;
      uint16_t channel;
    } hci;

    struct
    {
      char path[108];
    } un;

    struct
    {
      uint8_t addr[4];
      uint16_t port;
    } ipv4;

    struct
    {
      uint8_t addr[16];
      uint16_t port;
    } ipv6;

    struct
    {
      uint32_t pid;
      uint32_t groups;
    } netlink;
  };
};

struct host_addrinfo
{
  int flags;
  int family;
  int socktype;
  int protocol;
  struct host_socket_addr addr;
  struct host_addrinfo *next;
};

#define HOST_IFNAMSIZ 64

struct host_ifaddrs
{
  struct host_ifaddrs *next;
  char name[HOST_IFNAMSIZ];
  unsigned int flags;
  uint8_t has_addr;
  struct host_socket_addr addr;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int host_socket(int domain, int type, int protocol);
int host_bind(int fd, const struct host_socket_addr *addr);
int host_connect(int fd, const struct host_socket_addr *addr);
int host_listen(int fd, int backlog);
int host_accept(int fd);
int host_setsockopt(int fd, int level, int option, const void *value,
                    uint32_t value_len);
int host_poll(int fd, int events, int *revents, int timeout_ms);
int host_getsockopt_error(int fd);
int host_getaddrinfo(const char *node, const char *service,
                     const struct host_addrinfo *hints,
                     struct host_addrinfo **res);
void host_freeaddrinfo(struct host_addrinfo *res);
int host_getifaddrs(struct host_ifaddrs **ifap);
void host_freeifaddrs(struct host_ifaddrs *ifa);
ssize_t host_recv(int fd, void *data, size_t len, int flags);
ssize_t host_send(int fd, const void *data, size_t len, int flags);

#endif /* __ARCH_SIM_SRC_SIM_HOSTSOCKET_H */
