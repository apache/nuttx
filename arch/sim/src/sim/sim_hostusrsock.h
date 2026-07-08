/****************************************************************************
 * arch/sim/src/sim/sim_hostusrsock.h
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

#ifndef __ARCH_SIM_SRC_SIM_HOSTUSRSOCK_H
#define __ARCH_SIM_SRC_SIM_HOSTUSRSOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "sim_hostsock.h"

#ifdef __SIM__

/* Event message flags */

#define NUTTX_USRSOCK_EVENT_ABORT          (1 << 1)
#define NUTTX_USRSOCK_EVENT_SENDTO_READY   (1 << 2)
#define NUTTX_USRSOCK_EVENT_RECVFROM_AVAIL (1 << 3)
#define NUTTX_USRSOCK_EVENT_REMOTE_CLOSED  (1 << 4)

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

#ifdef CONFIG_NET_IPv6
struct nuttx_sockaddr_storage
{
  sa_family_t ss_family;       /* Address family */
  char        ss_data[26];     /* 26-bytes of address data */
};
#else
struct nuttx_sockaddr_storage
{
  sa_family_t ss_family;       /* Address family */
  char        ss_data[14];     /* 14-bytes of address data */
};
#endif

struct nuttx_linger
{
  int  l_onoff;   /* Indicates whether linger option is enabled. */
  int  l_linger;  /* Linger time, in seconds. */
};

struct nuttx_iovec
{
  void  *iov_base;  /* Base address of I/O memory region */
  size_t iov_len;   /* Size of the memory pointed to by iov_base */
};

struct nuttx_msghdr
{
  void *msg_name;               /* Socket name */
  socklen_t msg_namelen;        /* Length of name */
  struct nuttx_iovec *msg_iov;  /* Data blocks */
  unsigned long msg_iovlen;     /* Number of blocks */
  void *msg_control;            /* Per protocol magic
                                 * (eg BSD file descriptor passing)
                                 */
  unsigned long msg_controllen; /* Length of cmsg list */
  unsigned int msg_flags;
};

struct nuttx_cmsghdr
{
  unsigned long cmsg_len;       /* Data byte count, including hdr */
  int cmsg_level;               /* Originating protocol */
  int cmsg_type;                /* Protocol-specific type */
};

#endif /* __SIM__ */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __SIM__
int usrsock_event_callback(int16_t usockid, uint16_t events);

int host_usrsock_socket(int domain, int type, int protocol);
int host_usrsock_close(int sockfd);
int host_usrsock_connect(int sockfd, const struct nuttx_sockaddr *addr,
                         nuttx_socklen_t addrlen);
ssize_t host_usrsock_sendto(int sockfd, const void *buf, size_t len,
                            int flags,
                            const struct nuttx_sockaddr *dest_addr,
                            nuttx_socklen_t addrlen);
ssize_t host_usrsock_recvfrom(int sockfd, void *buf, size_t len, int flags,
                              struct nuttx_sockaddr *src_addr,
                              nuttx_socklen_t *addrlen);
int host_usrsock_setsockopt(int sockfd, int level, int optname,
                            const void *optval, nuttx_socklen_t optlen);
int host_usrsock_getsockopt(int sockfd, int level, int optname,
                            void *optval, nuttx_socklen_t *optlen);
int host_usrsock_getsockname(int sockfd,
                             struct nuttx_sockaddr *addr,
                             nuttx_socklen_t *addrlen);
int host_usrsock_getpeername(int sockfd,
                             struct nuttx_sockaddr *addr,
                             nuttx_socklen_t *addrlen);
int host_usrsock_bind(int sockfd, const struct nuttx_sockaddr *addr,
                      nuttx_socklen_t addrlen);
int host_usrsock_listen(int sockfd, int backlog);
int host_usrsock_accept(int sockfd, struct nuttx_sockaddr *addr,
                        nuttx_socklen_t *addrlen);
int host_usrsock_ioctl(int fd, unsigned long request, ...);
int host_usrsock_shutdown(int sockfd, int how);
#else
int host_usrsock_socket(int domain, int type, int protocol);
int host_usrsock_close(int sockfd);
int host_usrsock_connect(int sockfd, const struct sockaddr *addr,
                         socklen_t addrlen);
ssize_t host_usrsock_sendto(int sockfd, const void *buf, size_t len,
                            int flags,
                            const struct sockaddr *dest_addr,
                            socklen_t addrlen);
ssize_t host_usrsock_recvfrom(int sockfd, void *buf, size_t len, int flags,
                              struct sockaddr *src_addr,
                              socklen_t *addrlen);
int host_usrsock_setsockopt(int sockfd, int level, int optname,
                            const void *optval, socklen_t optlen);
int host_usrsock_getsockopt(int sockfd, int level, int optname,
                            void *optval, socklen_t *optlen);
int host_usrsock_getsockname(int sockfd, struct sockaddr *addr,
                             socklen_t *addrlen);
int host_usrsock_getpeername(int sockfd, struct sockaddr *addr,
                             socklen_t *addrlen);
int host_usrsock_bind(int sockfd, const struct sockaddr *addr,
                      socklen_t addrlen);
int host_usrsock_listen(int sockfd, int backlog);
int host_usrsock_accept(int sockfd, struct sockaddr *addr,
                        socklen_t *addrlen);
int host_usrsock_ioctl(int fd, unsigned long request, ...);
int host_usrsock_shutdown(int sockfd, int how);
void host_usrsock_loop(void);
#endif /* __SIM__ */

#endif /* __ARCH_SIM_SRC_SIM_HOSTUSRSOCK_H */
