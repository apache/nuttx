/****************************************************************************
 * arch/sim/src/sim/up_usrsock_host.h
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

#ifndef __ARCH_SIM_SRC_SIM_UP_USRSOCK_HOST_H
#define __ARCH_SIM_SRC_SIM_UP_USRSOCK_HOST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __SIM__
#  include <sys/socket.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef __SIM__

#define NUTTX_IPPROTO_IP            0
#define NUTTX_IPPROTO_HOPOPTS       0
#define NUTTX_IPPROTO_ICMP          1
#define NUTTX_IPPROTO_IGMP          2
#define NUTTX_IPPROTO_IPIP          4
#define NUTTX_IPPROTO_TCP           6
#define NUTTX_IPPROTO_EGP           8
#define NUTTX_IPPROTO_PUP           12
#define NUTTX_IPPROTO_UDP           17
#define NUTTX_IPPROTO_IDP           22
#define NUTTX_IPPROTO_TP            29
#define NUTTX_IPPROTO_DCCP          33
#define NUTTX_IPPROTO_IPV6          41
#define NUTTX_IPPROTO_ROUTING       43
#define NUTTX_IPPROTO_FRAGMENT      44
#define NUTTX_IPPROTO_RSVP          46
#define NUTTX_IPPROTO_GRE           47
#define NUTTX_IPPROTO_ESP           50
#define NUTTX_IPPROTO_AH            51
#define NUTTX_IPPROTO_ICMP6         58
#define NUTTX_IPPROTO_NONE          59
#define NUTTX_IPPROTO_DSTOPTS       60
#define NUTTX_IPPROTO_MTP           92
#define NUTTX_IPPROTO_ENCAP         98
#define NUTTX_IPPROTO_BEETPH        94
#define NUTTX_IPPROTO_PIM           103
#define NUTTX_IPPROTO_COMP          108
#define NUTTX_IPPROTO_SCTP          132
#define NUTTX_IPPROTO_UDPLITE       136
#define NUTTX_IPPROTO_MPLS          137
#define NUTTX_IPPROTO_RAW           255

#define NUTTX_PF_UNSPEC             0
#define NUTTX_PF_UNIX               1
#define NUTTX_PF_LOCAL              1
#define NUTTX_PF_INET               2
#define NUTTX_PF_INET6              10
#define NUTTX_PF_NETLINK            16
#define NUTTX_PF_ROUTE              NUTTX_PF_NETLINK
#define NUTTX_PF_PACKET             17
#define NUTTX_PF_CAN                29
#define NUTTX_PF_BLUETOOTH          31
#define NUTTX_PF_IEEE802154         36
#define NUTTX_PF_PKTRADIO           64
#define NUTTX_PF_RPMSG              65

#define NUTTX_AF_UNSPEC             NUTTX_PF_UNSPEC
#define NUTTX_AF_UNIX               NUTTX_PF_UNIX
#define NUTTX_AF_LOCAL              NUTTX_PF_LOCAL
#define NUTTX_AF_INET               NUTTX_PF_INET
#define NUTTX_AF_INET6              NUTTX_PF_INET6
#define NUTTX_AF_NETLINK            NUTTX_PF_NETLINK
#define NUTTX_AF_ROUTE              NUTTX_PF_ROUTE
#define NUTTX_AF_PACKET             NUTTX_PF_PACKET
#define NUTTX_AF_CAN                NUTTX_PF_CAN
#define NUTTX_AF_BLUETOOTH          NUTTX_PF_BLUETOOTH
#define NUTTX_AF_IEEE802154         NUTTX_PF_IEEE802154
#define NUTTX_AF_PKTRADIO           NUTTX_PF_PKTRADIO
#define NUTTX_AF_RPMSG              NUTTX_PF_RPMSG

#define NUTTX_SOCK_UNSPEC            0
#define NUTTX_SOCK_STREAM            1
#define NUTTX_SOCK_DGRAM             2
#define NUTTX_SOCK_RAW               3
#define NUTTX_SOCK_RDM               4
#define NUTTX_SOCK_SEQPACKET         5
#define NUTTX_SOCK_PACKET           10
#define NUTTX_SOCK_CLOEXEC          02000000
#define NUTTX_SOCK_NONBLOCK         00004000

#define NUTTX_SOCK_MAX              (NUTTX_SOCK_PACKET + 1)
#define NUTTX_SOCK_TYPE_MASK        0xf

#define NUTTX_MSG_OOB               0x0001
#define NUTTX_MSG_PEEK              0x0002
#define NUTTX_MSG_DONTROUTE         0x0004
#define NUTTX_MSG_CTRUNC            0x0008
#define NUTTX_MSG_PROXY             0x0010
#define NUTTX_MSG_TRUNC             0x0020
#define NUTTX_MSG_DONTWAIT          0x0040
#define NUTTX_MSG_EOR               0x0080
#define NUTTX_MSG_WAITALL           0x0100
#define NUTTX_MSG_FIN               0x0200
#define NUTTX_MSG_SYN               0x0400
#define NUTTX_MSG_CONFIRM           0x0800
#define NUTTX_MSG_RST               0x1000
#define NUTTX_MSG_ERRQUEUE          0x2000
#define NUTTX_MSG_NOSIGNAL          0x4000
#define NUTTX_MSG_MORE              0x8000

#define NUTTX_SOL_SOCKET            1
#define NUTTX_SO_ACCEPTCONN         0
#define NUTTX_SO_BROADCAST          1
#define NUTTX_SO_DEBUG              2
#define NUTTX_SO_DONTROUTE          3
#define NUTTX_SO_ERROR              4
#define NUTTX_SO_KEEPALIVE          5
#define NUTTX_SO_LINGER             6
#define NUTTX_SO_OOBINLINE          7
#define NUTTX_SO_RCVBUF             8
#define NUTTX_SO_RCVLOWAT           9
#define NUTTX_SO_RCVTIMEO           10
#define NUTTX_SO_REUSEADDR          11
#define NUTTX_SO_SNDBUF             12
#define NUTTX_SO_SNDLOWAT           13
#define NUTTX_SO_SNDTIMEO           14
#define NUTTX_SO_TYPE               15
#define NUTTX_SO_TIMESTAMP          16
#define NUTTX_SO_BINDTODEVICE       17

#define NUTTX_SO_SNDBUFFORCE        32
#define NUTTX_SO_RCVBUFFORCE        33
#define NUTTX_SO_RXQ_OVFL           40

#define NUTTX_SOL_IP                NUTTX_IPPROTO_IP
#define NUTTX_SOL_IPV6              NUTTX_IPPROTO_IPV6
#define NUTTX_SOL_TCP               NUTTX_IPPROTO_TCP
#define NUTTX_SOL_UDP               NUTTX_IPPROTO_UDP

#define NUTTX_SOL_HCI               0
#define NUTTX_SOL_L2CAP             6
#define NUTTX_SOL_SCO               17
#define NUTTX_SOL_RFCOMM            18

#define NUTTX___SO_PROTOCOL         16

#define NUTTX_SHUT_RD               1
#define NUTTX_SHUT_WR               2
#define NUTTX_SHUT_RDWR             3

#define NUTTX_SCM_RIGHTS            0x01
#define NUTTX_SCM_CREDENTIALS       0x02
#define NUTTX_SCM_SECURITY          0x03

#define NUTTX_IP_MULTICAST_IF           (NUTTX__SO_PROTOCOL + 1)
#define NUTTX_IP_MULTICAST_TTL          (NUTTX__SO_PROTOCOL + 2)
#define NUTTX_IP_MULTICAST_LOOP         (NUTTX__SO_PROTOCOL + 3)
#define NUTTX_IP_ADD_MEMBERSHIP         (NUTTX__SO_PROTOCOL + 4)
#define NUTTX_IP_DROP_MEMBERSHIP        (NUTTX__SO_PROTOCOL + 5)
#define NUTTX_IP_UNBLOCK_SOURCE         (NUTTX__SO_PROTOCOL + 6)
#define NUTTX_IP_BLOCK_SOURCE           (NUTTX__SO_PROTOCOL + 7)
#define NUTTX_IP_ADD_SOURCE_MEMBERSHIP  (NUTTX__SO_PROTOCOL + 8)
#define NUTTX_IP_DROP_SOURCE_MEMBERSHIP (NUTTX__SO_PROTOCOL + 9)
#define NUTTX_IP_MSFILTER               (NUTTX__SO_PROTOCOL + 10)
#define NUTTX_IP_MULTICAST_ALL          (NUTTX__SO_PROTOCOL + 11)
#define NUTTX_IP_PKTINFO                (NUTTX__SO_PROTOCOL + 12)
#define NUTTX_IP_TOS                    (NUTTX__SO_PROTOCOL + 13)
#define NUTTX_IP_TTL                    (NUTTX__SO_PROTOCOL + 14)

/* Event message flags */

#define NUTTX_USRSOCK_EVENT_ABORT          (1 << 1)
#define NUTTX_USRSOCK_EVENT_SENDTO_READY   (1 << 2)
#define NUTTX_USRSOCK_EVENT_RECVFROM_AVAIL (1 << 3)
#define NUTTX_USRSOCK_EVENT_REMOTE_CLOSED  (1 << 4)

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

typedef unsigned int nuttx_socklen_t;

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

struct nuttx_sockaddr
{
  sa_family_t sa_family;       /* Address family: See AF_* definitions */
  char        sa_data[14];     /* 14-bytes data (actually variable length) */
};

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

int usrsock_host_socket(int domain, int type, int protocol);
int usrsock_host_close(int sockfd);
int usrsock_host_connect(int sockfd, const struct nuttx_sockaddr *addr,
                         nuttx_socklen_t addrlen);
ssize_t usrsock_host_sendto(int sockfd, const void *buf, size_t len,
                            int flags,
                            const struct nuttx_sockaddr *dest_addr,
                            nuttx_socklen_t addrlen);
ssize_t usrsock_host_recvfrom(int sockfd, void *buf, size_t len, int flags,
                              struct nuttx_sockaddr *src_addr,
                              nuttx_socklen_t *addrlen);
int usrsock_host_setsockopt(int sockfd, int level, int optname,
                            const void *optval, nuttx_socklen_t optlen);
int usrsock_host_getsockopt(int sockfd, int level, int optname,
                            void *optval, nuttx_socklen_t *optlen);
int usrsock_host_getsockname(int sockfd,
                             struct nuttx_sockaddr *addr,
                             nuttx_socklen_t *addrlen);
int usrsock_host_getpeername(int sockfd,
                             struct nuttx_sockaddr *addr,
                             nuttx_socklen_t *addrlen);
int usrsock_host_bind(int sockfd, const struct nuttx_sockaddr *addr,
                      nuttx_socklen_t addrlen);
int usrsock_host_listen(int sockfd, int backlog);
int usrsock_host_accept(int sockfd, struct nuttx_sockaddr *addr,
                        nuttx_socklen_t *addrlen);
int usrsock_host_ioctl(int fd, unsigned long request, ...);
#else
int usrsock_host_socket(int domain, int type, int protocol);
int usrsock_host_close(int sockfd);
int usrsock_host_connect(int sockfd, const struct sockaddr *addr,
                         socklen_t addrlen);
ssize_t usrsock_host_sendto(int sockfd, const void *buf, size_t len,
                            int flags,
                            const struct sockaddr *dest_addr,
                            socklen_t addrlen);
ssize_t usrsock_host_recvfrom(int sockfd, void *buf, size_t len, int flags,
                              struct sockaddr *src_addr,
                              socklen_t *addrlen);
int usrsock_host_setsockopt(int sockfd, int level, int optname,
                            const void *optval, socklen_t optlen);
int usrsock_host_getsockopt(int sockfd, int level, int optname,
                            void *optval, socklen_t *optlen);
int usrsock_host_getsockname(int sockfd, struct sockaddr *addr,
                             socklen_t *addrlen);
int usrsock_host_getpeername(int sockfd, struct sockaddr *addr,
                             socklen_t *addrlen);
int usrsock_host_bind(int sockfd, const struct sockaddr *addr,
                      socklen_t addrlen);
int usrsock_host_listen(int sockfd, int backlog);
int usrsock_host_accept(int sockfd, struct sockaddr *addr,
                        socklen_t *addrlen);
int usrsock_host_ioctl(int fd, unsigned long request, ...);

void usrsock_host_loop(void);
#endif /* __SIM__ */

#endif /* __ARCH_SIM_SRC_SIM_UP_USRSOCK_HOST_H */
