/****************************************************************************
 * include/sys/socket.h
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

#ifndef __INCLUDE_SYS_SOCKET_H
#define __INCLUDE_SYS_SOCKET_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <sys/uio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The socket()domain parameter specifies a communication domain; this
 * selects the protocol family which will be used for communication.
 */

/* Supported Protocol Families */

#define PF_UNSPEC      0         /* Protocol family unspecified */
#define PF_UNIX        1         /* Local communication */
#define PF_LOCAL       1         /* Local communication */
#define PF_INET        2         /* IPv4 Internet protocols */
#define PF_INET6      10         /* IPv6 Internet protocols */
#define PF_NETLINK    16         /* Netlink IPC socket */
#define PF_ROUTE      PF_NETLINK /* 4.4BSD Compatibility*/
#define PF_PACKET     17         /* Low level packet interface */
#define PF_CAN        29         /* Controller Area Network (SocketCAN) */
#define PF_BLUETOOTH  31         /* Bluetooth sockets */
#define PF_IEEE802154 36         /* Low level IEEE 802.15.4 radio frame interface */
#define PF_PKTRADIO   64         /* Low level packet radio interface */
#define PF_RPMSG      65         /* Remote core communication */

/* Supported Address Families. Opengroup.org requires only AF_UNSPEC,
 * AF_UNIX, AF_INET and AF_INET6.
 */

#define AF_UNSPEC      PF_UNSPEC
#define AF_UNIX        PF_UNIX
#define AF_LOCAL       PF_LOCAL
#define AF_INET        PF_INET
#define AF_INET6       PF_INET6
#define AF_NETLINK     PF_NETLINK
#define AF_ROUTE       PF_ROUTE
#define AF_PACKET      PF_PACKET
#define AF_CAN         PF_CAN
#define AF_BLUETOOTH   PF_BLUETOOTH
#define AF_IEEE802154  PF_IEEE802154
#define AF_PKTRADIO    PF_PKTRADIO
#define AF_RPMSG       PF_RPMSG

/* The socket created by socket() has the indicated type, which specifies
 * the communication semantics.
 */

#define SOCK_UNSPEC    0        /* Unspecified socket type */
#define SOCK_STREAM    1        /* Provides sequenced, reliable, two-way,
                                 * connection-based byte streams. An out-of-band data
                                 * transmission mechanism may be supported.
                                 */
#define SOCK_DGRAM     2        /* Supports  datagrams (connectionless, unreliable
                                 * messages of a fixed maximum length).
                                 */
#define SOCK_RAW       3        /* Provides raw network protocol access. */
#define SOCK_RDM       4        /* Provides a reliable datagram layer that does not
                                 * guarantee ordering.
                                 */
#define SOCK_SEQPACKET 5        /* Provides a sequenced, reliable, two-way
                                 * connection-based data transmission path for
                                 * datagrams of fixed maximum length; a consumer is
                                 * required to read an entire packet with each read
                                 * system call.
                                 */
#define SOCK_CTRL      6        /* SOCK_CTRL is the preferred socket type to use
                                 * when we just want a socket for performing driver
                                 * ioctls. This definition is not POSIX compliant.
                                 */
#define SOCK_PACKET   10        /* Obsolete and should not be used in new programs */

#define SOCK_CLOEXEC  02000000  /* Atomically set close-on-exec flag for the new
                                 * descriptor(s).
                                 */
#define SOCK_NONBLOCK 00004000  /* Atomically mark descriptor(s) as non-blocking. */

#define SOCK_MAX (SOCK_PACKET + 1)
#define SOCK_TYPE_MASK 0xf      /* Mask which covers at least up to SOCK_MASK-1.
                                 * The remaining bits are used as flags.
                                 */

/* Bits in the FLAGS argument to `send', `recv', et al. These are the bits
 * recognized by Linux, not all are supported by NuttX.
 */

#define MSG_OOB          0x000001 /* Process out-of-band data.  */
#define MSG_PEEK         0x000002 /* Peek at incoming messages.  */
#define MSG_DONTROUTE    0x000004 /* Don't use local routing.  */
#define MSG_CTRUNC       0x000008 /* Control data lost before delivery.  */
#define MSG_PROXY        0x000010 /* Supply or ask second address.  */
#define MSG_TRUNC        0x000020
#define MSG_DONTWAIT     0x000040 /* Enable nonblocking IO.  */
#define MSG_EOR          0x000080 /* End of record.  */
#define MSG_WAITALL      0x000100 /* Wait for a full request.  */
#define MSG_FIN          0x000200
#define MSG_SYN          0x000400
#define MSG_CONFIRM      0x000800 /* Confirm path validity.  */
#define MSG_RST          0x001000
#define MSG_ERRQUEUE     0x002000 /* Fetch message from error queue.  */
#define MSG_NOSIGNAL     0x004000 /* Do not generate SIGPIPE.  */
#define MSG_MORE         0x008000 /* Sender will send more.  */
#define MSG_CMSG_CLOEXEC 0x100000 /* Set close_on_exit for file
                                   * descriptor received through SCM_RIGHTS.
                                   */

/* Protocol levels supported by get/setsockopt(): */

#define SOL_SOCKET       1 /* Only socket-level options supported */

/* Socket-level options */

#define SO_ACCEPTCONN    0 /* Reports whether socket listening is enabled
                            * (get only).
                            * arg: pointer to integer containing a boolean
                            * value
                            */
#define SO_BROADCAST     1 /* Permits sending of broadcast messages (get/set).
                            * arg: pointer to integer containing a boolean
                            * value
                            */
#define SO_DEBUG         2 /* Enables recording of debugging information
                            * (get/set).
                            * arg: pointer to integer containing a boolean
                            * value
                            */
#define SO_DONTROUTE     3 /* Requests that outgoing messages bypass standard
                            * routing (get/set)
                            * arg: pointer to integer containing a boolean
                            * value
                            */
#define SO_ERROR         4 /* Reports and clears error status (get only).
                            * arg: returns an integer value
                            */
#define SO_KEEPALIVE     5 /* Keeps connections active by enabling the periodic
                            * transmission of messages (get/set).
                            * arg:  pointer to integer containing a boolean int
                            * value
                            */
#define SO_LINGER        6 /* Lingers on a close() if data is present (get/set)
                            * arg: struct linger
                            */
#define SO_OOBINLINE     7 /* Leaves received out-of-band data (data marked
                            * urgent) inline
                            * (get/set) arg: pointer to integer containing a
                            * boolean value
                            */
#define SO_RCVBUF        8 /* Sets receive buffer size.
                            * arg: integer value (get/set).
                            */
#define SO_RCVLOWAT      9 /* Sets the minimum number of bytes to process for
                            * socket input (get/set).
                            * arg: integer value
                            */
#define SO_RCVTIMEO     10 /* Sets the timeout value that specifies the maximum
                            * amount of time an input function waits until it
                            * completes (get/set).
                            * arg: struct timeval
                            */
#define SO_REUSEADDR    11 /* Allow reuse of local addresses (get/set)
                            * arg: pointer to integer containing a boolean
                            * value
                            */
#define SO_SNDBUF       12 /* Sets send buffer size (get/set).
                            * arg: integer value
                            */
#define SO_SNDLOWAT     13 /* Sets the minimum number of bytes to process for
                            * socket output (get/set).
                            * arg: integer value
                            */
#define SO_SNDTIMEO     14 /* Sets the timeout value specifying the amount of
                            * time that an output function blocks because flow
                            * control prevents data from being sent(get/set).
                            * arg: struct timeval
                            */
#define SO_TYPE         15 /* Reports the socket type (get only).
                            * return: int
                            */
#define SO_TIMESTAMP    16 /* Generates a timestamp for each incoming packet
                            * arg: integer value
                            */
#define SO_BINDTODEVICE 17 /* Bind this socket to a specific network device.
                            */
#define SO_PEERCRED     18 /* Return the credentials of the peer process
                            * connected to this socket.
                            */

/* The options are unsupported but included for compatibility
 * and portability
 */
#define SO_SNDBUFFORCE  32
#define SO_RCVBUFFORCE  33
#define SO_RXQ_OVFL     40

/* Protocol-level socket operations. */

#define SOL_IP          IPPROTO_IP   /* See options in include/netinet/ip.h */
#define SOL_IPV6        IPPROTO_IPV6 /* See options in include/netinet/ip6.h */
#define SOL_TCP         IPPROTO_TCP  /* See options in include/netinet/tcp.h */
#define SOL_UDP         IPPROTO_UDP  /* See options in include/netinit/udp.h */

/* Bluetooth-level operations. */

#define SOL_HCI         0  /* See options in include/netpacket/bluetooth.h */
#define SOL_L2CAP       6  /* See options in include/netpacket/bluetooth.h */
#define SOL_SCO         17 /* See options in include/netpacket/bluetooth.h */
#define SOL_RFCOMM      18 /* See options in include/netpacket/bluetooth.h */

/* Protocol-level socket options may begin with this value */

#define __SO_PROTOCOL  16

/* Values for the 'how' argument of shutdown() */

#define SHUT_RD         1 /* Bit 0: Disables further receive operations */
#define SHUT_WR         2 /* Bit 1: Disables further send operations */
#define SHUT_RDWR       3 /* Bits 0+1: Disables further send and receive
                           * operations
                           */

/* The maximum backlog queue length */

#ifdef CONFIG_NET_TCPBACKLOG_CONNS
#  define SOMAXCONN CONFIG_NET_TCPBACKLOG_CONNS
#else
#  define SOMAXCONN 8
#endif

/* Definitions associated with sendmsg/recvmsg */

#define CMSG_NXTHDR(mhdr, cmsg) cmsg_nxthdr((mhdr), (cmsg))

#define CMSG_ALIGN(len) \
  (((len)+sizeof(long)-1) & ~(sizeof(long)-1))
#define CMSG_DATA(cmsg) \
  ((FAR void *)((FAR char *)(cmsg) + CMSG_ALIGN(sizeof(struct cmsghdr))))
#define CMSG_SPACE(len) \
  (CMSG_ALIGN(sizeof(struct cmsghdr)) + CMSG_ALIGN(len))
#define CMSG_LEN(len)   \
  (CMSG_ALIGN(sizeof(struct cmsghdr)) + (len))

#define __CMSG_FIRSTHDR(ctl, len) \
  ((len) >= sizeof(struct cmsghdr) ? (FAR struct cmsghdr *)(ctl) : NULL)
#define CMSG_FIRSTHDR(msg) \
  __CMSG_FIRSTHDR((msg)->msg_control, (msg)->msg_controllen)
#define CMSG_OK(mhdr, cmsg) ((cmsg)->cmsg_len >= sizeof(struct cmsghdr) && \
                            (cmsg)->cmsg_len <= (unsigned long) \
                            ((mhdr)->msg_controllen - \
                             ((char *)(cmsg) - (char *)(mhdr)->msg_control)))
#define for_each_cmsghdr(cmsg, msg) \
       for (cmsg = CMSG_FIRSTHDR(msg); \
            cmsg; \
            cmsg = CMSG_NXTHDR(msg, cmsg))

/* "Socket"-level control message types: */

#define SCM_RIGHTS      0x01    /* rw: access rights (array of int) */
#define SCM_CREDENTIALS 0x02    /* rw: struct ucred */
#define SCM_SECURITY    0x03    /* rw: security label */

/* Desired design of maximum size and alignment (see RFC2553) */

#define SS_MAXSIZE      128  /* Implementation specific max size */
#define SS_ALIGNSIZE    (sizeof(FAR struct sockaddr *))
                             /* Implementation specific desired alignment */

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/* sockaddr_storage structure. This structure must be (1) large enough to
 * accommodate all supported protocol-specific address structures, and (2)
 * aligned at an appropriate boundary so that pointers to it can be cast
 * as pointers to protocol-specific address structures and used to access
 * the fields of those structures without alignment problems.
 */

struct sockaddr_storage
{
  sa_family_t ss_family;     /* Address family */
  char        ss_data[SS_MAXSIZE - sizeof(sa_family_t)];
}
aligned_data(SS_ALIGNSIZE);  /* Force desired alignment */

/* The sockaddr structure is used to define a socket address which is used
 * in the bind(), connect(), getpeername(), getsockname(), recvfrom(), and
 * sendto() functions.
 */

struct sockaddr
{
  sa_family_t sa_family;       /* Address family: See AF_* definitions */
  char        sa_data[14];     /* 14-bytes data (actually variable length) */
};

/* Used with the SO_LINGER socket option */

struct linger
{
  int  l_onoff;   /* Indicates whether linger option is enabled. */
  int  l_linger;  /* Linger time, in seconds. */
};

struct msghdr
{
  FAR void *msg_name;           /* Socket name */
  socklen_t msg_namelen;        /* Length of name */
  FAR struct iovec *msg_iov;    /* Data blocks */
  unsigned long msg_iovlen;     /* Number of blocks */
  FAR void *msg_control;        /* Per protocol magic (eg BSD file descriptor passing) */
  unsigned long msg_controllen; /* Length of cmsg list */
  unsigned int msg_flags;
};

struct cmsghdr
{
  unsigned long cmsg_len;       /* Data byte count, including hdr */
  int cmsg_level;               /* Originating protocol */
  int cmsg_type;                /* Protocol-specific type */
};

struct ucred
{
  pid_t pid;
  uid_t uid;
  gid_t gid;
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline FAR struct cmsghdr *__cmsg_nxthdr(FAR void *__ctl,
                                                unsigned int __size,
                                                FAR struct cmsghdr *__cmsg)
{
  size_t len = CMSG_ALIGN(__cmsg->cmsg_len);
  FAR struct cmsghdr *__ptr =
               (FAR struct cmsghdr *)(((FAR char *)__cmsg) + len);

  if (len < sizeof(*__cmsg) ||
      (unsigned long)((FAR char *)(__ptr + 1) - (FAR char *)__ctl) > __size)
    {
      return NULL;
    }

  return __ptr;
}

static inline FAR struct cmsghdr *cmsg_nxthdr(FAR struct msghdr *__msg,
                                              FAR struct cmsghdr *__cmsg)
{
  return __cmsg_nxthdr(__msg->msg_control, __msg->msg_controllen, __cmsg);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

int socket(int domain, int type, int protocol);
int socketpair(int domain, int type, int protocol, int sv[2]);
int bind(int sockfd, FAR const struct sockaddr *addr, socklen_t addrlen);
int connect(int sockfd, FAR const struct sockaddr *addr, socklen_t addrlen);

int listen(int sockfd, int backlog);
int accept(int sockfd, FAR struct sockaddr *addr, FAR socklen_t *addrlen);
int accept4(int sockfd, FAR struct sockaddr *addr, FAR socklen_t *addrlen,
            int flags);

ssize_t send(int sockfd, FAR const void *buf, size_t len, int flags);
ssize_t sendto(int sockfd, FAR const void *buf, size_t len, int flags,
               FAR const struct sockaddr *to, socklen_t tolen);

ssize_t recv(int sockfd, FAR void *buf, size_t len, int flags);
ssize_t recvfrom(int sockfd, FAR void *buf, size_t len, int flags,
                 FAR struct sockaddr *from, FAR socklen_t *fromlen);

int shutdown(int sockfd, int how);

int setsockopt(int sockfd, int level, int option,
               FAR const void *value, socklen_t value_len);
int getsockopt(int sockfd, int level, int option,
               FAR void *value, FAR socklen_t *value_len);

int getsockname(int sockfd, FAR struct sockaddr *addr,
                FAR socklen_t *addrlen);
int getpeername(int sockfd, FAR struct sockaddr *addr,
                FAR socklen_t *addrlen);

ssize_t recvmsg(int sockfd, FAR struct msghdr *msg, int flags);
ssize_t sendmsg(int sockfd, FAR struct msghdr *msg, int flags);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_SYS_SOCKET_H */
