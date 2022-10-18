/****************************************************************************
 * drivers/modem/alt1250/altcom_cmd_sock.h
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

#ifndef __DRIVERS_MODEM_ALT1250_ALTCOM_CMD_SOCK_H
#define __DRIVERS_MODEM_ALT1250_ALTCOM_CMD_SOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/modem/alt1250.h>

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define APICMD_NAME_LENGTH        (256)
#define APICMD_SERVNAME_LENGTH    (32)
#define APICMD_H_ADDR_LENGTH      (16)
#define APICMD_OPTVAL_LENGTH      (16)
#define APICMD_DATA_LENGTH        (1500)
#define APICMD_AI_COUNT           (2)

#define APICMD_SELECT_NONBLOCK    (0)
#define APICMD_SELECT_BLOCK       (1)
#define APICMD_SELECT_BLOCKCANCEL (2)
#define APICMD_SELECT_READ_BIT    (1 << 0)
#define APICMD_SELECT_WRITE_BIT   (1 << 1)
#define APICMD_SELECT_EXCEPT_BIT  (1 << 2)

/* Using for socket: Address family
 * Referenced from socket.h of lwIP-v2.02
 */

#define ALTCOM_AF_UNSPEC           0                /* Refer to AF_UNSPEC on lwIP */
#define ALTCOM_AF_INET             2                /* Refer to AF_INET on lwIP   */
#define ALTCOM_AF_INET6            10               /* Refer to AF_INET6 on lwIP  */
#define ALTCOM_PF_INET             ALTCOM_AF_INET   /* Refer to PF_INET on lwIP   */
#define ALTCOM_PF_INET6            ALTCOM_AF_INET6  /* Refer to PF_INET6 on lwIP  */
#define ALTCOM_PF_UNSPEC           ALTCOM_AF_UNSPEC /* Refer to PF_UNSPEC on lwIP */

/* Using for socket: Socket protocol type
 * Referenced from socket.h of lwIP-v2.02
 */

#define ALTCOM_SOCK_STREAM         1   /* Refer to SOCK_STREAM on lwIP     */
#define ALTCOM_SOCK_DGRAM          2   /* Refer to SOCK_DGRAM on lwIP      */
#define ALTCOM_SOCK_RAW            3   /* Refer to SOCK_RAW on lwIP        */
#define ALTCOM_SOCK_DGRAM_DTLS     130 /* Refer to SOCK_DGRAM_DTLS on lwIP */

/* Using for socket: Protocol
 * Referenced from socket.h of lwIP-v2.02
 */

#define ALTCOM_IPPROTO_IP          0   /* Refer to IPPROTO_IP on lwIP      */
#define ALTCOM_IPPROTO_ICMP        1   /* Refer to IPPROTO_ICMP on lwIP    */
#define ALTCOM_IPPROTO_TCP         6   /* Refer to IPPROTO_TCP on lwIP     */
#define ALTCOM_IPPROTO_UDP         17  /* Refer to IPPROTO_UDP on lwIP     */
#define ALTCOM_IPPROTO_IPV6        41  /* Refer to IPPROTO_IPV6 on lwIP    */
#define ALTCOM_IPPROTO_ICMPV6      58  /* Refer to IPPROTO_ICMPV6 on lwIP  */
#define ALTCOM_IPPROTO_UDPLITE     136 /* Refer to IPPROTO_UDPLITE on lwIP */
#define ALTCOM_IPPROTO_RAW         255 /* Refer to IPPROTO_RAW on lwIP     */

/* Using for recvfrom/sendto: Flags
 * Referenced from socket.h of lwIP-v2.02
 */

#define ALTCOM_MSG_PEEK            0x01 /* Refer to MSG_PEEK on lwIP     */
#define ALTCOM_MSG_WAITALL         0x02 /* Refer to MSG_WAITALL on lwIP  */
#define ALTCOM_MSG_OOB             0x04 /* Refer to MSG_OOB on lwIP      */
#define ALTCOM_MSG_DONTWAIT        0x08 /* Refer to MSG_DONTWAIT on lwIP */
#define ALTCOM_MSG_MORE            0x10 /* Refer to MSG_MORE on lwIP     */

/* Using for setsockopt/getsockopt: Level
 * Referenced from socket.h of lwIP-v2.02
 */

#define ALTCOM_SOL_SOCKET          0xfff /* Refer to SOL_SOCKET on lwIP */

/* Using for setsockopt/getsockopt: Option flags per-socket
 * Referenced from socket.h of lwIP-v2.02
 */

#define ALTCOM_SO_REUSEADDR        0x0004 /* Refer to SO_REUSEADDR on lwIP */
#define ALTCOM_SO_KEEPALIVE        0x0008 /* Refer to SO_KEEPALIVE on lwIP */
#define ALTCOM_SO_BROADCAST        0x0020 /* Refer to SO_BROADCAST on lwIP */

/* Using for setsockopt/getsockopt:
 * Additional options, not kept in so_options
 * Referenced from socket.h of lwIP-v2.02
 */

#define ALTCOM_SO_ACCEPTCONN       0x0002 /* Refer to SO_ACCEPTCONN on lwIP */
#define ALTCOM_SO_LINGER           0x0080 /* Refer to SO_LINGER on lwIP     */
#define ALTCOM_SO_RCVBUF           0x1002 /* Refer to SO_RCVBUF on lwIP     */
#define ALTCOM_SO_SNDTIMEO         0x1005 /* Refer to SO_SNDTIMEO on lwIP   */
#define ALTCOM_SO_RCVTIMEO         0x1006 /* Refer to SO_RCVTIMEO on lwIP   */
#define ALTCOM_SO_ERROR            0x1007 /* Refer to SO_ERROR on lwIP      */
#define ALTCOM_SO_TYPE             0x1008 /* Refer to SO_TYPE on lwIP       */
#define ALTCOM_SO_NO_CHECK         0x100a /* Refer to SO_NO_CHECK on lwIP   ss*/

/* Options for level IPPROTO_IP
 * Referenced from socket.h of lwIP-v2.02
 */

#define ALTCOM_IP_TOS              1 /* Refer to IP_TOS on lwIP */
#define ALTCOM_IP_TTL              2 /* Refer to IP_TTL on lwIP */

/* Options and types related to multicast membership
 * Referenced from socket.h of lwIP-v2.02
 */

#define ALTCOM_IP_ADD_MEMBERSHIP   3 /* Refer to IP_ADD_MEMBERSHIP on lwIP  */
#define ALTCOM_IP_DROP_MEMBERSHIP  4 /* Refer to IP_DROP_MEMBERSHIP on lwIP */

/* Options and types for UDP multicast traffic handling
 * Referenced from socket.h of lwIP-v2.02
 */

#define ALTCOM_IP_MULTICAST_TTL    5 /* Refer to IP_MULTICAST_TTL on lwIP  */
#define ALTCOM_IP_MULTICAST_IF     6 /* Refer to IP_MULTICAST_IF on lwIP   */
#define ALTCOM_IP_MULTICAST_LOOP   7 /* Refer to IP_MULTICAST_LOOP on lwIP */

/* Options for level ALTCOM_IPPROTO_TCP
 * Referenced from socket.h of lwIP-v2.02
 */

#define ALTCOM_TCP_NODELAY         0x01 /* Refer to TCP_NODELAY on lwIP   */
#define ALTCOM_TCP_KEEPALIVE       0x02 /* Refer to TCP_KEEPALIVE on lwIP */
#define ALTCOM_TCP_KEEPIDLE        0x03 /* Refer to TCP_KEEPIDLE on lwIP  */
#define ALTCOM_TCP_KEEPINTVL       0x04 /* Refer to TCP_KEEPINTVL on lwIP */
#define ALTCOM_TCP_KEEPCNT         0x05 /* Refer to TCP_KEEPCNT on lwIP   */

/* Options for level ALTCOM_IPPROTO_IPV6
 * Referenced from socket.h of lwIP-v2.02
 */

#define ALTCOM_IPV6_V6ONLY         27 /* Refer to IPV6_V6ONLY on lwIP */

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint8_t  altcom_sa_family_t;
typedef uint32_t altcom_socklen_t;
typedef uint16_t altcom_in_port_t;

typedef uint32_t altcom_in_addr_t;

struct altcom_in_addr
{
  altcom_in_addr_t s_addr;
};

struct altcom_in6_addr
{
  union
  {
    uint32_t u32_addr[4];
    uint16_t u16_addr[8];
    uint8_t  u8_addr[16];
  } un;
#define altcom_s6_addr  un.u8_addr
};

struct altcom_sockaddr_storage
{
  uint8_t            s2_len;
  altcom_sa_family_t ss_family;
  char               s2_data1[2];
  uint32_t           s2_data2[3];
  uint32_t           s2_data3[3];
};

struct altcom_sockaddr_in
{
  uint8_t                sin_len;
  altcom_sa_family_t     sin_family;
  altcom_in_port_t       sin_port;
  struct altcom_in_addr  sin_addr;
#define ALTCOM_SIN_ZERO_LEN 8
  char                   sin_zero[ALTCOM_SIN_ZERO_LEN];
};

struct altcom_sockaddr_in6
{
  uint8_t                sin6_len;
  altcom_sa_family_t     sin6_family;
  altcom_in_port_t       sin6_port;
  uint32_t               sin6_flowinfo;
  struct altcom_in6_addr sin6_addr;
  uint32_t               sin6_scope_id;
};

struct altcom_ip_mreq
{
  struct altcom_in_addr imr_multiaddr;
  struct altcom_in_addr imr_interface;
};

struct altcom_linger
{
  int l_onoff;
  int l_linger;
};

/* structure for APICMDID_SOCK_ACCEPT and APICMDID_SOCK_GETSOCKNAME */

begin_packed_struct struct altmdmpkt_sockaddrlen_s
{
  int32_t sockfd;
  uint32_t addrlen;
} end_packed_struct;

/* structure for APICMDID_SOCK_BIND and APICMDID_SOCK_CONNECT */

begin_packed_struct struct altmdmpkt_sockaddr_s
{
  int32_t sockfd;
  uint32_t namelen;
  struct altcom_sockaddr_storage name;
} end_packed_struct;

/* structure for APICMDID_SOCK_CLOSE */

begin_packed_struct struct apicmd_close_s
{
  int32_t sockfd;
} end_packed_struct;

/* structure for APICMDID_SOCK_FCNTL */

begin_packed_struct struct apicmd_fcntl_s
{
  int32_t sockfd;
  int32_t cmd;
  int32_t val;
} end_packed_struct;

/* structure for APICMDID_SOCK_GETADDRINFO */

begin_packed_struct struct apicmd_getaddrinfo_s
{
  uint32_t nodenamelen;
  int8_t nodename[APICMD_NAME_LENGTH];
  uint32_t servnamelen;
  int8_t servname[APICMD_SERVNAME_LENGTH];
  int32_t hints_flag;
  int32_t ai_flags;
  int32_t ai_family;
  int32_t ai_socktype;
  int32_t ai_protocol;
} end_packed_struct;

/* structure for APICMDID_SOCK_GETHOSTBYNAME */

begin_packed_struct struct apicmd_gethostbyname_s
{
  uint32_t namelen;
  int8_t name[APICMD_NAME_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_GETHOSTBYNAMER */

begin_packed_struct struct apicmd_gethostbynamer_s
{
  uint32_t namelen;
  int8_t name[APICMD_NAME_LENGTH];
  int32_t buflen;
} end_packed_struct;

/* structure for APICMDID_SOCK_GETSOCKOPT */

begin_packed_struct struct apicmd_getsockopt_s
{
  int32_t sockfd;
  int32_t level;
  int32_t optname;
  int32_t optlen;
} end_packed_struct;

/* structure for APICMDID_SOCK_LISTEN */

begin_packed_struct struct apicmd_listen_s
{
  int32_t sockfd;
  int32_t backlog;
} end_packed_struct;

/* structure for APICMDID_SOCK_RECV */

begin_packed_struct struct apicmd_recv_s
{
  int32_t sockfd;
  int32_t recvlen;
  int32_t flags;
} end_packed_struct;

/* structure for APICMDID_SOCK_RECVFROM */

begin_packed_struct struct apicmd_recvfrom_s
{
  int32_t sockfd;
  int32_t recvlen;
  int32_t flags;
  uint32_t fromlen;
} end_packed_struct;

/* structure for APICMDID_SOCK_SELECT */

begin_packed_struct struct apicmd_select_s
{
  int32_t request;
  int32_t id;
  int32_t maxfds;
  uint16_t used_setbit;
  altcom_fd_set readset;
  altcom_fd_set writeset;
  altcom_fd_set exceptset;
} end_packed_struct;

/* structure for APICMDID_SOCK_SEND */

begin_packed_struct struct apicmd_send_s
{
  int32_t sockfd;
  int32_t flags;
  int32_t datalen;
  int8_t senddata[APICMD_DATA_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_SENDTO */

begin_packed_struct struct apicmd_sendto_s
{
  int32_t sockfd;
  int32_t flags;
  int32_t datalen;
  uint32_t tolen;
  struct altcom_sockaddr_storage to;
  int8_t senddata[APICMD_DATA_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_SETSOCKOPT */

begin_packed_struct struct apicmd_setsockopt_s
{
  int32_t sockfd;
  int32_t level;
  int32_t optname;
  int32_t optlen;
  int8_t optval[APICMD_OPTVAL_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_SHUTDOWN */

begin_packed_struct struct apicmd_shutdown_s
{
  int32_t sockfd;
  int32_t how;
} end_packed_struct;

/* structure for APICMDID_SOCK_SOCKET */

begin_packed_struct struct apicmd_socket_s
{
  int32_t domain;
  int32_t type;
  int32_t protocol;
} end_packed_struct;

/****************************************************************************/

/* structure for APICMDID_SOCK_BIND and APICMDID_SOCK_CLOSE,
 * APICMDID_SOCK_CONNECT, APICMDID_SOCK_FCNTL, APICMDID_SOCK_LISTEN,
 * APICMDID_SOCK_SEND, APICMDID_SOCK_SENDTO, APICMDID_SOCK_SETSOCKOPT,
 * APICMDID_SOCK_SHUTDOWN, APICMDID_SOCK_SOCKET
 */

begin_packed_struct struct altmdmpktr_sockcomm_s
{
  int32_t ret_code;
  int32_t err_code;
} end_packed_struct;

/* structure for APICMDID_SOCK_ACCEPT and APICMDID_SOCK_GETSOCKNAME */

begin_packed_struct struct altmdmpktr_sockaddr_s
{
  int32_t ret_code;
  int32_t err_code;
  uint32_t addrlen;
  struct altcom_sockaddr_storage address;
} end_packed_struct;

/* structure for APICMDID_SOCK_GETADDRINFO */

begin_packed_struct struct apicmd_getaddrinfo_ai_s
{
  int32_t ai_flags;
  int32_t ai_family;
  int32_t ai_socktype;
  int32_t ai_protocol;
  altcom_socklen_t ai_addrlen;
  struct altcom_sockaddr_storage ai_addr;
  uint32_t ai_cnamelen;
  int8_t ai_canonname[APICMD_NAME_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_GETADDRINFO */

begin_packed_struct struct apicmd_getaddrinfores_s
{
  int32_t ret_code;
  uint32_t ai_num;
  struct apicmd_getaddrinfo_ai_s ai[APICMD_AI_COUNT];
} end_packed_struct;

/* structure for APICMDID_SOCK_GETHOSTBYNAME */

begin_packed_struct struct apicmd_gethostbynameres_s
{
  int32_t ret_code;
  int8_t h_name[APICMD_NAME_LENGTH];
  int8_t h_aliases[APICMD_NAME_LENGTH];
  int32_t h_addrtype;
  int32_t h_length;
  int8_t h_addr_list[APICMD_H_ADDR_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_GETHOSTBYNAMER */

begin_packed_struct struct apicmd_gethostbynamer_res_s
{
  int32_t ret_code;
  int32_t err_code;
  int8_t h_name[APICMD_NAME_LENGTH];
  int8_t h_aliases[APICMD_NAME_LENGTH];
  int32_t h_addrtype;
  int32_t h_length;
  int8_t h_addr_list[APICMD_H_ADDR_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_GETSOCKOPT */

begin_packed_struct struct apicmd_getsockoptres_s
{
  int32_t ret_code;
  int32_t err_code;
  int32_t optlen;
  int8_t optval[APICMD_OPTVAL_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_RECV */

begin_packed_struct struct apicmd_recvres_s
{
  int32_t ret_code;
  int32_t err_code;
  int8_t recvdata[APICMD_DATA_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_RECVFROM */

begin_packed_struct struct apicmd_recvfromres_s
{
  int32_t ret_code;
  int32_t err_code;
  uint32_t fromlen;
  struct altcom_sockaddr_storage from;
  int8_t recvdata[APICMD_DATA_LENGTH];
} end_packed_struct;

/* structure for APICMDID_SOCK_SELECT */

begin_packed_struct struct apicmd_selectres_s
{
  int32_t ret_code;
  int32_t err_code;
  int32_t id;
  uint16_t used_setbit;
  altcom_fd_set readset;
  altcom_fd_set writeset;
  altcom_fd_set exceptset;
} end_packed_struct;

#endif  /* __DRIVERS_MODEM_ALT1250_ALTCOM_CMD_SOCK_H */
