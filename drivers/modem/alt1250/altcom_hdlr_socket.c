/****************************************************************************
 * drivers/modem/alt1250/altcom_hdlr_socket.c
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

#include <nuttx/config.h>

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <nuttx/modem/alt1250.h>

#include "altcom_cmd_sock.h"
#include "altcom_errno.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define READSET_BIT   (1 << 0)
#define WRITESET_BIT  (1 << 1)
#define EXCEPTSET_BIT (1 << 2)

#define ALTCOM_SO_SETMODE_8BIT   (1)
#define ALTCOM_SO_SETMODE_32BIT  (2)
#define ALTCOM_SO_SETMODE_LINGER (3)
#define ALTCOM_SO_SETMODE_INADDR (4)
#define ALTCOM_SO_SETMODE_IPMREQ (5)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void sockaddr2altstorage(FAR const struct sockaddr *from,
                                FAR struct altcom_sockaddr_storage *to)
{
  FAR struct sockaddr_in         *inaddr_from;
  FAR struct sockaddr_in6        *in6addr_from;
  FAR struct altcom_sockaddr_in  *inaddr_to;
  FAR struct altcom_sockaddr_in6 *in6addr_to;

  if (from->sa_family == AF_INET)
    {
      inaddr_from = (FAR struct sockaddr_in *)from;
      inaddr_to   = (FAR struct altcom_sockaddr_in *)to;

      inaddr_to->sin_len    = sizeof(struct altcom_sockaddr_in);
      inaddr_to->sin_family = ALTCOM_AF_INET;
      inaddr_to->sin_port   = inaddr_from->sin_port;
      memcpy(&inaddr_to->sin_addr, &inaddr_from->sin_addr,
        sizeof(struct altcom_in_addr));
    }
  else if (from->sa_family == AF_INET6)
    {
      in6addr_from = (FAR struct sockaddr_in6 *)from;
      in6addr_to   = (FAR struct altcom_sockaddr_in6 *)to;

      in6addr_to->sin6_len    = sizeof(struct altcom_sockaddr_in6);
      in6addr_to->sin6_family = ALTCOM_AF_INET6;
      in6addr_to->sin6_port   = in6addr_from->sin6_port;
      memcpy(&in6addr_to->sin6_addr, &in6addr_from->sin6_addr,
        sizeof(struct altcom_in6_addr));
    }
}

static void altstorage2sockaddr(
  FAR const struct altcom_sockaddr_storage *from, FAR struct sockaddr *to)
{
  FAR struct altcom_sockaddr_in  *inaddr_from;
  FAR struct altcom_sockaddr_in6 *in6addr_from;
  FAR struct sockaddr_in         *inaddr_to;
  FAR struct sockaddr_in6        *in6addr_to;

  if (from->ss_family == ALTCOM_AF_INET)
    {
      inaddr_from = (FAR struct altcom_sockaddr_in *)from;
      inaddr_to   = (FAR struct sockaddr_in *)to;

      inaddr_to->sin_family = AF_INET;
      inaddr_to->sin_port   = inaddr_from->sin_port;
      memcpy(&inaddr_to->sin_addr, &inaddr_from->sin_addr,
        sizeof(struct in_addr));

      /* LwIP does not use this member, so it should be set to 0 */

      memset(inaddr_to->sin_zero, 0, sizeof(inaddr_to->sin_zero));
    }
  else if (from->ss_family == ALTCOM_AF_INET6)
    {
      in6addr_from = (FAR struct altcom_sockaddr_in6 *)from;
      in6addr_to   = (FAR struct sockaddr_in6 *)to;

      in6addr_to->sin6_family = AF_INET6;
      in6addr_to->sin6_port   = in6addr_from->sin6_port;
      memcpy(&in6addr_to->sin6_addr, &in6addr_from->sin6_addr,
        sizeof(struct in6_addr));

      /* LwIP does not use thease members, so it should be set to 0 */

      in6addr_to->sin6_flowinfo = 0;
      in6addr_to->sin6_scope_id = 0;
    }
}

static int flags2altflags(int32_t flags, FAR int32_t *altflags)
{
  if (flags & (MSG_DONTROUTE | MSG_CTRUNC | MSG_PROXY | MSG_TRUNC |
               MSG_EOR | MSG_FIN | MSG_SYN | MSG_CONFIRM |
               MSG_RST | MSG_ERRQUEUE | MSG_NOSIGNAL))
    {
      return -ENOPROTOOPT;
    }

  *altflags = 0;

  if (flags & MSG_PEEK)
    {
      *altflags |= ALTCOM_MSG_PEEK;
    }

  if (flags & MSG_WAITALL)
    {
      *altflags |= ALTCOM_MSG_WAITALL;
    }

  if (flags & MSG_OOB)
    {
      *altflags |= ALTCOM_MSG_OOB;
    }

  if (flags & MSG_DONTWAIT)
    {
      *altflags |= ALTCOM_MSG_DONTWAIT;
    }

  if (flags & MSG_MORE)
    {
      *altflags |= ALTCOM_MSG_MORE;
    }

  return 0;
}

static int get_so_setmode(uint16_t level, uint16_t option)
{
  int setmode = 0;

  switch (level)
    {
      case SOL_SOCKET:
        switch (option)
          {
            case SO_ACCEPTCONN:
            case SO_ERROR:
            case SO_BROADCAST:
            case SO_KEEPALIVE:
            case SO_REUSEADDR:
            case SO_TYPE:
            case SO_RCVBUF:
              setmode = ALTCOM_SO_SETMODE_32BIT;
              break;
#ifdef CONFIG_NET_SOLINGER
            case SO_LINGER:
              setmode = ALTCOM_SO_SETMODE_LINGER;
              break;
#endif
            default:
              m_err("Not support option: %u\n", option);
              setmode = -EILSEQ;
              break;
          }
        break;
      case IPPROTO_IP:
        switch (option)
          {
            case IP_TOS:
            case IP_TTL:
              setmode = ALTCOM_SO_SETMODE_32BIT;
              break;
            case IP_MULTICAST_TTL:
            case IP_MULTICAST_LOOP:
              setmode = ALTCOM_SO_SETMODE_8BIT;
              break;
            case IP_MULTICAST_IF:
              setmode = ALTCOM_SO_SETMODE_INADDR;
              break;
            case IP_ADD_MEMBERSHIP:
            case IP_DROP_MEMBERSHIP:
              setmode = ALTCOM_SO_SETMODE_IPMREQ;
              break;
            default:
              m_err("Not support option: %u\n", option);
              setmode = -EILSEQ;
              break;
          }
        break;
      case IPPROTO_TCP:
        switch (option)
          {
            case TCP_NODELAY:
            case TCP_KEEPIDLE:
            case TCP_KEEPINTVL:
              setmode = ALTCOM_SO_SETMODE_32BIT;
              break;
            default:
              m_err("Not support option: %u\n", option);
              setmode = -EILSEQ;
              break;
          }
        break;
      case IPPROTO_IPV6:
        switch (option)
          {
            case IPV6_V6ONLY:
              setmode = ALTCOM_SO_SETMODE_32BIT;
              break;
            default:
              m_err("Not support option: %u\n", option);
              setmode = -EILSEQ;
              break;
          }
        break;
      default:
        m_err("Not support level: %u\n", level);
        setmode = -EILSEQ;
        break;
    }

  return setmode;
}

static int32_t sockaddrlen_pkt_compose(FAR void **arg, size_t arglen,
                                       uint8_t altver, FAR uint8_t *pktbuf,
                                       const size_t pktsz)
{
  int32_t size = 0;
  FAR int16_t *usockid = (FAR int16_t *)arg[0];
  FAR int16_t *addrlen = (FAR int16_t *)arg[1];

  FAR struct altmdmpkt_sockaddrlen_s *out =
    (FAR struct altmdmpkt_sockaddrlen_s *)pktbuf;

  out->sockfd = htonl(*usockid);
  if (*addrlen == sizeof(struct sockaddr_in))
    {
      out->addrlen = htonl(sizeof(struct altcom_sockaddr_in));
    }
  else if (*addrlen == sizeof(struct sockaddr_in6))
    {
      out->addrlen = htonl(sizeof(struct altcom_sockaddr_in6));
    }
  else
    {
      size = -EINVAL;
    }

  if (size == 0)
    {
      size = sizeof(struct altmdmpkt_sockaddrlen_s);
    }

  return size;
}

static int32_t sockaddr_pkt_compose(FAR void **arg, size_t arglen,
                                    uint8_t altver, FAR uint8_t *pktbuf,
                                    const size_t pktsz)
{
  int32_t size = 0;
  FAR int16_t *usockid = (FAR int16_t *)arg[0];
  FAR int16_t *addrlen = (FAR int16_t *)arg[1];
  FAR struct sockaddr_storage *sa = (FAR struct sockaddr_storage *)arg[2];

  FAR struct altmdmpkt_sockaddr_s *out =
    (FAR struct altmdmpkt_sockaddr_s *)pktbuf;
  struct altcom_sockaddr_storage altsa;

  out->sockfd = htonl(*usockid);
  if (*addrlen == sizeof(struct sockaddr_in))
    {
      out->namelen = htonl(sizeof(struct altcom_sockaddr_in));
    }
  else if (*addrlen == sizeof(struct sockaddr_in6))
    {
      out->namelen = htonl(sizeof(struct altcom_sockaddr_in6));
    }
  else
    {
      size = -EINVAL;
    }

  sockaddr2altstorage((struct sockaddr *)sa, &altsa);
  memcpy(&out->name, &altsa, sizeof(struct altcom_sockaddr_storage));

  if (size == 0)
    {
      size = sizeof(struct altmdmpkt_sockaddr_s);
    }

  return size;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int32_t altcom_socket_pkt_compose(FAR void **arg, size_t arglen,
                                  uint8_t altver, FAR uint8_t *pktbuf,
                                  const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int16_t *domain = (FAR int16_t *)arg[0];
  FAR int16_t *type = (FAR int16_t *)arg[1];
  FAR int16_t *protocol = (FAR int16_t *)arg[2];

  FAR struct apicmd_socket_s *out =
    (FAR struct apicmd_socket_s *)pktbuf;

  /* convert domain */

  if (*domain == PF_UNSPEC)
    {
      out->domain = htonl(ALTCOM_PF_UNSPEC);
    }
  else if (*domain == PF_INET)
    {
      out->domain = htonl(ALTCOM_PF_INET);
    }
  else if (*domain == PF_INET6)
    {
      out->domain = htonl(ALTCOM_PF_INET6);
    }
  else
    {
      size = -EAFNOSUPPORT;
    }

  /* convert type */

  if (*type == SOCK_STREAM)
    {
      out->type = htonl(ALTCOM_SOCK_STREAM);
    }
  else if (*type == SOCK_DGRAM)
    {
      out->type = htonl(ALTCOM_SOCK_DGRAM);
    }
  else if (*type == SOCK_RAW)
    {
      out->type = htonl(ALTCOM_SOCK_RAW);
    }
  else
    {
      size = -EAFNOSUPPORT;
    }

  /* convert protocol */

  if (*protocol == IPPROTO_IP)
    {
      out->protocol = htonl(ALTCOM_IPPROTO_IP);
    }
  else if (*protocol == IPPROTO_ICMP)
    {
      out->protocol = htonl(ALTCOM_IPPROTO_ICMP);
    }
  else if (*protocol == IPPROTO_TCP)
    {
      out->protocol = htonl(ALTCOM_IPPROTO_TCP);
    }
  else if (*protocol == IPPROTO_UDP)
    {
      out->protocol = htonl(ALTCOM_IPPROTO_UDP);
    }
  else if (*protocol == IPPROTO_IPV6)
    {
      out->protocol = htonl(ALTCOM_IPPROTO_IPV6);
    }
  else if (*protocol == IPPROTO_ICMP6)
    {
      out->protocol = htonl(ALTCOM_IPPROTO_ICMPV6);
    }
  else if (*protocol == IPPROTO_UDPLITE)
    {
      out->protocol = htonl(ALTCOM_IPPROTO_UDPLITE);
    }
  else if (*protocol == IPPROTO_RAW)
    {
      out->protocol = htonl(ALTCOM_IPPROTO_RAW);
    }
  else
    {
      size = -EAFNOSUPPORT;
    }

  if (size == 0)
    {
      size = sizeof(struct apicmd_socket_s);
    }

  *altcid = APICMDID_SOCK_SOCKET;

  return size;
}

int32_t altcom_close_pkt_compose(FAR void **arg, size_t arglen,
                                 uint8_t altver, FAR uint8_t *pktbuf,
                                 const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int16_t *usockid = (FAR int16_t *)arg[0];

  FAR struct apicmd_close_s *out =
    (FAR struct apicmd_close_s *)pktbuf;

  out->sockfd = htonl(*usockid);

  size = sizeof(struct apicmd_close_s);

  *altcid = APICMDID_SOCK_CLOSE;

  return size;
}

int32_t altcom_accept_pkt_compose(FAR void **arg, size_t arglen,
                                  uint8_t altver, FAR uint8_t *pktbuf,
                                  const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  size = sockaddrlen_pkt_compose(arg, arglen, altver, pktbuf, pktsz);

  *altcid = APICMDID_SOCK_ACCEPT;

  return size;
}

int32_t altcom_bind_pkt_compose(FAR void **arg, size_t arglen,
                                uint8_t altver, FAR uint8_t *pktbuf,
                                const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  size = sockaddr_pkt_compose(arg, arglen, altver, pktbuf, pktsz);

  *altcid = APICMDID_SOCK_BIND;

  return size;
}

int32_t altcom_connect_pkt_compose(FAR void **arg, size_t arglen,
                                   uint8_t altver, FAR uint8_t *pktbuf,
                                   const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;

  size = sockaddr_pkt_compose(arg, arglen, altver, pktbuf, pktsz);

  *altcid = APICMDID_SOCK_CONNECT;

  return size;
}

int32_t altcom_fcntl_pkt_compose(FAR void **arg, size_t arglen,
                                 uint8_t altver, FAR uint8_t *pktbuf,
                                 const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *sockfd = (FAR int32_t *)arg[0];
  FAR int32_t *cmd = (FAR int32_t *)arg[1];
  FAR int32_t *val = (FAR int32_t *)arg[2];

  FAR struct apicmd_fcntl_s *out =
    (FAR struct apicmd_fcntl_s *)pktbuf;

  out->sockfd = htonl(*sockfd);
  out->cmd = htonl(*cmd);
  out->val = htonl(*val);

  if (size == 0)
    {
      size = sizeof(struct apicmd_fcntl_s);
    }

  *altcid = APICMDID_SOCK_FCNTL;

  return size;
}

int32_t altcom_getsockname_pkt_compose(FAR void **arg, size_t arglen,
                                       uint8_t altver, FAR uint8_t *pktbuf,
                                       const size_t pktsz,
                                       FAR uint16_t *altcid)
{
  int32_t size = 0;

  size = sockaddrlen_pkt_compose(arg, arglen, altver, pktbuf, pktsz);

  *altcid = APICMDID_SOCK_GETSOCKNAME;

  return size;
}

int32_t altcom_getsockopt_pkt_compose(FAR void **arg, size_t arglen,
                                      uint8_t altver, FAR uint8_t *pktbuf,
                                      const size_t pktsz,
                                      FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *sockfd = (FAR int32_t *)arg[0];
  FAR int16_t *level = (FAR int16_t *)arg[1];
  FAR int16_t *option = (FAR int16_t *)arg[2];
  FAR uint16_t *max_valuelen = (FAR uint16_t *)arg[3];

  FAR struct apicmd_getsockopt_s *out =
    (FAR struct apicmd_getsockopt_s *)pktbuf;

  out->sockfd = htonl(*sockfd);
  if (*level == SOL_SOCKET)
    {
      out->level = htonl(ALTCOM_SOL_SOCKET);
      out->optlen = htonl(*max_valuelen);

      if (*option == SO_ACCEPTCONN)
        {
          out->optname = htonl(ALTCOM_SO_ACCEPTCONN);
        }
      else if (*option == SO_ERROR)
        {
          out->optname = htonl(ALTCOM_SO_ERROR);
        }
      else if (*option == SO_BROADCAST)
        {
          out->optname = htonl(ALTCOM_SO_BROADCAST);
        }
      else if (*option == SO_KEEPALIVE)
        {
          out->optname = htonl(ALTCOM_SO_KEEPALIVE);
        }
      else if (*option == SO_REUSEADDR)
        {
          out->optname = htonl(ALTCOM_SO_REUSEADDR);
        }
      else if (*option == SO_TYPE)
        {
          out->optname = htonl(ALTCOM_SO_TYPE);
        }
      else if (*option == SO_RCVBUF)
        {
          out->optname = htonl(ALTCOM_SO_RCVBUF);
        }
#ifdef CONFIG_NET_SOLINGER
      else if (*option == SO_LINGER)
        {
          out->optname = htonl(ALTCOM_SO_LINGER);
        }
#endif
      else
        {
          size = -ENOPROTOOPT;
        }
    }
  else if (*level == IPPROTO_IP)
    {
      out->level = htonl(ALTCOM_IPPROTO_IP);
      out->optlen = htonl(*max_valuelen);

      if (*option == IP_TOS)
        {
          out->optname = htonl(ALTCOM_IP_TOS);
        }
      else if (*option == IP_TTL)
        {
          out->optname = htonl(ALTCOM_IP_TTL);
        }
      else if (*option == IP_MULTICAST_TTL)
        {
          out->optname = htonl(ALTCOM_IP_MULTICAST_TTL);
        }
      else if (*option == IP_MULTICAST_LOOP)
        {
          out->optname = htonl(ALTCOM_IP_MULTICAST_LOOP);
        }
      else if (*option == IP_MULTICAST_IF)
        {
          out->optname = htonl(ALTCOM_IP_MULTICAST_IF);
        }
      else
        {
          size = -ENOPROTOOPT;
        }
    }
  else if (*level == IPPROTO_TCP)
    {
      out->level = htonl(ALTCOM_IPPROTO_TCP);
      out->optlen = htonl(*max_valuelen);
      if (*option == TCP_NODELAY)
        {
          out->optname = htonl(ALTCOM_TCP_NODELAY);
        }
      else if (*option == TCP_KEEPIDLE)
        {
          out->optname = htonl(ALTCOM_TCP_KEEPIDLE);
        }
      else if (*option == TCP_KEEPINTVL)
        {
          out->optname = htonl(ALTCOM_TCP_KEEPINTVL);
        }
      else
        {
          size = -ENOPROTOOPT;
        }
    }
  else if (*level == IPPROTO_IPV6)
    {
      out->level = htonl(ALTCOM_IPPROTO_IPV6);
      out->optlen = htonl(*max_valuelen);
      if (*option == IPV6_V6ONLY)
        {
          out->optname = htonl(ALTCOM_IPV6_V6ONLY);
        }
      else
        {
          size = -ENOPROTOOPT;
        }
    }
  else
    {
      size = -ENOPROTOOPT;
    }

  if (size == 0)
    {
      size = sizeof(struct apicmd_getsockopt_s);
    }

  *altcid = APICMDID_SOCK_GETSOCKOPT;

  return size;
}

int32_t altcom_listen_pkt_compose(FAR void **arg, size_t arglen,
                                  uint8_t altver, FAR uint8_t *pktbuf,
                                  const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *sockfd = (FAR int32_t *)arg[0];
  FAR uint16_t *backlog = (FAR uint16_t *)arg[1];

  FAR struct apicmd_listen_s *out =
    (FAR struct apicmd_listen_s *)pktbuf;

  out->sockfd = htonl(*sockfd);
  out->backlog = htonl(*backlog);

  size = sizeof(struct apicmd_listen_s);

  *altcid = APICMDID_SOCK_LISTEN;

  return size;
}

int32_t altcom_recvfrom_pkt_compose(FAR void **arg, size_t arglen,
                                    uint8_t altver, FAR uint8_t *pktbuf,
                                    const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *sockfd = (FAR int32_t *)arg[0];
  FAR int32_t *flags = (FAR int32_t *)arg[1];
  FAR uint16_t *max_buflen = (FAR uint16_t *)arg[2];
  FAR uint16_t *max_addrlen = (FAR uint16_t *)arg[3];

  FAR struct apicmd_recvfrom_s *out =
    (FAR struct apicmd_recvfrom_s *)pktbuf;
  int32_t flg;

  out->sockfd = htonl(*sockfd);
  if (*max_buflen > APICMD_DATA_LENGTH)
    {
      /* Truncate the length to the maximum transfer size */

      *max_buflen = APICMD_DATA_LENGTH;
    }

  out->recvlen = htonl(*max_buflen);
  size = flags2altflags(*flags, &flg);
  out->flags = htonl(flg);
  if (*max_addrlen == sizeof(struct sockaddr_in))
    {
      out->fromlen = htonl(sizeof(struct altcom_sockaddr_in));
    }
  else if (*max_addrlen == sizeof(struct sockaddr_in6))
    {
      out->fromlen = htonl(sizeof(struct altcom_sockaddr_in6));
    }
  else if (*max_addrlen == 0)
    {
      out->fromlen = htonl(0);
    }
  else
    {
      size = -EINVAL;
    }

  if (size == 0)
    {
      size = sizeof(struct apicmd_recvfrom_s);
    }

  *altcid = APICMDID_SOCK_RECVFROM;

  return size;
}

int32_t altcom_sendto_pkt_compose(FAR void **arg, size_t arglen,
                                  uint8_t altver, FAR uint8_t *pktbuf,
                                  const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *sockfd = (FAR int32_t *)arg[0];
  FAR int32_t *flags = (FAR int32_t *)arg[1];
  FAR uint16_t *addrlen = (FAR uint16_t *)arg[2];
  FAR uint16_t *buflen = (FAR uint16_t *)arg[3];
  FAR struct sockaddr_storage *sa = (FAR struct sockaddr_storage *)arg[4];
  FAR uint8_t *buf = (FAR uint8_t *)arg[5];

  FAR struct apicmd_sendto_s *out =
    (FAR struct apicmd_sendto_s *)pktbuf;
  int32_t flg;
  struct altcom_sockaddr_storage altsa;

  if (*buflen > APICMD_DATA_LENGTH)
    {
      /* Truncate the length to the maximum transfer size */

      *buflen = APICMD_DATA_LENGTH;
    }
  else if (*buflen < 0)
    {
      size = -EINVAL;
      goto err_out;
    }

  if (*buflen > 0 && !buf)
    {
      size = -EINVAL;
      goto err_out;
    }

  if (sa && !(*addrlen))
    {
      size = -EINVAL;
      goto err_out;
    }

  out->sockfd = htonl(*sockfd);
  size = flags2altflags(*flags, &flg);
  if (size != 0)
    {
      goto err_out;
    }

  out->flags = htonl(flg);
  out->datalen = htonl(*buflen);
  if (*addrlen == sizeof(struct sockaddr_in))
    {
      out->tolen = htonl(sizeof(struct altcom_sockaddr_in));
    }
  else if (*addrlen == sizeof(struct sockaddr_in6))
    {
      out->tolen = htonl(sizeof(struct altcom_sockaddr_in6));
    }
  else if (*addrlen == 0)
    {
      out->tolen = htonl(0);
    }
  else
    {
      size = -EINVAL;
    }

  if (size == 0)
    {
      memset(&altsa, 0, sizeof(struct altcom_sockaddr_storage));
      sockaddr2altstorage((struct sockaddr *)sa, &altsa);
      memcpy(&out->to, &altsa, *addrlen);
      memcpy(out->senddata, buf, *buflen);
      size = sizeof(struct apicmd_sendto_s) - sizeof(out->senddata) +
        *buflen;
    }

err_out:
  *altcid = APICMDID_SOCK_SENDTO;

  return size;
}

int32_t altcom_setsockopt_pkt_compose(FAR void **arg, size_t arglen,
                                      uint8_t altver, FAR uint8_t *pktbuf,
                                      const size_t pktsz,
                                      FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *sockfd = (FAR int32_t *)arg[0];
  FAR int16_t *level = (FAR int16_t *)arg[1];
  FAR int16_t *option = (FAR int16_t *)arg[2];
  FAR uint16_t *valuelen = (FAR uint16_t *)arg[3];
  FAR uint8_t *value = (FAR uint8_t *)arg[4];

  FAR struct apicmd_setsockopt_s *out =
    (FAR struct apicmd_setsockopt_s *)pktbuf;

  int setmode = 0;

  out->sockfd = htonl(*sockfd);
  if (*level == SOL_SOCKET)
    {
      out->level = htonl(ALTCOM_SOL_SOCKET);
      out->optlen = htonl(*valuelen);

      if (*option == SO_BROADCAST)
        {
          out->optname = htonl(ALTCOM_SO_BROADCAST);
        }
      else if (*option == SO_REUSEADDR)
        {
          out->optname = htonl(ALTCOM_SO_REUSEADDR);
        }
      else if (*option == SO_KEEPALIVE)
        {
          out->optname = htonl(ALTCOM_SO_KEEPALIVE);
        }
      else if (*option == SO_RCVBUF)
        {
          out->optname = htonl(ALTCOM_SO_RCVBUF);
        }
#ifdef CONFIG_NET_SOLINGER
      else if (*option == SO_LINGER)
        {
          out->optname = htonl(ALTCOM_SO_LINGER);
        }
#endif
      else
        {
          size = -ENOPROTOOPT;
        }
    }
  else if (*level == IPPROTO_IP)
    {
      out->level = htonl(ALTCOM_IPPROTO_IP);
      out->optlen = htonl(*valuelen);

      if (*option == IP_TOS)
        {
          out->optname = htonl(ALTCOM_IP_TOS);
        }
      else if (*option == IP_TTL)
        {
          out->optname = htonl(ALTCOM_IP_TTL);
        }
      else if (*option == IP_MULTICAST_TTL)
        {
          out->optname = htonl(ALTCOM_IP_MULTICAST_TTL);
        }
      else if (*option == IP_MULTICAST_LOOP)
        {
          out->optname = htonl(ALTCOM_IP_MULTICAST_LOOP);
        }
      else if (*option == IP_MULTICAST_IF)
        {
          out->optname = htonl(ALTCOM_IP_MULTICAST_IF);
        }
      else if (*option == IP_ADD_MEMBERSHIP)
        {
          out->optname = htonl(ALTCOM_IP_ADD_MEMBERSHIP);
        }
      else if (*option == IP_DROP_MEMBERSHIP)
        {
          out->optname = htonl(ALTCOM_IP_DROP_MEMBERSHIP);
        }
      else
        {
          size = -ENOPROTOOPT;
        }
    }
  else if (*level == IPPROTO_TCP)
    {
      out->level = htonl(ALTCOM_IPPROTO_TCP);
      out->optlen = htonl(*valuelen);
      if (*option == TCP_NODELAY)
        {
          out->optname = htonl(ALTCOM_TCP_NODELAY);
        }
      else if (*option == TCP_KEEPIDLE)
        {
          out->optname = htonl(ALTCOM_TCP_KEEPIDLE);
        }
      else if (*option == TCP_KEEPINTVL)
        {
          out->optname = htonl(ALTCOM_TCP_KEEPINTVL);
        }
      else if (*option == TCP_KEEPCNT)
        {
          out->optname = htonl(ALTCOM_TCP_KEEPCNT);
        }
      else
        {
          size = -ENOPROTOOPT;
        }
    }
  else if (*level == IPPROTO_IPV6)
    {
      out->level = htonl(ALTCOM_IPPROTO_IPV6);
      out->optlen = htonl(*valuelen);
      if (*option == IPV6_V6ONLY)
        {
          out->optname = htonl(ALTCOM_IPV6_V6ONLY);
        }
      else
        {
          size = -ENOPROTOOPT;
        }
    }
  else
    {
      size = -ENOPROTOOPT;
    }

  if (size < 0)
    {
      goto exit;
    }

  setmode = get_so_setmode(*level, *option);

  switch (setmode)
    {
      case ALTCOM_SO_SETMODE_8BIT:
        if (*valuelen < sizeof(int8_t))
          {
            m_err("Unexpected valuelen: actual=%lu expect=%lu\n",
              *valuelen, sizeof(int8_t));
            size = -EINVAL;
            break;
          }

        *out->optval = value[0];
        break;
      case ALTCOM_SO_SETMODE_32BIT:
        if (*valuelen < sizeof(int32_t))
          {
            m_err("Unexpected valuelen: actual=%lu expect=%lu\n",
              *valuelen, sizeof(int32_t));
            size = -EINVAL;
            break;
          }

        *((FAR int32_t *)out->optval) =
          htonl(*((FAR int32_t *)value));
        break;
      case ALTCOM_SO_SETMODE_LINGER:
        if (*valuelen < sizeof(struct linger))
          {
            m_err("Unexpected valuelen: actual=%lu expect=%lu\n",
              *valuelen, sizeof(struct linger));
            size = -EINVAL;
            break;
          }

        ((FAR struct altcom_linger *)out->optval)->l_onoff =
          htonl(((FAR struct linger *)value)->l_onoff);
        ((FAR struct altcom_linger *)out->optval)->l_linger =
          htonl(((FAR struct linger *)value)->l_linger);
        break;
      case ALTCOM_SO_SETMODE_INADDR:
        if (*valuelen < sizeof(struct in_addr))
          {
            m_err("Unexpected valuelen: actual=%lu expect=%lu\n",
              *valuelen, sizeof(struct in_addr));
            size = -EINVAL;
            break;
          }

        ((FAR struct altcom_in_addr *)out->optval)->s_addr =
          htonl(((FAR struct in_addr *)value)->s_addr);
        break;
      case ALTCOM_SO_SETMODE_IPMREQ:
        if (*valuelen < sizeof(struct ip_mreq))
          {
            m_err("Unexpected valuelen: actual=%lu expect=%lu\n",
              *valuelen, sizeof(struct ip_mreq));
            size = -EINVAL;
            break;
          }

        ((FAR struct altcom_ip_mreq *)out->optval)->imr_multiaddr.s_addr =
          htonl(((FAR struct ip_mreq *)value)->imr_multiaddr.s_addr);
        ((FAR struct altcom_ip_mreq *)out->optval)->imr_interface.s_addr =
          htonl(((FAR struct ip_mreq *)value)->imr_interface.s_addr);
        break;
      default:
        size = -EINVAL;
        break;
    }

exit:
  if (size == 0)
    {
      size = sizeof(struct apicmd_setsockopt_s);
    }

  *altcid = APICMDID_SOCK_SETSOCKOPT;

  return size;
}

int32_t altcom_select_pkt_compose(FAR void **arg, size_t arglen,
                                  uint8_t altver, FAR uint8_t *pktbuf,
                                  const size_t pktsz, FAR uint16_t *altcid)
{
  int32_t size = 0;
  FAR int32_t *request = (FAR int32_t *)arg[0];
  FAR int32_t *id = (FAR int32_t *)arg[1];
  FAR int32_t *maxfds = (FAR int32_t *)arg[2];
  FAR uint16_t *used_setbit = (FAR uint16_t *)arg[3];
  FAR altcom_fd_set *readset = (FAR altcom_fd_set *)arg[4];
  FAR altcom_fd_set *writeset = (FAR altcom_fd_set *)arg[5];
  FAR altcom_fd_set *exceptset = (FAR altcom_fd_set *)arg[6];

  FAR struct apicmd_select_s *out =
    (FAR struct apicmd_select_s *)pktbuf;

  out->request = htonl(*request);
  out->id = htonl(*id);
  out->maxfds = htonl(*maxfds);
  out->used_setbit = htons(*used_setbit);
  memcpy(&out->readset, readset, sizeof(altcom_fd_set));
  memcpy(&out->writeset, writeset, sizeof(altcom_fd_set));
  memcpy(&out->exceptset, exceptset, sizeof(altcom_fd_set));

  size = sizeof(struct apicmd_select_s);

  *altcid = APICMDID_SOCK_SELECT;

  return size;
}

int32_t altcom_sockcomm_pkt_parse(FAR struct alt1250_dev_s *dev,
                                  FAR uint8_t *pktbuf, size_t pktsz,
                                  uint8_t altver, FAR void **arg,
                                  size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR int32_t *errcode = (FAR int32_t *)arg[1];

  FAR struct altmdmpktr_sockcomm_s *in =
    (FAR struct altmdmpktr_sockcomm_s *)pktbuf;

  *ret = ntohl(in->ret_code);
  *errcode = altcom_errno2nuttx(ntohl(in->err_code));

  return 0;
}

int32_t altcom_scokaddr_pkt_parse(FAR struct alt1250_dev_s *dev,
                                  FAR uint8_t *pktbuf, size_t pktsz,
                                  uint8_t altver, FAR void **arg,
                                  size_t arglen, FAR uint64_t *bitmap)
{
  int32_t rc = OK;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR int32_t *errcode = (FAR int32_t *)arg[1];
  FAR uint32_t *addrlen = (FAR uint32_t *)arg[2];
  FAR struct sockaddr_storage *sa = (FAR struct sockaddr_storage *)arg[3];

  FAR struct altmdmpktr_sockaddr_s *in =
    (FAR struct altmdmpktr_sockaddr_s *)pktbuf;
  struct altcom_sockaddr_storage altsa;

  *ret = ntohl(in->ret_code);
  *errcode = altcom_errno2nuttx(ntohl(in->err_code));

  if (*ret >= 0)
    {
      *addrlen = ntohl(in->addrlen);
      if (*addrlen == sizeof(struct altcom_sockaddr_in))
        {
          *addrlen = sizeof(struct sockaddr_in);
        }
      else if (*addrlen == sizeof(struct altcom_sockaddr_in6))
        {
          *addrlen = sizeof(struct sockaddr_in6);
        }
      else
        {
          rc = -EILSEQ;
        }

      memcpy(&altsa, &in->address, sizeof(struct altcom_sockaddr_storage));
      altstorage2sockaddr(&altsa, (FAR struct sockaddr *)sa);
    }

  return rc;
}

int32_t altcom_getsockopt_pkt_parse(FAR struct alt1250_dev_s *dev,
                                    FAR uint8_t *pktbuf, size_t pktsz,
                                    uint8_t altver, FAR void **arg,
                                    size_t arglen, FAR uint64_t *bitmap)
{
  int32_t rc = OK;
  FAR int32_t  *ret     = (FAR int32_t *)arg[0];
  FAR int32_t  *errcode = (FAR int32_t *)arg[1];
  FAR uint32_t *optlen  = (FAR uint32_t *)arg[2];
  FAR int8_t   *optval  = (FAR int8_t *)arg[3];
  FAR uint16_t *level   = (FAR uint16_t *)arg[4];
  FAR uint16_t *option  = (FAR uint16_t *)arg[5];

  FAR struct apicmd_getsockoptres_s *in =
    (FAR struct apicmd_getsockoptres_s *)pktbuf;

  int setmode = 0;

  *ret = ntohl(in->ret_code);
  *errcode = altcom_errno2nuttx(ntohl(in->err_code));

  if (*ret >= 0)
    {
      *optlen = ntohl(in->optlen);
      if (*optlen > APICMD_OPTVAL_LENGTH)
        {
          rc = -EILSEQ;
        }
      else
        {
          setmode = get_so_setmode(*level, *option);

          switch (setmode)
            {
              case ALTCOM_SO_SETMODE_8BIT:
                if (*optlen < sizeof(int8_t))
                  {
                    m_err("Unexpected optlen: actual=%lu expect=%lu\n",
                      *optlen, sizeof(int8_t));
                    rc = -EILSEQ;
                    break;
                  }

                *optval = in->optval[0];
                break;
              case ALTCOM_SO_SETMODE_32BIT:
                if (*optlen < sizeof(int32_t))
                  {
                    m_err("Unexpected optlen: actual=%lu expect=%lu\n",
                      *optlen, sizeof(int32_t));
                    rc = -EILSEQ;
                    break;
                  }

                *((FAR int32_t *)optval) =
                  ntohl(*((FAR int32_t *)in->optval));
                break;
              case ALTCOM_SO_SETMODE_LINGER:
                if (*optlen < sizeof(struct linger))
                  {
                    m_err("Unexpected optlen: actual=%lu expect=%lu\n",
                      *optlen, sizeof(struct linger));
                    rc = -EILSEQ;
                    break;
                  }

                FAR struct altcom_linger *plinger;

                plinger = (FAR struct altcom_linger *)&in->optval[0];
                ((FAR struct linger *)optval)->l_onoff =
                  ntohl(plinger->l_onoff);
                ((FAR struct linger *)optval)->l_linger =
                  ntohl(plinger->l_linger);
                break;
              case ALTCOM_SO_SETMODE_INADDR:
                if (*optlen < sizeof(struct in_addr))
                  {
                    m_err("Unexpected optlen: actual=%lu expect=%lu\n",
                      *optlen, sizeof(struct in_addr));
                    rc = -EILSEQ;
                    break;
                  }

                FAR struct altcom_in_addr *pinaddr;

                pinaddr = (FAR struct altcom_in_addr *)&in->optval[0];
                ((FAR struct in_addr *)optval)->s_addr =
                  ntohl(pinaddr->s_addr);
                break;
              default:
                break;
            }
        }
    }

  return rc;
}

int32_t altcom_recvfrom_pkt_parse(FAR struct alt1250_dev_s *dev,
                                  FAR uint8_t *pktbuf, size_t pktsz,
                                  uint8_t altver, FAR void **arg,
                                  size_t arglen, FAR uint64_t *bitmap)
{
  int32_t rc = OK;
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR int32_t *errcode = (FAR int32_t *)arg[1];
  FAR uint32_t *fromlen = (FAR uint32_t *)arg[2];
  FAR struct sockaddr_storage *sa = (FAR struct sockaddr_storage *)arg[3];
  FAR int8_t *buf = (FAR int8_t *)arg[4];

  FAR struct apicmd_recvfromres_s *in =
    (FAR struct apicmd_recvfromres_s *)pktbuf;
  struct altcom_sockaddr_storage altsa;

  *ret = ntohl(in->ret_code);
  *errcode = altcom_errno2nuttx(ntohl(in->err_code));

  if (*ret >= 0)
    {
      *fromlen = ntohl(in->fromlen);
      if (*fromlen == sizeof(struct altcom_sockaddr_in))
        {
          *fromlen = sizeof(struct sockaddr_in);
        }
      else if (*fromlen == sizeof(struct altcom_sockaddr_in6))
        {
          *fromlen = sizeof(struct sockaddr_in6);
        }
      else if (*fromlen != 0)
        {
          rc = -EILSEQ;
        }

      if ((rc == OK) && (*fromlen != 0))
        {
          memcpy(&altsa, &in->from, *fromlen);
          altstorage2sockaddr(&altsa, (FAR struct sockaddr *)sa);
        }

      if (*ret > APICMD_DATA_LENGTH)
        {
          rc = -EILSEQ;
        }
      else
        {
          memcpy(buf, in->recvdata, *ret);
        }
    }

  return rc;
}

int32_t altcom_select_pkt_parse(FAR struct alt1250_dev_s *dev,
                                FAR uint8_t *pktbuf, size_t pktsz,
                                uint8_t altver, FAR void **arg,
                                size_t arglen, FAR uint64_t *bitmap)
{
  FAR int32_t *ret = (FAR int32_t *)arg[0];
  FAR int32_t *errcode = (FAR int32_t *)arg[1];
  FAR int32_t *id = (FAR int32_t *)arg[2];
  FAR altcom_fd_set *readset = (FAR altcom_fd_set *)arg[3];
  FAR altcom_fd_set *writeset = (FAR altcom_fd_set *)arg[4];
  FAR altcom_fd_set *exceptset = (FAR altcom_fd_set *)arg[5];

  FAR struct apicmd_selectres_s *in =
    (FAR struct apicmd_selectres_s *)pktbuf;
  uint16_t used_setbit;

  *ret = ntohl(in->ret_code);
  *errcode = altcom_errno2nuttx(ntohl(in->err_code));

  if (*ret >= 0)
    {
      *id = ntohl(in->id);
      used_setbit = ntohs(in->used_setbit);
      memset(readset, 0, sizeof(altcom_fd_set));
      memset(writeset, 0, sizeof(altcom_fd_set));
      memset(exceptset, 0, sizeof(altcom_fd_set));
      if (used_setbit & READSET_BIT)
        {
          memcpy(readset, &in->readset, sizeof(altcom_fd_set));
        }

      if (used_setbit & WRITESET_BIT)
        {
          memcpy(writeset, &in->writeset, sizeof(altcom_fd_set));
        }

      if (used_setbit & EXCEPTSET_BIT)
        {
          memcpy(exceptset, &in->exceptset, sizeof(altcom_fd_set));
        }
    }

  return 0;
}
