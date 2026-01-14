/****************************************************************************
 * net/socket/net_sockif.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netconfig.h>

#include "inet/inet.h"
#include "local/local.h"
#include "rpmsg/rpmsg.h"
#include "can/can.h"
#include "netlink/netlink.h"
#include "pkt/pkt.h"
#include "bluetooth/bluetooth.h"
#include "ieee802154/ieee802154.h"
#include "socket/socket.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ctrl_setup(FAR struct socket *psock);
static int ctrl_close(FAR struct socket *psock);
#ifdef CONFIG_NET_SOCKOPTS
static int ctrl_getsockopt(FAR struct socket *psock, int level, int option,
                           FAR void *value, FAR socklen_t *value_len);
static int ctrl_setsockopt(FAR struct socket *psock, int level, int option,
                           FAR const void *value, socklen_t value_len);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct sock_intf_s g_ctrl_sockif =
{
  ctrl_setup,       /* si_setup */
  NULL,             /* si_sockcaps */
  NULL,             /* si_addref */
  NULL,             /* si_bind */
  NULL,             /* si_getsockname */
  NULL,             /* si_getpeername */
  NULL,             /* si_listen */
  NULL,             /* si_connect */
  NULL,             /* si_accept */
  NULL,             /* si_poll */
  NULL,             /* si_sendmsg */
  NULL,             /* si_recvmsg */
  ctrl_close,       /* si_close */
  NULL,             /* si_ioctl */
  NULL,             /* si_socketpair */
  NULL              /* si_shutdown */
#ifdef CONFIG_NET_SOCKOPTS
  , ctrl_getsockopt /* si_getsockopt */
  , ctrl_setsockopt /* si_setsockopt */
#endif
};

static struct socket_conn_s g_ctrl_conn =
{
  .s_lock = NXRMUTEX_INITIALIZER,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ctrl_setup
 *
 * Description:
 *   Called for socket() to verify that the provided socket type and
 *   protocol are usable by this address family.  Perform any family-
 *   specific socket fields.
 *
 *   NOTE:  This is common logic for SOCK_CTRL
 *
 * Input Parameters:
 *   psock    A pointer to a user allocated socket structure to be
 *            initialized.
 *   protocol (see sys/socket.h)
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  Otherwise, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

static int ctrl_setup(FAR struct socket *psock)
{
  psock->s_conn = &g_ctrl_conn;
  return 0;
}

static int ctrl_close(FAR struct socket *psock)
{
  return 0;
}

/****************************************************************************
 * Name: ctrl_getsockopt / ctrl_setsockopt
 *
 * Description:
 *   Only support IPPROTO_IP and IPPROTO_IPV6.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_SOCKOPTS
static int ctrl_getsockopt(FAR struct socket *psock, int level, int option,
                           FAR void *value, FAR socklen_t *value_len)
{
  switch (level)
    {
#ifdef CONFIG_NET_IPv4
      case IPPROTO_IP:/* IPv4 protocol socket options (see include/netinet/in.h) */
        return ipv4_getsockopt(psock, option, value, value_len);
#endif

#ifdef CONFIG_NET_IPv6
      case IPPROTO_IPV6:/* IPv6 protocol socket options (see include/netinet/in.h) */
        return ipv6_getsockopt(psock, option, value, value_len);
#endif

      default:
        return -ENOPROTOOPT;
    }
}

static int ctrl_setsockopt(FAR struct socket *psock, int level, int option,
                           FAR const void *value, socklen_t value_len)
{
  switch (level)
    {
#ifdef CONFIG_NET_IPv4
      case IPPROTO_IP:/* IPv4 protocol socket options (see include/netinet/in.h) */
        return ipv4_setsockopt(psock, option, value, value_len);
#endif

#ifdef CONFIG_NET_IPv6
      case IPPROTO_IPV6:/* IPv6 protocol socket options (see include/netinet/in.h) */
        return ipv6_setsockopt(psock, option, value, value_len);
#endif
      default:
        return -ENOPROTOOPT;
    }
}
#endif /* CONFIG_NET_SOCKOPTS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_sockif
 *
 * Description:
 *   Return the socket interface associated with this address family.
 *
 * Input Parameters:
 *   family   - Socket address family
 *   type     - Socket type
 *   protocol - Socket protocol
 *
 * Returned Value:
 *   On success, a non-NULL instance of struct sock_intf_s is returned.  NULL
 *   is returned only if the address family is not supported.
 *
 ****************************************************************************/

FAR const struct sock_intf_s *
net_sockif(sa_family_t family, int type, int protocol)
{
  FAR const struct sock_intf_s *sockif = NULL;

  /* Get the socket interface.
   *
   * REVISIT:  Should also support PF_UNSPEC which would permit the socket
   * to be used for anything.
   */

  if ((type & SOCK_TYPE_MASK) == SOCK_CTRL)
    {
      return &g_ctrl_sockif;
    }

  switch (family)
    {
#if defined(HAVE_PFINET_SOCKETS) || defined(HAVE_PFINET6_SOCKETS)
#  ifdef HAVE_PFINET_SOCKETS
    case PF_INET:
#  endif
#  ifdef HAVE_PFINET6_SOCKETS
    case PF_INET6:
#  endif
      sockif = inet_sockif(family, type, protocol);
      break;
#endif

#ifdef CONFIG_NET_LOCAL
    case PF_LOCAL:
      sockif = &g_local_sockif;
      break;
#endif

#ifdef CONFIG_NET_CAN
    case PF_CAN:
      sockif = &g_can_sockif;
      break;
#endif

#ifdef CONFIG_NET_NETLINK
    case PF_NETLINK:
      sockif = &g_netlink_sockif;
      break;
#endif

#ifdef CONFIG_NET_PKT
    case PF_PACKET:
      sockif = &g_pkt_sockif;
      break;
#endif

#ifdef CONFIG_NET_BLUETOOTH
    case PF_BLUETOOTH:
      sockif = &g_bluetooth_sockif;
      break;
#endif

#ifdef CONFIG_NET_IEEE802154
    case PF_IEEE802154:
      sockif = &g_ieee802154_sockif;
      break;
#endif

#ifdef CONFIG_NET_RPMSG
    case PF_RPMSG:
      sockif = &g_rpmsg_sockif;
      break;
#endif

    default:
      nerr("ERROR: Address family unsupported: %d\n", family);
    }

  return sockif;
}
