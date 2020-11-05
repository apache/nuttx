/****************************************************************************
 * net/socket/net_sockif.c
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

  switch (family)
    {
#ifdef HAVE_INET_SOCKETS
#ifdef HAVE_PFINET_SOCKETS
    case PF_INET:
#endif
#ifdef HAVE_PFINET6_SOCKETS
    case PF_INET6:
#endif
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
