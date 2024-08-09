/****************************************************************************
 * net/net_initialize.c
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
#ifdef CONFIG_NET

#include <debug.h>

#include <nuttx/net/net.h>

#include "socket/socket.h"
#include "devif/devif.h"
#include "netdev/netdev.h"
#include "ipforward/ipforward.h"
#include "sixlowpan/sixlowpan.h"
#include "icmp/icmp.h"
#include "icmpv6/icmpv6.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "pkt/pkt.h"
#include "bluetooth/bluetooth.h"
#include "ieee802154/ieee802154.h"
#include "can/can.h"
#include "netlink/netlink.h"
#include "route/route.h"
#include "usrsock/usrsock.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_initialize
 *
 * Description:
 *   This is called from the OS initialization logic at power-up reset in
 *   order to configure networking data structures.  This is called prior
 *   to platform-specific driver initialization so that the networking
 *   subsystem is prepared to deal with network driver initialization
 *   actions.
 *
 *   Actions performed in this initialization phase assume that base OS
 *   facilities such as semaphores are available but this logic cannot
 *   depend upon OS resources such as interrupts or timers which are not
 *   yet available.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void net_initialize(void)
{
  /* Initialize the device interface layer */

  devif_initialize();

#ifdef CONFIG_NET_BLUETOOTH
  /* Initialize Bluetooth  socket support */

  bluetooth_initialize();
#endif

#ifdef CONFIG_NET_CAN
  /* Initialize SocketCAN support */

  can_initialize();
#endif

#ifdef CONFIG_NET_IEEE802154
  /* Initialize IEEE 802.15.4  socket support */

  ieee802154_initialize();
#endif

#ifdef CONFIG_NET_NETLINK
  /* Initialize the Netlink IPC support */

  netlink_initialize();
#endif

#ifdef CONFIG_NET_PKT
  /* Initialize packet socket support */

  pkt_initialize();
#endif

#ifdef CONFIG_NET_ROUTE
  /* Initialize the routing table */

  net_init_route();
#endif

#ifdef CONFIG_NET_IPv6
#ifdef CONFIG_NET_6LOWPAN
  /* Initialize 6LoWPAN data structures */

  sixlowpan_initialize();
#endif
#endif /* CONFIG_NET_IPv6 */

#ifdef HAVE_FWDALLOC
  /* Initialize IP forwarding support */

  ipfwd_initialize();
#endif

#ifdef CONFIG_NET_ICMP_SOCKET
  /* Initialize IPPPROTO_ICMP socket support */

  icmp_sock_initialize();
#endif

#ifdef CONFIG_NET_ICMPv6_SOCKET
  /* Initialize IPPPROTO_ICMP6 socket support */

  icmpv6_sock_initialize();
#endif

#ifdef NET_TCP_HAVE_STACK
  /* Initialize the TCP/IP connection structures */

  tcp_initialize();

  /* Initialize the TCP/IP write buffering */

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  tcp_wrbuffer_initialize();
#endif
#endif /* CONFIG_NET_TCP */

#ifdef NET_UDP_HAVE_STACK
  /* Initialize the UDP connection structures */

  udp_initialize();

#ifdef CONFIG_NET_UDP_WRITE_BUFFERS
  udp_wrbuffer_initialize();
#endif
#endif

#ifdef CONFIG_NET_USRSOCK
  /* Initialize the user-space socket API */

  usrsock_initialize();
#endif

#ifdef CONFIG_NETUTILS_IPTLITE
  /* Initialize the iptlite packet filter modules */

  nflite_initialize();
#endif
}

#endif /* CONFIG_NET */
