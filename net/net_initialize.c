/****************************************************************************
 * net/net_initialize.c
 *
 *   Copyright (C) 2007-2009, 2011-2015, 2017-2018 Gregory Nutt. All rights
 *     reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
#include "arp/arp.h"
#include "sixlowpan/sixlowpan.h"
#include "neighbor/neighbor.h"
#include "icmp/icmp.h"
#include "icmpv6/icmpv6.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "pkt/pkt.h"
#include "bluetooth/bluetooth.h"
#include "ieee802154/ieee802154.h"
#include "local/local.h"
#include "netlink/netlink.h"
#include "igmp/igmp.h"
#include "route/route.h"
#include "usrsock/usrsock.h"
#include "utils/utils.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_setup
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

void net_setup(void)
{
  /* Initialize the locking facility */

  net_lockinitialize();

  /* Clear the ARP table */

  arp_reset();

#ifdef CONFIG_NET_IPv6
  /* Initialize the Neighbor Table data structures */

  neighbor_initialize();

#ifdef CONFIG_NET_6LOWPAN
  /* Initialize 6LoWPAN data structures */

  sixlowpan_initialize();
#endif
#endif /* CONFIG_NET_IPv6 */

  /* Initialize the device interface layer */

  devif_initialize();

#ifdef HAVE_FWDALLOC
  /* Initialize IP forwarding support */

  ipfwd_initialize();
#endif

#ifdef CONFIG_NET_PKT
  /* Initialize packet socket support */

  pkt_initialize();
#endif

#ifdef CONFIG_NET_ICMP_SOCKET
  /* Initialize IPPPROTO_ICMP socket support */

  icmp_sock_initialize();
#endif

#ifdef CONFIG_NET_ICMPv6_SOCKET
  /* Initialize IPPPROTO_ICMP6 socket support */

  icmpv6_sock_initialize();
#endif

#ifdef CONFIG_NET_BLUETOOTH
  /* Initialize Bluetooth  socket support */

  bluetooth_initialize();
#endif

#ifdef CONFIG_NET_IEEE802154
  /* Initialize IEEE 802.15.4  socket support */

  ieee802154_initialize();
#endif

#ifdef CONFIG_NET_LOCAL
  /* Initialize the local, "Unix domain" socket support */

  local_initialize();
#endif

#ifdef CONFIG_NET_NETLINK
  /* Initialize the Netlink IPC support */

  netlink_initialize();
#endif

#ifdef NET_TCP_HAVE_STACK
  /* Initialize the listening port structures */

  tcp_listen_initialize();

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

#ifdef CONFIG_NET_IGMP
  /* Initialize IGMP support */

  igmp_initialize();
#endif

#ifdef CONFIG_NET_ROUTE
  /* Initialize the routing table */

  net_init_route();
#endif

#ifdef CONFIG_NET_USRSOCK
  /* Initialize the user-space socket API */

  usrsock_initialize();
#endif
}

/****************************************************************************
 * Name: net_initialize
 *
 * Description:
 *   This function is called from the OS initialization logic at power-up
 *   reset AFTER initialization of hardware facilities such as timers and
 *   interrupts.   This logic completes the initialization started by
 *   net_setup().
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
  /* Initialize the periodic ARP timer */

  arp_timer_initialize();
}

#endif /* CONFIG_NET */
