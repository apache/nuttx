/****************************************************************************
 * net/net_sockets.c
 *
 *   Copyright (C) 2007-2009, 2011-2015 Gregory Nutt. All rights reserved.
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

#include <nuttx/net/iob.h>
#include <nuttx/net/net.h>

#include "socket/socket.h"
#include "devif/devif.h"
#include "netdev/netdev.h"
#include "arp/arp.h"
#include "neighbor/neighbor.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "pkt/pkt.h"
#include "local/local.h"
#include "igmp/igmp.h"
#include "route/route.h"
#include "utils/utils.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

  neighbor_setup();
#endif

#ifdef CONFIG_NET_IOB
  /* Initialize I/O buffering */

  iob_initialize();
#endif

  /* Initialize the device interface layer */

  devif_initialize();

#ifdef CONFIG_NET_PKT
  /* Initialize packet socket support */

  pkt_initialize();
#endif

#ifdef CONFIG_NET_LOCAL
  /* Initialize the local, "Unix domain" socket support */

  local_initialize();
#endif

#ifdef CONFIG_NET_TCP
  /* Initialize the listening port structures */

  tcp_listen_initialize();

  /* Initialize the TCP/IP connection structures */

  tcp_initialize();

  /* Initialize the TCP/IP write buffering */

#ifdef CONFIG_NET_TCP_WRITE_BUFFERS
  tcp_wrbuffer_initialize();
#endif
#endif /* CONFIG_NET_TCP */

#ifdef CONFIG_NET_UDP
  /* Initialize the UDP connection structures */

  udp_initialize();
#endif

#ifdef CONFIG_NET_IGMP
  /* Initialize IGMP support */

  igmp_initialize();
#endif

#ifdef CONFIG_NET_ROUTE
  /* Initialize the routing table */

  net_initroute();
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
#ifdef CONFIG_NET_IPv6
  /* Configure Neighbor Table ageing */

  neighbor_initialize();
#endif

  /* Initialize the periodic ARP timer */

  arp_timer_initialize();
}

#endif /* CONFIG_NET */
