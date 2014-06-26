/****************************************************************************
 * net/net_sockets.c
 *
 *   Copyright (C) 2007-2009, 2011-2014 Gregory Nutt. All rights reserved.
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
#include <nuttx/net/arp.h>

#include "net.h"
#include "tcp/tcp.h"
#include "udp/udp.h"
#include "pkt/pkt.h"
#include "igmp/igmp.h"
#include "route/route.h"

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

/* This is called from the initialization logic to configure the socket layer */

void net_initialize(void)
{
  /* Initialize the locking facility */

  net_lockinitialize();

  /* Initialize I/O buffering */

#ifdef CONFIG_NET_IOB
  iob_initialize();
#endif

  /* Initialize the device interface layer */

  uip_initialize();

#ifdef CONFIG_NET_PKT
  /* Initialize packet socket support */

  pkt_initialize();
#endif

#ifdef CONFIG_NET_TCP
  /* Initialize the listening port structures */

  tcp_listeninit();

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

#if CONFIG_NSOCKET_DESCRIPTORS > 0
  /* Initialize the socket layer */

  netdev_seminit();
#endif

  /* Initialize the periodic ARP timer */

  arp_timer_init();
}

#endif /* CONFIG_NET */
