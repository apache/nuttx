/****************************************************************************
 * net/icmpv6/icmpv6_poll.c
 *
 *   Copyright (C) 2015, 2019, 2020 Gregory Nutt. All rights reserved.
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
#if defined(CONFIG_NET_ICMPv6_SOCKET) || defined(CONFIG_NET_ICMPv6_NEIGHBOR)

#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/icmpv6.h>

#include "devif/devif.h"
#include "icmpv6/icmpv6.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: icmpv6_poll
 *
 * Description:
 *   Poll a UDP "connection" structure for availability of TX data
 *
 * Input Parameters:
 *   dev  - The device driver structure to use in the send operation
 *   conn - A pointer to the ICMPv6 connection structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *
 ****************************************************************************/

void icmpv6_poll(FAR struct net_driver_s *dev,
                 FAR struct icmpv6_conn_s *conn)
{
  /* Setup for the application callback */

  dev->d_appdata = &dev->d_buf[NET_LL_HDRLEN(dev) + IPICMPv6_HDRLEN];
  dev->d_len     = 0;
  dev->d_sndlen  = 0;

  /* Perform the application callback */

  devif_conn_event(dev, conn, ICMPv6_POLL, conn ? conn->list : dev->d_conncb);
}

#endif /* CONFIG_NET_ICMPv6_SOCKET || CONFIG_NET_ICMPv6_NEIGHBOR */
