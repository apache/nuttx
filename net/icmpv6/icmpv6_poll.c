/****************************************************************************
 * net/icmpv6/icmpv6_poll.c
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
 *   dev is not NULL.
 *   conn may be NULL.
 *   The connection (conn), if not NULL, is bound to
 *   the polling device (dev).
 *
 ****************************************************************************/

void icmpv6_poll(FAR struct net_driver_s *dev,
                 FAR struct icmpv6_conn_s *conn)
{
  DEBUGASSERT(dev != NULL && (conn == NULL || dev == conn->dev));

  /* Setup for the application callback */

  dev->d_appdata = &dev->d_buf[NET_LL_HDRLEN(dev) + IPICMPv6_HDRLEN];
  dev->d_len     = 0;
  dev->d_sndlen  = 0;

  /* Perform the application callback */

  devif_conn_event(dev, conn, ICMPv6_POLL,
                   conn ? conn->sconn.list : dev->d_conncb);
}

#endif /* CONFIG_NET_ICMPv6_SOCKET || CONFIG_NET_ICMPv6_NEIGHBOR */
