/****************************************************************************
 * net/pkt/pkt_callback.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_PKT)

#include <stdint.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>

#include "devif/devif.h"
#include "pkt/pkt.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pkt_callback
 *
 * Description:
 *   Inform the application holding the packet socket of a change in state.
 *
 * Returned Value:
 *   OK if packet has been processed, otherwise ERROR.
 *
 * Assumptions:
 *   This function is called with the network locked.
 *
 ****************************************************************************/

uint16_t pkt_callback(FAR struct net_driver_s *dev,
                      FAR struct pkt_conn_s *conn, uint16_t flags)
{
  ninfo("flags: %04x\n", flags);

  /* Some sanity checking */

  if (conn)
    {
      /* Perform the callback */

      flags = devif_conn_event(dev, flags, conn->sconn.list);
    }

  return flags;
}

#endif /* CONFIG_NET && CONFIG_NET_PKT */
