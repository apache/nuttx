/****************************************************************************
 * net/can/can_poll.c
 * Poll for the availability of packet TX data
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_CAN)

#include <assert.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/netdev.h>

#include "devif/devif.h"
#include "can/can.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_poll
 *
 * Description:
 *   Poll a packet "connection" structure for availability of TX data
 *
 * Input Parameters:
 *   dev - The device driver structure to use in the send operation
 *   conn - The packet "connection" to poll for TX data
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The network is locked.
 *   dev is not NULL.
 *   conn is not NULL.
 *   The connection (conn) is bound to the polling device (dev).
 *
 ****************************************************************************/

void can_poll(FAR struct net_driver_s *dev, FAR struct can_conn_s *conn)
{
  DEBUGASSERT(dev != NULL && conn != NULL && dev == conn->dev);

  /* Setup for the application callback */

  dev->d_len    = 0;
  dev->d_sndlen = 0;

  /* Perform the application callback */

  can_callback(dev, conn, CAN_POLL);

  /* Check if the application has data to send */

  if (dev->d_sndlen > 0)
    {
      return;
    }

  /* Make sure that d_len is zero meaning that there is nothing to be sent */

  dev->d_len = 0;
}

#endif /* CONFIG_NET && CONFIG_NET_CAN */
