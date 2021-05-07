/****************************************************************************
 * net/ieee802154/ieee802154_poll.c
 * Poll for the availability of ougoing IEEE 802.15.4 frames
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

#include <assert.h>
#include <debug.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/ieee802154.h>

#include "devif/devif.h"
#include "ieee802154/ieee802154.h"

#ifdef CONFIG_NET_IEEE802154

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ieee802154_poll
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
 *
 ****************************************************************************/

void ieee802154_poll(FAR struct net_driver_s *dev,
                     FAR struct ieee802154_conn_s *conn)
{
  FAR struct radio_driver_s *radio;

  DEBUGASSERT(dev != NULL && conn != NULL);
  radio = (FAR struct radio_driver_s *)dev;

  /* Verify that the packet connection is valid */

  if (conn != NULL)
    {
      /* Setup for the application callback (NOTE:  These should not be
       * used by PF_IEEE802154 sockets).
       */

      radio->r_dev.d_appdata = radio->r_dev.d_buf;
      radio->r_dev.d_len     = 0;
      radio->r_dev.d_sndlen  = 0;

      /* Perform the application callback.
       *
       * REVISIT:
       *  Need to pass the meta data and the IOB through the callback.
       */

#warning Missing logic

      /* Perform the application callback */

      ieee802154_callback(radio, conn, IEEE802154_POLL);
    }
}

#endif /* CONFIG_NET && CONFIG_NET_PKT */
