/****************************************************************************
 * net/can/can_input.c
 * Handling incoming packet input
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

#include <errno.h>
#include <debug.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/can.h>

#include "devif/devif.h"
#include "can/can.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

const uint8_t can_dlc_to_len[16] =
{
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    12,
    16,
    20,
    24,
    32,
    48,
    64,
};
const uint8_t len_to_can_dlc[65] =
{
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    9,
    9,
    9,
    10,
    10,
    10,
    10,
    11,
    11,
    11,
    11,
    12,
    12,
    12,
    12,
    13,
    13,
    13,
    13,
    13,
    13,
    13,
    13,
    14,
    14,
    14,
    14,
    14,
    14,
    14,
    14,
    14,
    14,
    14,
    14,
    14,
    14,
    14,
    14,
    15,
    15,
    15,
    15,
    15,
    15,
    15,
    15,
    15,
    15,
    15,
    15,
    15,
    15,
    15,
    15,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: can_input
 *
 * Description:
 *   Handle incoming packet input
 *
 * Input Parameters:
 *   dev - The device driver structure containing the received packet
 *
 * Returned Value:
 *   OK     The packet has been processed  and can be deleted
 *  -EAGAIN There is a matching connection, but could not dispatch the packet
 *          yet.  Useful when a packet arrives before a recv call is in
 *          place.
 *
 * Assumptions:
 *   This function can be called from an interrupt.
 *
 ****************************************************************************/

int can_input(struct net_driver_s *dev)
{
  FAR struct can_conn_s *conn = NULL;
  int ret = OK;
  uint16_t buflen = dev->d_len;

  do
    {
      conn = can_nextconn(conn);

      if (conn && (conn->dev == 0x0 || dev == conn->dev))
        {
          uint16_t flags;

          /* Setup for the application callback */

          dev->d_appdata = dev->d_buf;
          dev->d_sndlen  = 0;
          dev->d_len     = buflen;

          /* Perform the application callback */

          flags = can_callback(dev, conn, CAN_NEWDATA);

          /* If the operation was successful, the CAN_NEWDATA flag is removed
           * and thus the packet can be deleted (OK will be returned).
           */

          if ((flags & CAN_NEWDATA) != 0)
            {
              /* No.. the packet was not processed now.  Return -EAGAIN so
               * that the driver may retry again later.  We still need to
               * set d_len to zero so that the driver is aware that there
               * is nothing to be sent.
               */

               nwarn("WARNING: Packet not processed\n");
               ret = -EAGAIN;
            }
        }
    }
  while (conn);

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_CAN */
