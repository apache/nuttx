/****************************************************************************
 * net/pkt/pkt_input.c
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
#if defined(CONFIG_NET) && defined(CONFIG_NET_PKT)

#include <errno.h>
#include <debug.h>

#include <nuttx/net/netdev.h>
#include <nuttx/net/pkt.h>

#include "devif/devif.h"
#include "pkt/pkt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PKTBUF ((FAR struct eth_hdr_s *)&dev->d_buf)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pkt_input
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
 *   The network is locked.
 *
 ****************************************************************************/

int pkt_input(struct net_driver_s *dev)
{
  FAR struct pkt_conn_s *conn;
  FAR struct eth_hdr_s  *pbuf = (FAR struct eth_hdr_s *)dev->d_buf;
  int ret = OK;

  conn = pkt_active(pbuf);
  if (conn)
    {
      uint16_t flags;

      /* Setup for the application callback */

      dev->d_appdata = dev->d_buf;
      dev->d_sndlen  = 0;

      /* Perform the application callback */

      flags = pkt_callback(dev, conn, PKT_NEWDATA);

      /* If the operation was successful, the PKT_NEWDATA flag is removed
       * and thus the packet can be deleted (OK will be returned).
       */

      if ((flags & PKT_NEWDATA) != 0)
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
  else
    {
      ninfo("No PKT listener\n");
    }

  return ret;
}

#endif /* CONFIG_NET && CONFIG_NET_PKT */
