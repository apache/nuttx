/****************************************************************************
 * net/netpoll/netpoll_pkt.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/net/netpoll.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* How many times to reclaim TX packets from the driver before giving up */

#define NETPOLL_PKT_ALLOC_RETRY 8

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netpoll_pkt_alloc
 *
 * Description:
 *   Allocate a TX packet for netpoll.
 *   When the TX quota is used up, ask the driver to reclaim finished TX
 *   packets and try again a bounded number of times.
 *
 * Input Parameters:
 *   dev - The lower half device driver structure
 *
 * Returned Value:
 *   Pointer to the packet, NULL on failure
 *
 ****************************************************************************/

FAR netpkt_t *netpoll_pkt_alloc(FAR struct netdev_lowerhalf_s *dev)
{
  FAR netpkt_t *pkt;
  int retry;

  for (retry = 0; retry < NETPOLL_PKT_ALLOC_RETRY; retry++)
    {
      pkt = netpkt_alloc(dev, NETPKT_TX);
      if (pkt != NULL)
        {
          return pkt;
        }

      /* Try to release TX packets the driver has finished with */

      dev->ops->txpoll(dev);
    }

  return NULL;
}
