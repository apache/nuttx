/****************************************************************************
 * net/netdev/netdev_carrier.c
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

#include <sys/socket.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <net/if.h>
#include <net/ethernet.h>
#include <nuttx/net/netdev.h>

#include "ipfrag/ipfrag.h"
#include "netdev/netdev.h"
#include "netlink/netlink.h"
#include "arp/arp.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_carrier_on
 *
 * Description:
 *   Notifies the networking layer about an available carrier.
 *   (e.g. a cable was plugged in)
 *
 * Input Parameters:
 *   dev - The device driver structure
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

int netdev_carrier_on(FAR struct net_driver_s *dev)
{
  if (dev && !IFF_IS_RUNNING(dev->d_flags))
    {
      dev->d_flags |= IFF_RUNNING;
      netlink_device_notify(dev);
      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: netdev_carrier_off
 *
 * Description:
 *   Notifies the networking layer about an disappeared carrier.
 *   (e.g. a cable was unplugged)
 *
 * Input Parameters:
 *   dev - The device driver structure
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

int netdev_carrier_off(FAR struct net_driver_s *dev)
{
  if (dev && IFF_IS_RUNNING(dev->d_flags))
    {
      dev->d_flags &= ~IFF_RUNNING;
      netlink_device_notify(dev);

#ifdef CONFIG_NET_IPFRAG
      /* Clean up fragment data for this NIC (if any) */

      ip_frag_stop(dev);
#endif

      /* Notify clients that the network has been taken down */

      devif_dev_event(dev, NETDEV_DOWN);
      arp_cleanup(dev);

      return OK;
    }

  return -EINVAL;
}
