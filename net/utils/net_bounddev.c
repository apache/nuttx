/****************************************************************************
 * net/utils/net_bounddev.c
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

#include <nuttx/net/netdev.h>
#include <nuttx/net/net.h>

#include "netdev/netdev.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: net_bound_device
 *
 * Description:
 *   If the socket is bound to a device, return the reference to the
 *   bound device.
 *
 * Input Parameters:
 *   sconn - Socket connection structure (not currently used).
 *
 * Returned Value:
 *   A reference to the bound device.  If the retained interface index no
 *   longer refers to a valid device, this function will unbind the device
 *   and return an arbitrary network device at the head of the list of
 *   registered devices.  This supports legacy IPv4 DHCPD behavior when
 *   there is only a single registered network device.
 *
 ****************************************************************************/

#ifdef CONFIG_NET_BINDTODEVICE
FAR struct net_driver_s *net_bound_device(FAR struct socket_conn_s *sconn)
{
  FAR struct net_driver_s *dev = NULL;

  /* Is the socket bound to a device? */

  if (sconn->s_boundto != 0)
    {
      /* Yes..This socket has been bound to an interface.  Convert the
       * interface index into a device structure reference.
       */

      dev = netdev_findbyindex(sconn->s_boundto);
      if (dev == NULL)
        {
          /* No device?  It must have been unregistered.  Un-bind the
           * socket.
           */

          sconn->s_boundto = 0;
        }
    }

  /* If no device was bound or the bound device is no longer valid,
   * then let's try the default network device.
   */

  return dev == NULL ? netdev_default() : dev;
}
#endif

