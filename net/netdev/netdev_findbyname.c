/****************************************************************************
 * net/netdev/netdev_findbyname.c
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

#include <string.h>
#include <errno.h>

#include <nuttx/net/netdev.h>

#include "utils/utils.h"
#include "netdev/netdev.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_findbyname
 *
 * Description:
 *   Find a previously registered network device using its assigned
 *   network interface name
 *
 * Input Parameters:
 *   ifname The interface name of the device of interest
 *
 * Returned Value:
 *  Pointer to driver on success; null on failure
 *
 ****************************************************************************/

FAR struct net_driver_s *netdev_findbyname(FAR const char *ifname)
{
  FAR struct net_driver_s *dev;

  if (ifname)
    {
      net_lock();
      for (dev = g_netdevices; dev; dev = dev->flink)
        {
          if (strcmp(ifname, dev->d_ifname) == 0)
            {
              net_unlock();
              return dev;
            }
        }

      net_unlock();
    }

  return NULL;
}
