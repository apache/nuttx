/****************************************************************************
 * net/netdev/netdev_foreach.c
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

#include <debug.h>

#include <nuttx/net/netdev.h>

#include "netdev/netdev.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_foreach
 *
 * Description:
 *   Enumerate each registered network device.  This function will terminate
 *   when either (1) all devices have been enumerated or (2) when a callback
 *   returns any non-zero value.
 *
 *   NOTE 1:  The network must be locked throughout the enumeration.
 *   NOTE 2:  No checks are made on devices.  For examples, callbacks will
 *            will be made on network devices that are in the 'down' state.
 *            The callback implementations must take into account all
 *            network device state.  Typically, a network in the down state
 *            would not terminate the traversal.
 *
 * Input Parameters:
 *   callback - Will be called for each registered device
 *   arg      - Opaque user argument passed to callback()
 *
 * Returned Value:
 *  0: Enumeration completed
 *  1: Enumeration terminated early by callback
 *
 * Assumptions:
 *  The network is locked.
 *
 ****************************************************************************/

int netdev_foreach(netdev_callback_t callback, FAR void *arg)
{
  FAR struct net_driver_s *dev;
  int ret = 0;

  if (callback != NULL)
    {
      for (dev = g_netdevices; dev; dev = dev->flink)
        {
          if (callback(dev, arg) != 0)
            {
              ret = 1;
              break;
            }
        }
    }

  return ret;
}
