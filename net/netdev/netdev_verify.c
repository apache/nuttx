/****************************************************************************
 * net/netdev/netdev_verify.c
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

#include <stdbool.h>

#include <nuttx/net/netdev.h>

#include "utils/utils.h"
#include "netdev/netdev.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_verify
 *
 * Description:
 *   Verify that the specified device still exists
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

bool netdev_verify(FAR struct net_driver_s *dev)
{
  FAR struct net_driver_s *chkdev;
  bool valid = false;

  /* Search the list of registered devices */

  net_lock();
  for (chkdev = g_netdevices; chkdev != NULL; chkdev = chkdev->flink)
    {
      /* Is the network device that we are looking for? */

      if (chkdev == dev)
        {
          /* Yes.. return true  */

          valid = true;
          break;
        }
    }

  net_unlock();
  return valid;
}
