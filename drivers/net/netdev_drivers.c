/****************************************************************************
 * drivers/net/netdev_drivers.c
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

#include <debug.h>
#include <string.h>

#ifdef CONFIG_DRIVERS_NOTENET
#  include <nuttx/note/notenet_driver.h>
#endif

#include "netdev_drivers.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_drivers_init
 *
 * Description:
 *   Register all enabled NETDEV drivers.
 *
 * Input Parameters:
 *   dev    - The lower half device driver structure
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 ****************************************************************************/

int netdev_drivers_init(FAR struct netdev_lowerhalf_s *dev)
{
  int ret = OK;

#ifdef CONFIG_DRIVERS_NOTENET
  if (strcmp(dev->netdev.d_ifname, CONFIG_DRIVERS_NOTENET_INTF) == 0)
    {
      ninfo("bind NOTE with netdev = %p\n", dev);
      ret = notenet_init(dev);
      if (ret < 0)
        {
          nerr("ERROR: netdev_note_init failed: %d\n", ret);
          return ret;
        }
    }
#endif

  return ret;
}
