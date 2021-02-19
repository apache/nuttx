/****************************************************************************
 * net/netdev/netdev_nametoindex.c
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
#include <errno.h>

#include <net/if.h>

#include "nuttx/net/net.h"
#include "nuttx/net/netdev.h"

#include "netdev/netdev.h"

#ifdef CONFIG_NETDEV_IFINDEX

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_nametoindex
 *
 * Description:
 *   The if_nametoindex() function returns the interface index corresponding
 *   to name ifname.
 *
 * Input Parameters:
 *   ifname - The interface name
 *
 * Returned Value:
 *   The corresponding index if ifname is the name of an interface;
 *   otherwise, a negated errno value is returned.
 *
 ****************************************************************************/

unsigned int netdev_nametoindex(FAR const char *ifname)
{
  FAR struct net_driver_s *dev;
  unsigned int ifindex = -ENODEV;

  /* Find the driver with this name */

  net_lock();
  dev = netdev_findbyname(ifname);
  if (dev != NULL)
    {
      ifindex = dev->d_ifindex;
    }

  net_unlock();
  return ifindex;
}

/****************************************************************************
 * Name: if_nametoindex
 *
 * Description:
 *   The if_nametoindex() function returns the interface index corresponding
 *   to name ifname.
 *
 * Input Parameters:
 *   ifname - The interface name
 *
 * Returned Value:
 *   The corresponding index if ifname is the name of an interface;
 *   otherwise, zero.  Although not specified, the errno value will be set.
 *
 ****************************************************************************/

unsigned int if_nametoindex(FAR const char *ifname)
{
  int ret;

  /* Let netdev_nametoindex to the work */

  ret = netdev_nametoindex(ifname);
  if (ret < 0)
    {
      set_errno(-ret);
      return 0;
    }

  return ret;
}

#endif /* CONFIG_NETDEV_IFINDEX */
