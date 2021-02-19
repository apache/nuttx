/****************************************************************************
 * net/netdev/netdev_indextoname.c
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
 * Name: netdev_indextoname
 *
 * Description:
 *   The if_indextoname() function maps an interface index to its
 *   corresponding name.
 *
 * Input Parameters:
 *   ifname  - Points to a buffer of at least IF_NAMESIZE bytes.
 *             if_indextoname() will place in this buffer the name of the
 *             interface with index ifindex.
 *
 * Returned Value:
 *   If ifindex is an interface index, then the function will return zero
 *   (OK). Otherwise, the function returns a negated errno value;
 *
 ****************************************************************************/

int netdev_indextoname(unsigned int ifindex, FAR char *ifname)
{
  FAR struct net_driver_s *dev;
  int ret = -ENODEV;

  DEBUGASSERT(ifindex > 0 && ifindex <= MAX_IFINDEX);
  DEBUGASSERT(ifname != NULL);

  /* Find the driver with this name */

  net_lock();
  dev = netdev_findbyindex(ifindex);
  if (dev != NULL)
    {
      memcpy(ifname, dev->d_ifname, IF_NAMESIZE);
      ret = OK;
    }

  net_unlock();
  return ret;
}

/****************************************************************************
 * Name: if_indextoname
 *
 * Description:
 *   The if_indextoname() function maps an interface index to its
 *   corresponding name.
 *
 * Input Parameters:
 *   ifname  - Points to a buffer of at least IF_NAMESIZE bytes.
 *             if_indextoname() will place in this buffer the name of the
 *             interface with index ifindex.
 *
 * Returned Value:
 *   If ifindex is an interface index, then the function will return the
 *   value supplied by ifname. Otherwise, the function returns a NULL pointer
 *   and sets errno to indicate the error.
 *
 ****************************************************************************/

FAR char *if_indextoname(unsigned int ifindex, FAR char *ifname)
{
  int ret;

  /* Let netdev_indextoname to the work */

  ret = netdev_indextoname(ifindex, ifname);
  if (ret < 0)
    {
      set_errno(-ret);
      return NULL;
    }

  return ifname;
}

#endif /* CONFIG_NETDEV_IFINDEX */
