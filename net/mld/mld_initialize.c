/****************************************************************************
 * net/mld/mld_initialize.c
 * MLD initialization logic
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
#include <debug.h>

#include <nuttx/wdog.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/mld.h>

#include "devif/devif.h"
#include "inet/inet.h"
#include "mld/mld.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  mld_devinit
 *
 * Description:
 *   Called when a new network device is registered to configure that device
 *   for MLD support.
 *
 ****************************************************************************/

void mld_devinit(struct net_driver_s *dev)
{
  mldinfo("MLD initializing dev %p\n", dev);

  /* Initialize the MLD state in the device structure */

  memset(&dev->d_mld, 0, sizeof(struct mld_netdev_s));

  /* All routers start up as a Querier on each of their attached links. */

  SET_MLD_QUERIER(dev->d_mld.flags);

  /* Add the all nodes address to the group
   * REVISIT: Do we need this?  What is it for?  It is clone from IGMP and
   * probably is not relevant here.
   */

  mld_grpalloc(dev, g_ipv6_allnodes);

  /* Allow the MLD messages at the MAC level */

  mld_addmcastmac(dev, g_ipv6_allnodes);
  mld_addmcastmac(dev, g_ipv6_allrouters);
  mld_addmcastmac(dev, g_ipv6_allmldv2routers);

#ifdef CONFIG_NET_MLD_ROUTER
  /* Start the general query timer. */

  mld_start_gentimer(dev, MSEC2TICK(MLD_QUERY_MSEC));
#endif
}
