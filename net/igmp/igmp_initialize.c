/****************************************************************************
 * net/igmp/igmp_initialize.c
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

#include <assert.h>
#include <nuttx/debug.h>

#include <nuttx/net/ip.h>
#include <nuttx/net/igmp.h>

#include "devif/devif.h"
#include "igmp/igmp.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Public Data
 ****************************************************************************/

const in_addr_t g_ipv4_allsystems = HTONL(0xe0000001);
const in_addr_t g_ipv4_allrouters = HTONL(0xe0000002);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_devinit
 *
 * Description:
 *   Called when a new network device is registered to configure that device
 *   for IGMP support.
 *
 ****************************************************************************/

void igmp_devinit(struct net_driver_s *dev)
{
  ninfo("IGMP initializing dev %p\n", dev);
  DEBUGASSERT(dev->d_igmp_grplist.head == NULL);

  /* Add the all systems address to the group */

  igmp_grpalloc(dev, &g_ipv4_allsystems);

  /* Allow the IGMP messages at the MAC level */

  igmp_addmcastmac(dev, &g_ipv4_allrouters);
  igmp_addmcastmac(dev, &g_ipv4_allsystems);
}

#endif /* CONFIG_NET_IGMP */
