/****************************************************************************
 * net/igmp/igmp_mcastmac.c
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
#include <inttypes.h>
#include <stdint.h>

#include <nuttx/net/netconfig.h>
#include <nuttx/net/ip.h>
#include <nuttx/net/igmp.h>

#include "devif/devif.h"
#include "igmp/igmp.h"

#ifdef CONFIG_NET_IGMP

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_mcastmac
 *
 * Description:
 *   Given an IP address (in network order), create a IGMP multicast MAC
 *   address.
 *
 ****************************************************************************/

static void igmp_mcastmac(FAR const in_addr_t *ip, FAR uint8_t *mac)
{
  /* This mapping is from the IETF IN RFC 1700 */

  mac[0] = 0x01;
  mac[1] = 0x00;
  mac[2] = 0x5e;
  mac[3] = ip4_addr2(*ip) & 0x7f;
  mac[4] = ip4_addr3(*ip);
  mac[5] = ip4_addr4(*ip);

  ninfo("IP: %08" PRIx32 " -> MAC: %02x%02x%02x%02x%02x%02x\n",
        (uint32_t)*ip, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  igmp_addmcastmac
 *
 * Description:
 *   Add an IGMP MAC address to the device's MAC filter table.
 *
 ****************************************************************************/

void igmp_addmcastmac(FAR struct net_driver_s *dev, FAR const in_addr_t *ip)
{
  uint8_t mcastmac[6];

  ninfo("Adding: IP %08" PRIx32 "\n", (uint32_t)*ip);
  if (dev->d_addmac)
    {
      igmp_mcastmac(ip, mcastmac);
      dev->d_addmac(dev, mcastmac);
    }
}

/****************************************************************************
 * Name:  igmp_removemcastmac
 *
 * Description:
 *   Remove an IGMP MAC address from the device's MAC filter table.
 *
 ****************************************************************************/

void igmp_removemcastmac(FAR struct net_driver_s *dev,
                         FAR const in_addr_t *ip)
{
  uint8_t mcastmac[6];

  ninfo("Removing: IP %08" PRIx32 "\n", (uint32_t)*ip);
  if (dev->d_rmmac)
    {
      igmp_mcastmac(ip, mcastmac);
      dev->d_rmmac(dev, mcastmac);
    }
}

#endif /* CONFIG_NET_IGMP */
