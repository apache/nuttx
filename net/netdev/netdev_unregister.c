/****************************************************************************
 * net/netdev/netdev_unregister.c
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

#include <sys/socket.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <net/if.h>
#include <net/ethernet.h>
#include <nuttx/net/netdev.h>

#include "utils/utils.h"
#include "netdev/netdev.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_NET_SLIP
#  define NETDEV_FORMAT "sl%d"
#else
#  define NETDEV_FORMAT "eth%d"
#endif

/****************************************************************************
 * Name: free_ifindex
 *
 * Description:
 *   Free a interface index to the assigned to device.
 *
 * Input Parameters:
 *   dev - Instance of device structure for the unregistered device.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IFINDEX
static inline void free_ifindex(int ndx)
{
  uint32_t bit;

  /* The bit index is the interface index minus one.  Zero is reserved in
   * POSIX to mean no interface index.
   */

  DEBUGASSERT(ndx > 0 && ndx <= MAX_IFINDEX);
  ndx--;

  /* Set the bit in g_devset corresponding to the zero based index */

  bit = 1 << ndx;

  /* The bit should be in the set state */

  DEBUGASSERT((g_devset & bit) != 0 && (g_devfreed & bit) == 0);
  g_devset   &= ~bit;
  g_devfreed |= bit;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_unregister
 *
 * Description:
 *   Unregister a network device driver.
 *
 * Input Parameters:
 *   dev - The device driver structure to un-register
 *
 * Returned Value:
 *   0:Success; negated errno on failure
 *
 * Assumptions:
 *   Currently only called for USB networking devices when the device is
 *   physically removed from the slot
 *
 ****************************************************************************/

int netdev_unregister(FAR struct net_driver_s *dev)
{
  struct net_driver_s *prev;
  struct net_driver_s *curr;

  if (dev)
    {
      net_lock();

      /* Find the device in the list of known network devices */

      for (prev = NULL, curr = g_netdevices;
           curr && curr != dev;
           prev = curr, curr = curr->flink);

      /* Remove the device to the list of known network devices */

      if (curr)
        {
          /* Where was the entry */

          if (prev)
            {
              /* The entry was in the middle or at the end of the list */

              prev->flink = curr->flink;
            }
          else
            {
              /* The entry was at the beginning of the list */

              g_netdevices = curr->flink;
            }

          curr->flink = NULL;
        }

#ifdef CONFIG_NETDEV_IFINDEX
      free_ifindex(dev->d_ifindex);
#endif
      net_unlock();

#ifdef CONFIG_NET_ETHERNET
      ninfo("Unregistered MAC: %02x:%02x:%02x:%02x:%02x:%02x as dev: %s\n",
            dev->d_mac.ether.ether_addr_octet[0],
            dev->d_mac.ether.ether_addr_octet[1],
            dev->d_mac.ether.ether_addr_octet[2],
            dev->d_mac.ether.ether_addr_octet[3],
            dev->d_mac.ether.ether_addr_octet[4],
            dev->d_mac.ether.ether_addr_octet[5], dev->d_ifname);
#else
      ninfo("Unregistered dev: %s\n", dev->d_ifname);
#endif
      return OK;
    }

  return -EINVAL;
}
