/****************************************************************************
 * net/netdev/netdev_findbyindex.c
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

#include <nuttx/net/netdev.h>

#include "utils/utils.h"
#include "netdev/netdev.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_findbyindex
 *
 * Description:
 *   Find a previously registered network device by assigned interface index.
 *
 * Input Parameters:
 *   ifindex - The interface index.  This is a one-based index and must be
 *             greater than zero.
 *
 * Returned Value:
 *  Pointer to driver on success; NULL on failure.  This function will return
 *  NULL only if there is no device corresponding to the provided index.
 *
 ****************************************************************************/

FAR struct net_driver_s *netdev_findbyindex(int ifindex)
{
  FAR struct net_driver_s *dev;
  int i;

#ifdef CONFIG_NETDEV_IFINDEX
  /* The bit index is the interface index minus one.  Zero is reserved in
   * POSIX to mean no interface index.
   */

  DEBUGASSERT(ifindex > 0 && ifindex <= MAX_IFINDEX);
  if (ifindex < 1 || ifindex > MAX_IFINDEX)
    {
      return NULL;
    }
#endif

  net_lock();

#ifdef CONFIG_NETDEV_IFINDEX
  /* Check if this index has been assigned */

  if ((g_devset & (1UL << (ifindex - 1))) == 0)
    {
      /* This index has not been assigned */

      net_unlock();
      return NULL;
    }
#endif

  for (i = 0, dev = g_netdevices; dev; i++, dev = dev->flink)
    {
#ifdef CONFIG_NETDEV_IFINDEX
      /* Check if the index matches the index assigned when the device was
       * registered.
       */

      if (dev->d_ifindex == ifindex)
#else
      /* NOTE that this option is not a safe way to enumerate network
       * devices:  There could be changes to the list of registered device
       * causing a given index to be meaningless (unless, of course, the
       * caller keeps the network locked).
       */

      if (i == (ifindex - 1))
#endif
        {
          net_unlock();
          return dev;
        }
    }

  net_unlock();
  return NULL;
}

/****************************************************************************
 * Name: netdev_nextindex
 *
 * Description:
 *   Return the interface index to the next valid device.
 *
 * Input Parameters:
 *   ifindex - The first interface index to check.  Usually in a traversal
 *             this would be the previous interface index plus 1.
 *
 * Returned Value:
 *   The interface index for the next network driver.  -ENODEV is returned if
 *   there are no further devices with assigned interface indices.
 *
 ****************************************************************************/

#ifdef CONFIG_NETDEV_IFINDEX
int netdev_nextindex(int ifindex)
{
  /* The bit index is the interface index minus one.  Zero is reserved in
   * POSIX to mean no interface index.
   */

  DEBUGASSERT(ifindex > 0 && ifindex <= MAX_IFINDEX);
  ifindex--;

  if (ifindex >= 0 && ifindex < MAX_IFINDEX)
    {
      net_lock();
      for (; ifindex < MAX_IFINDEX; ifindex++)
        {
          if ((g_devset & (1UL << ifindex)) != 0)
            {
              /* NOTE that the index + 1 is returned.  Zero is reserved to
               * mean no-index in the POSIX standards.
               */

              net_unlock();
              return ifindex + 1;
            }
        }

      net_unlock();
    }

  return -ENODEV;
}
#endif
