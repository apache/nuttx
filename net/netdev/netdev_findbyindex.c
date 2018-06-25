/****************************************************************************
 * net/netdev/netdev_findbyindex.c
 *
 *   Copyright (C) 2015, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

#if CONFIG_NSOCKET_DESCRIPTORS > 0

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

  if ((g_devset & (1L << (ifindex - 1))) == 0)
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
          if ((g_devset & (1L << ifindex)) != 0)
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

#endif /* CONFIG_NSOCKET_DESCRIPTORS */
