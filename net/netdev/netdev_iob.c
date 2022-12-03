/****************************************************************************
 * net/netdev/netdev_iob.c
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
#include <errno.h>

#include <nuttx/net/netdev.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_iob_prepare
 *
 * Description:
 *   Prepare data buffer for a given NIC
 *   The iob offset will be updated to l2 gruard size by default:
 *  ----------------------------------------------------------------
 *  |                     iob entry                                |
 *  ---------------------------------------------------------------|
 *  |<--- CONFIG_NET_LL_GUARDSIZE -->|<--- io_len/io_pktlen(0) --->|
 *  ---------------------------------------------------------------|
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 * Returned Value:
 *   A non-zero copy is returned on success.
 *
 ****************************************************************************/

int netdev_iob_prepare(FAR struct net_driver_s *dev, bool throttled,
                       unsigned int timeout)
{
  /* Prepare iob buffer */

  if (dev->d_iob == NULL)
    {
      dev->d_iob = net_iobtimedalloc(false, timeout);
      if (dev->d_iob == NULL && throttled)
        {
          dev->d_iob = net_iobtimedalloc(true, timeout);
        }
    }

  if (dev->d_iob == NULL)
    {
      return -ENOMEM;
    }

  /* Set the device buffer to l2 */

  dev->d_buf = &dev->d_iob->io_data[CONFIG_NET_LL_GUARDSIZE -
                                    NET_LL_HDRLEN(dev)];

  /* Update l2 gruard size */

  iob_reserve(dev->d_iob, CONFIG_NET_LL_GUARDSIZE);

  return OK;
}

/****************************************************************************
 * Name: netdev_iob_clear
 *
 * Description:
 *   Clean up buffer resources for a given NIC
 *
 * Assumptions:
 *   The caller has locked the network and dev->d_iob has been
 *   released or taken away.
 *
 ****************************************************************************/

void netdev_iob_clear(FAR struct net_driver_s *dev)
{
  /* Clear the device buffer */

  dev->d_iob = NULL;
  dev->d_buf = NULL;
  dev->d_len = 0;
}

/****************************************************************************
 * Name: netdev_iob_release
 *
 * Description:
 *   Release buffer resources for a given NIC
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

void netdev_iob_release(FAR struct net_driver_s *dev)
{
  /* Release device buffer */

  if (dev->d_iob != NULL)
    {
      iob_free_chain(dev->d_iob);
      dev->d_iob = NULL;
      dev->d_buf = NULL;
    }
}
