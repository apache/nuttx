/****************************************************************************
 * net/netdev/netdev_iob.c
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
      nwarn("WARNING: IOB Prepare failed for dev %s!\n", dev->d_ifname);
      return -ENOMEM;
    }

  /* Update l2 gruard size */

  iob_reserve(dev->d_iob, CONFIG_NET_LL_GUARDSIZE);

  /* Set the device buffer to l2 */

  dev->d_buf = NETLLBUF;

  return OK;
}

/****************************************************************************
 * Name: netdev_iob_prepare_dynamic
 *
 * Description:
 *   Pre-alloc the iob for the data to be sent.
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

#ifdef CONFIG_IOB_ALLOC
void netdev_iob_prepare_dynamic(FAR struct net_driver_s *dev, uint16_t size)
{
  FAR struct iob_s *iob;
  size += CONFIG_NET_LL_GUARDSIZE;

  if (dev->d_iob && size <= IOB_BUFSIZE(dev->d_iob))
    {
      return;
    }

  /* alloc new iob for jumbo frame */

  iob = iob_alloc_dynamic(size);
  if (iob == NULL)
    {
      nerr("ERROR: Failed to allocate an I/O buffer.");
      return;
    }

  iob_reserve(iob, CONFIG_NET_LL_GUARDSIZE);

  netdev_iob_replace(dev, iob);
}
#endif

/****************************************************************************
 * Name: netdev_iob_replace
 *
 * Description:
 *   Replace buffer resources for a given NIC
 *
 * Assumptions:
 *   The caller has locked the network and new iob is prepared with
 *   l2 gruard size as offset.
 *
 ****************************************************************************/

void netdev_iob_replace(FAR struct net_driver_s *dev, FAR struct iob_s *iob)
{
  /* Release previous buffer */

  netdev_iob_release(dev);

  /* Set new buffer */

  dev->d_iob = iob;
  dev->d_buf = NETLLBUF;
  dev->d_len = iob->io_pktlen;
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
    }

  dev->d_buf = NULL;
}

/****************************************************************************
 * Name: netdev_iob_clone
 *
 * Description:
 *   Backup the current iob buffer for a given NIC by cloning it.
 *
 * Assumptions:
 *   The caller has locked the network.
 *
 ****************************************************************************/

FAR struct iob_s *netdev_iob_clone(FAR struct net_driver_s *dev,
                                   bool throttled)
{
  FAR struct iob_s *iob;
  int ret;

  iob = iob_tryalloc(throttled);
  if (iob == NULL)
    {
      nwarn("WARNING: IOB alloc failed for dev %s!\n", dev->d_ifname);
      return NULL;
    }

  iob_reserve(iob, CONFIG_NET_LL_GUARDSIZE);
  ret = iob_clone_partial(dev->d_iob, dev->d_iob->io_pktlen, 0,
                          iob, 0, throttled, false);
  if (ret < 0)
    {
      iob_free_chain(iob);
      nwarn("WARNING: IOB clone failed for dev %s, ret=%d!\n",
            dev->d_ifname, ret);
      return NULL;
    }

  return iob;
}
