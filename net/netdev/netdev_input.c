/****************************************************************************
 * net/netdev/netdev_input.c
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

#include <nuttx/net/netdev.h>

#include "utils/utils.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_input
 *
 * Description:
 *   This function will copy the flat buffer that does not support
 *   Scatter/gather to the iob vector buffer.
 *
 *   Compatible with all old flat buffer NICs:
 *
 *   [tcp|udp|icmp|...]ipv[4|6]_data_handler()
 *                     |                    (iob_concat/append to readahead)
 *                     |
 *              pkt/ipv[4/6]_in()/...
 *                     |
 *                     |
 *                netdev_input()  // new interface, Scatter/gather flat/iobs
 *                     |
 *                     |
 *           pkt/ipv[4|6]_input()/...
 *                     |
 *                     |
 *     NICs io vector receive(Orignal flat buffer)
 *
 * Input Parameters:
 *   NULL
 *
 * Returned Value:
 *  Pointer to default network driver on success; null on failure
 *
 ****************************************************************************/

int netdev_input(FAR struct net_driver_s *dev,
                 devif_poll_callback_t callback, bool reply)
{
  uint16_t llhdrlen = NET_LL_HDRLEN(dev);
  FAR uint8_t *buf = dev->d_buf;
  int ret;

  /* Prepare iob buffer */

  ret = netdev_iob_prepare(dev, false, 0);
  if (ret != OK)
    {
      return ret;
    }

  /* Copy data to iob entry */

  ret = iob_trycopyin(dev->d_iob, buf, dev->d_len, -llhdrlen, false);
  if (ret == dev->d_len)
    {
      /* Update device buffer to l2 start */

      dev->d_buf = NETLLBUF;

      ret = callback(dev);
      if (dev->d_iob != NULL && reply)
        {
          if (ret == OK && dev->d_len > 0)
            {
              iob_copyout(buf, dev->d_iob, dev->d_len, -llhdrlen);
            }
        }
    }

  netdev_iob_release(dev);

  dev->d_buf = buf;

  return ret;
}
