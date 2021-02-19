/****************************************************************************
 * net/bluetooth/bluetooth_finddev.c
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

#include <nuttx/net/net.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>

#include "netdev/netdev.h"
#include "bluetooth/bluetooth.h"

#ifdef CONFIG_NET_BLUETOOTH

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bluetooth_finddev_s
{
  FAR const bt_addr_t       *bf_addr;
  FAR struct radio_driver_s *bf_radio;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bluetooth_dev_callback
 *
 * Description:
 *   Check if this device matches the connections local address.
 *
 * Input Parameters:
 *   dev - The next network device in the enumeration
 *
 * Returned Value:
 *   0 if there is no match (meaning to continue looking); 1 if there is a
 *   match (meaning to stop the search).
 *
 ****************************************************************************/

static int bluetooth_dev_callback(FAR struct net_driver_s *dev,
                                  FAR void *arg)
{
  FAR struct bluetooth_finddev_s *match =
    (FAR struct bluetooth_finddev_s *)arg;

  DEBUGASSERT(dev != NULL && match != NULL && match->bf_addr != NULL);

  /* First, check if this network device is an Bluetooth radio and that
   * it has an assigned Bluetooth address.
   */

  if (dev->d_lltype == NET_LL_BLUETOOTH && dev->d_mac.radio.nv_addrlen > 0)
    {
      DEBUGASSERT(dev->d_mac.radio.nv_addrlen == BLUETOOTH_ADDRSIZE);

      /* Does the device address match */

      if (BLUETOOTH_ADDRCMP(dev->d_mac.radio.nv_addr, match->bf_addr))
        {
          /* Yes.. save the match and return 1 to stop the search */

          match->bf_radio = (FAR struct radio_driver_s *)dev;
          return 1;
        }
    }

  /* Keep looking */

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bluetooth_find_device
 *
 * Description:
 *   Select the network driver to use with the Bluetooth transaction.
 *
 * Input Parameters:
 *   conn - Bluetooth connection structure (not currently used).
 *   addr - The address to match the devices assigned address
 *
 * Returned Value:
 *   A pointer to the network driver to use.  NULL is returned on any
 *   failure.
 *
 ****************************************************************************/

FAR struct radio_driver_s *
  bluetooth_find_device(FAR struct bluetooth_conn_s *conn,
                        FAR const bt_addr_t *addr)
{
  struct bluetooth_finddev_s match;
  int ret;

  DEBUGASSERT(conn != NULL);
  match.bf_addr  = addr;
  match.bf_radio = NULL;

  /* Search for the Bluetooth network device whose MAC is equal to the
   * sockets bound local address.
   */

  ret = netdev_foreach(bluetooth_dev_callback, (FAR void *)&match);
  if (ret == 1)
    {
      DEBUGASSERT(match.bf_radio != NULL);
      return match.bf_radio;
    }

  return NULL;
}

#endif /* CONFIG_NET_BLUETOOTH */
