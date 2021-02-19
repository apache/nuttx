/****************************************************************************
 * net/ieee802154/ieee802154_finddev.c
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
#include <nuttx/net/ieee802154.h>

#include "netdev/netdev.h"
#include "ieee802154/ieee802154.h"

#ifdef CONFIG_NET_IEEE802154

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ieee802154_finddev_s
{
  FAR const struct ieee802154_saddr_s *addr;
  FAR struct radio_driver_s *radio;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ieee802154_dev_callback
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

static int ieee802154_dev_callback(FAR struct net_driver_s *dev,
                                   FAR void *arg)
{
  FAR struct ieee802154_finddev_s *match =
    (FAR struct ieee802154_finddev_s *)arg;

  DEBUGASSERT(dev != NULL && match != NULL && match->addr != NULL);

  /* First, check if this network device is an IEEE 802.15.4 radio and that
   * it has an assigned IEEE 802.15.4 address.
   */

  if (dev->d_lltype == NET_LL_IEEE802154 && dev->d_mac.radio.nv_addrlen > 0)
    {
      DEBUGASSERT(dev->d_mac.radio.nv_addrlen == 2 ||
                  dev->d_mac.radio.nv_addrlen == 8);

      /* Does the address mode match? */

      if (match->addr->s_mode == IEEE802154_ADDRMODE_SHORT &&
          dev->d_mac.radio.nv_addrlen == 2)
        {
          /* Does the device address match */

          if (IEEE802154_SADDRCMP(dev->d_mac.radio.nv_addr,
                                  match->addr->s_saddr))
            {
              /* Yes.. save the match and return 1 to stop the search */

              match->radio = (FAR struct radio_driver_s *)dev;
              return 1;
            }
        }
      else if (match->addr->s_mode == IEEE802154_ADDRMODE_EXTENDED &&
               dev->d_mac.radio.nv_addrlen == 8)
        {
          /* Does the device address match */

          if (IEEE802154_EADDRCMP(dev->d_mac.radio.nv_addr,
                                  match->addr->s_eaddr))
            {
              /* Yes.. save the match and return 1 to stop the search */

              match->radio = (FAR struct radio_driver_s *)dev;
              return 1;
            }
        }
    }

  /* Keep looking */

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ieee802154_find_device
 *
 * Description:
 *   Select the network driver to use with the IEEE802154 transaction.
 *
 * Input Parameters:
 *   conn - IEEE802154 connection structure (not currently used).
 *   addr - The address to match the devices assigned address
 *
 * Returned Value:
 *   A pointer to the network driver to use.  NULL is returned on any
 *   failure.
 *
 ****************************************************************************/

FAR struct radio_driver_s *
  ieee802154_find_device(FAR struct ieee802154_conn_s *conn,
                         FAR const struct ieee802154_saddr_s *addr)
{
  struct ieee802154_finddev_s match;
  int ret;

  DEBUGASSERT(conn != NULL);
  match.addr  = addr;
  match.radio = NULL;

  /* If the socket is not bound to a local address, then return a failure */

  if (addr->s_mode == IEEE802154_ADDRMODE_NONE)
    {
      return NULL;
    }

  DEBUGASSERT(addr->s_mode == IEEE802154_ADDRMODE_SHORT ||
              addr->s_mode == IEEE802154_ADDRMODE_EXTENDED);

  /* Other, search for the IEEE 802.15.4 network device whose MAC is equal to
   * the sockets bound local address.
   */

  ret = netdev_foreach(ieee802154_dev_callback, (FAR void *)&match);
  if (ret == 1)
    {
      DEBUGASSERT(match.radio != NULL);
      return match.radio;
    }

  return NULL;
}

#endif /* CONFIG_NET_IEEE802154 */
