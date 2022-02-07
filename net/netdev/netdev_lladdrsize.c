/****************************************************************************
 * net/netdev/netdev_lladdrsize.c
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

#include <net/if.h>

#include <nuttx/net/net.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/radiodev.h>
#include <nuttx/net/bluetooth.h>
#include <nuttx/net/sixlowpan.h>

#include "netdev/netdev.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_pktradio_addrlen
 *
 * Description:
 *   Returns the size of the node address associated with a packet radio.
 *   This is probably CONFIG_PKTRADIO_ADDRLEN but we cannot be sure in the
 *   case that there are multiple packet radios.  In that case, we have to
 *   query the radio for its address length.
 *
 * Input Parameters:
 *   dev - A reference to the device of interest
 *
 * Returned Value:
 *   The size of the MAC address associated with this radio
 *
 ****************************************************************************/

#if defined(CONFIG_NET_6LOWPAN) && (defined(CONFIG_WIRELESS_PKTRADIO) || \
    defined(CONFIG_NET_BLUETOOTH))
static inline int netdev_pktradio_addrlen(FAR struct net_driver_s *dev)
{
  FAR struct radio_driver_s *radio = (FAR struct radio_driver_s *)dev;
  struct radiodev_properties_s properties;
  int ret;

  DEBUGASSERT(radio != NULL && radio->r_properties != NULL);
  ret = radio->r_properties(radio, &properties);
  if (ret < 0)
    {
      return ret;
    }

  return properties.sp_addrlen;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: netdev_lladdrsize
 *
 * Description:
 *   Returns the size of the MAC address associated with a network device.
 *
 * Input Parameters:
 *   dev - A reference to the device of interest
 *
 * Returned Value:
 *   The size of the MAC address associated with this device
 *
 ****************************************************************************/

int netdev_lladdrsize(FAR struct net_driver_s *dev)
{
  DEBUGASSERT(dev != NULL);

  /* Get the length of the address for this link layer type */

  switch (dev->d_lltype)
    {
#ifdef CONFIG_NET_ETHERNET
      case NET_LL_ETHERNET:
      case NET_LL_IEEE80211:
        {
          /* Size of the Ethernet MAC address */

          return IFHWADDRLEN;
        }
#endif

#ifdef CONFIG_NET_6LOWPAN

#ifdef CONFIG_WIRELESS_IEEE802154
      case NET_LL_IEEE802154:
        {
          /* 6LoWPAN can be configured to use either extended or short
           * addressing.
           */

#ifdef CONFIG_NET_6LOWPAN_EXTENDEDADDR
          return NET_6LOWPAN_EADDRSIZE;
#else
          return NET_6LOWPAN_SADDRSIZE;
#endif
        }
#endif /* CONFIG_WIRELESS_IEEE802154 */

#if defined(CONFIG_WIRELESS_PKTRADIO) || defined(CONFIG_WIRELESS_BLUETOOTH)
#ifdef CONFIG_WIRELESS_PKTRADIO
      case NET_LL_PKTRADIO:
#endif
#ifdef CONFIG_WIRELESS_BLUETOOTH
      case NET_LL_BLUETOOTH:
#endif
        {
           /* Return the size of the packet radio address */

           return netdev_pktradio_addrlen(dev);
        }
#endif /* CONFIG_WIRELESS_PKTRADIO || CONFIG_WIRELESS_BLUETOOTH */
#endif /* CONFIG_NET_6LOWPAN */

       default:
        {
          /* The link layer type associated has no address */

          return 0;
        }
    }
}
