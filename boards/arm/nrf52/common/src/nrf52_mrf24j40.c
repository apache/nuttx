/****************************************************************************
 * boards/arm/nrf52/common/src/nrf52_mrf24j40.c
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

#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>
#include <nuttx/wireless/ieee802154/mrf24j40.h>

#include "nrf52_gpio.h"
#include "nrf52_spi.h"

#include "nrf52_mrf24j40.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_DRIVERS_WIRELESS
#  error Wireless support requires CONFIG_DRIVERS_WIRELESS
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_mrf24j40_devsetup
 *
 * Description:
 *   Initialize one the MRF24J40 device in one mikroBUS slot
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int nrf52_mrf24j40_devsetup(struct nrf52_mrf24j40_s *priv)
{
  struct ieee802154_radio_s *radio;
  MACHANDLE                  mac;
  struct spi_dev_s          *spi;
  int                        ret;

  /* Configure pins */

  nrf52_gpio_config(priv->intcfg);

  /* Initialize the SPI bus and get an instance of the SPI interface */

  spi = nrf52_spibus_initialize(priv->spidev);
  if (spi == NULL)
    {
      wlerr("Failed to initialize SPI bus %d\n", priv->spidev);
      return -ENODEV;
    }

  /* Initialize and register the SPI MRF24J40 device */

  radio = mrf24j40_init(spi, &priv->dev);
  if (radio == NULL)
    {
      wlerr("Failed to initialize SPI bus %d\n", priv->spidev);
      return -ENODEV;
    }

  /* Create a 802.15.4 MAC device from a 802.15.4 compatible radio device. */

  mac = mac802154_create(radio);
  if (mac == NULL)
    {
      wlerr("Failed to initialize IEEE802.15.4 MAC\n");
      return -ENODEV;
    }

#ifdef CONFIG_IEEE802154_NETDEV
  /* Use the IEEE802.15.4 MAC interface instance to create a 6LoWPAN
   * network interface by wrapping the MAC intrface instance in a
   * network device driver via mac802154dev_register().
   */

  ret = mac802154netdev_register(mac);
  if (ret < 0)
    {
      wlerr("Failed to register the MAC network driver wpan%d: %d\n",
            0, ret);
      return ret;
    }
#endif

#ifdef CONFIG_IEEE802154_MACDEV
  /* If want to call these APIs from userspace, you have to wrap the MAC
   * interface in a character device viamac802154dev_register().
   */

  ret = mac802154dev_register(mac, 0);
  if (ret < 0)
    {
      wlerr("Failed to register the MAC character driver /dev/ieee%d: %d\n",
            0, ret);
      return ret;
    }
#endif

  UNUSED(ret);
  return OK;
}
