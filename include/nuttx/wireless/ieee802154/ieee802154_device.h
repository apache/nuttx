/****************************************************************************
 * include/nuttx/wireless/ieee802154/ieee802154_device.h
 * IEEE802.15.4 character driver
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

#ifndef __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_DEVICE_H
#define __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_DEVICE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/wireless/ioctl.h>

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#ifdef CONFIG_WIRELESS_IEEE802154

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct mac802154dev_txframe_s
{
  struct ieee802154_frame_meta_s meta;
  FAR uint8_t *payload;
  uint16_t length;
};

struct mac802154dev_rxframe_s
{
  struct ieee802154_data_ind_s meta;
  uint8_t payload[IEEE802154_MAX_PHY_PACKET_SIZE];
  uint8_t length;

  /* In promiscuous mode, the entire frame is passed to the application
   * inside the payload field. The offset field is used to specify the start
   * of the actual payload, skipping the 802.15.4 header.
   */

  uint8_t offset;
};

#endif /* CONFIG_WIRELESS_IEEE802154 */
#endif /* __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_DEVICE_H */
