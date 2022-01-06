/****************************************************************************
 * wireless/ieee802154/mac802154_get_mhrlen.c
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

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include "mac802154.h"
#include "mac802154_internal.h"

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Map between ieee802154_addrmode_e enum and actual address length */

static const uint8_t mac802154_addr_length[4] =
{
  0, 0, 2, 8
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_get_mhrlen
 *
 * Description:
 *   Calculate the MAC header length given the frame meta-data.
 *
 ****************************************************************************/

int mac802154_get_mhrlen(MACHANDLE mac,
                         FAR const struct ieee802154_frame_meta_s *meta)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  int ret = 3; /* Always frame control (2 bytes) and seq. num (1 byte) */

  /* Check to make sure both the dest address and the source address are not
   * set to NONE
   */

  if (meta->destaddr.mode == IEEE802154_ADDRMODE_NONE &&
      meta->srcmode == IEEE802154_ADDRMODE_NONE)
    {
      return -EINVAL;
    }

  /* The source address can only be set to NONE
   * if the device is the PAN coord
   */

  if (meta->srcmode == IEEE802154_ADDRMODE_NONE &&
      priv->devmode != IEEE802154_DEVMODE_PANCOORD)
    {
      return -EINVAL;
    }

  /* Add the destination address length */

  ret += mac802154_addr_length[meta->destaddr.mode];

  /* Add the source address length */

  ret += mac802154_addr_length[meta->srcmode];

  /* If both destination and source addressing information is present, the
   * MAC sublayer shall compare the destination and source PAN identifiers.
   * [1] pg. 41.
   */

  if (meta->srcmode  != IEEE802154_ADDRMODE_NONE &&
      meta->destaddr.mode != IEEE802154_ADDRMODE_NONE)
    {
      /* If the PAN identifiers are identical, the PAN ID Compression field
       * shall be set to one, and the source PAN identifier shall be omitted
       * from the transmitted frame. [1] pg. 41.
       */

      if (IEEE802154_PANIDCMP(meta->destaddr.panid, priv->addr.panid))
        {
          ret += 2; /* 2 bytes for destination PAN ID */
          return ret;
        }
    }

  /* If we are here, PAN ID compression is off, so include the dest and
   * source PAN ID if the respective address is included
   */

  if (meta->srcmode != IEEE802154_ADDRMODE_NONE)
    {
      ret += 2; /* 2 bytes for source PAN ID */
    }

  if (meta->destaddr.mode != IEEE802154_ADDRMODE_NONE)
    {
      ret += 2; /* 2 bytes for destination PAN ID */
    }

  return ret;
}
