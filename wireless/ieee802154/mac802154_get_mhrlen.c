/****************************************************************************
 * wireless/ieee802154/mac802154_get_mhrlen.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Author: Anthony Merlino <anthony@vergeaero.com>
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

#include <stdlib.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <string.h>

#include "mac802154.h"
#include "mac802154_internal.h"

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Public MAC Functions
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

  /* Check to make sure both the dest address and the source address are not set
   * to NONE */

  if (meta->destaddr.mode == IEEE802154_ADDRMODE_NONE &&
      meta->srcmode == IEEE802154_ADDRMODE_NONE)
    {
      return -EINVAL;
    }

  /* The source address can only be set to NONE if the device is the PAN coord */

  if (meta->srcmode == IEEE802154_ADDRMODE_NONE &&
      priv->devmode != IEEE802154_DEVMODE_PANCOORD)
    {
      return -EINVAL;
    }

  /* Add the destination address length */

  ret += mac802154_addr_length[meta->destaddr.mode];

  /* Add the source address length */

  ret += mac802154_addr_length[ meta->srcmode];

  /* If both destination and source addressing information is present, the MAC
   * sublayer shall compare the destination and source PAN identifiers.
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

  /* If we are here, PAN ID compression is off, so include the dest and source
   * PAN ID if the respective address is included
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
