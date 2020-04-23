/****************************************************************************
 * wireless/ieee802154/mac802154_reset.c
 *
 *   Copyright (C) 2016 Sebastien Lorquet. All rights reserved.
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_req_reset
 *
 * Description:
 *   The MLME-RESET.request primitive allows the next higher layer to request
 *   that the MLME performs a reset operation.
 *
 *   NOTE: The standard specifies that confirmation should be provided via
 *   via the asynchronous MLME-RESET.confirm primitive.  However, in our
 *   implementation we synchronously return the value immediately.
 *   Therefore, we merge the functionality of the MLME-RESET.request and
 *   MLME-RESET.confirm primitives together.
 *
 * Input Parameters:
 *   mac       - Handle to the MAC layer instance
 *   resetattr - Whether or not to reset the MAC PIB attributes to defaults
 *
 ****************************************************************************/

int mac802154_req_reset(MACHANDLE mac, bool resetattr)
{
  FAR struct ieee802154_privmac_s * priv =
    (FAR struct ieee802154_privmac_s *) mac;
  union ieee802154_attr_u attr;

  if (resetattr)
    {
      /* We do not reset the extended address. The extended address must be
       * manually overwritten.
       */

      priv->radio->reset(priv->radio);

      priv->isassoc = false;            /* Not associated with a PAN */
      priv->trackingbeacon = false;     /* Not tracking beacon by default */
      priv->sfspec.assocpermit = false; /* Dev (if coord) not accepting assoc */
      priv->autoreq = true;             /* Auto send data req if addr in beacon */
      priv->sfspec.ble = false;         /* BLE disabled */
      priv->beaconpayloadlength = 0;    /* Beacon payload NULL */
      priv->sfspec.beaconorder = 15;    /* Non-beacon enabled network */
      priv->sfspec.sforder = 15;        /* Length of active portion of outgoing SF */
      priv->beacon_txtime = 0;          /* Device never sent a beacon */
      priv->dsn = 0;                    /* Data sequence number */
      priv->gtspermit = true;           /* PAN Coord accepting GTS requests */
      priv->minbe = 3;                  /* Min value of backoff exponent (BE) */
      priv->maxbe = 5;                  /* Max value of backoff exponent (BE) */
      priv->max_csmabackoffs = 4;       /* Max # of backoffs before failure */
      priv->maxretries = 3;             /* Max # of retries allowed after failure */
      priv->promisc = false;            /* Device not in promiscuous mode */
      priv->rngsupport = false;         /* Ranging not yet supported */
      priv->resp_waittime = 32;         /* 32 SF durations */
      priv->sec_enabled = false;        /* Security disabled by default */
      priv->tx_totaldur = 0;            /* 0 transmit duration */

      priv->trans_persisttime = 0x01f4;

      /* Reset the short address and PAN ID. The extended address does not
       * get reset. It is a read-only attribute and the radio driver should
       * be in charge of managing it. We pull a local copy for us to use
       * below.
       */

      priv->addr.mode = IEEE802154_ADDRMODE_EXTENDED;
      mac802154_setsaddr(priv, IEEE802154_SADDR_UNSPEC);
      mac802154_setpanid(priv, IEEE802154_PANID_UNSPEC);

      priv->pandesc.coordaddr.mode = IEEE802154_ADDRMODE_NONE;
      mac802154_setcoordeaddr(priv, IEEE802154_EADDR_UNSPEC);
      mac802154_setcoordsaddr(priv, IEEE802154_SADDR_UNSPEC);

      mac802154_setdevmode(priv, IEEE802154_DEVMODE_ENDPOINT);

      /* The radio is in control of certain attributes, but we keep a mirror
       * for easy access.  Copy in the radio's values now that they've been
       * reset.
       */

      priv->radio->getattr(priv->radio,
                           IEEE802154_ATTR_MAC_EADDR, &attr);
      IEEE802154_EADDRCOPY(priv->addr.eaddr,
                           attr.mac.eaddr);

      priv->radio->getattr(priv->radio,
                           IEEE802154_ATTR_MAC_MAX_FRAME_WAITTIME,
                           &attr);
      priv->max_frame_waittime = attr.mac.max_frame_waittime;
    }

  priv->nrxusers = 0;

  return OK;
}
