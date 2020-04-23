/****************************************************************************
 * wireless/ieee802154/mac802154_data.c
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

#include <nuttx/mm/iob.h>

#include "mac802154.h"
#include "mac802154_internal.h"
#include "mac802154_data.h"

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_req_data
 *
 * Description:
 *   The MCPS-DATA.request primitive requests the transfer of a data SPDU
 *   (i.e., MSDU) from a local SSCS entity to a single peer SSCS entity.
 *   Confirmation is returned via the
 *   struct mac802154_maccb_s->conf_data callback.
 *
 ****************************************************************************/

int mac802154_req_data(MACHANDLE mac,
                       FAR const struct ieee802154_frame_meta_s *meta,
                       FAR struct iob_s *frame, bool allowinterrupt)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  FAR struct ieee802154_txdesc_s *txdesc;
  uint16_t *frame_ctrl;
  uint8_t mhr_len = 3;
  int ret;

  wlinfo("Accepting outbound frame io_len=%u io_offset=%u\n",
         frame->io_len, frame->io_offset);

  /* Check the required frame size */

  if (frame->io_len > IEEE802154_MAX_PHY_PACKET_SIZE)
    {
      return -E2BIG;
    }

  /* Cast the first two bytes of the IOB to a uint16_t frame control field */

  frame_ctrl = (FAR uint16_t *)&frame->io_data[0];

  /* Ensure we start with a clear frame control field */

  *frame_ctrl = 0;

  /* Set the frame type to Data */

  *frame_ctrl |= IEEE802154_FRAME_DATA << IEEE802154_FRAMECTRL_SHIFT_FTYPE;

  /* If the msduLength is greater than aMaxMACSafePayloadSize, the MAC
   * sublayer will set the Frame Version to one. [1] pg. 118.
   */

  if ((frame->io_len - frame->io_offset) >
       IEEE802154_MAX_SAFE_MAC_PAYLOAD_SIZE)
    {
      *frame_ctrl |= (1 << IEEE802154_FRAMECTRL_SHIFT_VERSION);
    }

  /* If the TXOptions parameter specifies that an acknowledged transmission
   * is required, the AR field will be set appropriately, as described in
   * 5.1.6.4 [1] pg. 118.
   */

  *frame_ctrl |= (meta->flags.ackreq << IEEE802154_FRAMECTRL_SHIFT_ACKREQ);

  /* If the destination address is present, copy the PAN ID and one of the
   * addresses, depending on mode, into the MHR.
   */

  if (meta->destaddr.mode != IEEE802154_ADDRMODE_NONE)
    {
      IEEE802154_PANIDCOPY(&frame->io_data[mhr_len], meta->destaddr.panid);
      mhr_len += IEEE802154_PANIDSIZE;

      if (meta->destaddr.mode == IEEE802154_ADDRMODE_SHORT)
        {
          IEEE802154_SADDRCOPY(&frame->io_data[mhr_len],
                               meta->destaddr.saddr);
          mhr_len += IEEE802154_SADDRSIZE;
        }
      else if (meta->destaddr.mode == IEEE802154_ADDRMODE_EXTENDED)
        {
          int index;

          /* The IEEE 802.15.4 Standard is confusing with regards to
           * byte-order for * extended address. More research discovers
           * that the extended address should be sent in reverse-canonical
           * form.
           */

          for (index = IEEE802154_EADDRSIZE - 1; index >= 0; index--)
            {
              frame->io_data[mhr_len++] = meta->destaddr.eaddr[index];
            }
        }
    }

  /* Set the destination addr mode inside the frame control field */

  *frame_ctrl |= (meta->destaddr.mode << IEEE802154_FRAMECTRL_SHIFT_DADDR);

  /* From this point on, we need exclusive access to the privmac struct */

  ret = mac802154_lock(priv, allowinterrupt);
  if (ret < 0)
    {
      /* Should only fail if interrupted by a signal */

      wlwarn("WARNING: mac802154_takesem failed: %d\n", ret);
      return ret;
    }

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
          *frame_ctrl |= IEEE802154_FRAMECTRL_PANIDCOMP;
        }
    }

  if (meta->srcmode != IEEE802154_ADDRMODE_NONE)
    {
      /* If the destination address is not included, or if PAN ID Compression
       * is off, we need to include the Source PAN ID.
       */

      if ((meta->destaddr.mode == IEEE802154_ADDRMODE_NONE) ||
          (!(*frame_ctrl & IEEE802154_FRAMECTRL_PANIDCOMP)))
        {
          IEEE802154_PANIDCOPY(&frame->io_data[mhr_len], priv->addr.panid);
          mhr_len += IEEE802154_PANIDSIZE;
        }

      if (meta->srcmode == IEEE802154_ADDRMODE_SHORT)
        {
          IEEE802154_SADDRCOPY(&frame->io_data[mhr_len], priv->addr.saddr);
          mhr_len += IEEE802154_SADDRSIZE;
        }
      else if (meta->srcmode == IEEE802154_ADDRMODE_EXTENDED)
        {
          int index;

          /* The IEEE 802.15.4 Standard is confusing with regards to
           * byte-order for * extended address. More research discovers
           * that the extended address should be sent in reverse-canonical
           * form.
           */

          for (index = IEEE802154_EADDRSIZE - 1; index >= 0; index--)
            {
              frame->io_data[mhr_len++] = priv->addr.eaddr[index];
            }
        }
    }
  else
    {
      /* If this device is not the PAN coordinator, it shouldn't be sending
       * frames with a source address mode of NONE
       */

      if (priv->devmode != IEEE802154_DEVMODE_PANCOORD)
        {
          ret = -EINVAL;
          goto errout_with_sem;
        }
    }

  /* Set the source addr mode inside the frame control field */

  *frame_ctrl |= (meta->srcmode << IEEE802154_FRAMECTRL_SHIFT_SADDR);

  /* Each time a data or a MAC command frame is generated, the MAC sublayer
   * shall copy the value of macDSN into the Sequence Number field of the MHR
   * of the outgoing frame and then increment it by one. [1] pg. 40.
   */

  frame->io_data[2] = priv->dsn++;

  /* The MAC header we just created must never have exceeded where the app
   * data starts.  This should never happen since the offset should have
   * been set via the same logic to calculate the header length as the logic
   * here that created the header
   */

  wlinfo("mhr_len=%u\n", mhr_len);
  DEBUGASSERT(mhr_len == frame->io_offset);

  /* Allocate the txdesc, waiting if necessary, allow interruptions */

  ret = mac802154_txdesc_alloc(priv, &txdesc, true);
  if (ret < 0)
    {
      /* Should only fail if interrupted by a signal while re-acquiring
       * exclsem.  So the lock is not held if a failure is returned.
       */

      wlwarn("WARNING: mac802154_txdesc_alloc failed: %d\n", ret);
      return ret;
    }

  /* Set the offset to 0 to include the header ( we do not want to
   * modify the frame until AFTER the last place that -EINTR could
   * be returned and could generate a retry.  Subsequent error returns
   * are fatal and no retry should occur.
   */

  frame->io_offset = 0;

  /* Then initialize the TX descriptor */

  txdesc->conf->handle = meta->handle;
  txdesc->frame = frame;
  txdesc->frametype = IEEE802154_FRAME_DATA;
  txdesc->ackreq = meta->flags.ackreq;

  /* If the TxOptions parameter specifies that a GTS transmission is
   * required, the MAC sublayer will determine whether it has a valid GTS as
   * described 5.1.7.3. If a valid GTS could not be found, the MAC sublayer
   * will discard the MSDU. If a valid GTS was found, the MAC sublayer will
   * defer, if necessary, until the GTS. If the TxOptions parameter specifies
   * that a GTS transmission is not required, the MAC sublayer will transmit
   * the MSDU using either slotted CSMA-CA in the CAP for a beacon-enabled
   * PAN or unslotted CSMA-CA for a nonbeacon-enabled PAN. Specifying a GTS
   * transmission in the TxOptions parameter overrides an indirect
   * transmission request. [1] pg. 118.
   */

  if (meta->flags.usegts)
    {
      /* TODO:
       * Support GTS transmission. This should just change where we link
       * the transaction.  Instead of going in the CSMA transaction list, it
       * should be linked to the GTS' transaction list. We'll need to check
       * if the GTS is valid, and then find the GTS, before linking.
       * Note, we also don't have to try and kick-off any transmission here.
       */

      ret = -ENOTSUP;
      goto errout_with_txdesc;
    }
  else
    {
      /* If the TxOptions parameter specifies that an indirect transmission
       * is required and this primitive is received by the MAC sublayer of a
       * coordinator, the data frame is sent using indirect transmission, as
       * described in 5.1.5 and 5.1.6.3. [1]
       */

      if (meta->flags.indirect)
        {
          /* If the TxOptions parameter specifies that an indirect
           * transmission is required and if the device receiving this
           * primitive is not a coordinator, the destination address is not
           * present, or the TxOptions parameter also specifies a
           * GTS transmission, the indirect transmission option will be
           * ignored. [1]
           *
           * NOTE: We don't just ignore the parameter.  Instead, we throw an
           * error, since this really shouldn't be happening.
           */

          if (priv->devmode >= IEEE802154_DEVMODE_COORD &&
              meta->destaddr.mode != IEEE802154_ADDRMODE_NONE)
            {
              /* Copy in a reference to the destination address to assist in
               * searching when data is requested.
               */

              memcpy(&txdesc->destaddr, &meta->destaddr,
                     sizeof(struct ieee802154_addr_s));
              mac802154_setupindirect(priv, txdesc);
              mac802154_unlock(priv)
            }
          else
            {
              ret = -EINVAL;
              goto errout_with_txdesc;
            }
        }
      else
        {
          /* Link the transaction into the CSMA transaction list */

          sq_addlast((FAR sq_entry_t *)txdesc, &priv->csma_queue);

          /* We no longer need to have the MAC layer locked. */

          mac802154_unlock(priv)

          /* Notify the radio driver that there is data available */

          priv->radio->txnotify(priv->radio, false);
        }
    }

  return OK;

errout_with_txdesc:

  /* Free TX the descriptor, but preserve the IOB. */

  txdesc->frame = NULL;
  mac802154_txdesc_free(priv, txdesc);

errout_with_sem:
  mac802154_unlock(priv)
  return ret;
}

/****************************************************************************
 * Internal MAC Functions
 ****************************************************************************/
