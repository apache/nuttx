/****************************************************************************
 * include/nuttx/wireless/ieee802154/ieee802154_radio.h
 *
 *   Copyright (C) 2014-2016 Sebastien Lorquet. All rights reserved.
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
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

#ifndef __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_RADIO_H
#define __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_RADIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Data only used between radio and MAC layer */

struct ieee802154_txdesc_s
{
  /* Support a singly linked list of tx descriptors */

  FAR struct ieee802154_txdesc_s *flink;

  /* Destination Address */

  struct ieee802154_addr_s destaddr; /* Only used for indirect transactions */

  /* Pointer to the frame IOB */

  FAR struct iob_s *frame;

  /* Pointer to the data confirmation structure to be populated upon
   * success/failure of the transmission.
   */

  FAR struct ieee802154_data_conf_s *conf;

  /* Frame type.  Used by MAC layer to control how tx done is handled */

  enum ieee802154_frametype_e frametype;

  bool ackreq;          /* Are we requesting an ACK? */
  bool framepending;    /* Did the ACK have the frame pending bit set */
  uint32_t purgetime;   /* Time to purge transaction */
  uint8_t  retrycount;  /* Number of remaining retries. Set to macMaxFrameRetries
                         * when txdescriptor is allocated
                         */

  /* TODO: Add slotting information for GTS transactions */
};

struct ieee802154_beaconframe_s
{
  uint8_t bf_data[IEEE802154_MAX_PHY_PACKET_SIZE];
  uint8_t bf_len;
  uint8_t bf_offset;
};

enum ieee802154_sfevent_e
{
  IEEE802154_SFEVENT_ENDOFACTIVE,
};

/* IEEE802.15.4 Radio Interface Operations **********************************/

struct ieee802154_radiocb_s
{
  CODE int (*poll) (FAR const struct ieee802154_radiocb_s *radiocb,
             bool gts, FAR struct ieee802154_txdesc_s **tx_desc);
  CODE void (*txdone) (FAR const struct ieee802154_radiocb_s *radiocb,
             FAR struct ieee802154_txdesc_s *tx_desc);
  CODE void (*rxframe) (FAR const struct ieee802154_radiocb_s *radiocb,
             FAR struct ieee802154_data_ind_s *ind);
  CODE void (*sfevent) (FAR const struct ieee802154_radiocb_s *radiocb,
             enum ieee802154_sfevent_e sfevent);
};

struct ieee802154_radio_s
{
  CODE int (*bind) (FAR struct ieee802154_radio_s *radio,
             FAR struct ieee802154_radiocb_s *radiocb);
  CODE int (*reset) (FAR struct ieee802154_radio_s *radio);
  CODE int (*getattr) (FAR struct ieee802154_radio_s *radio,
             enum ieee802154_attr_e ,
             FAR union ieee802154_attr_u *attrval);
  CODE int (*setattr) (FAR struct ieee802154_radio_s *radio,
             enum ieee802154_attr_e ,
             FAR const union ieee802154_attr_u *attrval);
  CODE int (*txnotify)(FAR struct ieee802154_radio_s *radio, bool gts);
  CODE int (*txdelayed)(FAR struct ieee802154_radio_s *radio,
             FAR struct ieee802154_txdesc_s *txdesc,
             uint32_t symboldelay);
  CODE int (*rxenable) (FAR struct ieee802154_radio_s *radio, bool enable);
  CODE int (*beaconstart)(FAR struct ieee802154_radio_s *radio,
             FAR const struct ieee802154_superframespec_s *sfspec,
             FAR struct ieee802154_beaconframe_s *beacon);
  CODE int (*beaconupdate)(FAR struct ieee802154_radio_s *radio,
             FAR struct ieee802154_beaconframe_s *beacon);
  CODE int (*beaconstop)(FAR struct ieee802154_radio_s *radio);
  CODE int (*sfupdate)(FAR struct ieee802154_radio_s *radio,
             FAR const struct ieee802154_superframespec_s *sfspec);
};

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_RADIO_H */
