/****************************************************************************
 * include/nuttx/wireless/ieee802154/ieee802154_radio.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_RADIO_H
#define __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_RADIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

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
  CODE void (*edresult) (FAR const struct ieee802154_radiocb_s *radiocb,
             uint8_t edval);
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
  CODE int (*energydetect) (FAR struct ieee802154_radio_s *radio,
                            uint32_t symboldelay);
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
