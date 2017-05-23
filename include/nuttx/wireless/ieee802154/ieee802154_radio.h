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

  /* Pointer to the data confirmation structure to be populated upon
   * success/failure of the transmission.
   */

  FAR struct ieee802154_data_conf_s *conf;

  enum ieee802154_frametype_e frametype; /* Frame type.  Used by MAC layer to
                                          * control how tx done is handled */

  /* TODO: Add slotting information for GTS transactions */
};

/* IEEE802.15.4 Radio Interface Operations **********************************/

struct ieee802154_radiocb_s
{
  CODE int (*poll_csma) (FAR const struct ieee802154_radiocb_s *radiocb,
             FAR struct ieee802154_txdesc_s **tx_desc,
             FAR struct iob_s **frame);
  CODE int (*poll_gts) (FAR const struct ieee802154_radiocb_s *radiocb,
             FAR struct ieee802154_txdesc_s **tx_desc,
             FAR struct iob_s **frame);
  CODE void (*txdone) (FAR const struct ieee802154_radiocb_s *radiocb,
             FAR const struct ieee802154_txdesc_s *tx_desc);
  CODE void (*rxframe) (FAR const struct ieee802154_radiocb_s *radiocb,
             FAR struct ieee802154_data_ind_s *ind);
};

struct ieee802154_radio_s; /* Forward reference */

struct ieee802154_radioops_s
{
  CODE int (*bind) (FAR struct ieee802154_radio_s *radio,
             FAR struct ieee802154_radiocb_s *radiocb);
  CODE int (*txnotify_csma)(FAR struct ieee802154_radio_s *radio);
  CODE int (*txnotify_gts)(FAR struct ieee802154_radio_s *radio);
  CODE int (*get_attr) (FAR struct ieee802154_radio_s *radio,
             enum ieee802154_pib_attr_e pib_attr,
             FAR union ieee802154_attr_u *attrval);
  CODE int (*set_attr) (FAR struct ieee802154_radio_s *radio,
             enum ieee802154_pib_attr_e pib_attr,
             FAR const union ieee802154_attr_u *attrval);
};

struct ieee802154_radio_s
{
  FAR const struct ieee802154_radioops_s *ops;
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
