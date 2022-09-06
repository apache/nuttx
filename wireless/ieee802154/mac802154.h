/****************************************************************************
 * wireless/ieee802154/mac802154.h
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

#ifndef __WIRELESS_IEEE802154__MAC802154_H
#define __WIRELESS_IEEE802154__MAC802154_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* Callback operations to notify the next highest layer of various
 * asynchronous events, usually triggered by some previous request or
 * response invoked by the upper layer.
 */

struct mac802154_maccb_s
{
  FAR struct mac802154_maccb_s *flink;  /* Implements a singly linked list */
  uint8_t prio;                         /* RX frame callback priority */

  /* Callback for various MLME or MCPS service events.
   * Return value represents whether the callback accepts the primitive.
   * >= 0 means the callback has accepted the primitive and is responsible
   * for calling ieee802154_primitive_free(). In the case of DATA.indication
   * primitive, only one callback can accept the frame. The callbacks are
   * stored in order of receiver priority defined by the 'prio' field above.
   * All other notifications are offered to all callbacks and all can accept
   * and free separately since the primitive will not be freed until the
   * nclients count reaches 0.
   */

  CODE int (*notify)(FAR struct mac802154_maccb_s *maccb,
                     FAR struct ieee802154_primitive_s *primitive);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct iob_s;  /* Forward reference */

/****************************************************************************
 * Name: mac802154_bind
 *
 * Description:
 *   Bind the MAC callback table to the MAC state.
 *
 * Input Parameters:
 *   mac - Reference to the MAC driver state structure
 *   cb  - MAC callback operations
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int mac802154_bind(MACHANDLE mac, FAR struct mac802154_maccb_s *cb);

/****************************************************************************
 * Name: mac802154_ioctl
 *
 * Description:
 *   Handle MAC and radio IOCTL commands directed to the MAC.
 *
 * Input Parameters:
 *   mac - Reference to the MAC driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int mac802154_ioctl(MACHANDLE mac, int cmd, unsigned long arg);

/****************************************************************************
 * MAC Interface Operations
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_get_mhrlen
 *
 * Description:
 *   Calculate the MAC header length given the frame meta-data.
 *
 ****************************************************************************/

int mac802154_get_mhrlen(MACHANDLE mac,
                         FAR const struct ieee802154_frame_meta_s *meta);

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
                       FAR struct iob_s *frame);

/****************************************************************************
 * Name: mac802154_req_purge
 *
 * Description:
 *   The MCPS-PURGE.request primitive allows the next higher layer to purge
 *   an MSDU from the transaction queue. Confirmation is returned via
 *   the struct mac802154_maccb_s->conf_purge callback.
 *
 *   NOTE: The standard specifies that confirmation should be indicated via
 *   the asynchronous MLME-PURGE.confirm primitive.  However, in our
 *   implementation we synchronously return the status from the request.
 *   Therefore, we merge the functionality of the MLME-PURGE.request and
 *   MLME-PURGE.confirm primitives together.
 *
 ****************************************************************************/

int mac802154_req_purge(MACHANDLE mac, uint8_t msdu_handle);

/****************************************************************************
 * Name: mac802154_req_associate
 *
 * Description:
 *   The MLME-ASSOCIATE.request primitive allows a device to request an
 *   association with a coordinator. Confirmation is returned via the
 *   struct mac802154_maccb_s->conf_associate callback.
 *
 ****************************************************************************/

int mac802154_req_associate(MACHANDLE mac,
                            FAR struct ieee802154_assoc_req_s *req);

/****************************************************************************
 * Name: mac802154_req_disassociate
 *
 * Description:
 *   The MLME-DISASSOCIATE.request primitive is used by an associated device
 *   to notify the coordinator of its intent to leave the PAN. It is also
 *   used by the coordinator to instruct an associated device to leave the
 *   PAN.
 *
 *   Confirmation is returned via the
 *   struct mac802154_maccb_s->conf_disassociate callback.
 *
 ****************************************************************************/

int mac802154_req_disassociate(MACHANDLE mac,
                               FAR struct ieee802154_disassoc_req_s *req);

/****************************************************************************
 * Name: mac802154_req_gts
 *
 * Description:
 *   The MLME-GTS.request primitive allows a device to send a request to the
 *   PAN coordinator to allocate a new GTS or to deallocate an existing GTS.
 *   Confirmation is returned via the
 *   struct mac802154_maccb_s->conf_gts callback.
 *
 ****************************************************************************/

int mac802154_req_gts(MACHANDLE mac, FAR struct ieee802154_gts_req_s *req);

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
 *   mac         - Handle to the MAC layer instance
 *   reset_attr  - Whether or not to reset the MAC PIB attributes to defaults
 *
 ****************************************************************************/

int mac802154_req_reset(MACHANDLE mac, bool restattr);

/****************************************************************************
 * Name: mac802154_req_rxenable
 *
 * Description:
 *   The MLME-RX-ENABLE.request primitive allows the next higher layer to
 *   request that the receiver is enable for a finite period of time.
 *   Confirmation is returned via the
 *   struct mac802154_maccb_s->conf_rxenable callback.
 *
 ****************************************************************************/

int mac802154_req_rxenable(MACHANDLE mac,
                           FAR struct ieee802154_rxenable_req_s *req);

/****************************************************************************
 * Name: mac802154_req_scan
 *
 * Description:
 *   The MLME-SCAN.request primitive is used to initiate a channel scan over
 *   a given list of channels. A device can use a channel scan to measure
 *   the energy on the channel, search for the coordinator with which it
 *   associated, or search for all coordinators transmitting beacon frames
 *   within the POS of the scanning device. Scan results are returned
 *   via MULTIPLE calls to the struct mac802154_maccb_s->conf_scan
 *   callback.  This is a difference with the official 802.15.4
 *   specification, implemented here to save memory.
 *
 ****************************************************************************/

int mac802154_req_scan(MACHANDLE mac, FAR struct ieee802154_scan_req_s *req);

/****************************************************************************
 * Name: mac802154_req_get
 *
 * Description:
 *   The MLME-GET.request primitive requests information about a given PIB
 *   attribute.
 *
 *   NOTE: The standard specifies that the attribute value should be returned
 *   via the asynchronous MLME-GET.confirm primitive.  However, in our
 *   implementation, we synchronously return the value immediately.Therefore,
 *   we merge the functionality of the MLME-GET.request and MLME-GET.confirm
 *   primitives together.
 *
 ****************************************************************************/

int mac802154_req_get(MACHANDLE mac, enum ieee802154_attr_e ,
                      FAR union ieee802154_attr_u *attrval);

/****************************************************************************
 * Name: mac802154_req_set
 *
 * Description:
 *   The MLME-SET.request primitive attempts to write the given value to the
 *   indicated MAC PIB attribute.
 *
 *   NOTE: The standard specifies that confirmation should be indicated via
 *   the asynchronous MLME-SET.confirm primitive.
 *   However, in our implementation we synchronously return the status from
 *   the request.
 *   Therefore, we do merge the functionality of the MLME-SET.request and
 *   MLME-SET.confirm primitives together.
 *
 ****************************************************************************/

int mac802154_req_set(MACHANDLE mac, enum ieee802154_attr_e ,
                      FAR const union ieee802154_attr_u *attrval);

/****************************************************************************
 * Name: mac802154_req_start
 *
 * Description:
 *   The MLME-START.request primitive makes a request for the device to
 *   start using a new superframe configuration. Confirmation is returned
 *   via the struct mac802154_maccb_s->conf_start callback.
 *
 ****************************************************************************/

int mac802154_req_start(MACHANDLE mac,
                        FAR struct ieee802154_start_req_s *req);

/****************************************************************************
 * Name: mac802154_req_sync
 *
 * Description:
 *   The MLME-SYNC.request primitive requests to synchronize with the
 *   coordinator by acquiring and, if specified, tracking its beacons.
 *   Confirmation is returned via the
 *   struct mac802154_maccb_s->int_commstatus callback. TOCHECK.
 *
 ****************************************************************************/

int mac802154_req_sync(MACHANDLE mac, FAR struct ieee802154_sync_req_s *req);

/****************************************************************************
 * Name: mac802154_req_poll
 *
 * Description:
 *   The MLME-POLL.request primitive prompts the device to request data from
 *   the coordinator. Confirmation is returned via the
 *   struct mac802154_maccb_s->conf_poll callback, followed by a
 *   struct mac802154_maccb_s->ind_data callback.
 *
 ****************************************************************************/

int mac802154_req_poll(MACHANDLE mac, FAR struct ieee802154_poll_req_s *req);

/****************************************************************************
 * Name: mac802154_resp_associate
 *
 * Description:
 *   The MLME-ASSOCIATE.response primitive is used to initiate a response to
 *   an MLME-ASSOCIATE.indication primitive.
 *
 ****************************************************************************/

int mac802154_resp_associate(MACHANDLE mac,
                             FAR struct ieee802154_assoc_resp_s *resp);

/****************************************************************************
 * Name: mac802154_resp_orphan
 *
 * Description:
 *   The MLME-ORPHAN.response primitive allows the next higher layer of a
 *   coordinator to respond to the MLME-ORPHAN.indication primitive.
 *
 ****************************************************************************/

int mac802154_resp_orphan(MACHANDLE mac,
                          FAR struct ieee802154_orphan_resp_s *resp);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __WIRELESS_IEEE802154__MAC802154_H */
