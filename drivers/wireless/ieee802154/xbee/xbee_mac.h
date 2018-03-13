/****************************************************************************
 * drivers/wireless/ieee802154/xbee/xbee_mac.h
 *
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *   Author:  Anthony Merlino <anthony@vergeaero.com>
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

#ifndef __DRIVERS_WIRELESS_IEEE802154_XBEE_MAC_H
#define __DRIVERS_WIRELESS_IEEE802154_XBEE_MAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

#include <nuttx/wqueue.h>
#include <nuttx/spi/spi.h>

#include <nuttx/wireless/ieee802154/xbee.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Callback operations to notify the next highest layer of various
 * asynchronous events, usually triggered by some previous request or
 * response invoked by the upper layer.
 */

struct xbee_maccb_s
{
  FAR struct xbee_maccb_s *flink;  /* Implements a singly linked list */
  uint8_t prio;                    /* RX frame callback priority */

  /* Callback for various MLME or MCPS service events.  Return value represents
   * whether the callback accepts the primitive. >= 0 means the callback has
   * accepted the primitive and is responsible for calling
   * ieee802154_primitive_free(). In the case of DATA.indication primitive, only
   * one callback can accept the frame. The callbacks are stored in order of
   * receiver priority defined by the 'prio' field above. All other
   * notifications are offered to all callbacks and all can accept and free
   * separately since the primitive will not be freed until the nclients count
   * reaches 0. */

  CODE int (*notify)(FAR struct xbee_maccb_s *maccb,
                     FAR struct ieee802154_primitive_s *primitive);
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

 /****************************************************************************
 * Name: xbee_bind
 *
 * Description:
 *   Bind the XBee callback table to the MAC state.
 *
 * Input Parameters:
 *   xbee - Reference to the XBee driver state structure
 *   cb  -  XBee callback operations
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int xbee_bind(XBEEHANDLE xbee, FAR struct xbee_maccb_s *cb);

/****************************************************************************
 * Name: xbee_ioctl
 *
 * Description:
 *   Handle MAC and radio IOCTL commands directed to the XBee device.
 *
 * Input Parameters:
 *   mac - Reference to the Xbee driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int xbee_ioctl(XBEEHANDLE xbee, int cmd, unsigned long arg);

/****************************************************************************
 * MAC Interface Operations
 ****************************************************************************/

/****************************************************************************
 * Name: xbee_get_mhrlen
 *
 * Description:
 *   Calculate the MAC header length given the frame meta-data. For the XBee,
 *   we use the header to store the entire API frame for the TX request. The
 *   size we need is fixed based on the address mode we are using as it changes
 *   which API frame we need to issue.
 *
 ****************************************************************************/

int xbee_get_mhrlen(XBEEHANDLE xbee, FAR const struct ieee802154_frame_meta_s *meta);

/****************************************************************************
 * Name: xbee_req_data
 *
 * Description:
 *   The MCPS-DATA.request primitive requests the transfer of a data SPDU
 *   (i.e., MSDU) from a local SSCS entity to a single peer SSCS entity.
 *   Confirmation is returned via the
 *   struct xbee_maccb_s->conf_data callback.
 *
 ****************************************************************************/

int xbee_req_data(XBEEHANDLE xbee, FAR const struct ieee802154_frame_meta_s *meta,
                  FAR struct iob_s *frame);

/****************************************************************************
 * Name: xbee_req_get
 *
 * Description:
 *   The MLME-GET.request primitive requests information about a given PIB
 *   attribute.
 *
 *   NOTE: The standard specifies that the attribute value should be returned
 *   via the asynchronous MLME-GET.confirm primitve.  However, in our
 *   implementation, we synchronously return the value immediately.Therefore, we
 *   merge the functionality of the MLME-GET.request and MLME-GET.confirm
 *   primitives together.
 *
 ****************************************************************************/

int xbee_req_get(XBEEHANDLE xbee, enum ieee802154_attr_e attr,
                 FAR union ieee802154_attr_u *attrval);

/****************************************************************************
 * Name: xbee_req_set
 *
 * Description:
 *   The MLME-SET.request primitive attempts to write the given value to the
 *   indicated MAC PIB attribute.
 *
 *   NOTE: The standard specifies that confirmation should be indicated via
 *   the asynchronous MLME-SET.confirm primitve.  However, in our implementation
 *   we synchronously return the status from the request. Therefore, we do merge
 *   the functionality of the MLME-SET.request and MLME-SET.confirm primitives
 *   together.
 *
 ****************************************************************************/

int xbee_req_set(XBEEHANDLE xbee, enum ieee802154_attr_e attr,
                 FAR const union ieee802154_attr_u *attrval);

/****************************************************************************
 * Name: xbee_req_start
 *
 * Description:
 *   The MLME-START.request primitive makes a request for the device to start
 *   acting as a coordinator.  The XBee modules do not support beacon-enabled
 *   networking!
 *
 ****************************************************************************/

int xbee_req_start(XBEEHANDLE xbee, FAR struct ieee802154_start_req_s *req);

/****************************************************************************
 * Name: xbee_req_associate
 *
 * Description:
 *   The MLME-ASSOCIATE.request primitive allows a device to request an
 *   association with a coordinator.
 *
 *   On receipt of the MLME-ASSOCIATE.request primitive, the MLME of an
 *   unassociated device first updates the appropriate PHY and MAC PIB
 *   attributes, as described in 5.1.3.1, and then generates an association
 *   request command, as defined in 5.3.1 [1] pg.80
 *
 ****************************************************************************/

int xbee_req_associate(XBEEHANDLE xbee, FAR struct ieee802154_assoc_req_s *req);

/****************************************************************************
 * Name: xbee_req_reset
 *
 * Description:
 *   The MLME-RESET.request primitive allows the next higher layer to request
 *   that the MLME performs a reset operation.
 *
 * Input Parameters:
 *   xbee         - Handle to the XBee instance
 *   resetattr    - Whether or not to reset the MAC PIB attributes to defaults
 *
 ****************************************************************************/

int xbee_req_reset(XBEEHANDLE xbee, bool resetattr);

#endif /* __DRIVERS_WIRELESS_IEEE802154_XBEE_MAC_H */
