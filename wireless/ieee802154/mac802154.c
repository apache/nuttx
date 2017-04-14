/****************************************************************************
 * wireless/ieee802154/mac802154.c
 *
 *   Copyright (C) 2016 Sebastien Lorquet. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
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

#include <nuttx/kmalloc.h>
#include <nuttx/wireless/ieee802154/ieee802154_radio.h>
#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

#include "mac802154.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The privmac structure holds the internal state of the MAC and is the
 * underlying represention of the opaque MACHANDLE.  It contains storage for
 * the IEEE802.15.4 MIB attributes.
 */

struct ieee802154_privmac_s
{
  FAR struct ieee802154_radio_s *radio;     /* Contained IEEE802.15.4 radio dev */
  FAR const struct ieee802154_maccb_s *cb;  /* Contained MAC callbacks */

  /* MIB attributes, grouped to save memory */
  /* 0x40 */ uint8_t  macAckWaitDuration    : 1; /* 55 or 120(true) */
  /* 0x41 */ uint8_t  macAssociationPermit  : 1;
  /* 0x42 */ uint8_t  macAutoRequest        : 1;
  /* 0x43 */ uint8_t  macBattLifeExt        : 1;
  /* 0x44 */ uint8_t  macBattLifeExtPeriods : 1; /* 6 or 8(true) */
  /* 0x4E */ uint8_t  macMaxCSMABackoffs    : 3; /* 0-5 */

  /* 0x47 */ uint8_t  macBeaconOrder     : 4;
  /* 0x54 */ uint8_t  macSuperframeOrder : 4;

  /* 0x4F */ uint32_t macMinBE           : 2;
  /* 0x4D */ uint32_t macGTSPermit       : 1;
  /* 0x51 */ uint32_t macPromiscuousMode : 1;
  /* 0x52 */ uint32_t macRxOnWhenIdle    : 1;
             uint32_t macPad             : 3;
  /* 0x48 */ uint32_t macBeaconTxTime    : 24;

  /* 0x45 */ uint8_t  macBeaconPayload[IEEE802154_MAX_BEACON_PAYLOAD_LENGTH];
  /* 0x46 */ uint8_t  macBeaconPayloadLength;
  /* 0x49 */ uint8_t  macBSN;
  /* 0x4A */ uint8_t  macCoordExtendedAddress[8];
  /* 0x4B */ uint16_t macCoordShortAddress;
  /* 0x4C */ uint8_t  macDSN;
  /* 0x50 */ uint16_t macPANId;
  /* 0x53 */ uint16_t macShortAddress;
  /* 0x55 */ uint16_t macTransactionPersistenceTime;
#if 0
  /* Security MIB */
  /* 0x70 */ macACLEntryDescriptorSet
  /* 0x71 */ macACLEntryDescriptorSetSize
  /* 0x74 */ macDefaultSecurityMaterial
  /* 0x72 */ macDefaultSecurity:1
  /* 0x75 */ macDefaultSecuritySuite:3
  /* 0x73 */ macDefaultSecurityMaterialLength:6
  /* 0x76 */ macSecurityMode:2
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_defaultmib
 *
 * Description:
 *   Set the MIB to its default values.
 *
 ****************************************************************************/

static int mac802154_defaultmib(FAR struct ieee802154_privmac_s *priv)
{
  priv->macAckWaitDuration    = 0;
  priv->macAssociationPermit  = 0;
  priv->macAutoRequest        = 1;
  priv->macBattLifeExt        = 0;
  priv->macBattLifeExtPeriods = 0;
  priv->macMaxCSMABackoffs    = 4;

  priv->macBeaconOrder        = 15;
  priv->macSuperframeOrder    = 15;

  priv->macMinBE              = 3;
  priv->macGTSPermit          = 1;
  priv->macPromiscuousMode    = 0;
  priv->macRxOnWhenIdle       = 0;
  priv->macBeaconTxTime       = 0x000000;

  priv->macBeaconPayloadLength = 0;
  priv->macBSN                 = 0; /* Shall be random */
  //priv->macCoordExtendedAddress[8];
  priv->macCoordShortAddress   = 0xffff;
  priv->macDSN                 = 0; /* Shall be random */
  priv->macPANId               = 0xffff;
  priv->macShortAddress        = 0xffff;
  priv->macTransactionPersistenceTime = 0x01f4;

#if 0
  /* Security MIB */

  priv->macACLEntryDescriptorSetSize     = 0;
  priv->macDefaultSecurity               = 0;
  priv->macDefaultSecuritySuite          = 0;
  priv->macDefaultSecurityMaterialLength = 0x15;
  priv->macSecurityMode                  = 0;
#endif

  return OK;
}

/****************************************************************************
 * Name: mac802154_applymib
 *
 * Description:
 *   Some parts of the MIB must be sent to the radio device. This routine
 *   calls the radio device routines to store the related parameters in the
 *   radio driver. It must be called each time a MIB parameter is changed.
 *
 ****************************************************************************/

static int mac802154_applymib(FAR struct ieee802154_privmac_s *priv)
{
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_create
 *
 * Description:
 *   Create a 802.15.4 MAC device from a 802.15.4 compatible radio device.
 *
 *   The returned MAC structure should be passed to either the next highest
 *   layer in the network stack, or registered with a mac802154dev character
 *   or network drivers.  In any of these scenarios, the next highest layer
 *   should  register a set of callbacks with the MAC layer by setting the
 *   mac->cbs member.
 *
 *   NOTE: This API does not create any device accessible to userspace. If you
 *   want to call these APIs from userspace, you have to wrap your mac in a
 *   character device via mac802154_device.c.
 *
 * Input Parameters:
 *   radiodev - an instance of an IEEE 802.15.4 radio
 *
 * Returned Value:
 *   An opaque reference to the MAC state data.
 *
 ****************************************************************************/

MACHANDLE mac802154_create(FAR struct ieee802154_radio_s *radiodev)
{
  FAR struct ieee802154_privmac_s *mac;

  /* Allocate object */

  mac = (FAR struct ieee802154_privmac_s *)
    kmm_zalloc(sizeof(struct ieee802154_privmac_s));

  if (mac == NULL)
    {
      return NULL;
    }

  /* Initialize fields */

  mac->radio = radiodev;

  mac802154_defaultmib(mac);
  mac802154_applymib(mac);

  return (MACHANDLE)mac;
}

/****************************************************************************
 * Name: mac802154_bind
 *
 * Description:
 *   Bind the MAC callback table to the MAC state.
 *
 * Parameters:
 *   mac - Reference to the MAC driver state structure
 *   cb  - MAC callback operations
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int mac802154_bind(MACHANDLE mac, FAR const struct ieee802154_maccb_s *cb)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;

  priv->cb = cb;
  return OK;
}

/****************************************************************************
 * Name: mac802154_ioctl
 *
 * Description:
 *   Handle MAC and radio IOCTL commands directed to the MAC.
 *
 * Parameters:
 *   mac - Reference to the MAC driver state structure
 *   cmd - The IOCTL command
 *   arg - The argument for the IOCTL command
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int mac802154_ioctl(MACHANDLE mac, int cmd, unsigned long arg)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  int ret = -EINVAL;

  DEBUGASSERT(priv != NULL);

  /* Check for IOCTLs aimed at the IEEE802.15.4 MAC layer */

  if (_MAC802154IOCVALID(cmd))
    {
      /* Handle the MAC IOCTL command */
#warning Missing logic
    }

  /* No, other IOCTLs must be aimed at the IEEE802.15.4 radio layer */

  else
   {
     DEBUGASSERT(priv->radio != NULL &&
                 priv->radio->ops != NULL &&
                 priv->radio->ops->ioctl != NULL);

     ret = priv->radio->ops->ioctl(priv->radio, cmd, arg);
   }

 return ret;
}

/****************************************************************************
 * MAC Interface Operations
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_req_data
 *
 * Description:
 *   The MCPS-DATA.request primitive requests the transfer of a data SPDU
 *   (i.e., MSDU) from a local SSCS entity to a single peer SSCS entity. 
 *   Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_data callback.
 *
 ****************************************************************************/

int mac802154_req_data(MACHANDLE mac, FAR struct ieee802154_data_req_s *req)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_purge
 *
 * Description:
 *   The MCPS-PURGE.request primitive allows the next higher layer to purge an
 *   MSDU from the transaction queue. Confirmation is returned via
 *   the struct ieee802154_maccb_s->conf_purge callback.
 *
 ****************************************************************************/

int mac802154_req_purge(MACHANDLE mac, uint8_t handle)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_associate
 *
 * Description:
 *   The MLME-ASSOCIATE.request primitive allows a device to request an 
 *   association with a coordinator. Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_associate callback.
 *
 ****************************************************************************/

int mac802154_req_associate(MACHANDLE mac,
                            FAR struct ieee802154_assoc_req_s *req)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_disassociate
 *
 * Description:
 *   The MLME-DISASSOCIATE.request primitive is used by an associated device to
 *   notify the coordinator of its intent to leave the PAN. It is also used by
 *   the coordinator to instruct an associated device to leave the PAN.
 *   Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_disassociate callback.
 *
 ****************************************************************************/

int mac802154_req_disassociate(MACHANDLE mac,
                               FAR struct ieee802154_disassoc_req_s *req)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_get
 *
 * Description:
 *   The MLME-GET.request primitive requests information about a given PIB
 *   attribute. Actual data is returned via the
 *   struct ieee802154_maccb_s->conf_get callback.
 *
 ****************************************************************************/

int mac802154_req_get(MACHANDLE mac, enum ieee802154_pib_attr_e attr)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_gts
 *
 * Description:
 *   The MLME-GTS.request primitive allows a device to send a request to the PAN
 *   coordinator to allocate a new GTS or to deallocate an existing GTS.
 *   Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_gts callback.
 *
 ****************************************************************************/

int mac802154_req_gts(MACHANDLE mac, FAR uint8_t *characteristics)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_reset
 *
 * Description:
 *   The MLME-RESET.request primitive allows the next higher layer to request
 *   that the MLME performs a reset operation. Confirmation is returned via
 *   the struct ieee802154_maccb_s->conf_reset callback.
 *
 ****************************************************************************/

int mac802154_req_reset(MACHANDLE mac, bool setdefaults)
{
  FAR struct ieee802154_privmac_s * priv = (FAR struct ieee802154_privmac_s *) mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_rxenable
 *
 * Description:
 *   The MLME-RX-ENABLE.request primitive allows the next higher layer to
 *   request that the receiver is enable for a finite period of time.
 *   Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_rxenable callback.
 *
 ****************************************************************************/

int mac802154_req_rxenable(MACHANDLE mac, bool deferrable, int ontime,
                           int duration)
{
  FAR struct ieee802154_privmac_s * priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_scan
 *
 * Description:
 *   The MLME-SCAN.request primitive is used to initiate a channel scan over a
 *   given list of channels. A device can use a channel scan to measure the
 *   energy on the channel, search for the coordinator with which it associated,
 *   or search for all coordinators transmitting beacon frames within the POS of
 *   the scanning device. Scan results are returned
 *   via MULTIPLE calls to the struct ieee802154_maccb_s->conf_scan callback.
 *   This is a difference with the official 802.15.4 specification, implemented
 *   here to save memory.
 *
 ****************************************************************************/

int mac802154_req_scan(MACHANDLE mac, uint8_t type, uint32_t channels,
                       int duration)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_set
 *
 * Description:
 *   The MLME-SET.request primitive attempts to write the given value to the
 *   indicated MAC PIB attribute. Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_set callback.
 *
 ****************************************************************************/

int mac802154_req_set(MACHANDLE mac, int attribute, FAR uint8_t *value,
                      int valuelen)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_start
 *
 * Description:
 *   The MLME-START.request primitive makes a request for the device to start
 *   using a new superframe configuration. Confirmation is returned
 *   via the struct ieee802154_maccb_s->conf_start callback.
 *
 ****************************************************************************/

int mac802154_req_start(MACHANDLE mac, uint16_t panid, int channel,
                        uint8_t bo, uint8_t fo, bool coord, bool batext,
                        bool realign)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_sync
 *
 * Description:
 *   The MLME-SYNC.request primitive requests to synchronize with the
 *   coordinator by acquiring and, if specified, tracking its beacons.
 *   Confirmation is returned via the
 *   struct ieee802154_maccb_s->int_commstatus callback. TOCHECK.
 *
 ****************************************************************************/

int mac802154_req_sync(MACHANDLE mac, int channel, bool track)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_req_poll
 *
 * Description:
 *   The MLME-POLL.request primitive prompts the device to request data from the
 *   coordinator. Confirmation is returned via the 
 *   struct ieee802154_maccb_s->conf_poll callback, followed by a
 *   struct ieee802154_maccb_s->ind_data callback.
 *
 ****************************************************************************/

int mac802154_req_poll(MACHANDLE mac, FAR uint8_t *coordaddr)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_rsp_associate
 *
 * Description:
 *   The MLME-ASSOCIATE.response primitive is used to initiate a response to an 
 *   MLME-ASSOCIATE.indication primitive.
 *
 ****************************************************************************/

int mac802154_rsp_associate(MACHANDLE mac, uint8_t eadr, uint16_t saddr,
                            int status)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_rsp_orphan
 *
 * Description:
 *   The MLME-ORPHAN.response primitive allows the next higher layer of a
 *   coordinator to respond to the MLME-ORPHAN.indication primitive.
 *
 ****************************************************************************/

int mac802154_rsp_orphan(MACHANDLE mac, FAR uint8_t *orphanaddr,
                         uint16_t saddr, bool associated)
{
  FAR struct ieee802154_privmac_s *priv =
    (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}
