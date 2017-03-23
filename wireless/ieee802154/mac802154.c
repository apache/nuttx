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
#include <assert.h>
#include <debug.h>
#include <stdlib.h>

#include <nuttx/wireless/ieee802154/ieee802154_mac.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Frame Type */

#define IEEE802154_FRAME_BEACON  0x00
#define IEEE802154_FRAME_DATA    0x01
#define IEEE802154_FRAME_ACK     0x02
#define IEEE802154_FRAME_COMMAND 0x03

/* MAC commands */

#define IEEE802154_CMD_ASSOC_REQ      0x01
#define IEEE802154_CMD_ASSOC_RSP      0x02
#define IEEE802154_CMD_DIS_NOT        0x03
#define IEEE802154_CMD_DATA_REQ       0x04
#define IEEE802154_CMD_PANID_CONF_NOT 0x05
#define IEEE802154_CMD_ORPHAN_NOT     0x06
#define IEEE802154_CMD_BEACON_REQ     0x07
#define IEEE802154_CMD_COORD_REALIGN  0x08
#define IEEE802154_CMD_GTS_REQ        0x09

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The privmac structure is an extension of the public ieee802154_mac_s type.
 * It contains storage for the IEEE802.15.4 MIB attributes.
 */

struct ieee802154_privmac_s
{
  struct ieee802154_mac_s pubmac; /* This MUST be the first member */

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
  /* 0x52 */ uint32_t  macRxOnWhenIdle    : 1;
             uint32_t macPad             : 3;
  /* 0x48 */ uint32_t macBeaconTxTime    : 24;

  /* 0x45 */ uint8_t  macBeaconPayload[MAC802154_aMaxBeaconPayloadLength];
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
 * Private Function Prototypes
 ****************************************************************************/

static int mac802154_reqdata(FAR struct ieee802154_mac_s *mac,
             uint8_t handle, FAR uint8_t *buf, int len);
static int mac802154_reqpurge(FAR struct ieee802154_mac_s *mac,
             uint8_t handle);
static int mac802154_reqassociate(FAR struct ieee802154_mac_s *mac,
             uint16_t panid, FAR uint8_t *coordeadr);
static int mac802154_reqdisassociate(FAR struct ieee802154_mac_s *mac,
             FAR uint8_t *eadr, uint8_t reason);
static int mac802154_reqget(FAR struct ieee802154_mac_s *mac,
             int attribute);
static int mac802154_reqgts(FAR struct ieee802154_mac_s *mac,
             FAR uint8_t *characteristics);
static int mac802154_reqreset(FAR struct ieee802154_mac_s *mac,
             bool setdefaults);
static int mac802154_reqrxenable(FAR struct ieee802154_mac_s *mac,
             bool deferrable, int ontime, int duration);
static int mac802154_reqscan(FAR struct ieee802154_mac_s *mac,
             uint8_t type, uint32_t channels, int duration);
static int mac802154_reqset(FAR struct ieee802154_mac_s *mac,
             int attribute, FAR uint8_t *value, int valuelen);
static int mac802154_reqstart(FAR struct ieee802154_mac_s *mac,
             uint16_t panid, int channel, uint8_t bo, uint8_t fo,
             bool coord, bool batext, bool realign);
static int mac802154_reqsync(FAR struct ieee802154_mac_s *mac,
             int channel, bool track);
static int mac802154_reqpoll(FAR struct ieee802154_mac_s *mac,
             FAR uint8_t *coordaddr);
static int mac802154_rspassociate(FAR struct ieee802154_mac_s *mac,
             uint8_t eadr, uint16_t saddr, int status);
static int mac802154_rsporphan(FAR struct ieee802154_mac_s *mac,
             FAR uint8_t *orphanaddr, uint16_t saddr, bool associated);


/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ieee802154_macops_s mac802154ops =
{
  .req_data        = mac802154_reqdata
  .req_purge       = mac802154_reqpurge
  .req_associate   = mac802154_reqassociate
  .req_disassociate= mac802154_reqdisassociate
  .req_get         = mac802154_reqget
  .req_gts         = mac802154_reqgts
  .req_reset       = mac802154_reqreset
  .req_rxenable    = mac802154_reqrxenable
  .req_scan        = mac802154_reqscan
  .req_set         = mac802154_reqset
  .req_start       = mac802154_reqstart
  .req_sync        = mac802154_reqsync
  .req_poll        = mac802154_reqpoll
  .rsp_associate   = mac802154_rspassociate
  .rsp_orphan      = mac802154_rsporphan
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
  priv->macCoordExtendedAddress[8];
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
 * API Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_reqdata
 *
 * Description:
 *   The MCPS-DATA.request primitive requests the transfer of a data SPDU
 *   (i.e., MSDU) from a local SSCS entity to a single peer SSCS entity. 
 *   Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_data callback.
 *
 ****************************************************************************/

static int mac802154_reqdata(FAR struct ieee802154_mac_s *mac,
                             uint8_t handle, FAR uint8_t *buf, int len)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_reqpurge
 *
 * Description:
 *   The MCPS-PURGE.request primitive allows the next higher layer to purge an
 *   MSDU from the transaction queue. Confirmation is returned via
 *   the struct ieee802154_maccb_s->conf_purge callback.
 *
 ****************************************************************************/

static int mac802154_reqpurge(FAR struct ieee802154_mac_s *mac,
                              uint8_t handle)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_reqassociate
 *
 * Description:
 *   The MLME-ASSOCIATE.request primitive allows a device to request an 
 *   association with a coordinator. Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_associate callback.
 *
 ****************************************************************************/

static int mac802154_reqassociate(FAR struct ieee802154_mac_s *mac,
                                  uint16_t panid,
                                  uint8_t *coordeadr)
{
  FAR struct ieee802154_privmac_s * priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_reqdisassociate
 *
 * Description:
 *   The MLME-DISASSOCIATE.request primitive is used by an associated device to
 *   notify the coordinator of its intent to leave the PAN. It is also used by
 *   the coordinator to instruct an associated device to leave the PAN.
 *   Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_disassociate callback.
 *
 ****************************************************************************/

static int mac802154_reqdisassociate(FAR struct ieee802154_mac_s *mac,
                                     FAR uint8_t *eadr, uint8_t reason)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_reqget
 *
 * Description:
 *   The MLME-GET.request primitive requests information about a given PIB
 *   attribute. Actual data is returned via the
 *   struct ieee802154_maccb_s->conf_get callback.
 *
 ****************************************************************************/

static int mac802154_reqget(FAR struct ieee802154_mac_s *mac,
                            int attribute)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_reqgts
 *
 * Description:
 *   The MLME-GTS.request primitive allows a device to send a request to the PAN
 *   coordinator to allocate a new GTS or to deallocate an existing GTS.
 *   Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_gts callback.
 *
 ****************************************************************************/

static int mac802154_reqgts(FAR struct ieee802154_mac_s *mac,
                            FAR uint8_t *characteristics)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_reqreset
 *
 * Description:
 *   The MLME-RESET.request primitive allows the next higher layer to request
 *   that the MLME performs a reset operation. Confirmation is returned via
 *   the struct ieee802154_maccb_s->conf_reset callback.
 *
 ****************************************************************************/

static int mac802154_reqreset(FAR struct ieee802154_mac_s *mac,
                              bool setdefaults)
{
  FAR struct ieee802154_privmac_s * priv = (FAR struct ieee802154_privmac_s *) mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_reqrxenable
 *
 * Description:
 *   The MLME-RX-ENABLE.request primitive allows the next higher layer to
 *   request that the receiver is enable for a finite period of time.
 *   Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_rxenable callback.
 *
 ****************************************************************************/

static int mac802154_reqrxenable(FAR struct ieee802154_mac_s *mac,
                                 bool deferrable, int ontime, int duration)
{
  FAR struct ieee802154_privmac_s * priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_reqscan
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

static int mac802154_reqscan(FAR struct ieee802154_mac_s *mac,
                             uint8_t type, uint32_t channels, int duration)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_reqset
 *
 * Description:
 *   The MLME-SET.request primitive attempts to write the given value to the
 *   indicated MAC PIB attribute. Confirmation is returned via the
 *   struct ieee802154_maccb_s->conf_set callback.
 *
 ****************************************************************************/

static int mac802154_reqset(FAR struct ieee802154_mac_s *mac,
                            int attribute, FAR uint8_t *value, int valuelen)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_reqstart
 *
 * Description:
 *   The MLME-START.request primitive makes a request for the device to start
 *   using a new superframe configuration. Confirmation is returned
 *   via the struct ieee802154_maccb_s->conf_start callback.
 *
 ****************************************************************************/

static int mac802154_reqstart(FAR struct ieee802154_mac_s *mac,
                              uint16_t panid, int channel, uint8_t bo,
                              uint8_t fo, bool coord, bool batext,
                              bool realign)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_reqsync
 *
 * Description:
 *   The MLME-SYNC.request primitive requests to synchronize with the
 *   coordinator by acquiring and, if specified, tracking its beacons.
 *   Confirmation is returned via the
 *   struct ieee802154_maccb_s->int_commstatus callback. TOCHECK.
 *
 ****************************************************************************/

static int mac802154_reqsync(FAR struct ieee802154_mac_s *mac,
                             int channel, bool track)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_reqpoll
 *
 * Description:
 *   The MLME-POLL.request primitive prompts the device to request data from the
 *   coordinator. Confirmation is returned via the 
 *   struct ieee802154_maccb_s->conf_poll callback, followed by a
 *   struct ieee802154_maccb_s->ind_data callback.
 *
 ****************************************************************************/

static int mac802154_reqpoll(FAR struct ieee802154_mac_s *mac,
                             FAR uint8_t *coordaddr)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_rspassociate
 *
 * Description:
 *   The MLME-ASSOCIATE.response primitive is used to initiate a response to an 
 *   MLME-ASSOCIATE.indication primitive.
 *
 ****************************************************************************/

static int mac802154_rspassociate(FAR struct ieee802154_mac_s *mac,
                                  uint8_t eadr, uint16_t saddr, int status)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Name: mac802154_rsporphan
 *
 * Description:
 *   The MLME-ORPHAN.response primitive allows the next higher layer of a
 *   coordinator to respond to the MLME-ORPHAN.indication primitive.
 *
 ****************************************************************************/

static int mac802154_rsporphan(FAR struct ieee802154_mac_s *mac,
                               FAR uint8_t *orphanaddr, uint16_t saddr,
                               bool associated)
{
  FAR struct ieee802154_privmac_s *priv = (FAR struct ieee802154_privmac_s *)mac;
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mac802154_register
 *
 * Description:
 *   Create a 802.15.4 MAC device from a 802.15.4 compatible radio device.
 *   To create a 802.15.4 MAC, you need to pass:
 *
 *     - an instance of a radio driver in radiodev
 *     - a pointer to a structure that contains MAC callback routines to
 *       handle confirmations and indications. NULL entries indicate no
 *       callback.
 *
 *   In return you get a mac structure that has pointers to MAC operations
 *   and responses.
 *
 *   This API does not create any device accessible to userspace. If you
 *   want to call these APIs from userspace, you have to wrap your mac in a
 *   character device via mac802154_device.c.
 *
 ****************************************************************************/

FAR struct ieee802154_mac_s *
  mac802154_register(FAR struct ieee802154_radio_s *radiodev,
                     unsigned int minor)
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

  mac->pubmac.radio = radiodev;
  mac->pubmac.ops   = mac802154ops;

  mac802154_defaultmib(mac);
  mac802154_applymib(mac);

  return &mac->pubmac;
}
