/****************************************************************************
 * include/nuttx/wireless/ieee802154/ieee802154_mac.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_MAC_H
#define __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_MAC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* None at the moment */

/* IEEE 802.15.4 MAC Interface **********************************************/

/* Some addresses */

#define IEEE802154_PAN_UNSPEC   (uint16_t)0xFFFF
#define IEEE802154_SADDR_UNSPEC (uint16_t)0xFFFF
#define IEEE802154_SADDR_BCAST  (uint16_t)0xFFFE
#define IEEE802154_EADDR_UNSPEC (uint8_t*)"\xff\xff\xff\xff\xff\xff\xff\xff"

/* IEEE 802.15.4 MAC status codes */

enum
{
  MAC802154_STATUS_OK = 0,
  MAC802154_STATUS_BEACON_LOSS = 0xE0,
  MAC802154_STATUS_CHANNEL_ACCESS_FAILURE,
  MAC802154_STATUS_DENIED,
  MAC802154_STATUS_DISABLE_TRX_FAILURE,
  MAC802154_STATUS_FAILED_SECURITY_CHECK,
  MAC802154_STATUS_FRAME_TOO_LONG,
  MAC802154_STATUS_INVALID_GTS,
  MAC802154_STATUS_INVALID_HANDLE,
  MAC802154_STATUS_INVALID_PARAMETER,
  MAC802154_STATUS_NO_ACK,
  MAC802154_STATUS_NO_BEACON,
  MAC802154_STATUS_NO_DATA,
  MAC802154_STATUS_NO_SHORT_ADDRESS,
  MAC802154_STATUS_OUT_OF_CAP,
  MAC802154_STATUS_PAN_ID_CONFLICT,
  MAC802154_STATUS_REALIGNMENT,
  MAC802154_STATUS_TRANSACTION_EXPIRED,
  MAC802154_STATUS_TRANSACTION_OVERFLOW,
  MAC802154_STATUS_TX_ACTIVE,
  MAC802154_STATUS_UNAVAILABLE_KEY,
  MAC802154_STATUS_UNSUPPORTED_ATTRIBUTE
};

/* IEEE 802.15.4 PHY constants */
#define MAC802154_aMaxPHYPacketSize       127
#define MAC802154_aTurnaroundTime         12 /*symbol periods*/

/* IEEE 802.15.4 MAC constants */
#define MAC802154_aBaseSlotDuration       60
#define MAC802154_aNumSuperframeSlots     16
#define MAC802154_aBaseSuperframeDuration (MAC802154_aBaseSlotDuration * MAC802154_aNumSuperframeSlots)
#define MAC802154_aMaxBE                  5
#define MAC802154_aMaxBeaconOverhead      75
#define MAC802154_aMaxBeaconPayloadLength (MAC802154_aMaxPHYPacketSize - MAC802154_aMaxBeaconOverhead)
#define MAC802154_aGTSDescPersistenceTime 4
#define MAC802154_aMaxFrameOverhead       25
#define MAC802154_aMaxFrameResponseTime   1220
#define MAC802154_aMaxFrameRetries        3
#define MAC802154_aMaxLostBeacons         4
#define MAC802154_aMaxMACFrameSize        (MAC802154_aMaxPHYPacketSize - MAC802154_aMaxFrameOverhead)
#define MAC802154_aMaxSIFSFrameSize       18
#define MAC802154_aMinCAPLength           440
#define MAC802154_aMinLIFSPeriod          40
#define MAC802154_aMinSIFSPeriod          12
#define MAC802154_aResponseWaitTime       (32 * MAC802154_aBaseSuperframeDuration)
#define MAC802154_aUnitBackoffPeriod      20


/* IEEE 802.15.4 PHY/MAC PIB attributes IDs */
enum
{
  MAC802154_phyCurrentChannel = 0x00,
  MAC802154_phyChannelsSupported,
  MAC802154_phyTransmitPower,
  MAC802154_phyCCAMode,
  MAC802154_macAckWaitDuration = 0x40,
  MAC802154_macAssociationPermit,
  MAC802154_macAutoRequest,
  MAC802154_macBattLifeExt,
  MAC802154_macBattLifeExtPeriods,
  MAC802154_macBeaconPayload,
  MAC802154_macBeaconPayloadLength,
  MAC802154_macBeaconOrder,
  MAC802154_macBeaconTxTime,
  MAC802154_macBSN,
  MAC802154_macCoordExtendedAddress,
  MAC802154_macCoordShortAddress,
  MAC802154_macDSN,
  MAC802154_macGTSPermit,
  MAC802154_macMaxCSMABackoffs,
  MAC802154_macMinBE,
  MAC802154_macPANId,
  MAC802154_macPromiscuousMode,
  MAC802154_macRxOnWhenIdle,
  MAC802154_macShortAddress,
  MAC802154_macSuperframeOrder,
  MAC802154_macTransactionPersistenceTime,
  MAC802154_macACLEntryDescriptorSet = 0x70,
  MAC802154_macACLEntryDescriptorSetSize,
  MAC802154_macDefaultSecurity,
  MAC802154_macDefaultSecurityMaterialLength,
  MAC802154_macDefaultSecurityMaterial,
  MAC802154_macDefaultSecuritySuite,
  MAC802154_macSecurityMode
};

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* IEEE 802.15.4 Device address
 * The addresses in ieee802154 have several formats:
 * No address                : [none]
 * Short address + PAN id    : PPPP/SSSS
 * Extended address + PAN id : PPPP/LLLLLLLLLLLLLLLL
 */

struct ieee802154_addr_s
{
  uint8_t ia_len;            /* structure length, 0/2/8 */
  uint16_t ia_panid;         /* PAN identifier, can be IEEE802154_PAN_UNSPEC */
  union {
    uint16_t _ia_saddr;      /* short address */
    uint8_t  _ia_eaddr[8];   /* extended address */
  } ia_addr;
#define ia_saddr ia_addr._ia_saddr
#define ia_eaddr ia_addr._ia_eaddr
};
#define IEEE802154_ADDRSTRLEN 22 /* (2*2+1+8*2, PPPP/EEEEEEEEEEEEEEEE) */

/* Operations */

struct ieee802154_mac_s;

struct ieee802154_macops_s
{
  /* Requests, confirmed asynchronously via callbacks */

  CODE int req_data        (struct ieee802154_mac_s *mac, /* Transmit a data frame */
                            uint8_t handle,
                            uint8_t *buf,
                            int len);

  CODE int req_purge       (struct ieee802154_mac_s *mac, /* Cancel transmission of a data frame */
                            uint8_t handle);

  CODE int req_associate   (struct ieee802154_mac_s *mac, /* Start association with coordinator */
                            uint16_t panid,
                            uint8_t *coordeadr);

  CODE int req_disassociate(struct ieee802154_mac_s *mac, /* Start disassociation with coordinator */
                            uint8_t *eadr,
                            uint8_t reason);

  CODE int req_get         (struct ieee802154_mac_s *mac, /* Read the PIB */
                            int attribute);

  CODE int req_gts         (struct ieee802154_mac_s *mac, /* Allocate or deallocate a GTS */ 
                            uint8_t* characteristics);

  CODE int req_reset       (struct ieee802154_mac_s *mac, /* MAC layer reset */
                            bool setdefaults);

  CODE int req_rxenable    (struct ieee802154_mac_s *mac, /* PHY receiver control */
                            bool deferrable,
                            int ontime,
                            int duration);

  CODE int req_scan        (struct ieee802154_mac_s *mac, /* Start a network scan */
                            uint8_t type,
                            uint32_t channels,
                            int duration);

  CODE int req_set         (struct ieee802154_mac_s *mac, /* Change the PIB */
                            int attribute,
                            uint8_t *value,
                            int valuelen);

  CODE int req_start       (struct ieee802154_mac_s *mac, 
                            uint16_t panid,
                            int channel,
                            uint8_t bo,
                            uint8_t fo,
                            bool coord,
                            bool batext,
                            bool realign);

  CODE int req_sync        (struct ieee802154_mac_s *mac, 
                            int channel,
                            bool track);

  CODE int req_poll        (struct ieee802154_mac_s *mac, 
                            uint8_t *coordaddr);

  /* Synchronous Responses to Indications received via callbacks */

  CODE int rsp_associate   (struct ieee802154_mac_s *mac,  /* Reply to an association request */
                            uint8_t eadr,
                            uint16_t saddr,
                            int status);

  CODE int rsp_orphan      (struct ieee802154_mac_s *mac,  /* Orphan device management */
                            uint8_t *orphanaddr,
                            uint16_t saddr,
                            bool associated);
};

/* Notifications */

struct ieee802154_maccb_s
{
  /* Asynchronous confirmations to requests */

  CODE int conf_data        (struct ieee802154_mac_s *mac, /* Data frame was received by remote device */
                             uint8_t *buf,
                             int len);
  CODE int conf_purge       (struct ieee802154_mac_s *mac, /* Data frame was purged */
                             uint8_t handle,
                             int status);
  CODE int conf_associate   (struct ieee802154_mac_s *mac, /* Association request completed */
                             uint16_t saddr,
                             int status);
  CODE int conf_disassociate(struct ieee802154_mac_s *mac, /* Disassociation request completed */
                             int status);
  CODE int conf_get         (struct ieee802154_mac_s *mac, /* PIB data returned */
                             int status,
                             int attribute,
                             uint8_t *value,
                             int valuelen);
  CODE int conf_gts         (struct ieee802154_mac_s *mac, /* GTS management completed */
                             uint8_t *characteristics,
                             int status);
  CODE int conf_reset       (struct ieee802154_mac_s *mac, /* MAC reset completed */
                             int status);
  CODE int conf_rxenable    (struct ieee802154_mac_s *mac,
                             int status);
  CODE int conf_scan        (struct ieee802154_mac_s *mac,
                             int status,
                             uint8_t type,
                             uint32_t unscanned,
                             int rsltsize,
                             uint8_t *edlist,
                             uint8_t *pandescs);
  CODE int conf_set         (struct ieee802154_mac_s *mac,
                             int status, int attribute);
  CODE int conf_start       (struct ieee802154_mac_s *mac,
                             int status);
  CODE int conf_poll        (struct ieee802154_mac_s *mac,
                             int status);

  /* Asynchronous event indications, replied to synchronously with responses */

  CODE int ind_data         (struct ieee802154_mac_s *mac, /* Data frame received */
                             uint8_t *buf,
                             int len);
  CODE int ind_associate    (struct ieee802154_mac_s *mac, /* Association request received */
                             uint16_t clipanid,
                             uint8_t *clieaddr);
  CODE int ind_disassociate (struct ieee802154_mac_s *mac, /* Disassociation request received */
                             uint8_t *eadr,
                             uint8_t reason);
  CODE int ind_beaconnotify (struct ieee802154_mac_s *mac, /* Beacon notification */
                             uint8_t *bsn,
                             uint_t *pandesc,
                             uint8_t *sdu,
                             int sdulen);
  CODE int ind_gts          (struct ieee802154_mac_s *mac, /* GTS management request received */
                             uint8_t *devaddr,
                             uint8_t *characteristics);
  CODE int ind_orphan       (struct ieee802154_mac_s *mac, /* Orphan device detected */
                             uint8_t *orphanaddr);
  CODE int ind_commstatus   (struct ieee802154_mac_s *mac,
                             uint16_t panid,
                             uint8_t *src,
                             uint8_t *dst,
                             int status);
  CODE int ind_syncloss     (struct ieee802154_mac_s *mac,
                             int reason);
};

struct ieee802154_radio_s;

struct ieee802154_mac_s
{
  struct ieee802154_radio_s *radio;
  struct ieee802154_macops_s ops;
  struct ieee802154_maccb_s  cbs;
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

/*
 * Instanciate a 802.15.4 MAC from a 802.15.4 radio device.
 * To create a 802.15.4 MAC, you need to pass:
 *  - an instance of a radio driver in radiodev
 *  - a pointer to a structure that contains MAC callback routines to
 *     handle confirmations and indications. NULL entries indicate no callback.
 * In return you get a mac structure that has pointers to MAC operations and
 *  responses.
 * This API does not create any device accessible to userspace. If you want to
 * call these APIs from userspace, you have to wrap your mac in a character
 * device via mac802154_device.c . 
 */
FAR struct ieee802154_mac_s * mac802154_register(FAR struct ieee802154_radio_s *radiodev, struct ieee802154_maccb_s *callbacks);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_IEEE802154_MRF24J40_H */
