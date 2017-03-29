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

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* None at the moment */

/* IEEE 802.15.4 MAC Character Driver IOCTL Commands ************************/

/* The IEEE 802.15.4 standard specifies a MLME Service Access Point (SAP) 
 * including a series of primitives that are used as an interface between
 * the MLME and the next highest layer.  There are 4 types of primitives:
 *
 *   - Request
 *   - Indication
 *   - Response
 *   - Confirm
 * 
 * Of these, Request and Response primitives are sent from the next highest layer
 * to the MLME.  Indication and Confirm primitives are used to notify the next
 * highest layer of changes or actions that have taken place.
 *
 * The MAC802154 character driver exposed here provides IOCTL hooks for all
 * Request and Response primitives.
 */

#define MAC802154IOC_MLME_ASSOC_REQUEST        _MAC802154IOC(0x0001)
#define MAC802154IOC_MLME_ASSOC_RESPONSE       _MAC802154IOC(0x0002)
#define MAC802154IOC_MLME_DISASSOC_REQUEST     _MAC802154IOC(0x0003)
#define MAC802154IOC_MLME_GET_REQUEST          _MAC802154IOC(0x0004)
#define MAC802154IOC_MLME_GTS_REQUEST          _MAC802154IOC(0x0005)
#define MAC802154IOC_MLME_ORPHAN_RESPONSE      _MAC802154IOC(0x0006)
#define MAC802154IOC_MLME_RESET_REQUEST        _MAC802154IOC(0x0007)
#define MAC802154IOC_MLME_RXENABLE_REQUEST     _MAC802154IOC(0x0008)
#define MAC802154IOC_MLME_SCAN_REQUEST         _MAC802154IOC(0x0009)
#define MAC802154IOC_MLME_SET_REQUEST          _MAC802154IOC(0x000A)
#define MAC802154IOC_MLME_START_REQUEST        _MAC802154IOC(0x000B)
#define MAC802154IOC_MLME_SYNC_REQUEST         _MAC802154IOC(0x000C)
#define MAC802154IOC_MLME_POLL_REQUEST         _MAC802154IOC(0x000D)
#define MAC802154IOC_MLME_DPS_REQUEST          _MAC802154IOC(0x000E)
#define MAC802154IOC_MLME_SOUNDING_REQUEST     _MAC802154IOC(0x000F)
#define MAC802154IOC_MLME_CALIBRATE_REQUEST    _MAC802154IOC(0x0010)

/* IEEE 802.15.4 MAC Interface **********************************************/

/* Frame Type */

#define IEEE802154_FRAME_BEACON       0x00
#define IEEE802154_FRAME_DATA         0x01
#define IEEE802154_FRAME_ACK          0x02
#define IEEE802154_FRAME_COMMAND      0x03

/* MAC commands */

#define IEEE802154_CMD_ASSOC_REQ      0x01
#define IEEE802154_CMD_ASSOC_RSP      0x02
#define IEEE802154_CMD_DISASSOC_NOT   0x03
#define IEEE802154_CMD_DATA_REQ       0x04
#define IEEE802154_CMD_PANID_CONF_NOT 0x05
#define IEEE802154_CMD_ORPHAN_NOT     0x06
#define IEEE802154_CMD_BEACON_REQ     0x07
#define IEEE802154_CMD_COORD_REALIGN  0x08
#define IEEE802154_CMD_GTS_REQ        0x09

/* Some addresses */

#define IEEE802154_PAN_UNSPEC   (uint16_t)0xFFFF
#define IEEE802154_SADDR_UNSPEC (uint16_t)0xFFFF
#define IEEE802154_SADDR_BCAST  (uint16_t)0xFFFE
#define IEEE802154_EADDR_UNSPEC (uint8_t*)"\xff\xff\xff\xff\xff\xff\xff\xff"

/* Frame control field masks, 2 bytes 
 * Seee IEEE 802.15.4/2011 5.2.1.1 page 57
 */

#define IEEE802154_FRAMECTRL_FTYPE      0x0007  /* Frame type, bits 0-2 */
#define IEEE802154_FRAMECTRL_SEC        0x0008  /* Security Enabled, bit 3 */
#define IEEE802154_FRAMECTRL_PEND       0x0010  /* Frame pending, bit 4 */
#define IEEE802154_FRAMECTRL_ACKREQ     0x0020  /* Acknowledge request, bit 5 */
#define IEEE802154_FRAMECTRL_PANIDCOMP  0x0040  /* PAN ID Compression, bit 6 */
#define IEEE802154_FRAMECTRL_DADDR      0x0C00  /* Dest addressing mode, bits 10-11 */
#define IEEE802154_FRAMECTRL_VERSION    0x3000  /* Source addressing mode, bits 12-13 */
#define IEEE802154_FRAMECTRL_SADDR      0xC000  /* Source addressing mode, bits 14-15 */

#define IEEE802154_FRAMECTRL_SHIFT_FTYPE     0  /* Frame type, bits 0-2 */
#define IEEE802154_FRAMECTRL_SHIFT_SEC       3  /* Security Enabled, bit 3 */
#define IEEE802154_FRAMECTRL_SHIFT_PEND      4  /* Frame pending, bit 4 */
#define IEEE802154_FRAMECTRL_SHIFT_ACKREQ    5  /* Acknowledge request, bit 5 */
#define IEEE802154_FRAMECTRL_SHIFT_PANIDCOMP 6  /* PAN ID Compression, bit 6 */
#define IEEE802154_FRAMECTRL_SHIFT_DADDR     10 /* Dest addressing mode, bits 10-11 */
#define IEEE802154_FRAMECTRL_SHIFT_VERSION   12 /* Source addressing mode, bits 12-13 */
#define IEEE802154_FRAMECTRL_SHIFT_SADDR     14 /* Source addressing mode, bits 14-15 */

/* IEEE 802.15.4 PHY constants */

#define IEEE802154_aMaxPHYPacketSize       127
#define IEEE802154_aTurnaroundTime         12 /*symbol periods*/

/* IEEE 802.15.4 MAC constants */

#define IEEE802154_aBaseSlotDuration       60
#define IEEE802154_aNumSuperframeSlots     16
#define IEEE802154_aBaseSuperframeDuration (IEEE802154_aBaseSlotDuration * IEEE802154_aNumSuperframeSlots)
#define IEEE802154_aMaxBE                  5
#define IEEE802154_aMaxBeaconOverhead      75
#define IEEE802154_aMaxBeaconPayloadLength (IEEE802154_aMaxPHYPacketSize - IEEE802154_aMaxBeaconOverhead)
#define IEEE802154_aGTSDescPersistenceTime 4
#define IEEE802154_aMaxFrameOverhead       25
#define IEEE802154_aMaxFrameResponseTime   1220
#define IEEE802154_aMaxFrameRetries        3
#define IEEE802154_aMaxLostBeacons         4
#define IEEE802154_aMaxMACFrameSize        (IEEE802154_aMaxPHYPacketSize - IEEE802154_aMaxFrameOverhead)
#define IEEE802154_aMaxSIFSFrameSize       18
#define IEEE802154_aMinCAPLength           440
#define IEEE802154_aMinLIFSPeriod          40
#define IEEE802154_aMinSIFSPeriod          12
#define IEEE802154_aResponseWaitTime       (32 * IEEE802154_aBaseSuperframeDuration)
#define IEEE802154_aUnitBackoffPeriod      20

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* IEEE 802.15.4 MAC status codes */

enum ieee802154_status_e
{
  IEEE802154_STATUS_OK = 0,
  IEEE802154_STATUS_BEACON_LOSS = 0xE0,
  IEEE802154_STATUS_CHANNEL_ACCESS_FAILURE,
  IEEE802154_STATUS_DENIED,
  IEEE802154_STATUS_DISABLE_TRX_FAILURE,
  IEEE802154_STATUS_FAILED_SECURITY_CHECK,
  IEEE802154_STATUS_FRAME_TOO_LONG,
  IEEE802154_STATUS_INVALID_GTS,
  IEEE802154_STATUS_INVALID_HANDLE,
  IEEE802154_STATUS_INVALID_PARAMETER,
  IEEE802154_STATUS_NO_ACK,
  IEEE802154_STATUS_NO_BEACON,
  IEEE802154_STATUS_NO_DATA,
  IEEE802154_STATUS_NO_SHORT_ADDRESS,
  IEEE802154_STATUS_OUT_OF_CAP,
  IEEE802154_STATUS_PAN_ID_CONFLICT,
  IEEE802154_STATUS_REALIGNMENT,
  IEEE802154_STATUS_TRANSACTION_EXPIRED,
  IEEE802154_STATUS_TRANSACTION_OVERFLOW,
  IEEE802154_STATUS_TX_ACTIVE,
  IEEE802154_STATUS_UNAVAILABLE_KEY,
  IEEE802154_STATUS_UNSUPPORTED_ATTRIBUTE
};

/* IEEE 802.15.4 PHY/MAC PIB attributes IDs */

enum
{
  IEEE802154_phyCurrentChannel = 0x00,
  IEEE802154_phyChannelsSupported,
  IEEE802154_phyTransmitPower,
  IEEE802154_phyCCAMode,
  IEEE802154_macAckWaitDuration = 0x40,
  IEEE802154_macAssociationPermit,
  IEEE802154_macAutoRequest,
  IEEE802154_macBattLifeExt,
  IEEE802154_macBattLifeExtPeriods,
  IEEE802154_macBeaconPayload,
  IEEE802154_macBeaconPayloadLength,
  IEEE802154_macBeaconOrder,
  IEEE802154_macBeaconTxTime,
  IEEE802154_macBSN,
  IEEE802154_macCoordExtendedAddress,
  IEEE802154_macCoordShortAddress,
  IEEE802154_macDSN,
  IEEE802154_macGTSPermit,
  IEEE802154_macMaxCSMABackoffs,
  IEEE802154_macMinBE,
  IEEE802154_macPANId,
  IEEE802154_macPromiscuousMode,
  IEEE802154_macRxOnWhenIdle,
  IEEE802154_macShortAddress,
  IEEE802154_macSuperframeOrder,
  IEEE802154_macTransactionPersistenceTime,
  IEEE802154_macACLEntryDescriptorSet = 0x70,
  IEEE802154_macACLEntryDescriptorSetSize,
  IEEE802154_macDefaultSecurity,
  IEEE802154_macDefaultSecurityMaterialLength,
  IEEE802154_macDefaultSecurityMaterial,
  IEEE802154_macDefaultSecuritySuite,
  IEEE802154_macSecurityMode
};

/* IEEE 802.15.4 Device address
 * The addresses in ieee802154 have several formats:
 * No address                : [none]
 * Short address + PAN id    : PPPP/SSSS
 * Extended address + PAN id : PPPP/LLLLLLLLLLLLLLLL
 */

enum ieee802154_addr_mode_e
{
  IEEE802154_ADDRMODE_NONE = 0,
  IEEE802154_ADDRMODE_SHORT = 2,
  IEEE802154_ADDRMODE_EXTENDED
};

struct ieee802154_addr_s
{
  enum ieee802154_addr_mode_e ia_mode;  /* Address mode. Short or Extended */
  uint16_t ia_panid;                    /* PAN identifier, can be IEEE802154_PAN_UNSPEC */
  union
  {
    uint16_t _ia_saddr;                 /* short address */
    uint8_t  _ia_eaddr[8];              /* extended address */
  } ia_addr;

#define ia_saddr ia_addr._ia_saddr
#define ia_eaddr ia_addr._ia_eaddr
};

#define IEEE802154_ADDRSTRLEN 22 /* (2*2+1+8*2, PPPP/EEEEEEEEEEEEEEEE) */

struct ieee802154_framecontrol_s
{
  /* Frame type
   *
   * Should be a value from: ieee802154_frametype_e
   *
   * Bits 0-1
   */

  uint16_t frame_type     : 3;  

  uint16_t security_en    : 1;  /* Security Enabled flag, bit 3 */
  uint16_t frame_pending  : 1;  /* Frame Pending flag, bit 4 */
  uint16_t ack_req        : 1;  /* Acknowledge Request flag, bit 5 */
  uint16_t panid_comp     : 1;  /* PAN ID Compression flag, bit 6 */
  uint16_t reserved       : 3;  /* Reserved, bits 7-9 */

  /* Destination Addressing Mode
   *
   * Should be a value from: ieee802154_addr_mode_e
   *
   * Bits 10-11
   */

  uint16_t dest_addr_mode : 2;

  uint16_t frame_version  : 2;  /* Frame Version, bits 12-13 */

  /* Source Addressing Mode
   *
   * Should be a value from: ieee802154_addr_mode_e
   *
   * Bits 14-15
   */

  uint16_t src_addr_mode  : 2;
};

struct ieee802154_frame_s
{
  struct ieee802154_framecontrol_s frame_control;
  uint8_t seq_num;
  struct ieee802154_addr_s dest_addr;
  struct ieee802154_addr_s src_addr;
#ifdef CONFIG_IEEE802154_SECURITY
  struct ieee802154_auxsec_s aux_sec_hdr;
#endif
  void *payload;
  uint16_t fcs;
};

struct ieee802154_capability_info_s
{
  uint8_t reserved_0 : 1;     /* Reserved */
  uint8_t device_type : 1;    /* 0=RFD, 1=FFD */
  uint8_t power_source : 1;   /* 1=AC, 0=Other */
  uint8_t rx_on_idle : 1;     /* 0=Receiver off when idle
                               * 1=Receiver on when idle */
  uint8_t reserved_45 : 2;    /* Reserved */
  uint8_t security : 1;       /* 0=disabled, 1=enabled */
  uint8_t allocate_addr : 1;  /* 1=Coordinator allocates short address
                               * 0=otherwise */
};

#ifdef CONFIG_IEEE802154_SECURITY
struct ieee802154_security_s
{
  uint8_t level;            /* Security level to be used */
  uint8_t key_id_mode;      /* Mode used to identify the key to be used */
  uint8_t key_source[8];    /* Originator of the key to be used */
  uint8_t key_index;        /* Index of the key to be used */
};
#endif

struct ieee802154_superframe_spec_s
{
  uint16_t beacon_order     : 4;  /* Transmission interval of beacon */
  uint16_t superframe_order : 4;  /* Length of superframe */
  uint16_t final_cap_slot   : 4;  /* Last slot utilized by CAP */
  uint16_t ble              : 1;  /* Battery Life Extension (BLE) */
  uint16_t reserved         : 1;  /* Reserved bit */
  uint16_t pan_coordinator  : 1;  /* 1 if beacon sent by pan coordinator */
  uint16_t assoc_permit     : 1;  /* 1 if coordinator is accepting associaton */
};

struct ieee802154_pan_desc_s
{
  /* The coordinator address of the received beacon frame */

  struct ieee802154_addr_s coord_addr;

  uint8_t channel;          /* current channel occupied by the network */
  uint8_t channel_page;     /* current channel page occupied by the network */

  /* The superframe specifications received in the beacon frame */

  struct ieee802154_superframe_spec_s superframe_spec;

  uint8_t gts_permit;       /* 0=No GTS requests allowed
                             * 1=GTS request allowed */
  uint8_t link_quality;     /* LQI at which beacon was received */
  uint32_t timestamp;       /* Time at which the beacon frame was received
                             * in symbols */
};

struct ieee802154_pend_addr_s
{
  union
  {
    uint8_t pa_spec;
    struct
    {
      uint8_t num_short_addr  : 3;  /* Number of short addresses pending */
      uint8_t reserved_3      : 1;  /* Reserved bit */
      uint8_t num_ext_addr    : 3;  /* Number of extended addresses pending */       
      uint8_t reserved_7      : 1;  /* Reserved bit */
    };
  };
  struct ieee802154_addr_s addr[7]; /* Array of at most 7 addresses */
};

/* Primitive Semantics */

struct ieee802154_assoc_request_s
{
  uint8_t channel;          /* Channel number to attempt association */
  uint8_t channel_page;     /* Channel page to attempt association */

  /* Coordinator Address with which to associate */

  struct ieee802154_addr_s coord_addr;

  /* Capabilities of associating device */

  struct ieee802154_capability_info_s capabilities;

#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s security;
#endif
};

struct ieee802154_assoc_indication_s
{
  /* Address of device requesting association. Always in extended mode */

  struct ieee802154_addr_s dev_addr;

  /* Capabilities of associating device */

  struct ieee802154_capability_info_s capabilities;

#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s security;
#endif
};

struct ieee802154_assoc_response_s
{
  /* Address of device requesting association. Always in extended mode */

  struct ieee802154_addr_s dev_addr;

  /* Status of association attempt */

  enum ieee802154_status_e status; 

#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s security;
#endif
};

struct ieee802154_assoc_confirm_s
{
  /* Associated device address ALWAYS passed in short address mode. The
   * address will be IEEE802154_SADDR_UNSPEC if association was unsuccessful */

  struct ieee802154_addr_s dev_addr;

  /* Status of association attempt */

  enum ieee802154_status_e status;

#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s security;
#endif
};

struct ieee802154_disassoc_request_s
{
  /* Address of device to send disassociation notification */

  struct ieee802154_addr_s dev_addr;

  /* Reason for the disassosiation */

  enum ieee802154_status_e disassoc_reason;

  uint8_t tx_indirect;        /* 0=Send Direct, 1=Send Indirect */

#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s security;
#endif
};

struct ieee802154_disassoc_indication_s
{
  /* Address of device requesting disassociation. Always extended mode */

  struct ieee802154_addr_s dev_addr;

  /* Reason for the disassosiation */

  enum ieee802154_status_e disassoc_reason;

#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s security;
#endif
};

struct ieee802154_disassoc_confirm_s
{
  /* Status of the disassociation attempt */

  enum ieee802154_status_e status;

  /* Address of device either requesting or being intructed to disassociate */

  struct ieee802154_addr_s dev_addr;
};

struct ieee802154_beaconnotify_indication_s
{
  uint8_t bsn;        /* Beacon sequence number */

  /* PAN descriptor for the received beacon */

  struct ieee802154_pan_desc_s pan_desc;

  /* Beacon pending addresses */

  struct ieee802154_pend_addr_s pend_addr;

  uint8_t sdu_length; /* Number of octets contained in the beacon
                       * payload of the received beacond frame */
      
  /* Beacon payload */

  uint8_t sdu[IEEE802154_aMaxBeaconPayloadLength];
};

/* Operations */

struct ieee802154_mac_s; /* Forward reference */

struct ieee802154_macops_s
{
  /* Requests, confirmed asynchronously via callbacks */

  /* Transmit a data frame */

  CODE int (*req_data)(FAR struct ieee802154_mac_s *mac, uint8_t handle,
                       FAR uint8_t *buf, int len);

  /* Cancel transmission of a data frame */

  CODE int (*req_purge)(FAR struct ieee802154_mac_s *mac, uint8_t handle);

  /* Start association with coordinator */

  CODE int (*req_associate)(FAR struct ieee802154_mac_s *mac,
                            uint16_t panid, FAR uint8_t *coordeadr);

  /* Start disassociation with coordinator */

  CODE int (*req_disassociate)(FAR struct ieee802154_mac_s *mac,
                               FAR uint8_t *eadr, uint8_t reason);

  /* Read the PIB */

  CODE int (*req_get)(FAR struct ieee802154_mac_s *mac, int attribute);

  /* Allocate or deallocate a GTS */

  CODE int (*req_gts)(FAR struct ieee802154_mac_s *mac,
                      FAR uint8_t *characteristics);

  /* MAC layer reset */

  CODE int (*req_reset)(FAR struct ieee802154_mac_s *mac, bool setdefaults);

  /* PHY receiver control */

  CODE int (*req_rxenable)(FAR struct ieee802154_mac_s *mac, bool deferrable,
                           int ontime, int duration);

  /* Start a network scan */

  CODE int (*req_scan)(FAR struct ieee802154_mac_s *mac, uint8_t type,
                       uint32_t channels, int duration);

  /* Change the PIB */

  CODE int (*req_set)(FAR struct ieee802154_mac_s *mac, int attribute,
                      FAR uint8_t *value, int valuelen);

  CODE int (*req_start)(FAR struct ieee802154_mac_s *mac, uint16_t panid,
                        int channel, uint8_t bo, uint8_t fo, bool coord,
                        bool batext, bool realign);

  CODE int (*req_sync)(FAR struct ieee802154_mac_s *mac, int channel,
                       bool track);

  CODE int (*req_poll)(FAR struct ieee802154_mac_s *mac,
                       FAR uint8_t *coordaddr);

  /* Synchronous Responses to Indications received via callbacks */

  /* Reply to an association request */

  CODE int (*rsp_associate)(FAR struct ieee802154_mac_s *mac, uint8_t eadr,
                            uint16_t saddr, int status);

  /* Orphan device management */

  CODE int (*rsp_orphan)(FAR struct ieee802154_mac_s *mac,
                         FAR uint8_t *orphanaddr, uint16_t saddr,
                         bool associated);
};

/* Notifications */

struct ieee802154_maccb_s
{
  /* Asynchronous confirmations to requests */

  /* Data frame was received by remote device */

  CODE int (*conf_data)(FAR struct ieee802154_mac_s *mac,
                        FAR uint8_t *buf, int len);

  /* Data frame was purged */

  CODE int (*conf_purge)(FAR struct ieee802154_mac_s *mac, uint8_t handle,
                         int status);

  /* Association request completed */

  CODE int (*conf_associate)(FAR struct ieee802154_mac_s *mac,
                             uint16_t saddr, int status);

  /* Disassociation request completed */

  CODE int (*conf_disassociate)(FAR struct ieee802154_mac_s *mac,
                                int status);

  /* PIB data returned */

  CODE int (*conf_get)(FAR struct ieee802154_mac_s *mac, int status,
                       int attribute, FAR uint8_t *value,
                       int valuelen);

  /* GTS management completed */

  CODE int (*conf_gts)(FAR struct ieee802154_mac_s *mac,
                       FAR uint8_t *characteristics, int status);

  /* MAC reset completed */

  CODE int (*conf_reset)(FAR struct ieee802154_mac_s *mac, int status);

  CODE int (*conf_rxenable)(FAR struct ieee802154_mac_s *mac, int status);

  CODE int (*conf_scan)(FAR struct ieee802154_mac_s *mac, int status,
                        uint8_t type, uint32_t unscanned, int rsltsize,
                        FAR uint8_t *edlist, FAR uint8_t *pandescs);

  CODE int (*conf_set)(FAR struct ieee802154_mac_s *mac, int status,
                       int attribute);

  CODE int (*conf_start)(FAR struct ieee802154_mac_s *mac, int status);

  CODE int (*conf_poll)(FAR struct ieee802154_mac_s *mac, int status);

  /* Asynchronous event indications, replied to synchronously with responses */

  /* Data frame received */

  CODE int (*ind_data)(FAR struct ieee802154_mac_s *mac, FAR uint8_t *buf,
                       int len);

  /* Association request received */

  CODE int (*ind_associate)(FAR struct ieee802154_mac_s *mac,
                            uint16_t clipanid, FAR uint8_t *clieaddr);

   /* Disassociation request received */

  CODE int (*ind_disassociate)(FAR struct ieee802154_mac_s *mac,
                               FAR uint8_t *eadr, uint8_t reason);

  /* Beacon notification */

  CODE int (*ind_beaconnotify)(FAR struct ieee802154_mac_s *mac,
                               FAR uint8_t *bsn, FAR struct ieee802154_pan_desc_s *pandesc,
                               FAR uint8_t *sdu, int sdulen);

  /* GTS management request received */

  CODE int (*ind_gts)(FAR struct ieee802154_mac_s *mac,
                      FAR uint8_t *devaddr, FAR uint8_t *characteristics);

  /* Orphan device detected */

  CODE int (*ind_orphan)(FAR struct ieee802154_mac_s *mac,
                         FAR uint8_t *orphanaddr);

  CODE int (*ind_commstatus)(FAR struct ieee802154_mac_s *mac,
                             uint16_t panid, FAR uint8_t *src,
                             FAR uint8_t *dst, int status);

  CODE int (*ind_syncloss)(FAR struct ieee802154_mac_s *mac, int reason);
};

struct ieee802154_radio_s; /* Forware reference */

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

#if 0 /* REVISIT: This form is not currently used by the driver */
FAR struct ieee802154_mac_s *
  mac802154_register(FAR struct ieee802154_radio_s *radiodev,
                     FAR struct ieee802154_maccb_s *callbacks);
#else /* This is the form used by the driver */
FAR struct ieee802154_mac_s *
  mac802154_register(FAR struct ieee802154_radio_s *radiodev,
                     unsigned int minor);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_MAC_H */
