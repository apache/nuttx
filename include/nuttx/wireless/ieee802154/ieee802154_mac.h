/****************************************************************************
 * include/nuttx/wireless/ieee802154/ieee802154_mac.h
 *
 *   Copyright (C) 2016 Sebastien Lorquet. All rights reserved.
 *   Copyright (C) 2017 Verge Inc. All rights reserved.
 *
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *   Author: Anthony Merlino <anthony@vergeaero.com>
 *
 *   The naming and comments for various fields are taken directly
 *   from the IEEE 802.15.4 2011 standard.
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

#ifdef CONFIG_NET_6LOWPAN
#  include <net/if.h>
#endif

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

#define MAC802154IOC_MCPS_REGISTER             _MAC802154IOC(0x0001)

#define MAC802154IOC_MLME_REGISER              _MAC802154IOC(0x0002);
#define MAC802154IOC_MLME_ASSOC_REQUEST        _MAC802154IOC(0x0003)
#define MAC802154IOC_MLME_ASSOC_RESPONSE       _MAC802154IOC(0x0004)
#define MAC802154IOC_MLME_DISASSOC_REQUEST     _MAC802154IOC(0x0005)
#define MAC802154IOC_MLME_GET_REQUEST          _MAC802154IOC(0x0006)
#define MAC802154IOC_MLME_GTS_REQUEST          _MAC802154IOC(0x0007)
#define MAC802154IOC_MLME_ORPHAN_RESPONSE      _MAC802154IOC(0x0008)
#define MAC802154IOC_MLME_RESET_REQUEST        _MAC802154IOC(0x0009)
#define MAC802154IOC_MLME_RXENABLE_REQUEST     _MAC802154IOC(0x000A)
#define MAC802154IOC_MLME_SCAN_REQUEST         _MAC802154IOC(0x000B)
#define MAC802154IOC_MLME_SET_REQUEST          _MAC802154IOC(0x000C)
#define MAC802154IOC_MLME_START_REQUEST        _MAC802154IOC(0x000D)
#define MAC802154IOC_MLME_SYNC_REQUEST         _MAC802154IOC(0x000E)
#define MAC802154IOC_MLME_POLL_REQUEST         _MAC802154IOC(0x000F)
#define MAC802154IOC_MLME_DPS_REQUEST          _MAC802154IOC(0x0010)
#define MAC802154IOC_MLME_SOUNDING_REQUEST     _MAC802154IOC(0x0011)
#define MAC802154IOC_MLME_CALIBRATE_REQUEST    _MAC802154IOC(0x0012)



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

#define IEEE802154_MAX_PHY_PACKET_SIZE        127
#define IEEE802154_TURN_AROUND_TIME             12 /*symbol periods*/

/* IEEE 802.15.4 MAC constants */

#define IEEE802154_BASE_SLOT_DURATION         60
#define IEEE802154_NUM_SUPERFRAME_SLOTS       16

#define IEEE802154_BASE_SUPERFRAME_DURATION \
        (IEEE802154_BASE_SLOT_DURATION * IEEE802154_NUM_SUPERFRAME_SLOTS)

#define IEEE802154_GTS_DESC_PERSISTENCE_TIME  4
#define IEEE802154_MAX_BEACON_OVERHEAD        75

#define IEEE802154_MAX_BEACON_PAYLOAD_LENGTH \
        (IEEE802154_MAX_PHY_PACKET_SIZE - IEEE802154_MAX_BEACON_OVERHEAD)

#define IEEE802154_MAX_LOST_BEACONS           4
#define IEEE802514_MIN_MPDU_OVERHEAD          9
#define IEEE802154_MAX_MPDU_UNSEC_OVERHEAD    25

#define IEEE802154_MAX_SAFE_MAC_PAYLOAD_SIZE \
        (IEEE802154_MAX_PHY_PACKET_SIZE - IEEE802154_MAX_MPDU_UNSEC_OVERHEAD)
  
#define IEEE802154_MAX_MAC_PAYLOAD_SIZE \
        (IEEE802154_MAX_PHY_PACKET_SIZE - IEEE802154_MIN_MPDU_OVERHEAD)

#define IEEE802154_MAX_SIFS_FRAME_SIZE        18
#define IEEE802154_MIN_CAP_LENGTH             440
#define IEEE802154_UNIT_BACKOFF_PERIOD        20

/* IEEE 802.15.4 MAC PIB Attribut Defaults */

/* Definitions used by IOCTL calls */

#define MAX_ORPHAN_ADDR   32  /* REVISIT */

// TODO: Add macros

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

enum ieee802154_pib_attr_e
{
  /* PHY PIB Attributes */

  IEEE802154_PIB_PHY_CURRENT_CHANNEL = 0x00,
  IEEE802154_PIB_PHY_CHANNELS_SUPPORTED,
  IEEE802154_PIB_PHY_TX_POWER_TOLERANCE,
  IEEE802154_PIB_PHY_TX_POWER,
  IEEE802154_PIB_PHY_CCA_MODE,
  IEEE802154_PIB_PHY_CURRENT_PAGE,
  IEEE802154_PIB_PHY_MAX_FRAME_DURATION,
  IEEE802154_PIB_PHY_SHR_DURATION,
  IEEE802154_PIB_PHY_SYM_PER_OCTET,
  IEEE802154_PIB_PHY_PREAMBLE_SYM_LEN,
  IEEE802154_PIB_PHY_UWB_DATARATES_SUP,
  IEEE802154_PIB_PHY_CSS_LOW_DATARATE_SUP,
  IEEE802154_PIB_PHY_UWB_COU_PULSES_SUP,
  IEEE802154_PIB_PHY_UWB_CS_PULSES_SUP,
  IEEE802154_PIB_PHY_UWB_LCP_PULSES_SUP,
  IEEE802154_PIB_PHY_UWB_CURR_PULSE_SHAPE,
  IEEE802154_PIB_PHY_UWB_COU_PULSE,
  IEEE802154_PIB_PHY_UWB_CS_PULSE,
  IEEE802154_PIB_PHY_UWB_LCP_WEIGHT1,
  IEEE802154_PIB_PHY_UWB_LCP_WEIGHT2,
  IEEE802154_PIB_PHY_UWB_LCP_WEIGHT3,
  IEEE802154_PIB_PHY_UWB_LCP_WEIGHT4,
  IEEE802154_PIB_PHY_UWB_LCP_DELAY2,
  IEEE802154_PIB_PHY_UWB_LCP_DELAY3,
  IEEE802154_PIB_PHY_UWB_LCP_DELAY4,
  IEEE802154_PIB_PHY_RANGING,
  IEEE802154_PIB_PHY_RANGING_CRYSTAL_OFFSET,
  IEEE802154_PIB_PHY_RANGING_DPS,
  IEEE802154_PIB_PHY_CURRENT_CODE,
  IEEE802154_PIB_PHY_NATIVE_PRF,
  IEEE802154_PIB_PHY_UWB_SCAN_BINS_PER_CHAN,
  IEEE802154_PIB_PHY_UWB_INS_PREAMBLE_INTERVAL,
  IEEE802154_PIB_PHY_UWB_TX_RMARKER,
  IEEE802154_PIB_PHY_UWB_RX_RMARKER,
  IEEE802154_PIB_PHY_RFRAME_PROC_TIME,
  IEEE802154_PIB_PHY_CCA_DURATION,

  /* MAC PIB Attributes */

  IEEE802154_PIB_MAC_EXTENDED_ADDR = 0x40,
  IEEE802154_PIB_MAC_ACK_WAIT_DUR,
  IEEE802154_PIB_MAC_ASSOCIATED_PANCOORD,
  IEEE802154_PIB_MAC_ASSOCIATION_PERMIT,
  IEEE802154_PIB_MAC_AUTO_REQUEST,
  IEEE802154_PIB_MAC_BATT_LIFE_EXT,
  IEEE802154_PIB_MAC_BATT_LIFE_EXT_PERIODS,
  IEEE802154_PIB_MAC_BEACON_PAYLOAD,
  IEEE802154_PIB_MAC_BEACON_PAYLOAD_LEN,
  IEEE802154_PIB_MAC_BEACON_ORDER,
  IEEE802154_PIB_MAC_BEACON_TX_TIME,
  IEEE802154_PIB_MAC_BSN,
  IEEE802154_PIB_MAC_COORD_EXT_ADDR,
  IEEE802154_PIB_MAC_COORD_SHORT_ADDR,
  IEEE802154_PIB_MAC_DSN,
  IEEE802154_PIB_MAC_GTS_PERMIT,
  IEEE802154_PIB_MAC_MAX_BE,
  IEEE802154_PIB_MAC_MAX_CSMA_BACKOFFS,
  IEEE802154_PIB_MAC_FRAME_TOTAL_WAIT_TIME,
  IEEE802154_PIB_MAC_MAX_FRAME_RETRIES,
  IEEE802154_PIB_MAC_MIN_BE,
  IEEE802154_PIB_MAC_LIFS_PERIOD,
  IEEE802154_PIB_MAC_SIFS_PERIOD,
  IEEE802154_PIB_MAC_PAN_ID,
  IEEE802154_PIB_MAC_PROMISCUOUS_MODE,
  IEEE802154_PIB_MAC_RANGING_SUPPORT,
  IEEE802154_PIB_MAC_RESPONSE_WAIT_TIME,
  IEEE802154_PIB_MAC_RX_ON_WHEN_IDLE,
  IEEE802154_PIB_MAC_SECURITY_ENABLED,
  IEEE802154_PIB_MAC_SHORT_ADDRESS,
  IEEE802154_PIB_MAC_SUPERFRAME_ORDER,
  IEEE802154_PIB_MAC_SYNC_SYMBOL_OFFSET,
  IEEE802154_PIB_MAC_TIMESTAMP_SUPPORT,
  IEEE802154_PIB_MAC_TRANSACTION_PERSIST_TIME,
  IEEE802154_PIB_MAC_TX_CTRL_ACTIVE_DUR,
  IEEE802154_PIB_MAC_TX_CTRL_PAUSE_DUR,
  IEEE802154_PIB_MAC_TX_TOTAL_DUR,

  /* MAC Security Attributes */

  IEEE802154_PIB_MAC_KEY_TABLE = 0x70,
  IEEE802154_PIB_MAC_DEV_TABLE,
  IEEE802154_PIB_MAC_SEC_LVL_TABLE,
  IEEE802154_PIB_MAC_FRAME_COUNTER,
  IEEE802154_PIB_MAC_AUTOREQ_SEC_LVL,
  IEEE802154_PIB_MAC_AUTOREQ_KEY_ID_MODE,
  IEEE802154_PIB_MAC_AUTOREQ_KEY_SOURCE,
  IEEE802154_PIB_MAC_AUTOREQ_KEY_INDEX,
  IEEE802154_PIB_MAC_DEFAULT_KEY_SRC,
  IEEE802154_PIB_MAC_PANCOORD_EXT_ADDR,
  IEEE802154_PIB_MAC_PANCOORD_SHORT_ADDR,
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
  /* Address mode. Short or Extended */

  enum ieee802154_addr_mode_e ia_mode;  

  uint16_t ia_panid;        /* PAN identifier, can be IEEE802154_PAN_UNSPEC */
  union
  {
    uint16_t _ia_saddr;     /* short address */
    uint8_t  _ia_eaddr[8];  /* extended address */
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

#ifdef CONFIG_IEEE802154_SECURITY
struct ieee802154_security_s
{
  uint8_t level;            /* Security level to be used */
  uint8_t key_id_mode;      /* Mode used to identify the key to be used */
  uint8_t key_source[8];    /* Originator of the key to be used */
  uint8_t key_index;        /* Index of the key to be used */
};
#endif

#ifdef CONFIG_IEEE802154_UWB
enum ieee802154_uwbprf_e
{
  IEEE802154_UWBPRF_OFF = 0,
  IEEE802154_UWBPRF_4M,
  IEEE802154_UWBPRF_16M,
  IEEE802154_UWBPRF_64M
};

enum ieee802154_uwb_datarate_e
{
  IEEE802154_UWB_DATARATE_0 = 0,
  IEEE802154_UWB_DATARATE_16,
  IEEE802154_UWB_DATARATE_64,
  IEEE802154_UWB_DATARATE_1024,
  IEEE802154_UWB_DATARATE_4096
};
#endif

enum ieee802154_ranging_e
{
  IEEE802154_NON_RANGING = 0,
  IEEE802154_ALL_RANGING,
  IEEE802154_PHY_HEADER_ONLY
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

struct ieee802154_data_req_s
{
  enum ieee802154_addr_mode_e src_addr_mode;  /* Source Address Mode */
  struct ieee802154_addr_s dest__addr;        /* Destination Address */
  
  /* Number of bytes contained in the MAC Service Data Unit (MSDU) 
   * to be transmitted by the MAC sublayer enitity 
   * Note: This could be a uint8_t but if anyone ever wants to use
   * non-standard frame lengths, they may want a length larger than
   * a uint8_t */

  uint16_t msdu_length;


  uint8_t msdu_handle;    /* Handle assoc. with MSDU */
  struct
  {
    uint8_t ack_tx      : 1;  /* Acknowledge TX? */               
    uint8_t gts_tx      : 1;  /* 1=GTS used for TX, 0=CAP used for TX */
    uint8_t indirect_tx : 1;  /* Should indirect transmission be used? */
  };

#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s security;
#endif

#ifdef CONFIG_IEEE802154_UWB
  /* The UWB Pulse Repetion Frequency to be used for the transmission */

  enum ieee802154_uwbprf_e uwb_prf;

  /* The UWB preamble symbol repititions
   *  Should be one of:
   *    0, 16, 64, 1024, 4096
   */

  uint16_t uwb_presym_rep;  

  /* The UWB Data Rate to be used for the transmission */

  enum ieee802154_uwb_datarate_e data_rate;
#endif

  enum ieee802154_ranging_e ranging;

  /* The MAC service data unit array that is to be transmitted
   * This must be at the end of the struct to allow the array
   * to continue and make the struct "variable length"
   */ 

  uint8_t msdu[1];
};

#define SIZEOF_IEEE802154_DATA_REQ_S(n) \
        (sizeof(struct ieee802154_data_req_s) + (n))

struct ieee802154_data_conf_s
{
  uint8_t msdu_handle;              /* Handle assoc. with MSDU */

  /* The time, in symbols, at which the data were transmitted */

  uint32_t timestamp;

  enum ieee802154_status_e status;  /* The status of the MSDU transmission */

#ifdef CONFIG_IEEE802154_RANGING
  bool rng_rcvd;                    /* Ranging indicated by MSDU */

  /* A count of the time units corresponding to an RMARKER at the antenna at
   * the beginning of the ranging exchange
   */

  uint32_t rng_counter_start; 

  /* A count of the time units corresponding to an RMARKER at the antenna at
   * end of the ranging exchange
   */

  uint32_t rng_counter_stop; 

  /* A count of the time units in a message exchange over which the tracking
   * offset was measured
   */

  uint34_t rng_tracking_interval;

  /* A count of the time units slipped or advanced by the radio tracking
   * system over the course of the entire tracking interval
   */

  uint32_t rng_offset;

  /* The Figure of Merit (FoM) characterizing the ranging measurement */ 

  uint8_t rng_fom; 
#endif
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
  uint8_t lqi;              /* Link Quality Indication of the beacon */
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

struct ieee802154_assoc_req_s
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

struct ieee802154_assoc_ind_s
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

struct ieee802154_assoc_rsp_s
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

struct ieee802154_assoc_conf_s
{
  /* Associated device address ALWAYS passed in short address mode. The
   * address will be IEEE802154_SADDR_UNSPEC if association was unsuccessful.
   */

  struct ieee802154_addr_s dev_addr;

  /* Status of association attempt */

  enum ieee802154_status_e status;

#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s security;
#endif
};

struct ieee802154_disassoc_req_s
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

struct ieee802154_disassoc_ind_s
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

struct ieee802154_disassoc_conf_s
{
  /* Status of the disassociation attempt */

  enum ieee802154_status_e status;

  /* Address of device either requesting or being intructed to disassociate */

  struct ieee802154_addr_s dev_addr;
};

struct ieee802154_beaconnotify_ind_s
{
  uint8_t bsn;        /* Beacon sequence number */

  /* PAN descriptor for the received beacon */

  struct ieee802154_pan_desc_s pan_desc;

  /* Beacon pending addresses */

  struct ieee802154_pend_addr_s pend_addr;

  uint8_t sdu_length; /* Number of octets contained in the beacon
                       * payload of the received beacond frame */
      
  /* Beacon payload */

  uint8_t sdu[IEEE802154_MAX_BEACON_PAYLOAD_LENGTH];
};

#define SIZEOF_IEEE802154_BEACONNOTIFY_IND_S(n) \
  (sizeof(struct ieee802154_beaconnotify_ind_s) \
  - IEEE802154_MAX_BEACON_PAYLOAD_LENGTH + (n))

/* IOCTL data arguments *****************************************************/

/* Data returned with MAC802154IOC_MLME_ASSOC_RESPONSE */

struct ieee802154_assocresp_s
{
  uint8_t eadr;
  uint16_t saddr;
  int status;
};

/* Data provided to MAC802154IOC_MLME_GET_REQUEST */

struct ieee802154_getreq_s
{
  enum ieee802154_pib_attr_e attr;
};

/* Data provided to MAC802154IOC_MLME_GTS_REQUEST */

struct ieee802154_gtsreq_s
{
  uint8_t characteristics;
};

/* Data returned with MAC802154IOC_MLME_ORPHAN_RESPONSE */

struct ieee802154_orphanresp_s
{
  uint8_t orphanaddr[MAX_ORPHAN_ADDR];
  uint16_t saddr;
  bool associated;
};

/* Data provided with MAC802154IOC_MLME_RESET_REQUEST */

struct ieee802154_resetreq_s
{
  bool setdefaults;
};

/* Data provided with MAC802154IOC_MLME_RXENABLE_REQUEST */

struct ieee802154_rxenabreq_s
{
  bool deferrable;
  int ontime;
  int duration;
};

/* Data provided with MAC802154IOC_MLME_SCAN_REQUEST */

struct ieee802154_scanreq_s
{
  uint8_t type;
  uint32_t channels;
  int duration;
};

/* Data provided with MAC802154IOC_MLME_SET_REQUEST */

struct ieee802154_setreq_s
{
  FAR uint8_t *value;
  int valuelen;
  int attribute;
};

/* Data provided with MAC802154IOC_MLME_START_REQUEST */

struct ieee802154_startreq_s
{
  int channel;
  uint16_t panid;
  uint8_t bo;
  uint8_t fo;
  bool coord;
  bool batext;
  bool realign;
};

/* Data provided with MAC802154IOC_MLME_SYNC_REQUEST */

struct ieee802154_syncreq_s
{
  int channel;
  bool track;
};

/* Data provided with MAC802154IOC_MLME_POLL_REQUEST */

struct ieee802154_pollreq_s
{
  FAR uint8_t *coordaddr;
};

/* A pointer to this structure is passed as the argument of each IOCTL
 * command.
 */

union ieee802154_macarg_u
{
  struct ieee802154_assoc_req_s    assocreq;    /* MAC802154IOC_MLME_ASSOC_REQUEST */
  struct ieee802154_assocresp_s    assocresp;   /* MAC802154IOC_MLME_ASSOC_RESPONSE */
  struct ieee802154_disassoc_req_s disassocreq; /* MAC802154IOC_MLME_DISASSOC_REQUEST */
  struct ieee802154_getreq_s       getreq;      /* MAC802154IOC_MLME_GET_REQUEST */
  struct ieee802154_gtsreq_s       gtsreq;      /* MAC802154IOC_MLME_GTS_REQUEST */
  struct ieee802154_orphanresp_s   orphanresp;  /* MAC802154IOC_MLME_ORPHAN_RESPONSE */
  struct ieee802154_resetreq_s     resetreq;    /* MAC802154IOC_MLME_RESET_REQUEST */
  struct ieee802154_rxenabreq_s    rxenabreq;   /* MAC802154IOC_MLME_RXENABLE_REQUEST */
  struct ieee802154_scanreq_s      scanreq;     /* MAC802154IOC_MLME_SCAN_REQUEST */
  struct ieee802154_setreq_s       setreq;      /* MAC802154IOC_MLME_SET_REQUEST */
  struct ieee802154_startreq_s     startreq;    /* MAC802154IOC_MLME_START_REQUEST */
  struct ieee802154_syncreq_s      syncreq;     /* MAC802154IOC_MLME_SYNC_REQUEST */
  struct ieee802154_pollreq_s      pollreq;     /* MAC802154IOC_MLME_POLL_REQUEST */
  /* To be determined */                        /* MAC802154IOC_MLME_DPS_REQUEST */
  /* To be determined */                        /* MAC802154IOC_MLME_SOUNDING_REQUEST */
  /* To be determined */                        /* MAC802154IOC_MLME_CALIBRATE_REQUEST */
};

#ifdef CONFIG_NET_6LOWPAN
/* For the case of network IOCTLs, the network IOCTL to the MAC network
 * driver will include a device name like "wpan0" as the destination of
 * the IOCTL command.
 */

struct ieee802154_netmac_s
{
  char ifr_name[IFNAMSIZ];     /* Interface name, e.g. "wpan0" */
  union ieee802154_macarg_u u; /* Data payload */
};
#endif

/* This is an opaque reference to the MAC's internal private state.  It is
 * returned by mac802154_create() when it is created.  It may then be used
 * at other interfaces in order to interact with the MAC.
 */

typedef FAR void *MACHANDLE;

/* Notifications */

struct ieee802154_maccb_s
{
  /* Asynchronous confirmations to requests */

  /* Data frame was received by remote device */

  CODE void (*conf_data)(MACHANDLE mac,
                         FAR struct ieee802154_data_conf_s *conf);

  /* Data frame was purged */

  CODE void (*conf_purge)(MACHANDLE mac, uint8_t handle, int status);

  /* Association request completed */

  CODE void (*conf_associate)(MACHANDLE mac, uint16_t saddr, int status);

  /* Disassociation request completed */

  CODE void (*conf_disassociate)(MACHANDLE mac, int status);

  /* PIB data returned */

  CODE void (*conf_get)(MACHANDLE mac, int status, int attribute,
                        FAR uint8_t *value, int valuelen);

  /* GTS management completed */

  CODE void (*conf_gts)(MACHANDLE mac, FAR uint8_t *characteristics,
                        int status);

  /* MAC reset completed */

  CODE void (*conf_reset)(MACHANDLE mac, int status);

  CODE void (*conf_rxenable)(MACHANDLE mac, int status);

  CODE void (*conf_scan)(MACHANDLE mac, int status, uint8_t type,
                         uint32_t unscanned, int rsltsize,
                         FAR uint8_t *edlist, FAR uint8_t *pandescs);

  CODE void (*conf_set)(MACHANDLE mac, int status, int attribute);

  CODE void (*conf_start)(MACHANDLE mac, int status);

  CODE void (*conf_poll)(MACHANDLE mac, int status);

  /* Asynchronous event indications, replied to synchronously with responses */

  /* Data frame received */

  CODE void (*ind_data)(MACHANDLE mac, FAR uint8_t *buf, int len);

  /* Association request received */

  CODE void (*ind_associate)(MACHANDLE mac, uint16_t clipanid,
                             FAR uint8_t *clieaddr);

   /* Disassociation request received */

  CODE void (*ind_disassociate)(MACHANDLE mac, FAR uint8_t *eadr,
                                uint8_t reason);

  /* Beacon notification */

  CODE void (*ind_beaconnotify)(MACHANDLE mac, FAR uint8_t *bsn,
                                FAR struct ieee802154_pan_desc_s *pandesc,
                                FAR uint8_t *sdu, int sdulen);

  /* GTS management request received */

  CODE void (*ind_gts)(MACHANDLE mac, FAR uint8_t *devaddr,
                       FAR uint8_t *characteristics);

  /* Orphan device detected */

  CODE void (*ind_orphan)(MACHANDLE mac, FAR uint8_t *orphanaddr);

  CODE void (*ind_commstatus)(MACHANDLE mac, uint16_t panid,
                              FAR uint8_t *src, FAR uint8_t *dst,
                              int status);

  CODE void (*ind_syncloss)(MACHANDLE mac, int reason);
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

struct ieee802154_radio_s; /* Forward reference */

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

MACHANDLE mac802154_create(FAR struct ieee802154_radio_s *radiodev);

/****************************************************************************
 * Name: mac802154dev_register
 *
 * Description:
 *   Register a character driver to access the IEEE 802.15.4 MAC layer from
 *   user-space
 *
 * Input Parameters:
 *   mac - Pointer to the mac layer struct to be registered.
 *   minor - The device minor number.  The IEEE802.15.4 MAC character device
 *     will be registered as /dev/ieeeN where N is the minor number
 *
 * Returned Values:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int mac802154dev_register(MACHANDLE mac, int minor);

/****************************************************************************
 * Name: mac802154netdev_register
 *
 * Description:
 *   Register a network driver to access the IEEE 802.15.4 MAC layer from
 *   a socket using 6loWPAN
 *
 * Input Parameters:
 *   mac - Pointer to the mac layer struct to be registered.
 *
 * Returned Values:
 *   Zero (OK) is returned on success.  Otherwise a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int mac802154netdev_register(MACHANDLE mac);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_IEEE802154_IEEE802154_MAC_H */
