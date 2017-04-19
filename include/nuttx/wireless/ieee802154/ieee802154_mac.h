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

#define MAC802154IOC_MLME_REGISTER             _MAC802154IOC(0x0002)
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
#define IEEE802154_CMD_ASSOC_RESP     0x02
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

#define IEEE802154_FRAMECTRL_SHIFT_FTYPE      0  /* Frame type, bits 0-2 */
#define IEEE802154_FRAMECTRL_SHIFT_SEC        3  /* Security Enabled, bit 3 */
#define IEEE802154_FRAMECTRL_SHIFT_PEND       4  /* Frame pending, bit 4 */
#define IEEE802154_FRAMECTRL_SHIFT_ACKREQ     5  /* Acknowledge request, bit 5 */
#define IEEE802154_FRAMECTRL_SHIFT_PANIDCOMP  6  /* PAN ID Compression, bit 6 */
#define IEEE802154_FRAMECTRL_SHIFT_DADDR      10 /* Dest addressing mode, bits 10-11 */
#define IEEE802154_FRAMECTRL_SHIFT_VERSION    12 /* Source addressing mode, bits 12-13 */
#define IEEE802154_FRAMECTRL_SHIFT_SADDR      14 /* Source addressing mode, bits 14-15 */

/* IEEE 802.15.4 PHY constants */

#define IEEE802154_MAX_PHY_PACKET_SIZE        127
#define IEEE802154_TURN_AROUND_TIME           12 /*symbol periods*/

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
#define IEEE802154_MIN_MPDU_OVERHEAD          9
#define IEEE802154_MAX_UNSEC_MHR_OVERHEAD     23
#define IEEE802154_MFR_LENGTH                 2

#define IEEE802154_MAX_MPDU_UNSEC_OVERHEAD  \
        (IEEE802154_MAX_UNSEC_MHR_OVERHEAD + IEEE802154_MFR_LENGTH)
 

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

  enum ieee802154_addr_mode_e mode;  

  uint16_t panid;     /* PAN identifier, can be IEEE802154_PAN_UNSPEC */
  uint16_t saddr;     /* short address */
  uint8_t  eaddr[8];  /* extended address */
};

#define IEEE802154_ADDRSTRLEN 22 /* (2*2+1+8*2, PPPP/EEEEEEEEEEEEEEEE) */

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
    } pa_addr;
  } u;
  struct ieee802154_addr_s addr[7]; /* Array of at most 7 addresses */
};

/* Primitive Support Types ***************************************************/

union ieee802154_attr_val_u
{
  /* TODO: Finish this */
};

struct ieee802154_gts_info_s
{
  uint8_t length      : 4; /* Number of SF slots for GTS */
  uint8_t direction   : 1; /* 0=transmit-only, 1=receive-only */
  uint8_t type        : 1; /* 0=GTS deallocation, 1= GTS allocation */
  uint8_t reserved    : 2;
};

enum ieee802154_scantype_e
{
  IEEE802154_SCANTYPE_ED,
  IEEE802154_SCANTYPE_ACTIVE,
  IEEE802154_SCANTYPE_PASSIVE,
  IEEE802154_SCANTYPE_ORPHAN
};

/* Primitive Semantics *******************************************************/

/*****************************************************************************
 * Primitive: MCPS-DATA.request 
 *
 * Description:
 *    Requests the transfer of data to another device.  
 * 
 *****************************************************************************/

struct ieee802154_data_req_s
{
  enum ieee802154_addr_mode_e src_addr_mode;  /* Source Address Mode */
  struct ieee802154_addr_s dest_addr;         /* Destination Address */
  
  /* Number of bytes contained in the MAC Service Data Unit (MSDU) 
   * to be transmitted by the MAC sublayer enitity 
   * Note: This could be a uint8_t but if anyone ever wants to use
   * non-standard frame lengths, they may want a length larger than
   * a uint8_t.
   */

  uint16_t msdu_length;

  uint8_t msdu_handle;    /* Handle assoc. with MSDU */
  struct
  {
    uint8_t ack_tx      : 1;  /* Acknowledge TX? */               
    uint8_t gts_tx      : 1;  /* 1=GTS used for TX, 0=CAP used for TX */
    uint8_t indirect_tx : 1;  /* Should indirect transmission be used? */
  } msdu_flags;

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
        (sizeof(struct ieee802154_data_req_s) + (n) - 1)

/*****************************************************************************
 * Primitive: MCPS-DATA.confirm 
 *
 * Description:
 *    Reports the results of a request to transfer data to another device.
 * 
 *****************************************************************************/

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

/*****************************************************************************
 * Primitive: MCPS-DATA.indication 
 *
 * Description:
 *    Indicates the reception of data from another device.
 * 
 *****************************************************************************/

struct ieee802154_data_ind_s
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

/*****************************************************************************
 * Primitive: MCPS-PURGE.request 
 *
 * Description:
 *    Allows the next higher layer to purge an MSDU from the transaction queue.
 * 
 *****************************************************************************/

struct ieee802154_purge_req_s
{
  uint8_t msdu_handle;              /* Handle assoc. with MSDU */
};

/*****************************************************************************
 * Primitive: MCPS-PURGE.confirm 
 *
 * Description:
 *    Allows the MAC sublayer to notify the next higher layer of the success of
 *    its request to purge an MSDU from the transaction queue.
 * 
 *****************************************************************************/

struct ieee802154_purge_conf_s
{
  uint8_t msdu_handle;              /* Handle assoc. with MSDU */
  enum ieee802154_status_e status;  /* The status of the MSDU transmission */
};

/*****************************************************************************
 * Primitive: MLME-ASSOCIATE.request 
 *
 * Description:
 *    Used by a device to request an association with a coordinator.
 * 
 *****************************************************************************/

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

/*****************************************************************************
 * Primitive: MLME-ASSOCIATE.indication 
 *
 * Description:
 *    Used to indicate the reception of an association request command.
 * 
 *****************************************************************************/

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

/*****************************************************************************
 * Primitive: MLME-ASSOCIATE.response 
 *
 * Description:
 *    Used to initiate a response to an MLME-ASSOCIATE.indication primitive.
 * 
 *****************************************************************************/

struct ieee802154_assoc_resp_s
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

/*****************************************************************************
 * Primitive: MLME-ASSOCIATE.confirm 
 *
 * Description:
 *    Used to inform the next higher layer of the initiating device whether its
 *    request to associate was successful or unsuccessful.
 * 
 *****************************************************************************/

struct ieee802154_assoc_conf_s
{
  /* Associated device address ALWAYS passed in short address mode. The
   * address will be IEEE802154_SADDR_UNSPEC if association was
   * unsuccessful.
   */

  struct ieee802154_addr_s dev_addr;

  /* Status of association attempt */

  enum ieee802154_status_e status;

#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s security;
#endif
};

/*****************************************************************************
 * Primitive: MLME-DISASSOCIATE.request 
 *
 * Description:
 *    Used by an associated device to notify the coordinator of its intent to
 *    leave the PAN. It is also used by the coordinator to instruct an
 *    associated device to leave the PAN.
 * 
 *****************************************************************************/

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

/*****************************************************************************
 * Primitive: MLME-DISASSOCIATE.indication 
 *
 * Description:
 *    Used to indicate the reception of a disassociation notification command.
 * 
 *****************************************************************************/

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

/*****************************************************************************
 * Primitive: MLME-DISASSOCIATE.confirm 
 *
 * Description:
 *    Reports the results of an MLME-DISASSOCIATE.request primitive.
 * 
 *****************************************************************************/

struct ieee802154_disassoc_conf_s
{
  /* Status of the disassociation attempt */

  enum ieee802154_status_e status;

  /* Address of device either requesting or being intructed to disassociate */

  struct ieee802154_addr_s dev_addr;
};

/*****************************************************************************
 * Primitive: MLME-BEACONNOTIFY.indication 
 *
 * Description:
 *    Used to send parameters contained within a beacon frame received by the
 *    MAC sublayer to the next higher layer when either macAutoRequest is set to
 *    FALSE or when the beacon frame contains one or more octets of payload. The
 *    primitive also sends a measure of the LQI and the time the beacon frame
 *    was received.
 * 
 *****************************************************************************/

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

/*****************************************************************************
 * Primitive: MLME-COMM-STATUS.indication 
 *
 * Description:
 *    Allows the MLME to indicate a communications status.
 * 
 *****************************************************************************/

struct ieee802154_commstatus_ind_s
{
  struct ieee802154_addr_s src_addr;
  struct ieee802154_addr_s dest_addr;
  enum ieee802154_status_e status;
#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s security;
#endif
};

/*****************************************************************************
 * Primitive: MLME-GET.request 
 *
 * Description:
 *    Requests information about a given PIB attribute.
 * 
 *****************************************************************************/

struct ieee802154_get_req_s
{
  enum ieee802154_pib_attr_e pib_attr;
};

/*****************************************************************************
 * Primitive: MLME-GET.confirm 
 *
 * Description:
 *    Reports the results of an information request from the PIB.
 * 
 *****************************************************************************/

struct ieee802154_get_conf_s
{
  enum ieee802154_status_e status;
  enum ieee802154_pib_attr_e pib_attr;
  union ieee802154_attr_val_u attr_value;
};

/*****************************************************************************
 * Primitive: MLME-GTS.request
 *
 * Description:
 *    Allows a device to send a request to the PAN coordinator to allocate a new
 *    GTS or to deallocate an existing GTS. This primitive is also used by the
 *    PAN coordinator to initiate a GTS deallocation.
 * 
 *****************************************************************************/

struct ieee802154_gts_req_s
{
  struct ieee802154_gts_info_s gts_info;

#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s security;
#endif
};

/*****************************************************************************
 * Primitive: MLME-GTS.confirm
 *
 * Description:
 *    Reports the results of a request to allocate a new GTS or to deallocate an
 *    existing GTS.
 * 
 *****************************************************************************/

struct ieee802154_gts_conf_s
{
  struct ieee802154_gts_info_s gts_info;
  enum ieee802154_status_e status;
};

/*****************************************************************************
 * Primitive: MLME-GTS.indication
 *
 * Description:
 *    Indicates that a GTS has been allocated or that a previously allocated
 *    GTS has been deallocated.
 * 
 *****************************************************************************/

struct ieee802154_gts_ind_s
{
  uint16_t dev_addr;
  struct ieee802154_gts_info_s gts_info;
#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s security;
#endif
};

/*****************************************************************************
 * Primitive: MLME-ORPHAN.indication
 *
 * Description:
 *    Generated by the MLME of a coordinator and issued to its next higher layer
 *    on receipt of an orphan notification command.
 * 
 *****************************************************************************/

struct ieee802154_orphan_ind_s
{
  uint8_t orphan_addr[8];
#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s security;
#endif
};

/*****************************************************************************
 * Primitive: MLME-ORPHAN.response
 *
 * Description:
 *    Allows the next higher layer of a coordinator to respond to the
 *    MLME-ORPHAN.indication primitive.
 * 
 *****************************************************************************/

struct ieee802154_orphan_resp_s
{
  uint8_t orphan_addr[8];
#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s security;
#endif
};

/*****************************************************************************
 * Primitive: MLME-RESET.request
 *
 * Description:
 *    Used by the next higher layer to request that the MLME performs a reset
 *    operation.
 * 
 *****************************************************************************/

struct ieee802154_reset_req_s
{
  bool rst_pibattr;
};

/*****************************************************************************
 * Primitive: MLME-RESET.confirm
 *
 * Description:
 *    Reports the results of the reset operation.
 * 
 *****************************************************************************/

struct ieee802154_reset_conf_s
{
  enum ieee802154_status_e status;
};

/*****************************************************************************
 * Primitive: MLME-RXENABLE.request
 *
 * Description:
 *    Allows the next higher layer to request that the receiver is either
 *    enabled for a finite period of time or disabled.
 * 
 *****************************************************************************/

struct ieee802154_rxenable_req_s
{
  /* Number of symbols measured from the start of the superframe before the
   * receiver is to be enabled or disabled. */

  uint32_t rxon_time;        

  /* Number of symbols for which the receiver is to be enabled */

  uint32_t rxon_dur;

  uint8_t defer_permit  : 1; /* 0=Only attempt operation on current superframe
                              * 1=Operation can be deferred to next superframe */
  uint8_t rng_rxctrl    : 1; /* 0=RANGING_OFF, 1=RANGING_OFF */
};

/*****************************************************************************
 * Primitive: MLME-RXENABLE.confirm
 *
 * Description:
 *    Reports the results of the attempt to enable or disable the receiver.
 * 
 *****************************************************************************/

struct ieee802154_rxenable_conf_s
{
  enum ieee802154_status_e status;
};

/*****************************************************************************
 * Primitive: MLME-SCAN.request
 *
 * Description:
 *    Used to initiate a channel scan over a given list of channels.
 * 
 *****************************************************************************/

struct ieee802154_scan_req_s
{

  enum ieee802154_scantype_e type;
  uint8_t duration;
  uint8_t ch_page;

#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s security;
#endif

  uint8_t channels[1];
};

#define SIZEOF_IEEE802154_SCAN_REQ_S(n) \
  (sizeof(struct ieee802154_scan_req_s) + (n) - 1)

/*****************************************************************************
 * Primitive: MLME-SCAN.confirm
 *
 * Description:
 *    Reports the result of the channel scan request.
 * 
 *****************************************************************************/

struct ieee802154_scan_conf_s
{
  enum ieee802154_status_e status;
  enum ieee802154_scantype_e type;
  uint8_t ch_page;
  uint8_t num_channels;

#warning Figure out how to handle missing primitive semantics. See standard.
};

/*****************************************************************************
 * Primitive: MLME-SET.request
 *
 * Description:
 *    Attempts to write the given value to the indicated PIB attribute.
 * 
 *****************************************************************************/

struct ieee802154_set_req_s
{
  enum ieee802154_pib_attr_e pib_attr;
  union ieee802154_attr_val_u attr_value;
};

/*****************************************************************************
 * Primitive: MLME-SET.confirm
 *
 * Description:
 *    Reports the results of an attempt to write a value to a PIB attribute.
 * 
 *****************************************************************************/

struct ieee802154_set_conf_s
{
  enum ieee802154_status_e status;
  enum ieee802154_pib_attr_e pib_attr;
};

/*****************************************************************************
 * Primitive: MLME-START.request
 *
 * Description:
 *    Used by the PAN coordinator to initiate a new PAN or to begin using a new
 *    superframe configuration. This primitive is also used by a device already
 *    associated with an existing PAN to begin using a new superframe
 *    configuration.
 * 
 *****************************************************************************/

struct ieee802154_start_req_s
{
  uint16_t pan_id;
  uint8_t ch_num;
  uint8_t ch_page;

  uint32_t start_time   : 24; 
  uint32_t beacon_order : 8;

  uint8_t sf_order;

  uint8_t pan_coord     : 1;
  uint8_t batt_life_ext : 1;
  uint8_t coord_realign : 1;

#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s coord_realign;
  struct ieee802154_security_s beacon;
#endif
};

/*****************************************************************************
 * Primitive: MLME-START.confirm
 *
 * Description:
 *    Reports the results of the attempt to start using a new superframe
 *    configuration.
 * 
 *****************************************************************************/

struct ieee802154_start_conf_s
{
  enum ieee802154_status_e status;
};

/*****************************************************************************
 * Primitive: MLME-SYNC.request
 *
 * Description:
 *    Requests to synchronize with the coordinator by acquiring and, if
 *    specified, tracking its beacons.
 * 
 *****************************************************************************/

struct ieee802154_sync_req_s
{
  uint8_t ch_num;
  uint8_t ch_page;
  bool track_beacon;
};

/*****************************************************************************
 * Primitive: MLME-SYNC-LOSS.indication
 *
 * Description:
 *    Indicates the loss of synchronization with a coordinator.
 * 
 *****************************************************************************/

struct ieee802154_syncloss_ind_s
{
  enum ieee802154_status_e loss_reason;
  uint16_t pan_id;
  uint8_t ch_num;
  uint8_t ch_page;

#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s security;
#endif
};

/*****************************************************************************
 * Primitive: MLME-POLL.request
 *
 * Description:
 *    Prompts the device to request data from the coordinator. 
 * 
 *****************************************************************************/

struct ieee802154_poll_req_s
{
  struct ieee802154_addr_s coord_addr;

#ifdef CONFIG_IEEE802154_SECURITY
  /* Security information if enabled */

  struct ieee802154_security_s security;
#endif
};

/*****************************************************************************
 * Primitive: MLME-POLL.confirm
 *
 * Description:
 *    Reports the results of a request to poll the coordinator for data.
 * 
 *****************************************************************************/

struct ieee802154_poll_conf_s
{
  enum ieee802154_status_e status;
};

/* A pointer to this structure is passed as the argument of each IOCTL
 * command.
 */

union ieee802154_macarg_u
{
  struct ieee802154_assoc_req_s    assocreq;    /* MAC802154IOC_MLME_ASSOC_REQUEST */
  struct ieee802154_assoc_resp_s   assocresp;   /* MAC802154IOC_MLME_ASSOC_RESPONSE */
  struct ieee802154_disassoc_req_s disassocreq; /* MAC802154IOC_MLME_DISASSOC_REQUEST */
  struct ieee802154_get_req_s      getreq;      /* MAC802154IOC_MLME_GET_REQUEST */
  struct ieee802154_gts_req_s      gtsreq;      /* MAC802154IOC_MLME_GTS_REQUEST */
  struct ieee802154_orphan_resp_s  orphanresp;  /* MAC802154IOC_MLME_ORPHAN_RESPONSE */
  struct ieee802154_reset_req_s    resetreq;    /* MAC802154IOC_MLME_RESET_REQUEST */
  struct ieee802154_rxenable_req_s rxenabreq;   /* MAC802154IOC_MLME_RXENABLE_REQUEST */
  struct ieee802154_scan_req_s     scanreq;     /* MAC802154IOC_MLME_SCAN_REQUEST */
  struct ieee802154_set_req_s      setreq;      /* MAC802154IOC_MLME_SET_REQUEST */
  struct ieee802154_start_req_s    startreq;    /* MAC802154IOC_MLME_START_REQUEST */
  struct ieee802154_sync_req_s     syncreq;     /* MAC802154IOC_MLME_SYNC_REQUEST */
  struct ieee802154_poll_req_s     pollreq;     /* MAC802154IOC_MLME_POLL_REQUEST */
  /* To be determined */                        /* MAC802154IOC_MLME_DPS_REQUEST */
  /* To be determined */                        /* MAC802154IOC_MLME_SOUNDING_REQUEST */
  /* To be determined */                        /* MAC802154IOC_MLME_CALIBRATE_REQUEST */
};

union ieee802154_mlme_notify_u
{
  struct ieee802154_assoc_conf_s       assocconf;
  struct ieee802154_disassoc_conf_s    disassocconf;
  struct ieee802154_get_conf_s         getconf;
  struct ieee802154_gts_conf_s         gtsconf;
  struct ieee802154_reset_conf_s       resetconf;
  struct ieee802154_rxenable_conf_s    rxenableconf;
  struct ieee802154_scan_conf_s        scanconf;
  struct ieee802154_set_conf_s         setconf;
  struct ieee802154_start_conf_s       startconf;
  struct ieee802154_poll_conf_s        pollconf;

  struct ieee802154_assoc_ind_s        assocind;
  struct ieee802154_disassoc_ind_s     disassocind;
  struct ieee802154_beaconnotify_ind_s beaconnotifyind;
  struct ieee802154_gts_ind_s          gtsind;
  struct ieee802154_orphan_ind_s       orphanind;
  struct ieee802154_commstatus_ind_s   commstatusind;
  struct ieee802154_syncloss_ind_s     synclossind;
};

union ieee802154_mcps_notify_u
{
  struct ieee802154_data_conf_s        dataconf;
  struct ieee802154_purge_conf_s       purgeconf;
  struct ieee802154_data_ind_s         dataind;
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

/* MAC Service Notifications */

enum ieee802154_macnotify_e
{
  /* MCPS Notifications */

  IEEE802154_NOTIFY_CONF_DATA = 0x00,
  IEEE802154_NOTIFY_CONF_PURGE,
  IEEE802154_NOTIFY_IND_DATA,

  /* MLME Notifications */

  IEEE802154_NOTIFY_CONF_ASSOC,
  IEEE802154_NOTIFY_CONF_DISASSOC,
  IEEE802154_NOTIFY_CONF_GET,
  IEEE802154_NOTIFY_CONF_GTS,
  IEEE802154_NOTIFY_CONF_RESET,
  IEEE802154_NOTIFY_CONF_RXENABLE,
  IEEE802154_NOTIFY_CONF_SCAN,
  IEEE802154_NOTIFY_CONF_SET,
  IEEE802154_NOTIFY_CONF_START,
  IEEE802154_NOTIFY_CONF_POLL,

  IEEE802154_NOTIFY_IND_ASSOC,
  IEEE802154_NOTIFY_IND_DISASSOC,
  IEEE802154_NOTIFY_IND_BEACONNOTIFY,
  IEEE802154_NOTIFY_IND_GTS,
  IEEE802154_NOTIFY_IND_ORPHAN,
  IEEE802154_NOTIFY_IND_COMMSTATUS,
  IEEE802154_NOTIFY_IND_SYNCLOSS
};

/* Callback operations to notify the next highest layer of various asynchronous
 * events, usually triggered by some previous request or response invoked by the
 * upper layer */

struct ieee802154_maccb_s
{
  CODE void (*mlme_notify) (FAR struct ieee802154_maccb_s *maccb,
                            enum ieee802154_macnotify_e notif,
                            FAR union ieee802154_mlme_notify_u *arg);

  CODE void (*mcps_notify) (FAR struct ieee802154_maccb_s *maccb,
                            enum ieee802154_macnotify_e notif,
                            FAR union ieee802154_mcps_notify_u *arg);
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
