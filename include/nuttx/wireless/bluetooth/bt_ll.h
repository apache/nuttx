/****************************************************************************
 * include/nuttx/wireless/bluetooth/bt_ll.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_LL_H
#define __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_LL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <nuttx/wireless/bluetooth/bt_hci.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The following are definitions according to Bluetooth LE 4.0 spec. This
 * is the currently supported version of the standard in the Link Layer
 * implementation
 */

/* Link-layer definitions ***************************************************/

/* Possible states for the LL
 *
 * This is the SM definition:
 *
 * scanning <------> standby
 *
 * advertising <-> standby <-> initiating
 *      |             ^            |
 *      |             |            |
 *      |--------> connection <----|
 */

#define BT_LL_STATE_STANDBY     0   /* No transmisions */
#define BT_LL_STATE_ADVERTISING 1   /* Sending/responding to advertisements */
#define BT_LL_STATE_SCANNING    2   /* Listening for advertisements */
#define BT_LL_STATE_INITIATING  3   /* About to initiate connection to device */
#define BT_LL_STATE_CONNECTION  4   /* When connection is established */

/* Possible roles within connection state */

#define BT_LL_ROLE_MASTER       0
#define BT_LL_ROLE_SLAVE        1

/* BLE protocol definitions *************************************************/

#define BLUETOOTH_LE_FIRST_ADV_CH      37
#define BLUETOOTH_LE_LAST_ADV_CH       39

#define BLUETOOTH_LE_MAX_DATA_PAYLOAD  27    /* As per 4.0 spec */
#define BLUETOOTH_LE_PDU_MAXSIZE       39

/* Adverising channel PDU types */

#define BT_LE_PDUTYPE_ADV_IND          0b000
#define BT_LE_PDUTYPE_ADV_DIRECT_IND   0b001
#define BT_LE_PDUTYPE_ADV_NONCON_IND   0b010
#define BT_LE_PDUTYPE_SCAN_REQ         0b011
#define BT_LE_PDUTYPE_SCAN_RSP         0b100
#define BT_LE_PDUTYPE_CONNECT_REQ      0b101
#define BT_LE_PDUTYPE_ADV_SCAN_IND     0b110

/* Possible values for SCA field in CONNECT_REQ */

#define BT_LE_0_20_PPM                 7
#define BT_LE_21_30_PPM                6
#define BT_LE_31_50_PPM                5
#define BT_LE_51_75_PPM                4
#define BT_LE_76_100_PPM               3
#define BT_LE_101_150_PPM              2
#define BT_LE_151_250_PPM              1
#define BT_LE_251_500_PPM              0

/* Possible values for LLID field in data PDU */

#define BT_LE_DATA_PDU1                0b01   /* This is a data PDU (if length == 0, this is an "empty PDU") */
#define BT_LE_DATA_PDU2                0b10   /* This is a data PDU */
#define BT_LE_CONTROL_PDU              0b11   /* This is a control PDU */

#define BT_LL_CONNECTION_UPDATE_REQ    0x00
#define BT_LL_CHANNEL_MAP_REQ          0x01
#define BT_LL_TERMINATE_IND            0x02
#define BT_LL_ENC_REQ                  0x03
#define BT_LL_ENC_RSP                  0x04
#define BT_LL_START_ENC_REQ            0x05
#define BT_LL_START_ENC_RSP            0x06
#define BT_LL_UNKNOWN_RSP              0x07
#define BT_LL_FEATURE_REQ              0x08
#define BT_LL_FEATURE_RSP              0x09
#define BT_LL_PAUSE_ENC_REQ            0x0A
#define BT_LL_PAUSE_ENC_RSP            0x0B
#define BT_LL_VERSION_IND              0x0C
#define BT_LL_REJECT_IND               0x0D

/* 0x0E - 0x0F reserved for future use */

/* TODO: define contents of all control PDUs */

#define BT_LL_ADV_ACCESS_ADDRESS       0x8E89BED6

#define BT_LL_VERSNUM_4_0              6    /* Bluetooth 4.0 */
#define BT_LL_VERSNUM_4_1              7    /* Bluetooth 4.1 */
#define BT_LL_VERSNUM_4_2              8    /* Bluetooth 4.2 */
#define BT_LL_VERSNUM_5_0              9    /* Bluetooth 5.0 */
#define BT_LL_VERSNUM_5_1              10   /* Bluetooth 5.1 */
#define BT_LL_VERSNUM_5_2              11   /* Bluetooth 5.2 */

#define BT_LL_COMPID_NORDIC            0x0059 /* Nordic Semiconductor */
#define BT_LL_COMPID_UNKNOWN           0xFFFF

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct bt_ll_lowerhalf_s
{
  int (*state)(void);

  int (*get_remote_features)(uint8_t conn_id);

  int (*setup_scan)(struct bt_hci_cp_le_set_scan_params_s *params);
  int (*scan)(bool enable);

  int (*set_advdata)(struct bt_hci_cp_le_set_adv_data_s *advdata);
  int (*set_rspdata)(struct bt_hci_cp_le_set_scan_rsp_data_s *rspdata);
  int (*setup_advertise)(struct bt_hci_cp_le_set_adv_parameters_s *params);
  int (*advertise)(bool enable);

  void (*send_data)(const uint8_t *data, size_t len,
                    bool continuation, bool broadcast);

  void (*rand)(uint8_t *data, size_t len);

  void (*set_ltk)(uint8_t conn_id, uint8_t *ltk);

  int (*reset)(void);
  int (*txpower)(int8_t dbm);
  int (*setchannel)(uint8_t channel);
  int (*set_randaddr)(struct bt_addr_s *addr);

  /* Get public address */

  int (*getaddress)(struct bt_addr_s *addr);
};

/* Header of an advertisement channel PDU */

begin_packed_struct struct bt_adv_pdu_hdr_s
{
  uint16_t type   : 4;     /* Type of PDU */
  uint16_t rfu    : 2;     /* Reserved for future use */
  uint16_t txadd  : 1;     /* Type-dependant field */
  uint16_t rxadd  : 1;     /* Type-dependant field */
  uint16_t length : 6;     /* Payload length (6 to 37 octets) */
  uint16_t rfu2   : 2;     /* Reserved for future use */
} end_packed_struct;

/* Header of a data channel PDU */

begin_packed_struct struct bt_data_pdu_hdr_s
{
  uint16_t llid   : 2;     /* Link-layer PDU type (data / L2CAP or control) */
  uint16_t nesn   : 1;     /* Next expected sequence number */
  uint16_t sn     : 1;     /* Sequence number */
  uint16_t md     : 1;     /* More data */
  uint16_t rfu    : 3;     /* Reserved for future use */
  uint16_t length : 5;     /* Payload + MIC size in octets */
  uint16_t rfu2   : 3;     /* Reserved for future use */
} end_packed_struct;

/* Payload structure for ADV_IND PDU */

begin_packed_struct struct bt_adv_ind_pdu_payload_s
{
  uint8_t adv_addr[6];  /* Advertiser address */
  uint8_t adv_data[31]; /* Advertising data (0-31 octets) */
} end_packed_struct;

/* Payload structure for ADV_DIRECT_IND PDU */

begin_packed_struct struct bt_adv_direct_ind_pdu_payload_s
{
  uint8_t adv_addr[6];   /* Advertiser address */
  uint8_t init_addr[6];  /* Initiator (target) address */
} end_packed_struct;

/* Payload structure for ADV_NONCONN_IND PDU */

begin_packed_struct struct bt_adv_nonconn_ind_pdu_payload_s
{
  uint8_t adv_addr[6];  /* Advertiser address */
  uint8_t adv_data[31]; /* Advertising data (0-31 octets) */
} end_packed_struct;

/* Payload structure for ADV_SCAN_IND PDU */

begin_packed_struct struct bt_adv_scan_ind_pdu_payload_s
{
  uint8_t adv_addr[6];  /* Advertiser address */
  uint8_t adv_data[31]; /* Advertising data (0-31 octets) */
} end_packed_struct;

/* Payload structure for SCAN_REQ PDU */

begin_packed_struct struct bt_scan_req_pdu_payload_s
{
  uint8_t scan_addr[6]; /* Scanner's address */
  uint8_t adv_addr[6];  /* Advertiser's (target) address */
} end_packed_struct;

/* Since all but direct adv PDUs are the same, this typedef is useful
 * when processing these packets
  */

typedef struct bt_adv_ind_pdu_payload_s bt_adv_indirect_pdu_payload_t;

/* Payload structure for SCAN_RESP PDU */

begin_packed_struct struct bt_scan_resp_pdu_payload_s
{
  uint8_t adv_addr[6];        /* Advertiser address */
  uint8_t scan_resp_data[31]; /* Optional scan data from advertiser (0-31 octets) */
} end_packed_struct;

/* Payload structure for CONNECT_REQ PDU */

begin_packed_struct struct bt_connect_req_pdu_payload_s
{
  uint8_t init_addr[6];   /* Initiator's address */
  uint8_t adv_addr[6];    /* Advertiser's address */

  uint32_t aa;            /* Connection's access address */
  uint8_t crcint[3];      /* Initial CRC value */
  uint8_t winsize;        /* Transmit window size, in units of 1.25ms */
  uint16_t winoffset;     /* Transmit window offset, in units of 1.25ms */
  uint16_t interval;      /* Connection interval, in units of 1.25ms */
  uint16_t latency;       /* Slave latency, in units of 1.25ms */
  uint16_t timeout;       /* Supervision timeout, in units of 10ms */
  uint8_t chm[5];         /* Channel map (used/unused data channel, for each bit) */
  uint8_t hop : 5;        /* Hop increment for channel selection (5 to 16) */
  uint8_t sca : 3;        /* Worst case master's clock sleep accuracy */
} end_packed_struct;

begin_packed_struct struct bt_control_pdu_payload_s
{
  uint8_t opcode;             /* Type of control PDU */
  uint8_t ctr_data[22];       /* Data, 0-22 octets, size depends on opcode value */
} end_packed_struct;

begin_packed_struct struct bt_control_pdu_version_ind_s
{
  uint8_t versnum;            /* Controller version */
  uint16_t compid;            /* Company ID */
  uint8_t subvers;            /* Revision */
} end_packed_struct;

begin_packed_struct struct bt_control_pdu_enc_req_s
{
  uint64_t rand;              /* Random number */
  uint16_t ediv;              /* EDIV */
  uint64_t skdm;              /* Master's Session Key Identifier */
  uint32_t ivm;               /* Master's Initialization Vector */
} end_packed_struct;

begin_packed_struct struct bt_control_pdu_enc_resp_s
{
  uint64_t skds;              /* Slave's Session Key Identifier */
  uint32_t ivs;               /* Slave's Initialization Vector */
} end_packed_struct;

begin_packed_struct struct bt_control_pdu_channel_map_req_s
{
  uint8_t channel_map[5];     /* Channel map */
  uint16_t instant;           /* When to apply this */
} end_packed_struct;

begin_packed_struct struct bt_control_pdu_conn_update_req_s
{
  uint8_t winsize;        /* Transmit window size, in units of 1.25ms */
  uint16_t winoffset;     /* Transmit window offset, in units of 1.25ms */
  uint16_t interval;      /* Connection interval, in units of 1.25ms */
  uint16_t latency;       /* Slave latency */
  uint16_t timeout;       /* Supervision timeout, in units of 10ms */
  uint16_t instant;       /* When to apply this */
} end_packed_struct;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int bt_ll_initialize(const struct bt_ll_lowerhalf_s *lower);

void bt_ll_handle_adv_pdu(struct bt_adv_pdu_hdr_s *hdr, uint8_t *payload);

void bt_ll_on_connection(struct bt_adv_pdu_hdr_s *hdr,
                         struct bt_connect_req_pdu_payload_s *payload,
                         uint8_t status, uint8_t conn_id, uint8_t role);

void bt_ll_on_connection_closed(uint8_t conn_id, uint8_t reason);

bool bt_ll_whitelisted(struct bt_addr_le_s addr);

int bt_ll_advtype_to_pdutype(int advtype);

uint32_t bt_ll_sca_to_ppm(uint8_t sca);

void bt_ll_on_data(const struct bt_data_pdu_hdr_s *hdr,
                   const uint8_t *payload, uint8_t conn_id);

void bt_ll_ack_packets(uint8_t conn_id, size_t num_packets);

void bt_ll_send_rem_features(uint8_t conn, const uint8_t *features);

void bt_ll_update_conn_params(uint8_t conn, uint16_t interval,
                              uint16_t latency, uint16_t timeout);

void bt_ll_request_ltk(uint8_t conn, uint16_t ediv, uint64_t rand);

void bt_ll_encryption_changed(uint8_t conn, bool enabled);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif // __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_LL_H
