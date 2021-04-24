/****************************************************************************
 * include/nuttx/wireless/bluetooth/bt_hci.h
 * Bluetooth Host Control Interface definitions.
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Ported from the Intel/Zephyr arduino101_firmware_source-v1.tar package
 * where the code was released with a compatible 3-clause BSD license:
 *
 *   Copyright (c) 2016, Intel Corporation
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_HCI_H
#define __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_HCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#if defined(CONFIG_WIRELESS_BLUETOOTH) || defined(CONFIG_DRIVERS_BLUETOOTH)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BT_ADDR_LE_PUBLIC  0x00
#define BT_ADDR_LE_RANDOM  0x01

#define BT_ADDR_ANY    (&(bt_addr_t) {{0, 0, 0, 0, 0, 0}})
#define BT_ADDR_LE_ANY (&(bt_addr_le_t) { 0, {0, 0, 0, 0, 0, 0}})

/* HCI Error Codes */

#define BT_HCI_SUCCESS                    0x00
#define BT_HCI_PENDING                    0x00
#define BT_HCI_ERR_UNKNOWN_COMMAND        0x01
#define BT_HCI_ERR_CONNECTION_TIMEOUT     0x08
#define BT_HCI_ERR_AUTHENTICATION_FAIL    0x05
#define BT_HCI_ERR_COMMAND_DISALLOWED     0x0C
#define BT_HCI_ERR_INVALID_PARAMETERS     0x12
#define BT_HCI_ERR_REMOTE_USER_TERM_CONN  0x13
#define BT_HCI_ERR_UNSUPP_REMOTE_FEATURE  0x1a
#define BT_HCI_ERR_INSTANT_PASSED         0x28
#define BT_HCI_ERR_PAIRING_NOT_SUPPORTED  0x29
#define BT_HCI_ERR_UNACCEPT_CONN_PARAMS   0x3b

/* EIR/AD definitions */

#define BT_EIR_FLAGS             0x01 /* AD flags */
#define BT_EIR_UUID16_SOME       0x02 /* 16-bit UUID, more available */
#define BT_EIR_UUID16_ALL        0x03 /* 16-bit UUID, all listed */
#define BT_EIR_UUID32_SOME       0x04 /* 32-bit UUID, more available */
#define BT_EIR_UUID32_ALL        0x05 /* 32-bit UUID, all listed */
#define BT_EIR_UUID128_SOME      0x06 /* 128-bit UUID, more available */
#define BT_EIR_UUID128_ALL       0x07 /* 128-bit UUID, all listed */
#define BT_EIR_NAME_COMPLETE     0x09 /* Complete name */
#define BT_EIR_TX_POWER          0x0a /* Tx Power */
#define BT_EIR_SOLICIT16         0x14 /* Solicit UUIDs, 16-bit */
#define BT_EIR_SOLICIT128        0x15 /* Solicit UUIDs, 128-bit */
#define BT_EIR_SVC_DATA16        0x16 /* Service data, 16-bit UUID */
#define BT_EIR_GAP_APPEARANCE    0x19 /* GAP appearance */
#define BT_EIR_SOLICIT32         0x1f /* Solicit UUIDs, 32-bit */
#define BT_EIR_SVC_DATA32        0x20 /* Service data, 32-bit UUID */
#define BT_EIR_SVC_DATA128       0x21 /* Service data, 128-bit UUID */
#define BT_EIR_MANUFACTURER_DATA 0xff /* Manufacturer Specific Data */

#define BT_LE_AD_GENERAL         0x02 /* General Discoverable */
#define BT_LE_AD_NO_BREDR        0x04 /* BR/EDR not supported */

#define bt_acl_handle(h)         ((h) & 0x0fff)

/* LMP features */

#define BT_LMP_NO_BREDR          0x20
#define BT_LMP_LE                0x40

/* LE features */

#define BT_HCI_LE_ENCRYPTION     0x01

/* OpCode Group Fields */

#define BT_OGF_LINK_CTRL         0x01
#define BT_OGF_LINK_POLICY       0x02
#define BT_OGF_BASEBAND          0x03
#define BT_OGF_INFO              0x04
#define BT_OGF_STATUS            0x05
#define BT_OGF_LE                0x08

/* Construct OpCode from OGF and OCF */

#define BT_OGF_VENDOR                         0x18
#define BT_HC_VN_READ_CONT_FEATURES           BT_OP(BT_OGF_VENDOR, 0x06E)

#define BT_OP(ogf, ocf)                       ((ocf) | ((ogf) << 10))

#define BT_HCI_OP_DISCONNECT                  BT_OP(BT_OGF_LINK_CTRL, 0x0006)
#define BT_HCI_OP_READ_REMOTE_VERSION_INFO    BT_OP(BT_OGF_LINK_CTRL, 0x001d)
#define BT_HCI_OP_SET_EVENT_MASK              BT_OP(BT_OGF_BASEBAND, 0x0001)
#define BT_HCI_OP_RESET                       BT_OP(BT_OGF_BASEBAND, 0x0003)
#define BT_HCI_OP_SET_CTL_TO_HOST_FLOW        BT_OP(BT_OGF_BASEBAND, 0x0031)
#define BT_HCI_OP_HOST_BUFFER_SIZE            BT_OP(BT_OGF_BASEBAND, 0x0033)
#define BT_HCI_OP_HOST_NUM_COMPLETED_PACKETS  BT_OP(BT_OGF_BASEBAND, 0x0035)
#define BT_HCI_OP_LE_WRITE_LE_HOST_SUPP       BT_OP(BT_OGF_BASEBAND, 0x006d)
#define BT_HCI_OP_READ_LOCAL_VERSION_INFO     BT_OP(BT_OGF_INFO, 0x0001)
#define BT_HCI_OP_READ_LOCAL_FEATURES         BT_OP(BT_OGF_INFO, 0x0003)
#define BT_HCI_OP_READ_BUFFER_SIZE            BT_OP(BT_OGF_INFO, 0x0005)
#define BT_HCI_OP_READ_BD_ADDR                BT_OP(BT_OGF_INFO, 0x0009)
#define BT_HCI_OP_LE_SET_EVENT_MASK           BT_OP(BT_OGF_LE, 0x0001)
#define BT_HCI_OP_LE_READ_BUFFER_SIZE         BT_OP(BT_OGF_LE, 0x0002)
#define BT_HCI_OP_LE_READ_LOCAL_FEATURES      BT_OP(BT_OGF_LE, 0x0003)

#define BT_HCI_OP_LE_SET_RAND_ADDR            BT_OP(BT_OGF_LE, 0x0005)

/* Advertising types */

#define BT_LE_ADV_IND                         0x00
#define BT_LE_ADV_DIRECT_IND                  0x01
#define BT_LE_ADV_SCAN_IND                    0x02
#define BT_LE_ADV_NONCONN_IND                 0x03
#define BT_LE_ADV_SCAN_RSP                    0x04

#define BT_HCI_OP_LE_SET_ADV_PARAMETERS       BT_OP(BT_OGF_LE, 0x0006)
#define BT_HCI_OP_LE_SET_ADV_DATA             BT_OP(BT_OGF_LE, 0x0008)
#define BT_HCI_OP_LE_SET_SCAN_RSP_DATA        BT_OP(BT_OGF_LE, 0x0009)
#define BT_HCI_OP_LE_SET_ADV_ENABLE           BT_OP(BT_OGF_LE, 0x000a)

/* Scan types */

#define BT_HCI_OP_LE_SET_SCAN_PARAMS          BT_OP(BT_OGF_LE, 0x000b)
#  define BT_LE_SCAN_PASSIVE                  0x00
#  define BT_LE_SCAN_ACTIVE                   0x01
#define BT_HCI_OP_LE_SET_SCAN_ENABLE          BT_OP(BT_OGF_LE, 0x000c)
#  define BT_LE_SCAN_DISABLE                  0x00
#  define BT_LE_SCAN_ENABLE                   0x01
#  define BT_LE_SCAN_FILTER_DUP_DISABLE       0x00
#  define BT_LE_SCAN_FILTER_DUP_ENABLE        0x01
#define BT_HCI_OP_LE_CREATE_CONN              BT_OP(BT_OGF_LE, 0x000d)
#define BT_HCI_OP_LE_CREATE_CONN_CANCEL       BT_OP(BT_OGF_LE, 0x000e)
#define BT_HCI_OP_LE_READ_WHITE_LIST_SIZE     BT_OP(BT_OGF_LE, 0x000f)
#define BT_HCI_OP_LE_CLEAR_WHITE_LIST         BT_OP(BT_OGF_LE, 0x0010)
#define BT_HCI_OP_LE_ADD_DEV_TO_WHITE_LIST    BT_OP(BT_OGF_LE, 0x0011)
#define BT_HCI_OP_LE_REM_DEV_FROM_WHITE_LIST  BT_OP(BT_OGF_LE, 0x0012)
#define BT_HCI_OP_LE_CONN_UPDATE              BT_OP(BT_OGF_LE, 0x0013)
#define BT_HCI_OP_LE_READ_REMOTE_FEATURES     BT_OP(BT_OGF_LE, 0x0016)
#define BT_HCI_OP_LE_ENCRYPT                  BT_OP(BT_OGF_LE, 0x0017)
#define BT_HCI_OP_LE_RAND                     BT_OP(BT_OGF_LE, 0x0018)
#define BT_HCI_OP_LE_START_ENCRYPTION         BT_OP(BT_OGF_LE, 0x0019)
#define BT_HCI_OP_LE_LTK_REQ_REPLY            BT_OP(BT_OGF_LE, 0x001a)
#define BT_HCI_OP_LE_LTK_REQ_NEG_REPLY        BT_OP(BT_OGF_LE, 0x001b)

/* Event definitions */

#define BT_HCI_EVT_INQUIRY_COMPLETE             0x01
#define BT_HCI_EVT_INQUIRY_RESULT               0x02
#define BT_HCI_EVT_CONN_COMPLETE                0x03
#define BT_HCI_EVT_CONN_REQUEST                 0x04
#define BT_HCI_EVT_DISCONN_COMPLETE             0x05
#define BT_HCI_EVT_AUTH_COMPLETE                0x06
#define BT_HCI_EVT_REMOTE_NAME_REQ_COMPLETE     0x07
#define BT_HCI_EVT_ENCRYPT_CHANGE               0x08
#define BT_HCI_EVT_CHANGE_CONN_LINK_KEY         0x09
#define BT_HCI_EVT_CENTRAL_LINK_KEY_COMP        0x0a
#define BT_HCI_EVT_REMOTE_FEATURES              0x0b
#define BT_HCI_EVT_REMOTE_VERSION_INFO          0x0c
#define BT_HCI_EVT_QOS_SETUP_COMP               0x0d
#define BT_HCI_EVT_CMD_COMPLETE                 0x0e
#define BT_HCI_EVT_CMD_STATUS                   0x0f
#define BT_HCI_EVT_HARDWARE_ERROR               0x10
#define BT_HCI_FLUSH_OCCURED                    0x11
#define BT_HCI_EVT_ROLE_CHANGE                  0x12
#define BT_HCI_EVT_NUM_COMPLETED_PACKETS        0x13
#define BT_HCI_EVT_MODE_CHANGE                  0x14
#define BT_HCI_EVT_RETURN_LINK_KEYS             0x15
#define BT_HCI_EVT_PIN_CODE_REQ                 0x16
#define BT_HCI_EVT_LINK_KEY_REQ                 0x17
#define BT_HCI_EVT_LINK_KEY_NOTIFY              0x18
#define BT_HCI_EVT_LOOPBACK_COMMAND             0x19
#define BT_HCI_EVT_DATA_BUF_OVERFLOW            0x1a
#define BT_HCI_EVT_MAX_SLOTS_CHANGED            0x1b
#define BT_HCI_EVT_READ_CLOCK_OFF_COMP          0x1c
#define BT_HCI_EVT_CONN_PKT_TYPE_CHANGE         0x1d
#define BT_HCI_EVT_QOS_VIOLATION                0x1e
#define BT_HCI_EVT_PAGE_SCAN_MODE_CHANGE        0x1f
#define BT_HCI_EVT_PAGE_SCAN_REP_MODE_CHNG      0x20
#define BT_HCI_EVT_FLOW_SPECIFICATION_COMP      0x21
#define BT_HCI_EVT_INQUIRY_RESULT_WITH_RSSI     0x22
#define BT_HCI_EVT_REMOTE_EXT_FEATURES          0x23
#define BT_HCI_EVT_SYNC_CONN_COMPLETE           0x2c
#define BT_HCI_EVT_ESCO_CONNECTION_CHANGED      0x2d
#define BT_HCI_EVT_SNIFF_SUB_RATE               0x2e
#define BT_HCI_EVT_EXTENDED_INQUIRY_RESULT      0x2f
#define BT_HCI_EVT_ENCRYPT_KEY_REFRESH_COMPLETE 0x30
#define BT_HCI_EVT_IO_CAPA_REQ                  0x31
#define BT_HCI_EVT_IO_CAPA_RESP                 0x32
#define BT_HCI_EVT_USER_CONFIRM_REQ             0x33
#define BT_HCI_EVT_USER_PASSKEY_REQ             0x34
#define BT_HCI_EVT_REMOTE_OOB_DATA_REQUEST      0x35
#define BT_HCI_EVT_SSP_COMPLETE                 0x36
#define BT_HCI_EVT_LINK_SUPER_TOUT_CHANGED      0x38
#define BT_HCI_EVT_ENHANCED_FLUSH_COMPLETE      0x39
#define BT_HCI_EVT_USER_PASSKEY_NOTIFY          0x3b
#define BT_HCI_EVT_KEYPRESS_NOTIFY              0x3c
#define BT_HCI_EVT_RMT_HOST_SUP_FEAT_NOTIFY     0x3d
#define BT_HCI_EVT_LE_META_EVENT                0x3e

#define BT_HCI_EVT_LE_CONN_COMPLETE             0x01
#  define BT_HCI_ROLE_MASTER                    0x00
#  define BT_HCI_ROLE_SLAVE                     0x01
#define BT_HCI_EVT_LE_ADVERTISING_REPORT        0x02
#define BT_HCI_EVT_LE_CONN_UPDATE_COMPLETE      0x03
#define BT_HCI_EVT_LE_READ_REM_FEAT_COMPLETE    0x04
#define BT_HCI_EVT_LE_LTK_REQUEST               0x05
#define BT_HCI_EVT_LE_CONN_PARAM_REQ            0x06
#define BT_HCI_EVT_LE_DATA_LEN_CHANGE           0x07
#define BT_HCI_EVT_LE_P256_PUBLIC_KEY_COMPLETE  0x08
#define BT_HCI_EVT_LE_GENERATE_DHKEY_COMPLETE   0x09
#define BT_HCI_EVT_LE_ENH_CONN_COMPLETE         0x0a
#define BT_HCI_EVT_LE_DIRECT_ADV_REPORT         0x0b
#define BT_HCI_EVT_LE_PHY_UPDATE_COMPLETE       0x0c
#define BT_HCI_EVT_LE_EXT_ADVERTISING_REPORT    0x0d
#define BT_HCI_EVT_LE_PER_ADV_SYNC_ESTABLISHED  0x0e
#define BT_HCI_EVT_LE_PER_ADVERTISING_REPORT    0x0f
#define BT_HCI_EVT_LE_PER_ADV_SYNC_LOST         0x10
#define BT_HCI_EVT_LE_SCAN_TIMEOUT              0x11
#define BT_HCI_EVT_LE_ADV_SET_TERMINATED        0x12
#define BT_HCI_EVT_LE_SCAN_REQ_RECEIVED         0x13
#define BT_HCI_EVT_LE_CHAN_SEL_ALGO             0x14
#define BT_HCI_EVT_LE_CIS_ESTABLISHED           0x19
#define BT_HCI_EVT_LE_CIS_REQ                   0x1a
#define BT_HCI_EVT_LE_BIG_COMPLETE              0x1b
#define BT_HCI_EVT_LE_BIG_TERMINATE             0x1c
#define BT_HCI_EVT_LE_BIG_SYNC_ESTABLISHED      0x1d
#define BT_HCI_EVT_LE_BIG_SYNC_LOST             0x1e
#define BT_HCI_EVT_LE_REQ_PEER_SCA_COMPLETE     0x1f

/* Packet boundary flags for ACL packets */

#define BT_HCI_ACL_CONTINUATION               0x0001
#define BT_HCI_ACL_NEW                        0x0002

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef struct bt_addr_s
{
  uint8_t val[6];
} bt_addr_t;

typedef struct bt_addr_le_s
{
  uint8_t type;
  uint8_t val[6];
} bt_addr_le_t;

begin_packed_struct struct bt_hci_evt_hdr_s
{
  uint8_t evt;
  uint8_t len;
} end_packed_struct;

begin_packed_struct struct bt_hci_acl_hdr_s
{
  uint16_t handle          : 12;
  uint16_t packet_boundary : 2;
  uint16_t broadcast       : 2;
  uint16_t len;
} end_packed_struct;

begin_packed_struct struct bt_hci_iso_hdr_s
{
  uint16_t handle;
  uint16_t len;
} end_packed_struct;

begin_packed_struct struct bt_hci_cmd_hdr_s
{
  uint16_t opcode;
  uint8_t param_len;
} end_packed_struct;

begin_packed_struct struct bt_hci_cp_disconnect_s
{
  uint16_t handle;
  uint8_t reason;
} end_packed_struct;

begin_packed_struct struct bt_hci_cp_set_event_mask_s
{
  uint8_t events[8];
} end_packed_struct;

begin_packed_struct struct bt_hci_cp_set_ctl_to_host_flow_s
{
  uint8_t enable;
} end_packed_struct;

begin_packed_struct struct bt_hci_cp_host_buffer_size_s
{
  uint16_t acl_mtu;
  uint8_t sco_mtu;
  uint16_t acl_pkts;
  uint16_t sco_pkts;
} end_packed_struct;

begin_packed_struct struct bt_hci_handle_count_s
{
  uint16_t handle;
  uint16_t count;
} end_packed_struct;

begin_packed_struct struct bt_hci_cp_host_num_completed_packets_s
{
  uint8_t num_handles;
  struct bt_hci_handle_count_s h[0];
} end_packed_struct;

begin_packed_struct struct bt_hci_cp_write_le_host_supp_s
{
  uint8_t le;
  uint8_t simul;
} end_packed_struct;

begin_packed_struct struct bt_hci_le_read_remote_features_s
{
  uint16_t conn;
} end_packed_struct;

begin_packed_struct struct bt_hci_rp_read_local_version_info_s
{
  uint8_t status;
  uint8_t hci_version;
  uint16_t hci_revision;
  uint8_t lmp_version;
  uint16_t manufacturer;
  uint16_t lmp_subversion;
} end_packed_struct;

begin_packed_struct struct bt_hci_rp_read_local_features_s
{
  uint8_t status;
  uint8_t features[8];
} end_packed_struct;

begin_packed_struct struct bt_hci_rp_le_read_remote_features_s
{
  uint8_t status;
  uint16_t conn;
  uint8_t features[8];
} end_packed_struct;

begin_packed_struct struct bt_hci_rp_read_buffer_size_s
{
  uint8_t status;
  uint16_t acl_max_len;
  uint8_t sco_max_len;
  uint16_t acl_max_num;
  uint16_t sco_max_num;
} end_packed_struct;

begin_packed_struct struct bt_hci_rp_read_bd_addr_s
{
  uint8_t status;
  struct bt_addr_s bdaddr;
} end_packed_struct;

begin_packed_struct struct bt_hci_rp_le_read_buffer_size_s
{
  uint8_t status;
  uint16_t le_max_len;
  uint8_t le_max_num;
} end_packed_struct;

begin_packed_struct struct bt_hci_rp_le_read_local_features_s
{
  uint8_t status;
  uint8_t features[8];
} end_packed_struct;

begin_packed_struct struct bt_hci_cp_le_set_adv_parameters_s
{
  uint16_t min_interval;
  uint16_t max_interval;
  uint8_t type;
  uint8_t own_addr_type;
  bt_addr_le_t direct_addr;
  uint8_t channel_map;
  uint8_t filter_policy;
} end_packed_struct;

begin_packed_struct struct bt_hci_cp_le_set_adv_data_s
{
  uint8_t len;
  uint8_t data[31];
} end_packed_struct;

begin_packed_struct struct bt_hci_cp_le_set_scan_rsp_data_s
{
  uint8_t len;
  uint8_t data[31];
} end_packed_struct;

begin_packed_struct struct bt_hci_cp_le_set_adv_enable_s
{
  uint8_t enable;
} end_packed_struct;

begin_packed_struct struct bt_hci_cp_le_set_scan_params_s
{
  uint8_t scan_type;
  uint16_t interval;
  uint16_t window;
  uint8_t addr_type;
  uint8_t filter_policy;
} end_packed_struct;

begin_packed_struct struct bt_hci_cp_le_set_scan_enable_s
{
  uint8_t enable;
  uint8_t filter_dup;
} end_packed_struct;

begin_packed_struct struct bt_hci_cp_le_create_conn_s
{
  uint16_t scan_interval;
  uint16_t scan_window;
  uint8_t filter_policy;
  bt_addr_le_t peer_addr;
  uint8_t own_addr_type;
  uint16_t conn_interval_min;
  uint16_t conn_interval_max;
  uint16_t conn_latency;
  uint16_t supervision_timeout;
  uint16_t min_ce_len;
  uint16_t max_ce_len;
} end_packed_struct;

begin_packed_struct struct hci_cp_le_conn_update_s
{
  uint16_t handle;
  uint16_t conn_interval_min;
  uint16_t conn_interval_max;
  uint16_t conn_latency;
  uint16_t supervision_timeout;
  uint16_t min_ce_len;
  uint16_t max_ce_len;
} end_packed_struct;

begin_packed_struct struct bt_hci_cp_le_encrypt_s
{
  uint8_t key[16];
  uint8_t plaintext[16];
} end_packed_struct;

begin_packed_struct struct bt_hci_rp_le_encrypt_s
{
  uint8_t status;
  uint8_t enc_data[16];
} end_packed_struct;

begin_packed_struct struct bt_hci_rp_le_rand_s
{
  uint8_t status;
  uint8_t rand[8];
} end_packed_struct;

begin_packed_struct struct bt_hci_cp_le_start_encryption_s
{
  uint16_t handle;
  uint64_t rand;
  uint16_t ediv;
  uint8_t ltk[16];
} end_packed_struct;

begin_packed_struct struct bt_hci_cp_le_ltk_req_reply_s
{
  uint16_t handle;
  uint8_t ltk[16];
} end_packed_struct;

begin_packed_struct struct bt_hci_cp_le_ltk_req_neg_reply_s
{
  uint16_t handle;
} end_packed_struct;

/* Event definitions */

begin_packed_struct struct bt_hci_evt_disconn_complete_s
{
  uint8_t status;
  uint16_t handle;
  uint8_t reason;
} end_packed_struct;

begin_packed_struct struct bt_hci_evt_encrypt_change_s
{
  uint8_t status;
  uint16_t handle;
  uint8_t encrypt;
} end_packed_struct;

begin_packed_struct struct hci_evt_cmd_disallowed_s
{
  uint8_t ncmd;
  uint16_t opcode;
} end_packed_struct;

begin_packed_struct struct hci_evt_cmd_complete_s
{
  uint8_t ncmd;
  uint16_t opcode;
} end_packed_struct;

begin_packed_struct struct bt_hci_evt_cmd_status_s
{
  uint8_t status;
  uint8_t ncmd;
  uint16_t opcode;
} end_packed_struct;

begin_packed_struct struct bt_hci_evt_num_completed_packets_s
{
  uint8_t num_handles;
  struct bt_hci_handle_count_s h[0];
} end_packed_struct;

begin_packed_struct struct bt_hci_evt_encrypt_key_refresh_complete_s
{
  uint8_t status;
  uint16_t handle;
} end_packed_struct;

begin_packed_struct struct bt_hci_evt_remote_version_info_s
{
  uint8_t  status;
  uint16_t handle;
  uint8_t  version;
  uint16_t manufacturer;
  uint16_t subversion;
} end_packed_struct;

begin_packed_struct struct bt_hci_evt_conn_complete_s
{
  uint8_t   status;
  uint16_t  handle;
  bt_addr_t bdaddr;
  uint8_t   link_type;
  uint8_t   encr_enabled;
} end_packed_struct;

begin_packed_struct struct bt_hci_evt_le_meta_event_s
{
  uint8_t subevent;
} end_packed_struct;

begin_packed_struct struct bt_hci_evt_le_conn_complete_s
{
  uint8_t status;
  uint16_t handle;
  uint8_t role;
  bt_addr_le_t peer_addr;
  uint16_t interval;
  uint16_t latency;
  uint16_t supv_timeout;
  uint8_t clock_accuracy;
} end_packed_struct;

begin_packed_struct struct bt_hci_evt_le_enh_conn_complete_s
{
  uint8_t      status;
  uint16_t     handle;
  uint8_t      role;
  bt_addr_le_t peer_addr;
  bt_addr_t    local_rpa;
  bt_addr_t    peer_rpa;
  uint16_t     interval;
  uint16_t     latency;
  uint16_t     supv_timeout;
  uint8_t      clock_accuracy;
} end_packed_struct;

begin_packed_struct struct bt_hci_evt_le_conn_update_s
{
  uint8_t status;
  uint16_t handle;
  uint16_t interval;
  uint16_t latency;
  uint16_t timeout;
} end_packed_struct;

begin_packed_struct struct bt_hci_ev_le_advertising_report_s
{
  uint8_t evt_type;
  bt_addr_le_t addr;
  uint8_t length;
  uint8_t data[0];
} end_packed_struct;

begin_packed_struct struct bt_hci_evt_le_ltk_request_s
{
  uint16_t handle;
  uint64_t rand;
  uint16_t ediv;
} end_packed_struct;

#endif /* CONFIG_WIRELESS_BLUETOOTH */
#endif /* __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BT_HCI_H */
