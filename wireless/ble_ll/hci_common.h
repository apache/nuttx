/****************************************************************************
 * wireless/ble_ll/hci_common.h
 *
 * SPDX-License-Identifier: Apache-2.0
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
 * Derived from Apache Mynewt NimBLE (nimble/include/nimble/hci_common.h).
 *
 ****************************************************************************/

#ifndef __WIRELESS_BLE_LL_HCI_COMMON_H
#define __WIRELESS_BLE_LL_HCI_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "ble.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BLE_HCI_MAX_DATA_LEN \
  (CONFIG_BLE_LL_EVT_SIZE - sizeof(struct ble_hci_ev))

#define BLE_HCI_OPCODE_NOP (0)

/* Set opcode based on OCF and OGF */

#define BLE_HCI_OP(ogf, ocf) ((ocf) | ((ogf) << 10))

/* Get the OGF and OCF from the opcode in the command */

#define BLE_HCI_OGF(opcode) (((opcode) >> 10) & 0x003f)
#define BLE_HCI_OCF(opcode) ((opcode) & 0x03ff)

/* Opcode Group definitions (note: 0x07 not defined in spec) */

#define BLE_HCI_OGF_LINK_CTRL     (0x01)
#define BLE_HCI_OGF_LINK_POLICY   (0x02)
#define BLE_HCI_OGF_CTLR_BASEBAND (0x03)
#define BLE_HCI_OGF_INFO_PARAMS   (0x04)
#define BLE_HCI_OGF_STATUS_PARAMS (0x05)
#define BLE_HCI_OGF_TESTING       (0x06)
#define BLE_HCI_OGF_LE            (0x08)
#define BLE_HCI_OGF_VENDOR        (0x3f)

/* Number of LE commands. NOTE: this is really just used to size the array
 * containing the lengths of the LE commands.
 */

#define BLE_HCI_NUM_LE_CMDS (79)

/* List of OCF for Link Control commands (OGF=0x01) */

#define BLE_HCI_OCF_DISCONNECT_CMD  (0x0006)
#define BLE_HCI_OCF_RD_REM_VER_INFO (0x001d)

/* List of OCF for Controller and Baseband commands (OGF=0x03) */

#define BLE_HCI_OCF_CB_SET_EVENT_MASK      (0x0001)
#define BLE_HCI_OCF_CB_RESET               (0x0003)
#define BLE_HCI_OCF_CB_READ_TX_PWR         (0x002d)
#define BLE_HCI_OCF_CB_SET_CTLR_TO_HOST_FC (0x0031)
#define BLE_HCI_OCF_CB_HOST_BUF_SIZE       (0x0033)
#define BLE_HCI_OCF_CB_HOST_NUM_COMP_PKTS  (0x0035)
#define BLE_HCI_OCF_CB_SET_EVENT_MASK2     (0x0063)
#define BLE_HCI_OCF_CB_RD_AUTH_PYLD_TMO    (0x007b)
#define BLE_HCI_OCF_CB_WR_AUTH_PYLD_TMO    (0x007c)

/* List of OCF for Info Param commands (OGF=0x04) */

#define BLE_HCI_OCF_IP_RD_LOCAL_VER     (0x0001)
#define BLE_HCI_OCF_IP_RD_LOC_SUPP_CMD  (0x0002)
#define BLE_HCI_OCF_IP_RD_LOC_SUPP_FEAT (0x0003)
#define BLE_HCI_OCF_IP_RD_BUF_SIZE      (0x0005)
#define BLE_HCI_OCF_IP_RD_BD_ADDR       (0x0009)

/* List of OCF for Status parameters commands (OGF = 0x05) */

#define BLE_HCI_OCF_RD_RSSI (0x0005)

/* List of OCF for LE commands (OGF = 0x08) */

#define BLE_HCI_OCF_LE_SET_EVENT_MASK   (0x0001)
#define BLE_HCI_OCF_LE_RD_BUF_SIZE      (0x0002)
#define BLE_HCI_OCF_LE_RD_BUF_SIZE_V2   (0x0060)
#define BLE_HCI_OCF_LE_RD_LOC_SUPP_FEAT (0x0003)

/* NOTE: 0x0004 is intentionally left undefined */

#define BLE_HCI_OCF_LE_SET_RAND_ADDR                       (0x0005)
#define BLE_HCI_OCF_LE_SET_ADV_PARAMS                      (0x0006)
#define BLE_HCI_OCF_LE_RD_ADV_CHAN_TXPWR                   (0x0007)
#define BLE_HCI_OCF_LE_SET_ADV_DATA                        (0x0008)
#define BLE_HCI_MAX_ADV_DATA_LEN                           (31)
#define BLE_HCI_OCF_LE_SET_SCAN_RSP_DATA                   (0x0009)
#define BLE_HCI_MAX_SCAN_RSP_DATA_LEN                      (31)
#define BLE_HCI_OCF_LE_SET_ADV_ENABLE                      (0x000a)
#define BLE_HCI_OCF_LE_SET_SCAN_PARAMS                     (0x000b)
#define BLE_HCI_OCF_LE_SET_SCAN_ENABLE                     (0x000c)
#define BLE_HCI_OCF_LE_CREATE_CONN                         (0x000d)
#define BLE_HCI_OCF_LE_CREATE_CONN_CANCEL                  (0x000e)
#define BLE_HCI_OCF_LE_RD_WHITE_LIST_SIZE                  (0x000f)
#define BLE_HCI_OCF_LE_CLEAR_WHITE_LIST                    (0x0010)
#define BLE_HCI_OCF_LE_ADD_WHITE_LIST                      (0x0011)
#define BLE_HCI_OCF_LE_RMV_WHITE_LIST                      (0x0012)
#define BLE_HCI_OCF_LE_CONN_UPDATE                         (0x0013)
#define BLE_HCI_OCF_LE_SET_HOST_CHAN_CLASS                 (0x0014)
#define BLE_HCI_OCF_LE_RD_CHAN_MAP                         (0x0015)
#define BLE_HCI_OCF_LE_RD_REM_FEAT                         (0x0016)
#define BLE_HCI_OCF_LE_ENCRYPT                             (0x0017)
#define BLE_HCI_OCF_LE_RAND                                (0x0018)
#define BLE_HCI_OCF_LE_START_ENCRYPT                       (0x0019)
#define BLE_HCI_OCF_LE_LT_KEY_REQ_REPLY                    (0x001a)
#define BLE_HCI_OCF_LE_LT_KEY_REQ_NEG_REPLY                (0x001b)
#define BLE_HCI_OCF_LE_RD_SUPP_STATES                      (0x001c)
#define BLE_HCI_OCF_LE_RX_TEST                             (0x001d)
#define BLE_HCI_OCF_LE_TX_TEST                             (0x001e)
#define BLE_HCI_OCF_LE_TEST_END                            (0x001f)
#define BLE_HCI_OCF_LE_REM_CONN_PARAM_RR                   (0x0020)
#define BLE_HCI_OCF_LE_REM_CONN_PARAM_NRR                  (0x0021)
#define BLE_HCI_OCF_LE_SET_DATA_LEN                        (0x0022)
#define BLE_HCI_OCF_LE_RD_SUGG_DEF_DATA_LEN                (0x0023)
#define BLE_HCI_OCF_LE_WR_SUGG_DEF_DATA_LEN                (0x0024)
#define BLE_HCI_OCF_LE_RD_P256_PUBKEY                      (0x0025)
#define BLE_HCI_OCF_LE_GEN_DHKEY                           (0x0026)
#define BLE_HCI_OCF_LE_ADD_RESOLV_LIST                     (0x0027)
#define BLE_HCI_OCF_LE_RMV_RESOLV_LIST                     (0x0028)
#define BLE_HCI_OCF_LE_CLR_RESOLV_LIST                     (0x0029)
#define BLE_HCI_OCF_LE_RD_RESOLV_LIST_SIZE                 (0x002a)
#define BLE_HCI_OCF_LE_RD_PEER_RESOLV_ADDR                 (0x002b)
#define BLE_HCI_OCF_LE_RD_LOCAL_RESOLV_ADDR                (0x002c)
#define BLE_HCI_OCF_LE_SET_ADDR_RES_EN                     (0x002d)
#define BLE_HCI_OCF_LE_SET_RPA_TMO                         (0x002e)
#define BLE_HCI_OCF_LE_RD_MAX_DATA_LEN                     (0x002f)
#define BLE_HCI_OCF_LE_RD_PHY                              (0x0030)
#define BLE_HCI_OCF_LE_SET_DEFAULT_PHY                     (0x0031)
#define BLE_HCI_OCF_LE_SET_PHY                             (0x0032)
#define BLE_HCI_OCF_LE_RX_TEST_V2                          (0x0033)
#define BLE_HCI_OCF_LE_TX_TEST_V2                          (0x0034)
#define BLE_HCI_OCF_LE_SET_ADV_SET_RND_ADDR                (0x0035)
#define BLE_HCI_OCF_LE_SET_EXT_ADV_PARAM                   (0x0036)
#define BLE_HCI_OCF_LE_SET_EXT_ADV_DATA                    (0x0037)
#define BLE_HCI_OCF_LE_SET_EXT_SCAN_RSP_DATA               (0x0038)
#define BLE_HCI_OCF_LE_SET_EXT_ADV_ENABLE                  (0x0039)
#define BLE_HCI_OCF_LE_RD_MAX_ADV_DATA_LEN                 (0x003a)
#define BLE_HCI_OCF_LE_RD_NUM_OF_ADV_SETS                  (0x003b)
#define BLE_HCI_OCF_LE_REMOVE_ADV_SET                      (0x003c)
#define BLE_HCI_OCF_LE_CLEAR_ADV_SETS                      (0x003d)
#define BLE_HCI_OCF_LE_SET_PERIODIC_ADV_PARAMS             (0x003e)
#define BLE_HCI_OCF_LE_SET_PERIODIC_ADV_DATA               (0x003f)
#define BLE_HCI_OCF_LE_SET_PERIODIC_ADV_ENABLE             (0x0040)
#define BLE_HCI_OCF_LE_SET_EXT_SCAN_PARAM                  (0x0041)
#define BLE_HCI_OCF_LE_SET_EXT_SCAN_ENABLE                 (0x0042)
#define BLE_HCI_OCF_LE_EXT_CREATE_CONN                     (0x0043)
#define BLE_HCI_LE_PERIODIC_ADV_CREATE_SYNC_OPT_FILTER     0x01
#define BLE_HCI_LE_PERIODIC_ADV_CREATE_SYNC_OPT_DISABLED   0x02
#define BLE_HCI_LE_PERIODIC_ADV_CREATE_SYNC_OPT_DUPLICATES 0x04
#define BLE_HCI_OCF_LE_PERIODIC_ADV_CREATE_SYNC            (0x0044)
#define BLE_HCI_OCF_LE_PERIODIC_ADV_CREATE_SYNC_CANCEL     (0x0045)
#define BLE_HCI_OCF_LE_PERIODIC_ADV_TERM_SYNC              (0x0046)
#define BLE_HCI_OCF_LE_ADD_DEV_TO_PERIODIC_ADV_LIST        (0x0047)
#define BLE_HCI_OCF_LE_REM_DEV_FROM_PERIODIC_ADV_LIST      (0x0048)
#define BLE_HCI_OCF_LE_CLEAR_PERIODIC_ADV_LIST             (0x0049)
#define BLE_HCI_OCF_LE_RD_PERIODIC_ADV_LIST_SIZE           (0x004a)
#define BLE_HCI_OCF_LE_RD_TRANSMIT_POWER                   (0x004b)
#define BLE_HCI_OCF_LE_RD_RF_PATH_COMPENSATION             (0x004c)
#define BLE_HCI_OCF_LE_WR_RF_PATH_COMPENSATION             (0x004d)
#define BLE_HCI_OCF_LE_SET_PRIVACY_MODE                    (0x004e)
#define BLE_HCI_OCF_LE_RX_TEST_V3                          (0x004f)
#define BLE_HCI_OCF_LE_TX_TEST_V3                          (0x0050)
#define BLE_HCI_OCF_LE_SET_CONNLESS_CTE_TX_PARAMS          (0x0051)
#define BLE_HCI_OCF_LE_SET_CONNLESS_CTE_TX_ENABLE          (0x0052)
#define BLE_HCI_OCF_LE_SET_CONNLESS_IQ_SAMPLING_ENABLE     (0x0053)
#define BLE_HCI_OCF_LE_SET_CONN_CTE_RX_PARAMS              (0x0054)
#define BLE_HCI_OCF_LE_SET_CONN_CTE_TX_PARAMS              (0x0055)
#define BLE_HCI_OCF_LE_SET_CONN_CTE_REQ_ENABLE             (0x0056)
#define BLE_HCI_OCF_LE_SET_CONN_CTE_RESP_ENABLE            (0x0057)
#define BLE_HCI_OCF_LE_RD_ANTENNA_INFO                     (0x0058)
#define BLE_HCI_OCF_LE_PERIODIC_ADV_RECEIVE_ENABLE         (0x0059)
#define BLE_HCI_OCF_LE_PERIODIC_ADV_SYNC_TRANSFER          (0x005a)
#define BLE_HCI_OCF_LE_PERIODIC_ADV_SET_INFO_TRANSFER      (0x005b)
#define BLE_HCI_OCF_LE_PERIODIC_ADV_SYNC_TRANSFER_PARAMS   (0x005c)
#define BLE_HCI_OCF_LE_SET_DEFAULT_SYNC_TRANSFER_PARAMS    (0x005d)
#define BLE_HCI_OCF_LE_GENERATE_DHKEY_V2                   (0x005e)
#define BLE_HCI_OCF_LE_MODIFY_SCA                          (0x005f)
#define BLE_HCI_OCF_LE_READ_ISO_TX_SYNC                    (0x0061)
#define BLE_HCI_OCF_LE_SET_CIG_PARAMS                      (0x0062)
#define BLE_HCI_OCF_LE_SET_CIG_PARAMS_TEST                 (0x0063)
#define BLE_HCI_OCF_LE_CREATE_CIS                          (0x0064)
#define BLE_HCI_OCF_LE_REMOVE_CIG                          (0x0065)
#define BLE_HCI_OCF_LE_ACCEPT_CIS_REQ                      (0x0066)
#define BLE_HCI_OCF_LE_REJECT_CIS_REQ                      (0x0067)
#define BLE_HCI_OCF_LE_CREATE_BIG                          (0x0068)
#define BLE_HCI_OCF_LE_CREATE_BIG_TEST                     (0x0069)
#define BLE_HCI_OCF_LE_TERMINATE_BIG                       (0x006a)
#define BLE_HCI_OCF_LE_BIG_CREATE_SYNC                     (0x006b)
#define BLE_HCI_OCF_LE_BIG_TERMINATE_SYNC                  (0x006c)
#define BLE_HCI_OCF_LE_REQ_PEER_SCA                        (0x006d)
#define BLE_HCI_OCF_LE_SETUP_ISO_DATA_PATH                 (0x006e)
#define BLE_HCI_OCF_LE_REMOVE_ISO_DATA_PATH                (0x006f)
#define BLE_HCI_OCF_LE_ISO_TRANSMIT_TEST                   (0x0070)
#define BLE_HCI_OCF_LE_ISO_RECEIVE_TEST                    (0x0071)
#define BLE_HCI_OCF_LE_ISO_READ_TEST_COUNTERS              (0x0072)
#define BLE_HCI_OCF_LE_ISO_TEST_END                        (0x0073)
#define BLE_HCI_OCF_LE_SET_HOST_FEATURE                    (0x0074)
#define BLE_HCI_OCF_LE_READ_ISO_LINK_QUALITY               (0x0075)
#define BLE_HCI_OCF_LE_ENH_READ_TRANSMIT_POWER_LEVEL       (0x0076)
#define BLE_HCI_OCF_LE_READ_REMOTE_TRANSMIT_POWER_LEVEL    (0x0077)
#define BLE_HCI_OCF_LE_SET_PATH_LOSS_REPORT_PARAM          (0x0078)
#define BLE_HCI_OCF_LE_SET_PATH_LOSS_REPORT_ENABLE         (0x0079)
#define BLE_HCI_OCF_LE_SET_TRANS_PWR_REPORT_ENABLE         (0x007a)
#define BLE_HCI_OCF_LE_SET_DEFAULT_SUBRATE                 (0x007d)
#define BLE_HCI_OCF_LE_SUBRATE_REQ                         (0x007e)
#define BLE_HCI_OCF_LE_SET_EXT_ADV_PARAM_V2                (0x007f)
#define BLE_HCI_OCF_LE_CS_RD_LOC_SUPP_CAP                  (0x0089)
#define BLE_HCI_OCF_LE_CS_RD_REM_SUPP_CAP                  (0x008a)
#define BLE_HCI_OCF_LE_CS_WR_CACHED_REM_SUPP_CAP           (0x008b)
#define BLE_HCI_OCF_LE_CS_SEC_ENABLE                       (0x008c)
#define BLE_HCI_OCF_LE_CS_SET_DEF_SETTINGS                 (0x008d)
#define BLE_HCI_OCF_LE_CS_RD_REM_FAE                       (0x008e)
#define BLE_HCI_OCF_LE_CS_WR_CACHED_REM_FAE                (0x008f)
#define BLE_HCI_OCF_LE_CS_CREATE_CONFIG                    (0x0090)
#define BLE_HCI_OCF_LE_CS_REMOVE_CONFIG                    (0x0091)
#define BLE_HCI_OCF_LE_CS_SET_CHAN_CLASS                   (0x0092)
#define BLE_HCI_OCF_LE_CS_SET_PROC_PARAMS                  (0x0093)
#define BLE_HCI_OCF_LE_CS_PROC_ENABLE                      (0x0094)
#define BLE_HCI_OCF_LE_CS_TEST                             (0x0095)
#define BLE_HCI_OCF_LE_CS_TEST_END                         (0x0096)

/* --- Vendor specific commands (OGF 0x003F) */

/* Read Random Static Address */

#define BLE_HCI_OCF_VS_RD_STATIC_ADDR \
  (CONFIG_BLE_LL_HCI_VS_OCF_OFFSET + (0x0001))

/* Set default transmit power. Actual selected TX power is returned
 * in reply. Setting 0xff restores controller reset default.
 */

#define BLE_HCI_OCF_VS_SET_TX_PWR \
  (CONFIG_BLE_LL_HCI_VS_OCF_OFFSET + (0x0002))

#define BLE_HCI_OCF_VS_CSS_CONFIGURE \
  (CONFIG_BLE_LL_HCI_VS_OCF_OFFSET + (0x0003))

#define BLE_HCI_OCF_VS_CSS_ENABLE \
  (CONFIG_BLE_LL_HCI_VS_OCF_OFFSET + (0x0004))

#define BLE_HCI_OCF_VS_CSS_SET_NEXT_SLOT \
  (CONFIG_BLE_LL_HCI_VS_OCF_OFFSET + (0x0005))

#define BLE_HCI_OCF_VS_CSS_SET_CONN_SLOT \
  (CONFIG_BLE_LL_HCI_VS_OCF_OFFSET + (0x0006))

#define BLE_HCI_OCF_VS_CSS_READ_CONN_SLOT \
  (CONFIG_BLE_LL_HCI_VS_OCF_OFFSET + (0x0007))

#define BLE_HCI_OCF_VS_SET_DATA_LEN \
  (CONFIG_BLE_LL_HCI_VS_OCF_OFFSET + (0x0008))

#define BLE_HCI_OCF_VS_SET_ANTENNA \
  (CONFIG_BLE_LL_HCI_VS_OCF_OFFSET + (0x0009))

#define BLE_HCI_OCF_VS_SET_LOCAL_IRK \
  (CONFIG_BLE_LL_HCI_VS_OCF_OFFSET + (0x000a))

#define BLE_HCI_VS_SET_SCAN_CFG_FLAG_NO_LEGACY   (0x00000001)
#define BLE_HCI_VS_SET_SCAN_CFG_FLAG_NO_EXT      (0x00000002)
#define BLE_HCI_VS_SET_SCAN_CFG_FLAG_RSSI_FILTER (0x00000004)
#define BLE_HCI_OCF_VS_SET_SCAN_CFG \
  (CONFIG_BLE_LL_HCI_VS_OCF_OFFSET + (0x000b))

/* Command Specific Definitions */

/* --- Set controller to host flow control (OGF 0x03, OCF 0x0031) --- */

#define BLE_HCI_CTLR_TO_HOST_FC_OFF  (0)
#define BLE_HCI_CTLR_TO_HOST_FC_ACL  (1)
#define BLE_HCI_CTLR_TO_HOST_FC_SYNC (2)
#define BLE_HCI_CTLR_TO_HOST_FC_BOTH (3)

/* --- LE set advertising parameters (OCF 0x0006) */

/* Advertising types */

#define BLE_HCI_ADV_TYPE_ADV_IND           (0)
#define BLE_HCI_ADV_TYPE_ADV_DIRECT_IND_HD (1)
#define BLE_HCI_ADV_TYPE_ADV_SCAN_IND      (2)
#define BLE_HCI_ADV_TYPE_ADV_NONCONN_IND   (3)
#define BLE_HCI_ADV_TYPE_ADV_DIRECT_IND_LD (4)
#define BLE_HCI_ADV_TYPE_MAX               (4)
#define BLE_HCI_ADV_CONN_MASK              (0x0001)
#define BLE_HCI_ADV_SCAN_MASK              (0x0002)
#define BLE_HCI_ADV_DIRECT_MASK            (0x0004)
#define BLE_HCI_ADV_SCAN_RSP_MASK          (0x0008)
#define BLE_HCI_ADV_LEGACY_MASK            (0x0010)
#define BLE_HCI_ADV_DATA_STATUS_COMPLETE   (0x0000)
#define BLE_HCI_ADV_DATA_STATUS_INCOMPLETE (0x0020)
#define BLE_HCI_ADV_DATA_STATUS_TRUNCATED  (0x0040)
#define BLE_HCI_ADV_DATA_STATUS_MASK       (0x0060)

/* Own address types */

#define BLE_HCI_ADV_OWN_ADDR_PUBLIC    (0)
#define BLE_HCI_ADV_OWN_ADDR_RANDOM    (1)
#define BLE_HCI_ADV_OWN_ADDR_PRIV_PUB  (2)
#define BLE_HCI_ADV_OWN_ADDR_PRIV_RAND (3)
#define BLE_HCI_ADV_OWN_ADDR_MAX       (3)

/* Advertisement peer address Type */

#define BLE_HCI_ADV_PEER_ADDR_PUBLIC (0)
#define BLE_HCI_ADV_PEER_ADDR_RANDOM (1)
#define BLE_HCI_ADV_PEER_ADDR_MAX    (1)
#if CONFIG_BLE_LL_BT_VERSION == 50

/* --- LE advertising channel tx power (OCF 0x0007) */

#define BLE_HCI_ADV_CHAN_TXPWR_MIN (-20)
#define BLE_HCI_ADV_CHAN_TXPWR_MAX (10)
#elif CONFIG_BLE_LL_BT_VERSION == 51
#define BLE_HCI_ADV_CHAN_TXPWR_MIN (-20)
#define BLE_HCI_ADV_CHAN_TXPWR_MAX (20)
#elif CONFIG_BLE_LL_BT_VERSION >= 52
#define BLE_HCI_ADV_CHAN_TXPWR_MIN (-127)
#define BLE_HCI_ADV_CHAN_TXPWR_MAX (20)
#endif

/* --- LE set scan enable (OCF 0x000c) */

/* Connect peer address type */

#define BLE_HCI_CONN_PEER_ADDR_PUBLIC       (0)
#define BLE_HCI_CONN_PEER_ADDR_RANDOM       (1)
#define BLE_HCI_CONN_PEER_ADDR_PUBLIC_IDENT (2)
#define BLE_HCI_CONN_PEER_ADDR_RANDOM_IDENT (3)
#define BLE_HCI_CONN_PEER_ADDR_MAX          (3)

/* Advertising filter policy
 *
 * Determines how an advertiser filters scan and connection requests.
 *
 *  NONE: no filtering (default value). No whitelist used.
 *  SCAN: process all connection requests but only scans from white list.
 *  CONN: process all scan request but only connection requests from white
 * list BOTH: ignore all scan and connection requests unless in white list.
 */

#define BLE_HCI_ADV_FILT_NONE (0)
#define BLE_HCI_ADV_FILT_SCAN (1)
#define BLE_HCI_ADV_FILT_CONN (2)
#define BLE_HCI_ADV_FILT_BOTH (3)
#define BLE_HCI_ADV_FILT_MAX  (3)
#define BLE_HCI_ADV_FILT_DEF  (BLE_HCI_ADV_FILT_NONE)

/* Advertising interval */

#define BLE_HCI_ADV_ITVL             (625)   /* usecs */
#define BLE_HCI_ADV_ITVL_MIN         (32)    /* units */
#define BLE_HCI_ADV_ITVL_MAX         (16384) /* units */
#define BLE_HCI_ADV_ITVL_NONCONN_MIN (160)   /* units */
#define BLE_HCI_ADV_ITVL_DEF         (0x800) /* 1.28 seconds */
#define BLE_HCI_ADV_CHANMASK_DEF     (0x7)   /* all channels */
#define BLE_HCI_PERIODIC_ADV_ITVL    (1250)  /* usecs */

/* Set scan parameters */

#define BLE_HCI_SCAN_TYPE_PASSIVE (0)
#define BLE_HCI_SCAN_TYPE_ACTIVE  (1)

/* Scan interval and scan window timing */

#define BLE_HCI_SCAN_ITVL           (625)    /* usecs */
#define BLE_HCI_SCAN_ITVL_MIN       (0x0004) /* units */
#define BLE_HCI_SCAN_ITVL_MAX       (0x4000) /* units */
#define BLE_HCI_SCAN_ITVL_MAX_EXT   (0xffff) /* units */
#define BLE_HCI_SCAN_ITVL_DEF       (16)     /* units */
#define BLE_HCI_SCAN_WINDOW_MIN     (0x0004) /* units */
#define BLE_HCI_SCAN_WINDOW_MAX     (0x4000) /* units */
#define BLE_HCI_SCAN_WINDOW_MAX_EXT (0xffff) /* units */
#define BLE_HCI_SCAN_WINDOW_DEF     (16)     /* units */

/* Scanning filter policy
 *  NO_WL:
 *      Scanner processes all advertising packets (white list not used)
 * except directed, connectable advertising packets not sent to the scanner.
 *  USE_WL:
 *      Scanner processes advertisements from white list only. A
 *      connectable, directed advertisement is ignored unless it contains
 *      scanners address.
 *  NO_WL_INITA:
 *      Scanner process all advertising packets (white list not used). A
 *      connectable, directed advertisement shall not be ignored if the InitA
 *      is a resolvable private address.
 *  USE_WL_INITA:
 *      Scanner process advertisements from white list only. A connectable,
 *      directed advertisement shall not be ignored if the InitA is a
 *      resolvable private address.
 */

#define BLE_HCI_SCAN_FILT_NO_WL        (0)
#define BLE_HCI_SCAN_FILT_USE_WL       (1)
#define BLE_HCI_SCAN_FILT_NO_WL_INITA  (2)
#define BLE_HCI_SCAN_FILT_USE_WL_INITA (3)
#define BLE_HCI_SCAN_FILT_MAX          (3)

/* Whitelist commands */

#define BLE_HCI_ADD_WHITE_LIST_LEN (7)
#define BLE_HCI_RMV_WHITE_LIST_LEN (7)

/* Create Connection */

#define BLE_HCI_CREATE_CONN_LEN           (25)
#define BLE_HCI_CONN_ITVL                 (1250) /* usecs */
#define BLE_HCI_CONN_FILT_NO_WL           (0)
#define BLE_HCI_CONN_FILT_USE_WL          (1)
#define BLE_HCI_CONN_FILT_MAX             (1)
#define BLE_HCI_CONN_ITVL_MIN             (0x0006)
#define BLE_HCI_CONN_ITVL_MAX             (0x0c80)
#define BLE_HCI_CONN_LATENCY_MIN          (0x0000)
#define BLE_HCI_CONN_LATENCY_MAX          (0x01f3)
#define BLE_HCI_CONN_SPVN_TIMEOUT_MIN     (0x000a)
#define BLE_HCI_CONN_SPVN_TIMEOUT_MAX     (0x0c80)
#define BLE_HCI_CONN_SPVN_TMO_UNITS       (10) /* msecs */
#define BLE_HCI_INITIATOR_FILT_POLICY_MAX (1)

/* Peer Address Type */

#define BLE_HCI_CONN_PEER_ADDR_PUBLIC  (0)
#define BLE_HCI_CONN_PEER_ADDR_RANDOM  (1)
#define BLE_HCI_CONN_PEER_ADDR_PUB_ID  (2)
#define BLE_HCI_CONN_PEER_ADDR_RAND_ID (3)
#define BLE_HCI_CONN_PEER_ADDR_MAX     (3)

/* --- LE set data length (OCF 0x0022) */

#define BLE_HCI_SET_DATALEN_TX_OCTETS_MIN (0x001b)
#define BLE_HCI_SET_DATALEN_TX_OCTETS_MAX (0x00fb)
#define BLE_HCI_SET_DATALEN_TX_TIME_MIN   (0x0148)
#define BLE_HCI_SET_DATALEN_TX_TIME_MAX   (0x4290)

/* --- LE read/write suggested default data length (OCF 0x0023 and 0x0024) */

#define BLE_HCI_SUGG_DEF_DATALEN_TX_OCTETS_MIN (0x001b)
#define BLE_HCI_SUGG_DEF_DATALEN_TX_OCTETS_MAX (0x00fb)
#define BLE_HCI_SUGG_DEF_DATALEN_TX_TIME_MIN   (0x0148)
#define BLE_HCI_SUGG_DEF_DATALEN_TX_TIME_MAX   (0x4290)

/* --- LE read maximum default PHY (OCF 0x0030) */

#define BLE_HCI_LE_PHY_1M    (1)
#define BLE_HCI_LE_PHY_2M    (2)
#define BLE_HCI_LE_PHY_CODED (3)

/* --- LE set default PHY (OCF 0x0031) */

#define BLE_HCI_LE_PHY_NO_TX_PREF_MASK (0x01)
#define BLE_HCI_LE_PHY_NO_RX_PREF_MASK (0x02)
#define BLE_HCI_LE_PHY_1M_PREF_MASK    (0x01)
#define BLE_HCI_LE_PHY_2M_PREF_MASK    (0x02)
#define BLE_HCI_LE_PHY_CODED_PREF_MASK (0x04)
#define BLE_HCI_LE_PHY_PREF_MASK_ALL                           \
  (BLE_HCI_LE_PHY_1M_PREF_MASK | BLE_HCI_LE_PHY_2M_PREF_MASK | \
   BLE_HCI_LE_PHY_CODED_PREF_MASK)

/* --- LE set PHY (OCF 0x0032) */

#define BLE_HCI_LE_PHY_CODED_ANY     (0x0000)
#define BLE_HCI_LE_PHY_CODED_S2_PREF (0x0001)
#define BLE_HCI_LE_PHY_CODED_S8_PREF (0x0002)

/* --- LE enhanced receiver test (OCF 0x0033) */

#define BLE_HCI_LE_PHY_1M    (1)
#define BLE_HCI_LE_PHY_2M    (2)
#define BLE_HCI_LE_PHY_CODED (3)

/* --- LE enhanced transmitter test (OCF 0x0034) */

#define BLE_HCI_LE_PHY_CODED_S8 (3)
#define BLE_HCI_LE_PHY_CODED_S2 (4)

/* --- LE set extended advertising parameters (OCF 0x0036) */

#define BLE_HCI_LE_SET_EXT_ADV_PROP_CONNECTABLE    (0x0001)
#define BLE_HCI_LE_SET_EXT_ADV_PROP_SCANNABLE      (0x0002)
#define BLE_HCI_LE_SET_EXT_ADV_PROP_DIRECTED       (0x0004)
#define BLE_HCI_LE_SET_EXT_ADV_PROP_HD_DIRECTED    (0x0008)
#define BLE_HCI_LE_SET_EXT_ADV_PROP_LEGACY         (0x0010)
#define BLE_HCI_LE_SET_EXT_ADV_PROP_ANON_ADV       (0x0020)
#define BLE_HCI_LE_SET_EXT_ADV_PROP_INC_TX_PWR     (0x0040)
#define BLE_HCI_LE_SET_EXT_ADV_PROP_MASK           (0x7f)
#define BLE_HCI_LE_SET_EXT_ADV_PROP_LEGACY_IND     (0x0013)
#define BLE_HCI_LE_SET_EXT_ADV_PROP_LEGACY_LD_DIR  (0x0015)
#define BLE_HCI_LE_SET_EXT_ADV_PROP_LEGACY_HD_DIR  (0x001d)
#define BLE_HCI_LE_SET_EXT_ADV_PROP_LEGACY_SCAN    (0x0012)
#define BLE_HCI_LE_SET_EXT_ADV_PROP_LEGACY_NONCONN (0x0010)

/* --- LE set extended advertising data (OCF 0x0037) */

#define BLE_HCI_MAX_EXT_ADV_DATA_LEN       (251)
#define BLE_HCI_LE_SET_DATA_OPER_INT       (0)
#define BLE_HCI_LE_SET_DATA_OPER_FIRST     (1)
#define BLE_HCI_LE_SET_DATA_OPER_LAST      (2)
#define BLE_HCI_LE_SET_DATA_OPER_COMPLETE  (3)
#define BLE_HCI_LE_SET_DATA_OPER_UNCHANGED (4)

/* --- LE set extended scan response data (OCF 0x0038) */

#define BLE_HCI_MAX_EXT_SCAN_RSP_DATA_LEN (251)

/* --- LE set periodic advertising parameters (OCF 0x003E) */

#define BLE_HCI_LE_SET_PERIODIC_ADV_PROP_INC_TX_PWR (0x0040)
#define BLE_HCI_LE_SET_PERIODIC_ADV_PROP_MASK       (0x0040)

/* --- LE set periodic advertising data (OCF 0x003F) */

#define BLE_HCI_MAX_PERIODIC_ADV_DATA_LEN (252)

/* --- LE remove device from periodic advertising list (OCF 0x0048) */

#define BLE_HCI_PERIODIC_DATA_STATUS_COMPLETE   0x00
#define BLE_HCI_PERIODIC_DATA_STATUS_INCOMPLETE 0x01
#define BLE_HCI_PERIODIC_DATA_STATUS_TRUNCATED  0x02

/* --- LE set privacy mode (OCF 0x004E) */

#define BLE_HCI_PRIVACY_NETWORK (0)
#define BLE_HCI_PRIVACY_DEVICE  (1)

/* --- LE iso transmit test  payload type options (OCF 0x0070) */

#define BLE_HCI_PAYLOAD_TYPE_ZERO_LENGTH     (0x00)
#define BLE_HCI_PAYLOAD_TYPE_VARIABLE_LENGTH (0x01)
#define BLE_HCI_PAYLOAD_TYPE_MAXIMUM_LENGTH  (0x02)

/* --- LE set advertising coded PHY options (OCF 0x007F) */

#define BLE_HCI_ADVERTISING_PHY_OPT_NO_PREF 0x0
#define BLE_HCI_ADVERTISING_PHY_OPT_S2_PREF 0x1
#define BLE_HCI_ADVERTISING_PHY_OPT_S8_PREF 0x2
#define BLE_HCI_ADVERTISING_PHY_OPT_S2_REQ  0x3
#define BLE_HCI_ADVERTISING_PHY_OPT_S8_REQ  0x4

/* Event Codes */

#define BLE_HCI_EVCODE_INQUIRY_CMP          (0x01)
#define BLE_HCI_EVCODE_INQUIRY_RESULT       (0x02)
#define BLE_HCI_EVCODE_CONN_DONE            (0x03)
#define BLE_HCI_EVCODE_CONN_REQUEST         (0x04)
#define BLE_HCI_EVCODE_DISCONN_CMP          (0x05)
#define BLE_HCI_EVCODE_AUTH_CMP             (0x06)
#define BLE_HCI_EVCODE_REM_NAME_REQ_CMP     (0x07)
#define BLE_HCI_EVCODE_ENCRYPT_CHG          (0x08)
#define BLE_HCI_EVCODE_CHG_LINK_KEY_CMP     (0x09)
#define BLE_HCI_EVCODE_MASTER_LINK_KEY_CMP  (0x0a)
#define BLE_HCI_EVCODE_RD_REM_SUPP_FEAT_CMP (0x0b)
#define BLE_HCI_EVCODE_RD_REM_VER_INFO_CMP  (0x0c)
#define BLE_HCI_EVCODE_QOS_SETUP_CMP        (0x0d)
#define BLE_HCI_EVCODE_COMMAND_COMPLETE     (0x0e)
#define BLE_HCI_EVCODE_COMMAND_STATUS       (0x0f)
#define BLE_HCI_EVCODE_HW_ERROR             (0x10)
#define BLE_HCI_EVCODE_NUM_COMP_PKTS        (0x13)
#define BLE_HCI_EVCODE_MODE_CHANGE          (0x14)
#define BLE_HCI_EVCODE_RETURN_LINK_KEYS     (0x15)
#define BLE_HCI_EVCODE_PIN_CODE_REQ         (0x16)
#define BLE_HCI_EVCODE_LINK_KEY_REQ         (0x17)
#define BLE_HCI_EVCODE_LINK_KEY_NOTIFY      (0x18)
#define BLE_HCI_EVCODE_LOOPBACK_CMD         (0x19)
#define BLE_HCI_EVCODE_DATA_BUF_OVERFLOW    (0x1a)
#define BLE_HCI_EVCODE_MAX_SLOTS_CHG        (0x1b)
#define BLE_HCI_EVCODE_READ_CLK_OFF_COMP    (0x1c)
#define BLE_HCI_EVCODE_CONN_PKT_TYPE_CHG    (0x1d)
#define BLE_HCI_EVCODE_QOS_VIOLATION        (0x1e)

/* NOTE: 0x1F not defined */

#define BLE_HCI_EVCODE_PSR_MODE_CHG      (0x20)
#define BLE_HCI_EVCODE_FLOW_SPEC_COMP    (0x21)
#define BLE_HCI_EVCODE_INQ_RESULT_RSSI   (0x22)
#define BLE_HCI_EVCODE_READ_REM_EXT_FEAT (0x23)

/* NOTE: 0x24 - 0x2B not defined */

#define BLE_HCI_EVCODE_SYNCH_CONN_COMP  (0x2c)
#define BLE_HCI_EVCODE_SYNCH_CONN_CHG   (0x2d)
#define BLE_HCI_EVCODE_SNIFF_SUBRATING  (0x2e)
#define BLE_HCI_EVCODE_EXT_INQ_RESULT   (0x2f)
#define BLE_HCI_EVCODE_ENC_KEY_REFRESH  (0x30)
#define BLE_HCI_EVOCDE_IO_CAP_REQ       (0x31)
#define BLE_HCI_EVCODE_IO_CAP_RSP       (0x32)
#define BLE_HCI_EVCODE_USER_CONFIRM_REQ (0x33)
#define BLE_HCI_EVCODE_PASSKEY_REQ      (0x34)
#define BLE_HCI_EVCODE_REM_OOB_DATA_REQ (0x35)
#define BLE_HCI_EVCODE_SIMPLE_PAIR_COMP (0x36)

/* NOTE: 0x37 not defined */

#define BLE_HCI_EVCODE_LNK_SPVN_TMO_CHG    (0x38)
#define BLE_HCI_EVCODE_ENH_FLUSH_COMP      (0x39)
#define BLE_HCI_EVCODE_USER_PASSKEY_NOTIFY (0x3b)
#define BLE_HCI_EVCODE_KEYPRESS_NOTIFY     (0x3c)
#define BLE_HCI_EVCODE_REM_HOST_SUPP_FEAT  (0x3d)
#define BLE_HCI_EVCODE_LE_META             (0x3e)

/* NOTE: 0x3F not defined */

#define BLE_HCI_EVCODE_PHYS_LINK_COMP        (0x40)
#define BLE_HCI_EVCODE_CHAN_SELECTED         (0x41)
#define BLE_HCI_EVCODE_DISCONN_PHYS_LINK     (0x42)
#define BLE_HCI_EVCODE_PHYS_LINK_LOSS_EARLY  (0x43)
#define BLE_HCI_EVCODE_PHYS_LINK_RECOVERY    (0x44)
#define BLE_HCI_EVCODE_LOGICAL_LINK_COMP     (0x45)
#define BLE_HCI_EVCODE_DISCONN_LOGICAL_LINK  (0x46)
#define BLE_HCI_EVCODE_FLOW_SPEC_MODE_COMP   (0x47)
#define BLE_HCI_EVCODE_NUM_COMP_DATA_BLKS    (0x48)
#define BLE_HCI_EVCODE_AMP_START_TEST        (0x49)
#define BLE_HCI_EVOCDE_AMP_TEST_END          (0x4a)
#define BLE_HCI_EVOCDE_AMP_RCVR_REPORT       (0x4b)
#define BLE_HCI_EVCODE_SHORT_RANGE_MODE_CHG  (0x4c)
#define BLE_HCI_EVCODE_AMP_STATUS_CHG        (0x4d)
#define BLE_HCI_EVCODE_TRIG_CLK_CAPTURE      (0x4e)
#define BLE_HCI_EVCODE_SYNCH_TRAIN_COMP      (0x4f)
#define BLE_HCI_EVCODE_SYNCH_TRAIN_RCVD      (0x50)
#define BLE_HCI_EVCODE_SLAVE_BCAST_RX        (0x51)
#define BLE_HCI_EVCODE_SLAVE_BCAST_TMO       (0x52)
#define BLE_HCI_EVCODE_TRUNC_PAGE_COMP       (0x53)
#define BLE_HCI_EVCODE_SLAVE_PAGE_RSP_TMO    (0x54)
#define BLE_HCI_EVCODE_SLAVE_BCAST_CHAN_MAP  (0x55)
#define BLE_HCI_EVCODE_INQ_RSP_NOTIFY        (0x56)
#define BLE_HCI_EVCODE_AUTH_PYLD_TMO         (0x57)
#define BLE_HCI_EVCODE_SAM_STATUS_CHG        (0x58)
#define BLE_HCI_EVCODE_VS                    (0xff)
#define BLE_HCI_VS_SUBEV_ID_ASSERT           (0x01)
#define BLE_HCI_VS_SUBEV_ID_CSS_SLOT_CHANGED (0x02)
#define BLE_HCI_VS_SUBEV_ISO_HCI_FEEDBACK    (0x03)
#define BLE_HCI_VS_SUBEV_ID_LLCP_TRACE       (0x17)

/* LE sub-event codes */

#define BLE_HCI_LE_SUBEV_CONN_COMPLETE               (0x01)
#define BLE_HCI_LE_SUBEV_ADV_RPT                     (0x02)
#define BLE_HCI_LE_SUBEV_CONN_UPD_COMPLETE           (0x03)
#define BLE_HCI_LE_SUBEV_RD_REM_USED_FEAT            (0x04)
#define BLE_HCI_LE_SUBEV_LT_KEY_REQ                  (0x05)
#define BLE_HCI_LE_SUBEV_REM_CONN_PARM_REQ           (0x06)
#define BLE_HCI_LE_SUBEV_DATA_LEN_CHG                (0x07)
#define BLE_HCI_LE_SUBEV_RD_LOC_P256_PUBKEY          (0x08)
#define BLE_HCI_LE_SUBEV_GEN_DHKEY_COMPLETE          (0x09)
#define BLE_HCI_LE_SUBEV_ENH_CONN_COMPLETE           (0x0a)
#define BLE_HCI_LE_SUBEV_DIRECT_ADV_RPT              (0x0b)
#define BLE_HCI_LE_SUBEV_PHY_UPDATE_COMPLETE         (0x0c)
#define BLE_HCI_LE_SUBEV_EXT_ADV_RPT                 (0x0d)
#define BLE_HCI_LE_SUBEV_PERIODIC_ADV_SYNC_ESTAB     (0x0e)
#define BLE_HCI_LE_SUBEV_PERIODIC_ADV_RPT            (0x0f)
#define BLE_HCI_LE_SUBEV_PERIODIC_ADV_SYNC_LOST      (0x10)
#define BLE_HCI_LE_SUBEV_SCAN_TIMEOUT                (0x11)
#define BLE_HCI_LE_SUBEV_ADV_SET_TERMINATED          (0x12)
#define BLE_HCI_LE_SUBEV_SCAN_REQ_RCVD               (0x13)
#define BLE_HCI_LE_SUBEV_CHAN_SEL_ALG                (0x14)
#define BLE_HCI_LE_SUBEV_CONNLESS_IQ_RPT             (0x15)
#define BLE_HCI_LE_SUBEV_CONN_IQ_RPT                 (0x16)
#define BLE_HCI_LE_SUBEV_CTE_REQ_FAILED              (0x17)
#define BLE_HCI_LE_SUBEV_PERIODIC_ADV_SYNC_TRANSFER  (0x18)
#define BLE_HCI_LE_SUBEV_CIS_ESTABLISHED             (0x19)
#define BLE_HCI_LE_SUBEV_CIS_REQUEST                 (0x1a)
#define BLE_HCI_LE_SUBEV_CREATE_BIG_COMPLETE         (0x1b)
#define BLE_HCI_LE_SUBEV_TERMINATE_BIG_COMPLETE      (0x1c)
#define BLE_HCI_LE_SUBEV_BIG_SYNC_ESTABLISHED        (0x1d)
#define BLE_HCI_LE_SUBEV_BIG_SYNC_LOST               (0x1e)
#define BLE_HCI_LE_SUBEV_REQ_PEER_SCA_COMP           (0x1f)
#define BLE_HCI_LE_SUBEV_PATH_LOSS_THRESHOLD         (0x20)
#define BLE_HCI_LE_SUBEV_TRANSMIT_POWER_REPORT       (0x21)
#define BLE_HCI_LE_SUBEV_BIGINFO_ADV_REPORT          (0x22)
#define BLE_HCI_LE_SUBEV_SUBRATE_CHANGE              (0x23)
#define BLE_HCI_LE_SUBEV_CS_RD_REM_SUPP_CAP_COMPLETE (0x2c)
#define BLE_HCI_LE_SUBEV_CS_RD_REM_FAE_COMPLETE      (0x2d)
#define BLE_HCI_LE_SUBEV_CS_SEC_ENABLE_COMPLETE      (0x2e)
#define BLE_HCI_LE_SUBEV_CS_CONFIG_COMPLETE          (0x2f)
#define BLE_HCI_LE_SUBEV_CS_PROC_ENABLE_COMPLETE     (0x30)
#define BLE_HCI_LE_SUBEV_CS_SUBEVENT_RESULT          (0x31)
#define BLE_HCI_LE_SUBEV_CS_SUBEVENT_RESULT_CONTINUE (0x32)
#define BLE_HCI_LE_SUBEV_CS_TEST_END_COMPLETE        (0x33)

/* Data buffer overflow event */

#define BLE_HCI_EVENT_ACL_BUF_OVERFLOW (0x01)

/* Advertising report */

#define BLE_HCI_ADV_RPT_EVTYPE_ADV_IND     (0)
#define BLE_HCI_ADV_RPT_EVTYPE_DIR_IND     (1)
#define BLE_HCI_ADV_RPT_EVTYPE_SCAN_IND    (2)
#define BLE_HCI_ADV_RPT_EVTYPE_NONCONN_IND (3)
#define BLE_HCI_ADV_RPT_EVTYPE_SCAN_RSP    (4)

/* Bluetooth 5, Vol 2, Part E, 7.7.65.13 */

#define BLE_HCI_LEGACY_ADV_EVTYPE_ADV_IND               (0x13)
#define BLE_HCI_LEGACY_ADV_EVTYPE_ADV_DIRECT_IND        (0x15)
#define BLE_HCI_LEGACY_ADV_EVTYPE_ADV_SCAN_IND          (0x12)
#define BLE_HCI_LEGACY_ADV_EVTYPE_ADV_NONCON_IND        (0x10)
#define BLE_HCI_LEGACY_ADV_EVTYPE_SCAN_RSP_ADV_IND      (0x1b)
#define BLE_HCI_LEGACY_ADV_EVTYPE_SCAN_RSP_ADV_SCAN_IND (0x1a)

/* LE connection complete event (sub event 0x01) */

#define BLE_HCI_LE_CONN_COMPLETE_ROLE_MASTER (0x00)
#define BLE_HCI_LE_CONN_COMPLETE_ROLE_SLAVE  (0x01)

/* Maximum valid connection handle value */

#define BLE_HCI_LE_CONN_HANDLE_MAX (0x0eff)

/* LE advertising report event. (sub event 0x02) */

#define BLE_HCI_LE_ADV_RPT_NUM_RPTS_MIN (1)
#define BLE_HCI_LE_ADV_RPT_NUM_RPTS_MAX (0x19)

/* Bluetooth Assigned numbers for version information. */

#define BLE_HCI_VER_BCS_1_0b    (0)
#define BLE_HCI_VER_BCS_1_1     (1)
#define BLE_HCI_VER_BCS_1_2     (2)
#define BLE_HCI_VER_BCS_2_0_EDR (3)
#define BLE_HCI_VER_BCS_2_1_EDR (4)
#define BLE_HCI_VER_BCS_3_0_HCS (5)
#define BLE_HCI_VER_BCS_4_0     (6)
#define BLE_HCI_VER_BCS_4_1     (7)
#define BLE_HCI_VER_BCS_4_2     (8)
#define BLE_HCI_VER_BCS_5_0     (9)
#define BLE_HCI_VER_BCS_5_1     (10)
#define BLE_HCI_VER_BCS_5_2     (11)
#define BLE_HCI_VER_BCS_5_3     (12)
#define BLE_HCI_VER_BCS_5_4     (13)
#define BLE_HCI_VER_BCS_6_0     (14)
#define BLE_HCI_VER_BCS_6_1     (15)
#define BLE_LMP_VER_BCS_1_0b    (0)
#define BLE_LMP_VER_BCS_1_1     (1)
#define BLE_LMP_VER_BCS_1_2     (2)
#define BLE_LMP_VER_BCS_2_0_EDR (3)
#define BLE_LMP_VER_BCS_2_1_EDR (4)
#define BLE_LMP_VER_BCS_3_0_HCS (5)
#define BLE_LMP_VER_BCS_4_0     (6)
#define BLE_LMP_VER_BCS_4_1     (7)
#define BLE_LMP_VER_BCS_4_2     (8)
#define BLE_LMP_VER_BCS_5_0     (9)
#define BLE_LMP_VER_BCS_5_1     (10)
#define BLE_LMP_VER_BCS_5_2     (11)
#define BLE_LMP_VER_BCS_5_3     (12)
#define BLE_LMP_VER_BCS_5_4     (13)
#define BLE_LMP_VER_BCS_6_0     (14)
#define BLE_LMP_VER_BCS_6_1     (15)
#if CONFIG_BLE_LL_BT_VERSION == 50

/* selected HCI and LMP version */

#define BLE_HCI_VER_BCS BLE_HCI_VER_BCS_5_0
#define BLE_LMP_VER_BCS BLE_LMP_VER_BCS_5_0
#elif CONFIG_BLE_LL_BT_VERSION == 51
#define BLE_HCI_VER_BCS BLE_HCI_VER_BCS_5_1
#define BLE_LMP_VER_BCS BLE_LMP_VER_BCS_5_1
#elif CONFIG_BLE_LL_BT_VERSION == 52
#define BLE_HCI_VER_BCS BLE_HCI_VER_BCS_5_2
#define BLE_LMP_VER_BCS BLE_LMP_VER_BCS_5_2
#elif CONFIG_BLE_LL_BT_VERSION == 53
#define BLE_HCI_VER_BCS BLE_HCI_VER_BCS_5_3
#define BLE_LMP_VER_BCS BLE_LMP_VER_BCS_5_3
#elif CONFIG_BLE_LL_BT_VERSION == 54
#define BLE_HCI_VER_BCS BLE_HCI_VER_BCS_5_4
#define BLE_LMP_VER_BCS BLE_LMP_VER_BCS_5_4
#elif CONFIG_BLE_LL_BT_VERSION == 60
#define BLE_HCI_VER_BCS BLE_HCI_VER_BCS_6_0
#define BLE_LMP_VER_BCS BLE_LMP_VER_BCS_6_0
#elif CONFIG_BLE_LL_BT_VERSION == 61
#define BLE_HCI_VER_BCS BLE_HCI_VER_BCS_6_1
#define BLE_LMP_VER_BCS BLE_LMP_VER_BCS_6_1
#else
#error Unsupported BLE_VERSION selected
#endif

#define BLE_HCI_DATA_HDR_SZ               4
#define BLE_HCI_DATA_HANDLE(handle_pb_bc) (((handle_pb_bc) & 0x0fff) >> 0)
#define BLE_HCI_DATA_PB(handle_pb_bc)     (((handle_pb_bc) & 0x3000) >> 12)
#define BLE_HCI_DATA_BC(handle_pb_bc)     (((handle_pb_bc) & 0xc000) >> 14)
#define BLE_HCI_PB_FIRST_NON_FLUSH        0
#define BLE_HCI_PB_MIDDLE                 1
#define BLE_HCI_PB_FIRST_FLUSH            2
#define BLE_HCI_PB_FULL                   3
#define BLE_HCI_ISO_CONN_HANDLE_MASK      (0x07ff)
#define BLE_HCI_ISO_PB_FLAG_MASK          (0x3000)
#define BLE_HCI_ISO_TS_FLAG_MASK          (0x4000)
#define BLE_HCI_ISO_LENGTH_MASK           (0x3fff)
#define BLE_HCI_ISO_SDU_LENGTH_MASK       (0x0fff)
#define BLE_HCI_ISO_PKT_STATUS_FLAG_MASK  (0xc000)
#define BLE_HCI_ISO_HANDLE(ch, pb, ts) ((ch) | ((pb) << 12) | ((ts) << 14))
#define BLE_HCI_ISO_CONN_HANDLE(h)     ((h) & BLE_HCI_ISO_CONN_HANDLE_MASK)
#define BLE_HCI_ISO_PB_FLAG(h)    (((h) & BLE_HCI_ISO_PB_FLAG_MASK) >> 12)
#define BLE_HCI_ISO_TS_FLAG(h)    (((h) & BLE_HCI_ISO_TS_FLAG_MASK) >> 14)
#define BLE_HCI_ISO_LENGTH(l)     ((l) & BLE_HCI_ISO_LENGTH_MASK)
#define BLE_HCI_ISO_SDU_LENGTH(l) ((l) & BLE_HCI_ISO_SDU_LENGTH_MASK)
#define BLE_HCI_ISO_PKT_STATUS_FLAG(l) \
  (((l) & BLE_HCI_ISO_PKT_STATUS_FLAG_MASK) >> 14)

#define BLE_HCI_ISO_PB_FIRST                   (0)
#define BLE_HCI_ISO_PB_CONTINUATION            (1)
#define BLE_HCI_ISO_PB_COMPLETE                (2)
#define BLE_HCI_ISO_PB_LAST                    (3)
#define BLE_HCI_ISO_PKT_STATUS_VALID           0x00
#define BLE_HCI_ISO_PKT_STATUS_INVALID         0x01
#define BLE_HCI_ISO_PKT_STATUS_LOST            0x10
#define BLE_HCI_ISO_BIG_HANDLE_MIN             0x00
#define BLE_HCI_ISO_BIG_HANDLE_MAX             0xef
#define BLE_HCI_ISO_BIG_ENCRYPTION_UNENCRYPTED 0x00
#define BLE_HCI_ISO_BIG_ENCRYPTION_ENCRYPTED   0x01
#define BLE_HCI_ISO_DATA_PATH_DIR_INPUT        0x00
#define BLE_HCI_ISO_DATA_PATH_DIR_OUTPUT       0x01
#define BLE_HCI_ISO_DATA_PATH_ID_HCI           0x00
#define BLE_HCI_ISO_FRAMING_UNFRAMED           0x00
#define BLE_HCI_ISO_FRAMING_FRAMED_SEGMENTABLE 0x01
#define BLE_HCI_ISO_FRAMING_FRAMED_UNSEGMENTED 0x02
#define BLE_HCI_ISO_HDR_SDU_LENGTH_MASK        (0x07ff)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Generic command header */

begin_packed_struct struct ble_hci_cmd
{
  uint16_t opcode;
  uint8_t length;
  uint8_t data[0];
} end_packed_struct;

/* Generic event header */

begin_packed_struct struct ble_hci_ev
{
  uint8_t opcode;
  uint8_t length;
  uint8_t data[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_lc_disconnect_cp
{
  uint16_t conn_handle;
  uint8_t reason;
} end_packed_struct;

begin_packed_struct struct ble_hci_rd_rem_ver_info_cp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_cb_set_event_mask_cp
{
  uint64_t event_mask;
} end_packed_struct;

begin_packed_struct struct ble_hci_cb_read_tx_pwr_cp
{
  uint16_t conn_handle;
  uint8_t type;
} end_packed_struct;

begin_packed_struct struct ble_hci_cb_read_tx_pwr_rp
{
  uint16_t conn_handle;
  int8_t tx_level;
} end_packed_struct;

begin_packed_struct struct ble_hci_cb_ctlr_to_host_fc_cp
{
  uint8_t enable;
} end_packed_struct;

begin_packed_struct struct ble_hci_cb_host_buf_size_cp
{
  uint16_t acl_data_len;
  uint8_t sco_data_len;
  uint16_t acl_num;
  uint16_t sco_num;
} end_packed_struct;

begin_packed_struct struct ble_hci_cb_host_num_comp_pkts_entry
{
  uint16_t handle;
  uint16_t count;
} end_packed_struct;

begin_packed_struct struct ble_hci_cb_host_num_comp_pkts_cp
{
  uint8_t handles;
  struct ble_hci_cb_host_num_comp_pkts_entry h[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_cb_set_event_mask2_cp
{
  uint64_t event_mask2;
} end_packed_struct;

begin_packed_struct struct ble_hci_cb_rd_auth_pyld_tmo_cp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_cb_rd_auth_pyld_tmo_rp
{
  uint16_t conn_handle;
  uint16_t tmo;
} end_packed_struct;

begin_packed_struct struct ble_hci_cb_wr_auth_pyld_tmo_cp
{
  uint16_t conn_handle;
  uint16_t tmo;
} end_packed_struct;

begin_packed_struct struct ble_hci_cb_wr_auth_pyld_tmo_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_ip_rd_local_ver_rp
{
  uint8_t hci_ver;
  uint16_t hci_rev;
  uint8_t lmp_ver;
  uint16_t manufacturer;
  uint16_t lmp_subver;
} end_packed_struct;

begin_packed_struct struct ble_hci_ip_rd_loc_supp_cmd_rp
{
  uint8_t commands[64];
} end_packed_struct;

begin_packed_struct struct ble_hci_ip_rd_loc_supp_feat_rp
{
  uint64_t features;
} end_packed_struct;

begin_packed_struct struct ble_hci_ip_rd_buf_size_rp
{
  uint16_t acl_data_len;
  uint8_t sco_data_len;
  uint16_t acl_num;
  uint16_t sco_num;
} end_packed_struct;

begin_packed_struct struct ble_hci_ip_rd_bd_addr_rp
{
  uint8_t addr[6];
} end_packed_struct;

begin_packed_struct struct ble_hci_rd_rssi_cp
{
  uint16_t handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_rd_rssi_rp
{
  uint16_t handle;
  int8_t rssi;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_event_mask_cp
{
  uint64_t event_mask;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_buf_size_rp
{
  uint16_t data_len;
  uint8_t data_packets;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_buf_size_v2_rp
{
  uint16_t data_len;
  uint8_t data_packets;
  uint16_t iso_data_len;
  uint8_t iso_data_packets;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_loc_supp_feat_rp
{
  uint64_t features;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_rand_addr_cp
{
  uint8_t addr[6];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_adv_params_cp
{
  uint16_t min_interval;
  uint16_t max_interval;
  uint8_t type;
  uint8_t own_addr_type;
  uint8_t peer_addr_type;
  uint8_t peer_addr[6];
  uint8_t chan_map;
  uint8_t filter_policy;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_adv_chan_txpwr_rp
{
  int8_t power_level;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_adv_data_cp
{
  uint8_t adv_data_len;
  uint8_t adv_data[BLE_HCI_MAX_ADV_DATA_LEN];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_scan_rsp_data_cp
{
  uint8_t scan_rsp_len;
  uint8_t scan_rsp[BLE_HCI_MAX_SCAN_RSP_DATA_LEN];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_adv_enable_cp
{
  uint8_t enable;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_scan_params_cp
{
  uint8_t scan_type;
  uint16_t scan_itvl;
  uint16_t scan_window;
  uint8_t own_addr_type;
  uint8_t filter_policy;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_scan_enable_cp
{
  uint8_t enable;
  uint8_t filter_duplicates;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_create_conn_cp
{
  uint16_t scan_itvl;
  uint16_t scan_window;
  uint8_t filter_policy;
  uint8_t peer_addr_type;
  uint8_t peer_addr[6];
  uint8_t own_addr_type;
  uint16_t min_conn_itvl;
  uint16_t max_conn_itvl;
  uint16_t conn_latency;
  uint16_t tmo;
  uint16_t min_ce;
  uint16_t max_ce;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_white_list_rp
{
  uint8_t size;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_add_whte_list_cp
{
  uint8_t addr_type;
  uint8_t addr[6];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rmv_white_list_cp
{
  uint8_t addr_type;
  uint8_t addr[6];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_conn_update_cp
{
  uint16_t conn_handle;
  uint16_t conn_itvl_min;
  uint16_t conn_itvl_max;
  uint16_t conn_latency;
  uint16_t supervision_timeout;
  uint16_t min_ce_len;
  uint16_t max_ce_len;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_host_chan_class_cp
{
  uint8_t chan_map[5];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_chan_map_cp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_chan_map_rp
{
  uint16_t conn_handle;
  uint8_t chan_map[5];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_rem_feat_cp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_encrypt_cp
{
  uint8_t key[16];
  uint8_t data[16];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_encrypt_rp
{
  uint8_t data[16];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rand_rp
{
  uint64_t random_number;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_start_encrypt_cp
{
  uint16_t conn_handle;
  uint64_t rand;
  uint16_t div;
  uint8_t ltk[16];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_lt_key_req_reply_cp
{
  uint16_t conn_handle;
  uint8_t ltk[16];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_lt_key_req_reply_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_lt_key_req_neg_reply_cp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_lt_key_req_neg_reply_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_supp_states_rp
{
  uint64_t states;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rx_test_cp
{
  uint8_t rx_chan;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_tx_test_cp
{
  uint8_t tx_chan;
  uint8_t test_data_len;
  uint8_t payload;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_test_end_rp
{
  uint16_t num_packets;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rem_conn_param_rr_cp
{
  uint16_t conn_handle;
  uint16_t conn_itvl_min;
  uint16_t conn_itvl_max;
  uint16_t conn_latency;
  uint16_t supervision_timeout;
  uint16_t min_ce;
  uint16_t max_ce;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rem_conn_param_rr_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rem_conn_params_nrr_cp
{
  uint16_t conn_handle;
  uint8_t reason;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rem_conn_params_nrr_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_data_len_cp
{
  uint16_t conn_handle;
  uint16_t tx_octets;
  uint16_t tx_time;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_data_len_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_sugg_def_data_len_rp
{
  uint16_t max_tx_octets;
  uint16_t max_tx_time;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_wr_sugg_def_data_len_cp
{
  uint16_t max_tx_octets;
  uint16_t max_tx_time;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_gen_dhkey_cp
{
  uint8_t pkey[64];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_add_resolv_list_cp
{
  uint8_t peer_addr_type;
  uint8_t peer_id_addr[6];
  uint8_t peer_irk[16];
  uint8_t local_irk[16];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rmv_resolve_list_cp
{
  uint8_t peer_addr_type;
  uint8_t peer_id_addr[6];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_resolv_list_size_rp
{
  uint8_t size;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_peer_resolv_addr_cp
{
  uint8_t peer_addr_type;
  uint8_t peer_id_addr[6];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_peer_resolv_addr_rp
{
  uint8_t rpa[6];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_local_resolv_addr_cp
{
  uint8_t peer_addr_type;
  uint8_t peer_id_addr[6];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_local_resolv_addr_rp
{
  uint8_t rpa[6];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_addr_res_en_cp
{
  uint8_t enable;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_rpa_tmo_cp
{
  uint16_t rpa_timeout;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_max_data_len_rp
{
  uint16_t max_tx_octests;
  uint16_t max_tx_time;
  uint16_t max_rx_octests;
  uint16_t max_rx_time;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_phy_cp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_phy_rp
{
  uint16_t conn_handle;
  uint8_t tx_phy;
  uint8_t rx_phy;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_default_phy_cp
{
  uint8_t all_phys;
  uint8_t tx_phys;
  uint8_t rx_phys;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_phy_cp
{
  uint16_t conn_handle;
  uint8_t all_phys;
  uint8_t tx_phys;
  uint8_t rx_phys;
  uint16_t phy_options;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rx_test_v2_cp
{
  uint8_t rx_chan;
  uint8_t phy;
  uint8_t index;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_tx_test_v2_cp
{
  uint8_t tx_chan;
  uint8_t test_data_len;
  uint8_t payload;
  uint8_t phy;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_adv_set_rnd_addr_cp
{
  uint8_t adv_handle;
  uint8_t addr[6];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_ext_adv_params_cp
{
  uint8_t adv_handle;
  uint16_t props;
  uint8_t pri_itvl_min[3];
  uint8_t pri_itvl_max[3];
  uint8_t pri_chan_map;
  uint8_t own_addr_type;
  uint8_t peer_addr_type;
  uint8_t peer_addr[6];
  uint8_t filter_policy;
  int8_t tx_power;
  uint8_t pri_phy;
  uint8_t sec_max_skip;
  uint8_t sec_phy;
  uint8_t sid;
  uint8_t scan_req_notif;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_ext_adv_params_rp
{
  int8_t tx_power;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_ext_adv_data_cp
{
  uint8_t adv_handle;
  uint8_t operation;
  uint8_t fragment_pref;
  uint8_t adv_data_len;
  uint8_t adv_data[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_ext_scan_rsp_data_cp
{
  uint8_t adv_handle;
  uint8_t operation;
  uint8_t fragment_pref;
  uint8_t scan_rsp_len;
  uint8_t scan_rsp[0];
} end_packed_struct;

begin_packed_struct struct adv_set
{
  uint8_t adv_handle;
  uint16_t duration;
  uint8_t max_events;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_ext_adv_enable_cp
{
  uint8_t enable;
  uint8_t num_sets;
  struct adv_set sets[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_max_adv_data_len_rp
{
  uint16_t max_adv_data_len;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_num_of_adv_sets_rp
{
  uint8_t num_sets;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_remove_adv_set_cp
{
  uint8_t adv_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_periodic_adv_params_cp
{
  uint8_t adv_handle;
  uint16_t min_itvl;
  uint16_t max_itvl;
  uint16_t props;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_periodic_adv_data_cp
{
  uint8_t adv_handle;
  uint8_t operation;
  uint8_t adv_data_len;
  uint8_t adv_data[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_periodic_adv_enable_cp
{
  uint8_t enable;
  uint8_t adv_handle;
} end_packed_struct;

begin_packed_struct struct scan_params
{
  uint8_t type;
  uint16_t itvl;
  uint16_t window;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_ext_scan_params_cp
{
  uint8_t own_addr_type;
  uint8_t filter_policy;
  uint8_t phys;
  struct scan_params scans[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_ext_scan_enable_cp
{
  uint8_t enable;
  uint8_t filter_dup;
  uint16_t duration;
  uint16_t period;
} end_packed_struct;

begin_packed_struct struct conn_params
{
  uint16_t scan_itvl;
  uint16_t scan_window;
  uint16_t conn_min_itvl;
  uint16_t conn_max_itvl;
  uint16_t conn_latency;
  uint16_t supervision_timeout;
  uint16_t min_ce;
  uint16_t max_ce;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_ext_create_conn_cp
{
  uint8_t filter_policy;
  uint8_t own_addr_type;
  uint8_t peer_addr_type;
  uint8_t peer_addr[6];
  uint8_t init_phy_mask;
  struct conn_params conn_params[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_periodic_adv_create_sync_cp
{
  uint8_t options;
  uint8_t sid;
  uint8_t peer_addr_type;
  uint8_t peer_addr[6];
  uint16_t skip;
  uint16_t sync_timeout;
  uint8_t sync_cte_type;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_periodic_adv_term_sync_cp
{
  uint16_t sync_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_add_dev_to_periodic_adv_list_cp
{
  uint8_t peer_addr_type;
  uint8_t peer_addr[6];
  uint8_t sid;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rem_dev_from_periodic_adv_list_cp
{
  uint8_t peer_addr_type;
  uint8_t peer_addr[6];
  uint8_t sid;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_periodic_adv_list_size_rp
{
  uint8_t list_size;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_transmit_power_rp
{
  int8_t min_tx_power;
  int8_t max_tx_power;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_rd_rf_path_compensation_rp
{
  int16_t tx_path_compensation;
  int16_t rx_path_compensation;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_wr_rf_path_compensation_cp
{
  int16_t tx_path_compensation;
  int16_t rx_path_compensation;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_privacy_mode_cp
{
  uint8_t peer_id_addr_type;
  uint8_t peer_id_addr[6];
  uint8_t mode;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_periodic_adv_receive_enable_cp
{
  uint16_t sync_handle;
  uint8_t enable;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_periodic_adv_sync_transfer_cp
{
  uint16_t conn_handle;
  uint16_t service_data;
  uint16_t sync_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_periodic_adv_sync_transfer_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_periodic_adv_set_info_transfer_cp
{
  uint16_t conn_handle;
  uint16_t service_data;
  uint8_t adv_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_periodic_adv_set_info_transfer_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_periodic_adv_sync_transfer_params_cp
{
  uint16_t conn_handle;
  uint8_t mode;
  uint16_t skip;
  uint16_t sync_timeout;
  uint8_t sync_cte_type;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_periodic_adv_sync_transfer_params_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct
    ble_hci_le_set_default_periodic_sync_transfer_params_cp
{
  uint8_t mode;
  uint16_t skip;
  uint16_t sync_timeout;
  uint8_t sync_cte_type;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_read_iso_tx_sync_cp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_read_iso_tx_sync_rp
{
  uint16_t conn_handle;
  uint16_t packet_seq_num;
  uint32_t tx_timestamp;
  uint8_t time_offset[3];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cis_params
{
  uint8_t cis_id;
  uint16_t max_sdu_c_to_p;
  uint16_t max_sdu_p_to_c;
  uint8_t phy_c_to_p;
  uint8_t phy_p_to_c;
  uint8_t rnt_c_to_p;
  uint8_t rnt_p_to_c;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_cig_params_cp
{
  uint8_t cig_id;
  uint8_t sdu_interval_c_to_p[3];
  uint8_t sdu_interval_p_to_c[3];
  uint8_t worst_sca;
  uint8_t packing;
  uint8_t framing;
  uint16_t max_latency_c_to_p;
  uint16_t max_latency_p_to_c;
  uint8_t cis_count;
  struct ble_hci_le_cis_params cis[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_cig_params_rp
{
  uint8_t cig_id;
  uint8_t cis_count;
  uint16_t conn_handle[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cis_params_test
{
  uint8_t cis_id;
  uint8_t nse;
  uint16_t max_sdu_c_to_p;
  uint16_t max_sdu_p_to_c;
  uint16_t max_pdu_c_to_p;
  uint16_t max_pdu_p_to_c;
  uint8_t phy_c_to_p;
  uint8_t phy_p_to_c;
  uint8_t bn_c_to_p;
  uint8_t bn_p_to_c;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_cig_params_test_cp
{
  uint8_t cig_id;
  uint8_t sdu_interval_c_to_p[3];
  uint8_t sdu_interval_p_to_c[3];
  uint8_t ft_c_to_p;
  uint8_t ft_p_to_c;
  uint16_t iso_interval;
  uint8_t worst_sca;
  uint8_t packing;
  uint8_t framing;
  uint8_t cis_count;
  struct ble_hci_le_cis_params_test cis[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_cig_params_test_rp
{
  uint8_t cig_id;
  uint8_t cis_count;
  uint16_t conn_handle[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_create_cis_params
{
  uint16_t cis_handle;
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_create_cis_cp
{
  uint8_t cis_count;
  struct ble_hci_le_create_cis_params cis[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_remove_cig_cp
{
  uint8_t cig_id;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_remove_cig_rp
{
  uint8_t cig_id;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_accept_cis_request_cp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_reject_cis_request_cp
{
  uint16_t conn_handle;
  uint8_t reason;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_reject_cis_request_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_create_big_cp
{
  uint8_t big_handle;
  uint8_t adv_handle;
  uint8_t num_bis;
  uint8_t sdu_interval[3];
  uint16_t max_sdu;
  uint16_t max_transport_latency;
  uint8_t rtn;
  uint8_t phy;
  uint8_t packing;
  uint8_t framing;
  uint8_t encryption;
  uint8_t broadcast_code[16];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_create_big_test_cp
{
  uint8_t big_handle;
  uint8_t adv_handle;
  uint8_t num_bis;
  uint8_t sdu_interval[3];
  uint16_t iso_interval;
  uint8_t nse;
  uint16_t max_sdu;
  uint16_t max_pdu;
  uint8_t phy;
  uint8_t packing;
  uint8_t framing;
  uint8_t bn;
  uint8_t irc;
  uint8_t pto;
  uint8_t encryption;
  uint8_t broadcast_code[16];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_terminate_big_cp
{
  uint8_t big_handle;
  uint8_t reason;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_big_create_sync_cp
{
  uint8_t big_handle;
  uint16_t sync_handle;
  uint8_t encryption;
  uint8_t broadcast_code[16];
  uint8_t mse;
  uint16_t sync_timeout;
  uint8_t num_bis;
  uint8_t bis[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_big_terminate_sync_cp
{
  uint8_t big_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_big_terminate_sync_rp
{
  uint8_t big_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_request_peer_sca_cp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_setup_iso_data_path_cp
{
  uint16_t conn_handle;
  uint8_t data_path_dir;
  uint8_t data_path_id;
  uint8_t codec_id[5];
  uint8_t controller_delay[3];
  uint8_t codec_config_len;
  uint8_t codec_config[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_setup_iso_data_path_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_remove_iso_data_path_cp
{
  uint16_t conn_handle;
  uint8_t data_path_dir;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_remove_iso_data_path_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_iso_transmit_test_cp
{
  uint16_t conn_handle;
  uint8_t payload_type;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_iso_transmit_test_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_iso_receive_test_cp
{
  uint16_t conn_handle;
  uint8_t payload_type;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_iso_receive_test_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_iso_read_test_counters_cp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_iso_read_test_counters_rp
{
  uint16_t conn_handle;
  uint32_t received_sdu_count;
  uint32_t missed_sdu_count;
  uint32_t failed_sdu_count;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_iso_test_end_cp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_iso_test_end_rp
{
  uint16_t conn_handle;
  uint32_t received_sdu_count;
  uint32_t missed_sdu_count;
  uint32_t failed_sdu_count;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_host_feature_cp
{
  uint8_t bit_num;
  uint8_t bit_val;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_read_iso_link_quality_cp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_read_iso_link_quality_rp
{
  uint16_t conn_handle;
  uint32_t tx_unacked_pkts;
  uint32_t tx_flushed_pkts;
  uint32_t tx_last_subevent_pkts;
  uint32_t retransmitted_pkts;
  uint32_t crc_error_pkts;
  uint32_t rx_unreceived_pkts;
  uint32_t duplicate_pkts;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_enh_read_transmit_power_level_cp
{
  uint16_t conn_handle;
  uint8_t phy;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_enh_read_transmit_power_level_rp
{
  uint16_t conn_handle;
  uint8_t phy;
  uint8_t curr_tx_power_level;
  uint8_t max_tx_power_level;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_read_remote_transmit_power_level_cp
{
  uint16_t conn_handle;
  uint8_t phy;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_path_loss_report_param_cp
{
  uint16_t conn_handle;
  uint8_t high_threshold;
  uint8_t high_hysteresis;
  uint8_t low_threshold;
  uint8_t low_hysteresis;
  uint16_t min_time_spent;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_path_loss_report_enable_cp
{
  uint16_t conn_handle;
  uint8_t enable;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_transmit_power_report_enable_cp
{
  uint16_t conn_handle;
  uint8_t local_enable;
  uint8_t remote_enable;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_default_subrate_cp
{
  uint16_t subrate_min;
  uint16_t subrate_max;
  uint16_t max_latency;
  uint16_t cont_num;
  uint16_t supervision_tmo;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_subrate_req_cp
{
  uint16_t conn_handle;
  uint16_t subrate_min;
  uint16_t subrate_max;
  uint16_t max_latency;
  uint16_t cont_num;
  uint16_t supervision_tmo;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_set_ext_adv_params_v2_cp
{
  struct ble_hci_le_set_ext_adv_params_cp params_v1;
  uint8_t pri_phy_opt;
  uint8_t sec_phy_opt;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cs_rd_loc_supp_cap_rp
{
  uint8_t num_config_supported;
  uint16_t max_consecutive_procedures_supported;
  uint8_t num_antennas_supported;
  uint8_t max_antenna_paths_supported;
  uint8_t roles_supported;
  uint8_t optional_modes_supported;
  uint8_t rtt_capability;
  uint8_t rtt_aa_only_n;
  uint8_t rtt_sounding_n;
  uint8_t rtt_random_payload_n;
  uint16_t optional_nadm_sounding_capability;
  uint16_t optional_nadm_random_capability;
  uint8_t optional_cs_sync_phys_supported;
  uint16_t optional_subfeatures_supported;
  uint16_t optional_t_ip1_times_supported;
  uint16_t optional_t_ip2_times_supported;
  uint16_t optional_t_fcs_times_supported;
  uint16_t optional_t_pm_times_supported;
  uint8_t t_sw_time_supported;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cs_rd_rem_supp_cap_cp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cs_wr_cached_rem_supp_cap_cp
{
  uint16_t conn_handle;
  uint8_t num_config_supported;
  uint16_t max_consecutive_procedures_supported;
  uint8_t num_antennas_supported;
  uint8_t max_antenna_paths_supported;
  uint8_t roles_supported;
  uint8_t optional_modes_supported;
  uint8_t rtt_capability;
  uint8_t rtt_aa_only_n;
  uint8_t rtt_sounding_n;
  uint8_t rtt_random_payload_n;
  uint16_t optional_nadm_sounding_capability;
  uint16_t optional_nadm_random_capability;
  uint8_t optional_cs_sync_phys_supported;
  uint16_t optional_subfeatures_supported;
  uint16_t optional_t_ip1_times_supported;
  uint16_t optional_t_ip2_times_supported;
  uint16_t optional_t_fcs_times_supported;
  uint16_t optional_t_pm_times_supported;
  uint8_t t_sw_time_supported;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cs_wr_cached_rem_supp_cap_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cs_sec_enable_cp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cs_set_def_settings_cp
{
  uint16_t conn_handle;
  uint8_t role_enable;
  uint8_t cs_sync_antenna_selection;
  uint8_t max_tx_power;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cs_set_def_settings_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cs_rd_rem_fae_cp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cs_wr_cached_rem_fae_cp
{
  uint16_t conn_handle;
  uint8_t remote_fae_table[72];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cs_wr_cached_rem_fae_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cs_create_config_cp
{
  uint16_t conn_handle;
  uint8_t config_id;
  uint8_t create_context;
  uint8_t main_mode_type;
  uint8_t sub_mode_type;
  uint8_t min_main_mode_steps;
  uint8_t max_main_mode_steps;
  uint8_t main_mode_repetition;
  uint8_t mode_0_steps;
  uint8_t role;
  uint8_t rtt_type;
  uint8_t cs_sync_phy;
  uint8_t channel_map[10];
  uint8_t channel_map_repetition;
  uint8_t channel_selection_type;
  uint8_t ch3c_shape;
  uint8_t ch3c_jump;
  uint8_t companion_signal_enable;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cs_remove_config_cp
{
  uint16_t conn_handle;
  uint8_t config_id;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cs_set_chan_class_cp
{
  uint8_t channel_classification[10];
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cs_set_proc_params_cp
{
  uint16_t conn_handle;
  uint8_t config_id;
  uint16_t max_procedure_len;
  uint16_t min_procedure_interval;
  uint16_t max_procedure_interval;
  uint16_t max_procedure_count;
  uint8_t min_subevent_len[3];
  uint8_t max_subevent_len[3];
  uint8_t tone_antenna_config_selection;
  uint8_t phy;
  uint8_t tx_power_delta;
  uint8_t preferred_peer_antenna;
  uint8_t snr_control_initiator;
  uint8_t snr_control_reflector;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cs_set_proc_params_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cs_proc_enable_cp
{
  uint16_t conn_handle;
  uint8_t config_id;
  uint8_t enable;
} end_packed_struct;

begin_packed_struct struct ble_hci_le_cs_test_cp
{
  uint8_t main_mode_type;
  uint8_t sub_mode_type;
  uint8_t main_mode_repetition;
  uint8_t mode_0_steps;
  uint8_t role;
  uint8_t rtt_type;
  uint8_t cs_sync_phy;
  uint8_t cs_sync_antenna_selection;
  uint8_t subevent_len[3];
  uint16_t subevent_interval;
  uint8_t transmit_power_level;
  uint8_t t_ip1_time;
  uint8_t t_ip2_time;
  uint8_t t_fcs_time;
  uint8_t t_pm_time;
  uint8_t t_sw_time;
  uint8_t tone_antenna_config_selection;
  uint8_t companion_signal_enable;
  uint16_t drbg_nonce;
  uint16_t override_config;
  uint8_t override_parameters_length;
  uint8_t override_parameters_data[];
} end_packed_struct;

begin_packed_struct struct ble_hci_vs_rd_static_addr_rp
{
  uint8_t addr[6];
} end_packed_struct;

begin_packed_struct struct ble_hci_vs_set_tx_pwr_cp
{
  int8_t tx_power;
} end_packed_struct;

begin_packed_struct struct ble_hci_vs_set_tx_pwr_rp
{
  int8_t tx_power;
} end_packed_struct;

begin_packed_struct struct ble_hci_vs_css_configure_cp
{
  uint32_t slot_us;
  uint32_t period_slots;
} end_packed_struct;

begin_packed_struct struct ble_hci_vs_css_enable_cp
{
  uint8_t enable;
} end_packed_struct;

begin_packed_struct struct ble_hci_vs_css_set_next_slot_cp
{
  uint16_t slot_idx;
} end_packed_struct;

begin_packed_struct struct ble_hci_vs_css_set_conn_slot_cp
{
  uint16_t conn_handle;
  uint16_t slot_idx;
} end_packed_struct;

begin_packed_struct struct ble_hci_vs_css_read_conn_slot_cp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_vs_css_read_conn_slot_rp
{
  uint16_t conn_handle;
  uint16_t slot_idx;
} end_packed_struct;

begin_packed_struct struct ble_hci_vs_set_data_len_cp
{
  uint16_t conn_handle;
  uint16_t tx_octets;
  uint16_t tx_time;
  uint16_t rx_octets;
  uint16_t rx_time;
} end_packed_struct;

begin_packed_struct struct ble_hci_vs_set_data_len_rp
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_vs_set_antenna_cp
{
  uint8_t antenna;
} end_packed_struct;

begin_packed_struct struct ble_hci_vs_set_local_irk_cp
{
  uint8_t own_addr_type;
  uint8_t irk[16];
} end_packed_struct;

begin_packed_struct struct ble_hci_vs_set_scan_cfg_cp
{
  uint32_t flags;
  int8_t rssi_threshold;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_disconn_cmp
{
  uint8_t status;
  uint16_t conn_handle;
  uint8_t reason;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_enrypt_chg
{
  uint8_t status;
  uint16_t connection_handle;
  uint8_t enabled;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_rd_rem_ver_info_cmp
{
  uint8_t status;
  uint16_t conn_handle;
  uint8_t version;
  uint16_t manufacturer;
  uint16_t subversion;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_command_complete
{
  uint8_t num_packets;
  uint16_t opcode;
  uint8_t status;
  uint8_t return_params[0];
} end_packed_struct;

/* NOP is exception and has no return parameters */

begin_packed_struct struct ble_hci_ev_command_complete_nop
{
  uint8_t num_packets;
  uint16_t opcode;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_command_status
{
  uint8_t status;
  uint8_t num_packets;
  uint16_t opcode;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_hw_error
{
  uint8_t hw_code;
} end_packed_struct;

begin_packed_struct struct comp_pkt
{
  uint16_t handle;
  uint16_t packets;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_num_comp_pkts
{
  uint8_t count;
  struct comp_pkt completed[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_data_buf_overflow
{
  uint8_t link_type;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_enc_key_refresh
{
  uint8_t status;
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_meta
{
  uint8_t subevent;
  uint8_t data[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_auth_pyld_tmo
{
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_vs
{
  uint8_t id;
  uint8_t data[0];
} end_packed_struct;

struct ble_hci_ev_vs_css_slot_changed
{
  uint16_t conn_handle;
  uint16_t slot_idx;
};

begin_packed_struct struct feedback_pkt
{
  uint16_t handle;
  uint8_t sdu_per_interval;
  int8_t diff;
} end_packed_struct;

begin_packed_struct struct ble_hci_vs_subev_iso_hci_feedback
{
  uint8_t big_handle;
  uint8_t count;
  struct feedback_pkt feedback[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_conn_complete
{
  uint8_t subev_code;
  uint8_t status;
  uint16_t conn_handle;
  uint8_t role;
  uint8_t peer_addr_type;
  uint8_t peer_addr[6];
  uint16_t conn_itvl;
  uint16_t conn_latency;
  uint16_t supervision_timeout;
  uint8_t mca;
} end_packed_struct;

begin_packed_struct struct adv_report
{
  uint8_t type;
  uint8_t addr_type;
  uint8_t addr[6];
  uint8_t data_len;
  uint8_t data[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_adv_rpt
{
  uint8_t subev_code;
  uint8_t num_reports;
  struct adv_report reports[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_conn_upd_complete
{
  uint8_t subev_code;
  uint8_t status;
  uint16_t conn_handle;
  uint16_t conn_itvl;
  uint16_t conn_latency;
  uint16_t supervision_timeout;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_rd_rem_used_feat
{
  uint8_t subev_code;
  uint8_t status;
  uint16_t conn_handle;
  uint8_t features[8];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_lt_key_req
{
  uint8_t subev_code;
  uint16_t conn_handle;
  uint64_t rand;
  uint16_t div;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_rem_conn_param_req
{
  uint8_t subev_code;
  uint16_t conn_handle;
  uint16_t min_interval;
  uint16_t max_interval;
  uint16_t latency;
  uint16_t timeout;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_data_len_chg
{
  uint8_t subev_code;
  uint16_t conn_handle;
  uint16_t max_tx_octets;
  uint16_t max_tx_time;
  uint16_t max_rx_octets;
  uint16_t max_rx_time;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_rd_loc_p256_pubkey
{
  uint8_t subev_code;
  uint8_t status;
  uint8_t public_key[64];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_gen_dhkey_complete
{
  uint8_t subev_code;
  uint8_t status;
  uint8_t dh_key[32];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_enh_conn_complete
{
  uint8_t subev_code;
  uint8_t status;
  uint16_t conn_handle;
  uint8_t role;
  uint8_t peer_addr_type;
  uint8_t peer_addr[6];
  uint8_t local_rpa[6];
  uint8_t peer_rpa[6];
  uint16_t conn_itvl;
  uint16_t conn_latency;
  uint16_t supervision_timeout;
  uint8_t mca;
} end_packed_struct;

begin_packed_struct struct dir_adv_report
{
  uint8_t type;
  uint8_t addr_type;
  uint8_t addr[6];
  uint8_t dir_addr_type;
  uint8_t dir_addr[6];
  int8_t rssi;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_direct_adv_rpt
{
  uint8_t subev_code;
  uint8_t num_reports;
  struct dir_adv_report reports[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_phy_update_complete
{
  uint8_t subev_code;
  uint8_t status;
  uint16_t conn_handle;
  uint8_t tx_phy;
  uint8_t rx_phy;
} end_packed_struct;

begin_packed_struct struct ext_adv_report
{
  uint16_t evt_type;
  uint8_t addr_type;
  uint8_t addr[6];
  uint8_t pri_phy;
  uint8_t sec_phy;
  uint8_t sid;
  int8_t tx_power;
  int8_t rssi;
  uint16_t periodic_itvl;
  uint8_t dir_addr_type;
  uint8_t dir_addr[6];
  uint8_t data_len;
  uint8_t data[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_ext_adv_rpt
{
  uint8_t subev_code;
  uint8_t num_reports;
  struct ext_adv_report reports[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_periodic_adv_sync_estab
{
  uint8_t subev_code;
  uint8_t status;
  uint16_t sync_handle;
  uint8_t sid;
  uint8_t peer_addr_type;
  uint8_t peer_addr[6];
  uint8_t phy;
  uint16_t interval;
  uint8_t aca;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_periodic_adv_rpt
{
  uint8_t subev_code;
  uint16_t sync_handle;
  int8_t tx_power;
  int8_t rssi;
  uint8_t cte_type;
  uint8_t data_status;
  uint8_t data_len;
  uint8_t data[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_periodic_adv_sync_lost
{
  uint8_t subev_code;
  uint16_t sync_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_scan_timeout
{
  uint8_t subev_code;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_adv_set_terminated
{
  uint8_t subev_code;
  uint8_t status;
  uint8_t adv_handle;
  uint16_t conn_handle;
  uint8_t num_events;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_scan_req_rcvd
{
  uint8_t subev_code;
  uint8_t adv_handle;
  uint8_t peer_addr_type;
  uint8_t peer_addr[6];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_chan_sel_alg
{
  uint8_t subev_code;
  uint16_t conn_handle;
  uint8_t csa;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_periodic_adv_sync_transfer
{
  uint8_t subev_code;
  uint8_t status;
  uint16_t conn_handle;
  uint16_t service_data;
  uint16_t sync_handle;
  uint8_t sid;
  uint8_t peer_addr_type;
  uint8_t peer_addr[6];
  uint8_t phy;
  uint16_t interval;
  uint8_t aca;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_cis_established
{
  uint8_t subev_code;
  uint8_t status;
  uint16_t conn_handle;
  uint8_t cig_sync_delay[3];
  uint8_t cis_sync_delay[3];
  uint8_t transport_latency_c_to_p[3];
  uint8_t transport_latency_p_to_c[3];
  uint8_t phy_c_to_p;
  uint8_t phy_p_to_c;
  uint8_t nse;
  uint8_t bn_c_to_p;
  uint8_t bn_p_to_c;
  uint8_t ft_c_to_p;
  uint8_t ft_p_to_c;
  uint16_t max_pdu_c_to_p;
  uint16_t max_pdu_p_to_c;
  uint16_t iso_interval;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_cis_request
{
  uint8_t subev_code;
  uint16_t acl_conn_handle;
  uint16_t cis_conn_handle;
  uint8_t cig_id;
  uint8_t cis_id;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_create_big_complete
{
  uint8_t subev_code;
  uint8_t status;
  uint8_t big_handle;
  uint8_t big_sync_delay[3];
  uint8_t transport_latency_big[3];
  uint8_t phy;
  uint8_t nse;
  uint8_t bn;
  uint8_t pto;
  uint8_t irc;
  uint16_t max_pdu;
  uint16_t iso_interval;
  uint8_t num_bis;
  uint16_t conn_handle[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_terminate_big_complete
{
  uint8_t subev_code;
  uint8_t big_handle;
  uint8_t reason;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_big_sync_established
{
  uint8_t subev_code;
  uint8_t status;
  uint8_t big_handle;
  uint8_t transport_latency_big[3];
  uint8_t nse;
  uint8_t bn;
  uint8_t pto;
  uint8_t irc;
  uint16_t max_pdu;
  uint16_t iso_interval;
  uint8_t num_bis;
  uint16_t conn_handle[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_big_sync_lost
{
  uint8_t subev_code;
  uint8_t big_handle;
  uint8_t reason;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_peer_sca_complete
{
  uint8_t subev_code;
  uint8_t status;
  uint16_t conn_handle;
  uint8_t sca;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_path_loss_threshold
{
  uint8_t subev_code;
  uint16_t conn_handle;
  uint8_t current_path_loss;
  uint8_t zone_entered;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_transmit_power_report
{
  uint8_t subev_code;
  uint8_t status;
  uint16_t conn_handle;
  uint8_t reason;
  uint8_t phy;
  int8_t transmit_power_level;
  uint8_t transmit_power_level_flag;
  int8_t delta;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_biginfo_adv_report
{
  uint8_t subev_code;
  uint16_t sync_handle;
  uint8_t bis_cnt;
  uint8_t nse;
  uint16_t iso_interval;
  uint8_t bn;
  uint8_t pto;
  uint8_t irc;
  uint16_t max_pdu;
  uint8_t sdu_interval[3];
  uint16_t max_sdu;
  uint8_t phy;
  uint8_t framing;
  uint8_t encryption;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_subrate_change
{
  uint8_t subev_code;
  uint8_t status;
  uint16_t conn_handle;
  uint16_t subrate_factor;
  uint16_t periph_latency;
  uint16_t cont_num;
  uint16_t supervision_tmo;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_cs_rd_rem_supp_cap_complete
{
  uint8_t subev_code;
  uint8_t status;
  uint16_t conn_handle;
  uint8_t num_config_supported;
  uint16_t max_consecutive_procedures_supported;
  uint8_t num_antennas_supported;
  uint8_t max_antenna_paths_supported;
  uint8_t roles_supported;
  uint8_t optional_modes_supported;
  uint8_t rtt_capability;
  uint8_t rtt_aa_only_n;
  uint8_t rtt_sounding_n;
  uint8_t rtt_random_payload_n;
  uint16_t optional_nadm_sounding_capability;
  uint16_t optional_nadm_random_capability;
  uint8_t optional_cs_sync_phys_supported;
  uint16_t optional_subfeatures_supported;
  uint16_t optional_t_ip1_times_supported;
  uint16_t optional_t_ip2_times_supported;
  uint16_t optional_t_fcs_times_supported;
  uint16_t optional_t_pm_times_supported;
  uint8_t t_sw_time_supported;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_cs_rd_rem_fae_complete
{
  uint8_t subev_code;
  uint8_t status;
  uint16_t conn_handle;
  uint8_t remote_fae_table[72];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_cs_sec_enable_complete
{
  uint8_t subev_code;
  uint8_t status;
  uint16_t conn_handle;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_cs_config_complete
{
  uint8_t subev_code;
  uint8_t status;
  uint16_t conn_handle;
  uint8_t config_id;
  uint8_t action;
  uint8_t main_mode_type;
  uint8_t sub_mode_type;
  uint8_t min_main_mode_steps;
  uint8_t max_main_mode_steps;
  uint8_t main_mode_repetition;
  uint8_t mode_0_steps;
  uint8_t role;
  uint8_t rtt_type;
  uint8_t cs_sync_phy;
  uint8_t channel_map[10];
  uint8_t channel_map_repetition;
  uint8_t channel_selection_type;
  uint8_t ch3c_shape;
  uint8_t ch3c_jump;
  uint8_t companion_signal_enable;
  uint8_t t_ip1_time;
  uint8_t t_ip2_time;
  uint8_t t_fcs_time;
  uint8_t t_pm_time;
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_cs_proc_enable_complete
{
  uint8_t subev_code;
  uint8_t status;
  uint16_t conn_handle;
  uint8_t config_id;
  uint8_t state;
  uint8_t tone_antenna_config_selection;
  uint8_t selected_tx_power;
  uint8_t subevent_len[3];
  uint8_t subevents_per_event;
  uint16_t subevent_interval;
  uint16_t event_interval;
  uint16_t procedure_interval;
  uint16_t procedure_count;
} end_packed_struct;

begin_packed_struct struct cs_steps_data
{
  uint8_t mode;
  uint8_t channel;
  uint8_t data_len;
  uint8_t data[];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_cs_subevent_result
{
  uint8_t subev_code;
  uint16_t conn_handle;
  uint8_t config_id;
  uint16_t start_acl_conn_event_counter;
  uint16_t procedure_counter;
  uint16_t frequency_compensation;
  uint8_t reference_power_level;
  uint8_t procedure_done_status;
  uint8_t subevent_done_status;
  uint8_t abort_reason;
  uint8_t num_antenna_paths;
  uint8_t num_steps_reported;
  struct cs_steps_data steps[];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_cs_subevent_result_continue
{
  uint8_t subev_code;
  uint16_t conn_handle;
  uint8_t config_id;
  uint8_t procedure_done_status;
  uint8_t subevent_done_status;
  uint8_t abort_reason;
  uint8_t num_antenna_paths;
  uint8_t num_steps_reported;
  struct cs_steps_data steps[];
} end_packed_struct;

begin_packed_struct struct ble_hci_ev_le_subev_cs_test_end_complete
{
  uint8_t subev_code;
  uint8_t status;
} end_packed_struct;

struct hci_data_hdr
{
  uint16_t hdh_handle_pb_bc;
  uint16_t hdh_len;
};

begin_packed_struct struct ble_hci_iso
{
  uint16_t handle;
  uint16_t length;
  uint8_t data[0];
} end_packed_struct;

begin_packed_struct struct ble_hci_iso_data
{
  uint16_t packet_seq_num;
  uint16_t sdu_len;
  uint8_t data[0];
} end_packed_struct;

/****************************************************************************
 * Public Data
 ****************************************************************************/

;

#endif /* __WIRELESS_BLE_LL_HCI_COMMON_H */
