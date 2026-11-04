/****************************************************************************
 * wireless/ble_ll/ble_ll_hci_supp_cmd.c
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
 * Derived from Apache Mynewt NimBLE
 * (nimble/controller/src/ble_ll_hci_supp_cmd.c).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <string.h>
#include "ble_ll.h"
#include "ble_ll_hci.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Magic macros */

#define BIT(n)   (1 << (n)) |
#define OCTET(x) (0 | x 0)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t octet_0 = OCTET(
#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) || \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)
    BIT(5) /* HCI Disconnect */
#endif
);

static const uint8_t octet_2 = OCTET(
#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) || \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)
    BIT(7) /* HCI Read Remote Version Information */
#endif
);

static const uint8_t octet_5 = OCTET(BIT(6) /* HCI Set Event Mask */
                                     BIT(7) /* HCI Reset */
);

static const uint8_t octet_10 = OCTET(
#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) || \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)
    BIT(2) /* HCI Read Transmit Power Level */
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_CTRL_TO_HOST_FLOW_CONTROL
    BIT(5) /* HCI Set Controller To Host Flow Control */
    BIT(6) /* HCI Host Buffer Size */
    BIT(7) /* HCI Host Number Of Completed Packets */
#endif
);

static const uint8_t octet_14 =
    OCTET(BIT(3) /* HCI Read Local Version Information */
          BIT(5) /* HCI Read Local Supported Features */
    );

static const uint8_t octet_15 = OCTET(BIT(1) /* HCI Read BD ADDR */
#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) || \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)
                                      BIT(5) /* HCI Read RSSI */
#endif
);

static const uint8_t octet_22 = OCTET(BIT(2) /* HCI Set Event Mask Page 2 */
);

static const uint8_t octet_25 =
    OCTET(BIT(0) /* HCI LE Set Event Mask */
          BIT(1) /* HCI LE Read Buffer Size [v1] */
          BIT(2) /* HCI LE Read Local Supported Features */
          BIT(4) /* HCI LE Set Random Address */
#ifdef CONFIG_BLE_LL_ROLE_BROADCASTER
          BIT(5) /* HCI LE Set Advertising Parameters */
          BIT(6) /* HCI LE Read Advertising Physical Channel Tx Power */
          BIT(7) /* HCI LE Set Advertising Data */
#endif
    );

static const uint8_t octet_26 = OCTET(
#ifdef CONFIG_BLE_LL_ROLE_BROADCASTER
    BIT(0) /* HCI LE Set Scan Response Data */
    BIT(1) /* HCI LE Set Advertising Enable */
#endif
#ifdef CONFIG_BLE_LL_ROLE_OBSERVER
    BIT(2) /* HCI LE Set Scan Parameters */
    BIT(3) /* HCI LE Set Scan Enable */
#endif
#ifdef CONFIG_BLE_LL_ROLE_CENTRAL
    BIT(4) /* HCI LE Create Connection */
    BIT(5) /* HCI LE Create Connection Cancel */
#endif
    BIT(6) /* HCI LE Read Filter Accept List Size */
    BIT(7) /* HCI LE Clear Filter Accept List */
);

static const uint8_t octet_27 =
    OCTET(BIT(0) /* HCI LE Add Device To Filter Accept List */
          BIT(1) /* HCI LE Remove Device From Filter Accept List */
#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) || \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)
          BIT(2) /* HCI LE Connection Update */
#endif
          BIT(3) /* HCI LE Set Host Channel Classification */
#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) || \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)
          BIT(4) /* HCI LE Read Channel Map */
          BIT(5) /* HCI LE Read Remote Features */
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
          BIT(6) /* HCI LE Encrypt */
#endif
          BIT(7) /* HCI LE Rand */
    );

static const uint8_t octet_28 = OCTET(
#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
#ifdef CONFIG_BLE_LL_ROLE_CENTRAL
    BIT(0) /* HCI LE Enable Encryption */
#endif
#ifdef CONFIG_BLE_LL_ROLE_PERIPHERAL
    BIT(1) /* HCI LE Long Term Key Request Reply */
    BIT(2) /* HCI LE Long Term Key Request Negative Reply */
#endif
#endif
    BIT(3) /* HCI LE Read Supported States */
);

static const uint8_t octet_32 = OCTET(
#if (defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) || \
     defined(CONFIG_BLE_LL_ROLE_CENTRAL)) &&   \
    defined(CONFIG_BLE_LL_CFG_FEAT_LE_PING)
    BIT(4) /* HCI Read Authenticated Payload Timeout */
    BIT(5) /* HCI Write Authenticated Payload Timeout */
#endif
);

static const uint8_t octet_33 = OCTET(
#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) || \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)
    BIT(4) /* HCI LE Remote Connection Parameter Request Reply */
    BIT(5) /* HCI LE Remote Connection Parameter Request Negative Reply */
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_DATA_LEN_EXT
    BIT(6) /* HCI LE Set Data Length */
    BIT(7) /* HCI LE Read Suggested Default Data Length */
#endif
);

static const uint8_t octet_34 = OCTET(
#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) || \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)
#ifdef CONFIG_BLE_LL_CFG_FEAT_DATA_LEN_EXT
    BIT(0) /* HCI LE Write Suggested Data Length */
#endif
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
    BIT(3) /* HCI LE Add Device To Resolving List */
    BIT(4) /* HCI LE Remove Device From Resolving List */
    BIT(5) /* HCI LE Clear Resolving List */
    BIT(6) /* HCI LE Read Resolving List Size */
    BIT(7) /* HCI LE Read Peer Resolvable Address */
#endif
);

static const uint8_t octet_35 = OCTET(
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
    BIT(0) /* HCI LE Read Local Resolvable Address */
    BIT(1) /* HCI LE Set Address Resolution Enable */
    BIT(2) /* HCI LE Set Resolvable Private Address Timeout */
#endif
    BIT(3) /* HCI LE Read Maximum Data Length */
#ifdef CONFIG_BLE_LL_PHY
#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) || \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)
    BIT(4) /* HCI LE Read PHY */
    BIT(5) /* HCI LE Set Default PHY */
    BIT(6) /* HCI LE Set PHY */
#endif
#endif
);

static const uint8_t octet_36 = OCTET(
#if defined(CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV) && \
    defined(CONFIG_BLE_LL_ROLE_BROADCASTER)
    BIT(1) /* HCI LE Set Advertising Set Random Address */
    BIT(2) /* HCI LE Set Extended Advertising Parameters */
    BIT(3) /* HCI LE Set Extended Advertising Data */
    BIT(4) /* HCI LE Set Extended Scan Response Data */
    BIT(5) /* HCI LE Set Extended Advertising Enable */
    BIT(6) /* HCI LE Read Maximum Advertising Data Length */
    BIT(7) /* HCI LE Read Number of Supported Advertising Sets */
#endif
);

static const uint8_t octet_37 = OCTET(
#if defined(CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV) && \
    defined(CONFIG_BLE_LL_ROLE_BROADCASTER)
    BIT(0) /* HCI LE Remove Advertising Set */
    BIT(1) /* HCI LE Clear Advertising Sets */
#endif
#if defined(CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV) && \
    defined(CONFIG_BLE_LL_ROLE_BROADCASTER)
    BIT(2) /* HCI LE Set Periodic Advertising Parameters */
    BIT(3) /* HCI LE Set Periodic Advertising Data */
    BIT(4) /* HCI LE Set Periodic Advertising Enable */
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV
#ifdef CONFIG_BLE_LL_ROLE_OBSERVER
    BIT(5) /* HCI LE Set Extended Scan Parameters */
    BIT(6) /* HCI LE Set Extended Scan Enable */
#endif
#ifdef CONFIG_BLE_LL_ROLE_CENTRAL
    BIT(7) /* HCI LE Extended Create Connection */
#endif
#endif
);

static const uint8_t octet_38 = OCTET(
#if defined(CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV) && \
    defined(CONFIG_BLE_LL_ROLE_OBSERVER)
    BIT(0) /* HCI LE Periodic Advertising Create Sync */
    BIT(1) /* HCI LE Periodic Advertising Create Sync Cancel */
    BIT(2) /* HCI LE Periodic Advertising Terminate Sync */
    BIT(3) /* HCI LE Add Device To Periodic Advertiser List */
    BIT(4) /* HCI LE Remove Device From Periodic Advertiser List */
    BIT(5) /* HCI LE Clear Periodic Advertiser List */
    BIT(6) /* HCI LE Read Periodic Advertiser List Size */
#endif
    BIT(7) /* HCI LE Read Transmit Power */
);

static const uint8_t octet_39 =
    OCTET(BIT(0) /* HCI LE Read RF Path Compensation */
          BIT(1) /* HCI LE Write RF Path Compensation */
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
          BIT(2) /* HCI LE Set Privacy Mode */
#endif
    );

static const uint8_t octet_40 = OCTET(
#if CONFIG_BLE_LL_BT_VERSION >= 51 && \
    defined(CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV)
#ifdef CONFIG_BLE_LL_ROLE_OBSERVER
    BIT(5) /* HCI LE Set Periodic Advertising Receive Enable */
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV_SYNC_TRANSFER
#ifdef CONFIG_BLE_LL_ROLE_OBSERVER
    BIT(6) /* HCI LE Periodic Advertising Sync Transfer */
#endif
#ifdef CONFIG_BLE_LL_ROLE_BROADCASTER
    BIT(7) /* HCI LE Periodic Advertising Set Info Transfer */
#endif
#endif
#endif
);

static const uint8_t octet_41 = OCTET(
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV_SYNC_TRANSFER
    BIT(0) /* HCI LE Set Periodic Advertising Sync Transfer Parameters */
    BIT(1) /* HCI LE Set Default Periodic Advertising Sync Transfer Params */
#endif
);

static const uint8_t octet_42 = OCTET();

static const uint8_t octet_43 = OCTET(
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_SCA_UPDATE
    BIT(2) /* HCI LE Request Peer SCA */
#endif
);

static const uint8_t octet_44 = OCTET(
#if BLE_LL_HOST_CONTROLLED_FEATURES
    BIT(1) /* HCI LE Set Host Feature */
#endif
);

static const uint8_t octet_46 = OCTET(
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_ENHANCED_CONN_UPDATE
    BIT(0) /* HCI LE Set Default Subrate */
    BIT(1) /* HCI LE Subrate Request */
#endif
#if CONFIG_BLE_LL_BT_VERSION >= 54 && \
    defined(CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV)
    BIT(2) /* HCI LE Set Extended Advertising Parameters [v2] */
#endif
);

static const uint8_t g_ble_ll_hci_supp_cmds[64] =
{
  octet_0,  0,        octet_2,  0,        0,        octet_5,  0,
  0,        0,        0,        octet_10, 0,        0,        0,
  octet_14, octet_15, 0,        0,        0,        0,        0,
  0,        octet_22, 0,        0,        octet_25, octet_26, octet_27,
  octet_28, 0,        0,        0,        octet_32, octet_33, octet_34,
  octet_35, octet_36, octet_37, octet_38, octet_39, octet_40, octet_41,
  octet_42, octet_43, octet_44, 0,        octet_46,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_ll_hci_supp_cmd_get
 *
 ****************************************************************************/

void ble_ll_hci_supp_cmd_get(uint8_t *buf)
{
  memcpy(buf, g_ble_ll_hci_supp_cmds, sizeof(g_ble_ll_hci_supp_cmds));
}
