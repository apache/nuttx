/****************************************************************************
 * wireless/ble_ll/ble.h
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
 * Derived from Apache Mynewt NimBLE (nimble/include/nimble/ble.h).
 *
 ****************************************************************************/

#ifndef __WIRELESS_BLE_LL_BLE_H
#define __WIRELESS_BLE_LL_BLE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <inttypes.h>
#include <string.h>

#include <nuttx/wireless/bluetooth/ble_ll.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The number of advertising instances */

#define BLE_ADV_INSTANCES (CONFIG_BLE_LL_MULTI_ADV_INSTANCES + 1)

/* 4 byte header + 251 byte payload. */

#define BLE_ACL_MAX_PKT_SIZE 255

/* Length of host user header.  Only contains the peer's connection handle. */

#define BLE_MBUF_HS_HDR_LEN (2)

/* Length of a Bluetooth Device Address. */

#define BLE_DEV_ADDR_LEN (6)

/* HW error codes */

#define BLE_HW_ERR_DO_NOT_USE    (0) /* XXX: reserve this one for now */
#define BLE_HW_ERR_HCI_SYNC_LOSS (1)

/* Bluetooth Device Own Address Type: Public. */

#define BLE_OWN_ADDR_PUBLIC (0x00)

/* Bluetooth Device Own Address Type: Random. */

#define BLE_OWN_ADDR_RANDOM (0x01)

/* Bluetooth Device Own Address Type: Resolvable Private Address (RPA)
 * with Public fallback.
 */

#define BLE_OWN_ADDR_RPA_PUBLIC_DEFAULT (0x02)

/* Bluetooth Device Own Address Type: Resolvable Private Address (RPA)
 * with Random fallback.
 */

#define BLE_OWN_ADDR_RPA_RANDOM_DEFAULT (0x03)

/* Bluetooth Device Address Type: Public. */

#define BLE_ADDR_PUBLIC (0x00)

/* Bluetooth Device Address Type: Random. */

#define BLE_ADDR_RANDOM (0x01)

/* Bluetooth Device Address Type: Public Identity Address
 * (Corresponds to Resolved Private Address).
 */

#define BLE_ADDR_PUBLIC_ID (0x02)

/* Bluetooth Device Address Type: Random (Static) Identity Address
 * (Corresponds to Resolved Private Address).
 */

#define BLE_ADDR_RANDOM_ID (0x03)

/* Bluetooth Device Address Type: Anonymous
 * (Corresponds to devices sending anonymous advertisements).
 */

#define BLE_ADDR_ANONYMOUS (0xff)

/* @brief A macro representing any BLE address.
 *
 * This macro initializes a `ble_addr_t` structure to all zeros.
 */

#define BLE_ADDR_ANY (&(ble_addr_t){ 0, { 0, 0, 0, 0, 0, 0 } })

/* Checks if the given address is a Resolvable Private Address (RPA).
 *
 * @param addr              Pointer to the `ble_addr_t` structure.
 * @return                  Non-zero if the address is an RPA,
 *                          zero otherwise.
 */

#define BLE_ADDR_IS_RPA(addr) \
  (((addr)->type == BLE_ADDR_RANDOM) && ((addr)->val[5] & 0xc0) == 0x40)

/* Checks if the given address is a Non-Resolvable Private Address (NRPA).
 *
 * @param addr              Pointer to the `ble_addr_t` structure.
 * @return                  Non-zero if the address is an NRPA,
 *                          zero otherwise.
 */

#define BLE_ADDR_IS_NRPA(addr) \
  (((addr)->type == BLE_ADDR_RANDOM) && ((addr)->val[5] & 0xc0) == 0x00)

/* @brief Checks if the given address is a Static Random Address.
 *
 * @param addr              Pointer to the `ble_addr_t` structure.
 * @return                  Non-zero if the address is a Static Random
 * Address, zero otherwise.
 */

#define BLE_ADDR_IS_STATIC(addr) \
  (((addr)->type == BLE_ADDR_RANDOM) && ((addr)->val[5] & 0xc0) == 0xc0)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* BLE Error Codes (Core v4.2 Vol 2 part D) */

enum ble_error_codes
{
  /* An "error" code of 0x0 means success */

  BLE_ERR_SUCCESS = 0x00,
  BLE_ERR_UNKNOWN_HCI_CMD = 0x01,
  BLE_ERR_UNK_CONN_ID = 0x02,
  BLE_ERR_HW_FAIL = 0x03,
  BLE_ERR_PAGE_TMO = 0x04,
  BLE_ERR_AUTH_FAIL = 0x05,
  BLE_ERR_PINKEY_MISSING = 0x06,
  BLE_ERR_MEM_CAPACITY = 0x07,
  BLE_ERR_CONN_SPVN_TMO = 0x08,
  BLE_ERR_CONN_LIMIT = 0x09,
  BLE_ERR_SYNCH_CONN_LIMIT = 0x0a,
  BLE_ERR_ACL_CONN_EXISTS = 0x0b,
  BLE_ERR_CMD_DISALLOWED = 0x0c,
  BLE_ERR_CONN_REJ_RESOURCES = 0x0d,
  BLE_ERR_CONN_REJ_SECURITY = 0x0e,
  BLE_ERR_CONN_REJ_BD_ADDR = 0x0f,
  BLE_ERR_CONN_ACCEPT_TMO = 0x10,
  BLE_ERR_UNSUPPORTED = 0x11,
  BLE_ERR_INV_HCI_CMD_PARMS = 0x12,
  BLE_ERR_REM_USER_CONN_TERM = 0x13,
  BLE_ERR_RD_CONN_TERM_RESRCS = 0x14,
  BLE_ERR_RD_CONN_TERM_PWROFF = 0x15,
  BLE_ERR_CONN_TERM_LOCAL = 0x16,
  BLE_ERR_REPEATED_ATTEMPTS = 0x17,
  BLE_ERR_NO_PAIRING = 0x18,
  BLE_ERR_UNK_LMP = 0x19,
  BLE_ERR_UNSUPP_REM_FEATURE = 0x1a,
  BLE_ERR_SCO_OFFSET = 0x1b,
  BLE_ERR_SCO_ITVL = 0x1c,
  BLE_ERR_SCO_AIR_MODE = 0x1d,
  BLE_ERR_INV_LMP_LL_PARM = 0x1e,
  BLE_ERR_UNSPECIFIED = 0x1f,
  BLE_ERR_UNSUPP_LMP_LL_PARM = 0x20,
  BLE_ERR_NO_ROLE_CHANGE = 0x21,
  BLE_ERR_LMP_LL_RSP_TMO = 0x22,
  BLE_ERR_LMP_COLLISION = 0x23,
  BLE_ERR_LMP_PDU = 0x24,
  BLE_ERR_ENCRYPTION_MODE = 0x25,
  BLE_ERR_LINK_KEY_CHANGE = 0x26,
  BLE_ERR_UNSUPP_QOS = 0x27,
  BLE_ERR_INSTANT_PASSED = 0x28,
  BLE_ERR_UNIT_KEY_PAIRING = 0x29,
  BLE_ERR_DIFF_TRANS_COLL = 0x2a,

  /* BLE_ERR_RESERVED         = 0x2b */

  BLE_ERR_QOS_PARM = 0x2c,
  BLE_ERR_QOS_REJECTED = 0x2d,
  BLE_ERR_CHAN_CLASS = 0x2e,
  BLE_ERR_INSUFFICIENT_SEC = 0x2f,
  BLE_ERR_PARM_OUT_OF_RANGE = 0x30,

  /* BLE_ERR_RESERVED         = 0x31 */

  BLE_ERR_PENDING_ROLE_SW = 0x32,

  /* BLE_ERR_RESERVED         = 0x33 */

  BLE_ERR_RESERVED_SLOT = 0x34,
  BLE_ERR_ROLE_SW_FAIL = 0x35,
  BLE_ERR_INQ_RSP_TOO_BIG = 0x36,
  BLE_ERR_SEC_SIMPLE_PAIR = 0x37,
  BLE_ERR_HOST_BUSY_PAIR = 0x38,
  BLE_ERR_CONN_REJ_CHANNEL = 0x39,
  BLE_ERR_CTLR_BUSY = 0x3a,
  BLE_ERR_CONN_PARMS = 0x3b,
  BLE_ERR_DIR_ADV_TMO = 0x3c,
  BLE_ERR_CONN_TERM_MIC = 0x3d,
  BLE_ERR_CONN_ESTABLISHMENT = 0x3e,
  BLE_ERR_MAC_CONN_FAIL = 0x3f,
  BLE_ERR_COARSE_CLK_ADJ = 0x40,
  BLE_ERR_TYPE0_SUBMAP_NDEF = 0x41,
  BLE_ERR_UNK_ADV_INDENT = 0x42,
  BLE_ERR_LIMIT_REACHED = 0x43,
  BLE_ERR_OPERATION_CANCELLED = 0x44,
  BLE_ERR_PACKET_TOO_LONG = 0x45,
  BLE_ERR_MAX = 0xff
};

/* Structure representing a BLE address. */

typedef struct
{
  /* The type of the address. */

  uint8_t type;

  /* The value of the address as an array of 6 bytes. */

  uint8_t val[6];
} ble_addr_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint8_t g_dev_addr[BLE_DEV_ADDR_LEN];

extern uint8_t g_random_addr[BLE_DEV_ADDR_LEN];

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_addr_cmp
 *
 ****************************************************************************/

static inline int ble_addr_cmp(const ble_addr_t *a, const ble_addr_t *b)
{
  int type_diff;

  type_diff = a->type - b->type;
  if (type_diff != 0)
    {
      return type_diff;
    }

  return memcmp(a->val, b->val, sizeof(a->val));
}

#endif /* __WIRELESS_BLE_LL_BLE_H */
