/****************************************************************************
 * wireless/ble_ll/ble_ll_hci.h
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
 * (nimble/controller/include/controller/ble_ll_hci.h).
 *
 ****************************************************************************/

#ifndef __WIRELESS_BLE_LL_BLE_LL_HCI_H
#define __WIRELESS_BLE_LL_BLE_LL_HCI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ble_ll_os.h"
#include "hci_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The largest event the controller will send. */

#define BLE_LL_MAX_EVT_LEN CONFIG_BLE_LL_EVT_SIZE

/* This determines the number of outstanding commands allowed from the
 * host to the controller. NOTE: you cannot change this without modifying
 * other portions of the code as we currently use a global os event for
 * the command; you would need to allocate a pool of these.
 */

#define BLE_LL_CFG_NUM_HCI_CMD_PKTS (1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef void (*ble_ll_hci_post_cmd_complete_cb)(void *user_data);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialize LL HCI */

void ble_ll_hci_init(void);

/* HCI transport to the host (ble_ll_driver.c) */

void *ble_ll_hci_evt_alloc(int discardable);
void ble_ll_hci_buf_free(void *buf);
int ble_ll_hci_evt_send(void *buf);
int ble_ll_hci_acl_send(struct iob_s *iob);

int ble_ll_hci_cmd_rx(uint8_t *cmdbuf);

int ble_ll_hci_acl_rx(struct iob_s *om);

int ble_ll_hci_iso_rx(struct iob_s *om);

/* Used to determine if the LE event is enabled/disabled */

bool ble_ll_hci_is_le_event_enabled(unsigned int subev);

/* Used to determine if event is enabled/disabled */

bool ble_ll_hci_is_event_enabled(unsigned int evcode);

/* Send event from controller to host */

int ble_ll_hci_event_send(struct ble_hci_ev *hci_ev);

/* Sends a command complete with a no-op opcode to host */

void ble_ll_hci_send_noop(void);

/* Checks the preferref phy masks from set default phy and set phy commands */

int ble_ll_hci_chk_phy_masks(uint8_t all_phys, uint8_t tx_phys,
                             uint8_t rx_phys, uint8_t *txphy,
                             uint8_t *rxphy);

/* Returns true if Extended Advertising HCI commands are in use */

bool ble_ll_hci_adv_mode_ext(void);

/* Check if max octets/time are within allowed range */

int ble_ll_hci_check_dle(uint16_t max_octets, uint16_t max_time);

void ble_ll_hci_supp_cmd_get(uint8_t *buf);

/* Used to set post HCI command hook */

void ble_ll_hci_post_cmd_cb_set(ble_ll_hci_post_cmd_complete_cb cb,
                                void *user_data);

#endif /* __WIRELESS_BLE_LL_BLE_LL_HCI_H */
