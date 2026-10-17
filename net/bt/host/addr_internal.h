/****************************************************************************
 * net/bt/host/addr_internal.h
 *
 * Copyright (c) ByteDance Ltd. and/or its affiliates.
 *
 * Author: chao an <anchao.archer@bytedance.com>
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
 ****************************************************************************/

/** @file
 * @brief Internal header for Bluetooth address functions
 */

/* Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/addr.h>

void bt_addr_le_copy_resolved(bt_addr_le_t *dst, const bt_addr_le_t *src);

/**
 * @brief Determine whether an HCI LE event address was resolved by the
 * Controller
 *
 * This helper inspects an HCI LE event address field and reports whether the
 * Controller resolved a Resolvable Private Address (RPA) to an identity
 * address when producing the event.
 *
 * @warning The parameter is not a regular application-layer @ref
 * bt_addr_le_t. It must be the address field taken directly from an HCI LE
 * event structure. In those events, the address "type" uses the Identity
 * Address values (@c BT_ADDR_LE_PUBLIC_ID and @c BT_ADDR_LE_RANDOM_ID) to
 *    indicate that resolution has occurred; this function checks for exactly
 *    those two values. Do not use this with any @ref bt_addr_le_t obtained
 *    from Zephyr host APIs.
 *
 * The complete (at time of writing) list of events that contain at least one
 * field like this:
 * - LE Advertising Report event (@ref bt_hci_evt_le_advertising_report)
 * - LE Enhanced Connection Complete event (@ref
 * bt_hci_evt_le_enh_conn_complete)
 * - LE Directed Advertising Report event (@ref
 * bt_hci_evt_le_direct_adv_report)
 * - LE Extended Advertising Report event (@ref
 * bt_hci_evt_le_ext_advertising_report)
 * - LE Scan Request Received event (@ref bt_hci_evt_le_scan_req_received)
 * - LE Periodic Advertising Sync Transfer Received event (@ref
 * bt_hci_evt_le_past_received_v2)
 * - LE Monitored Advertisers Report event
 *
 * @note The exact "potentially resolved address field" type is not a
 * distinct
 * type in the Core Specification; it is inferred from the common layout of
 * the events above. This function only inspects the address type; it does
 * not perform address resolution nor consult the resolve list.
 *
 * @param hci_addr_field_value Address field taken directly from an HCI LE
 *                             event
 * @retval true The Controller resolved the address (the on-air RPA matched
 * an
 * IRK in the Controller's resolve list)
 * @retval false The address was not resolved by the Controller (resolution
 * disabled or no match), or the field holds one of the LE Extended
 * Advertising Report sentinels @c BT_ADDR_LE_UNRESOLVED or @c
 *    BT_ADDR_LE_ANONYMOUS
 *
 * @see bt_addr_le_copy_resolved() to convert a HCI event address type to a
 * regular @ref bt_addr_le_t.
 */

bool bt_addr_le_is_resolved(const bt_addr_le_t *hci_addr_field_value);
