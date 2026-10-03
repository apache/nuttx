/****************************************************************************
 * wireless/ble_ll/ble_ll_scan_aux.h
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
 * (nimble/controller/include/controller/ble_ll_scan_aux.h).
 *
 ****************************************************************************/

#ifndef __WIRELESS_BLE_LL_BLE_LL_SCAN_AUX_H
#define __WIRELESS_BLE_LL_BLE_LL_SCAN_AUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ble_ll_os.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV
struct ble_ll_scan_aux_data;

#endif /* CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV
void ble_ll_scan_aux_init(void);

int ble_ll_scan_aux_sched(struct ble_ll_scan_aux_data *aux,
                          uint32_t pdu_ticks, uint8_t pdu_rem_us,
                          uint32_t aux_ptr);

int ble_ll_scan_aux_rx_isr_start(uint8_t pdu_type,
                                 struct ble_mbuf_hdr *rxhdr);

int ble_ll_scan_aux_rx_isr_end(struct iob_s *rxpdu, uint8_t crcok);

void ble_ll_scan_aux_rx_pkt_in(struct iob_s *rxpdu,
                               struct ble_mbuf_hdr *rxhdr);

void ble_ll_scan_aux_break(struct ble_ll_scan_aux_data *aux);

void ble_ll_scan_aux_wfr_timer_exp(void);

void ble_ll_scan_aux_halt(void);

void ble_ll_scan_aux_sched_remove(struct ble_ll_sched_item *sch);

int ble_ll_scan_aux_rx_isr_end_on_ext(struct ble_ll_scan_sm *scansm,
                                      struct iob_s *rxpdu);

void ble_ll_scan_aux_pkt_in_on_ext(struct iob_s *rxpdu,
                                   struct ble_mbuf_hdr *rxhdr);

#endif /* CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV */

#endif /* __WIRELESS_BLE_LL_BLE_LL_SCAN_AUX_H */
