/****************************************************************************
 * wireless/ble_ll/ble_phy.h
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
 * (nimble/controller/include/controller/ble_phy.h).
 *
 ****************************************************************************/

#ifndef __WIRELESS_BLE_LL_BLE_PHY_H
#define __WIRELESS_BLE_LL_BLE_PHY_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/mm/iob.h>
#include <nuttx/wireless/bluetooth/ble_ll.h>

#include "hci_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Power */

#define BLE_PHY_MAX_PWR_DBM   (10)

/* Max. clock drift */

#define BLE_PHY_MAX_DRIFT_PPM (50)

/* PHY bitmasks (compatible with HCI) */

#define BLE_PHY_MASK_1M       (BLE_HCI_LE_PHY_1M_PREF_MASK)
#define BLE_PHY_MASK_2M       (BLE_HCI_LE_PHY_2M_PREF_MASK)
#define BLE_PHY_MASK_CODED    (BLE_HCI_LE_PHY_CODED_PREF_MASK)

/* PHY indices (for a zero-based array) */

#define BLE_PHY_IDX_1M        (0)
#define BLE_PHY_IDX_2M        (1)
#define BLE_PHY_IDX_CODED     (2)

/* Lower-half call helper */

#define BLE_PHY_OP(op, ...) \
  g_ble_ll_lower->ops->op(g_ble_ll_lower, ##__VA_ARGS__)

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern FAR struct ble_ll_lower_s *g_ble_ll_lower;

/* PDU metadata of the PDU being received, provided by the PHY */

extern FAR struct ble_mbuf_hdr *g_ble_ll_phy_rxhdr;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Copies the received PHY buffer into the allocated PDU */

void ble_phy_rxpdu_copy(FAR uint8_t *dptr, FAR struct iob_s *rxpdu);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline int ble_phy_init(void)
{
  return BLE_PHY_OP(init);
}

static inline int ble_phy_setchan(uint8_t chan, uint32_t access_addr,
                                  uint32_t crcinit)
{
  return BLE_PHY_OP(setchan, chan, access_addr, crcinit);
}

static inline uint8_t ble_phy_chan_get(void)
{
  return BLE_PHY_OP(chan_get);
}

static inline uint32_t ble_phy_access_addr_get(void)
{
  return BLE_PHY_OP(access_addr_get);
}

#ifdef CONFIG_BLE_LL_PHY_VARIABLE_TIFS
static inline void ble_phy_tifs_set(uint16_t tifs)
{
  BLE_PHY_OP(tifs_set, tifs);
}

#endif

static inline void ble_phy_tifs_txtx_set(uint16_t usecs, uint8_t anchor)
{
  BLE_PHY_OP(tifs_txtx_set, usecs, anchor);
}

static inline int ble_phy_tx_set_start_time(uint32_t cputime,
                                            uint8_t rem_usecs)
{
  return BLE_PHY_OP(tx_set_start_time, cputime, rem_usecs);
}

static inline int ble_phy_rx_set_start_time(uint32_t cputime,
                                            uint8_t rem_usecs)
{
  return BLE_PHY_OP(rx_set_start_time, cputime, rem_usecs);
}

static inline void ble_phy_set_txend_cb(ble_phy_tx_end_func txend_cb,
                                        FAR void *arg)
{
  BLE_PHY_OP(set_txend_cb, txend_cb, arg);
}

static inline int ble_phy_tx(ble_phy_tx_pducb_t pducb, FAR void *pducb_arg,
                             uint8_t end_trans)
{
  return BLE_PHY_OP(tx, pducb, pducb_arg, end_trans);
}

static inline void ble_phy_restart_rx(void)
{
  BLE_PHY_OP(restart_rx);
}

static inline void ble_phy_wfr_enable(int txrx, uint8_t tx_phy_mode,
                                      uint32_t wfr_usecs)
{
  BLE_PHY_OP(wfr_enable, txrx, tx_phy_mode, wfr_usecs);
}

static inline void ble_phy_disable(void)
{
  BLE_PHY_OP(disable);
}

static inline void ble_phy_rfclk_enable(void)
{
  BLE_PHY_OP(rfclk_enable);
}

static inline void ble_phy_rfclk_disable(void)
{
  BLE_PHY_OP(rfclk_disable);
}

static inline int ble_phy_state_get(void)
{
  return BLE_PHY_OP(state_get);
}

static inline uint8_t ble_phy_xcvr_state_get(void)
{
  return BLE_PHY_OP(xcvr_state_get);
}

static inline int ble_phy_rx_started(void)
{
  return BLE_PHY_OP(rx_started);
}

static inline uint8_t ble_phy_max_data_pdu_pyld(void)
{
  return BLE_PHY_OP(max_data_pdu_pyld);
}

static inline int ble_phy_tx_power_set(int dbm)
{
  return BLE_PHY_OP(tx_power_set, dbm);
}

static inline int ble_phy_tx_power_round(int dbm)
{
  return BLE_PHY_OP(tx_power_round, dbm);
}

static inline int ble_phy_tx_power_get(void)
{
  return BLE_PHY_OP(tx_power_get);
}

#if defined(CONFIG_BLE_LL_CFG_FEAT_LE_2M_PHY) || \
    defined(CONFIG_BLE_LL_CFG_FEAT_LE_CODED_PHY)
static inline void ble_phy_mode_set(uint8_t tx_phy_mode, uint8_t rx_phy_mode)
{
  BLE_PHY_OP(mode_set, tx_phy_mode, rx_phy_mode);
}

#endif

static inline void ble_phy_encrypt_enable(FAR const uint8_t *key)
{
  BLE_PHY_OP(encrypt_enable, key);
}

static inline void ble_phy_encrypt_header_mask_set(uint8_t mask)
{
  BLE_PHY_OP(encrypt_header_mask_set, mask);
}

static inline void ble_phy_encrypt_iv_set(FAR const uint8_t *iv)
{
  BLE_PHY_OP(encrypt_iv_set, iv);
}

static inline void ble_phy_encrypt_counter_set(uint64_t counter,
                                               uint8_t dir_bit)
{
  BLE_PHY_OP(encrypt_counter_set, counter, dir_bit);
}

static inline void ble_phy_encrypt_disable(void)
{
  BLE_PHY_OP(encrypt_disable);
}

static inline void ble_phy_resolv_list_enable(void)
{
  BLE_PHY_OP(resolv_list_enable);
}

static inline void ble_phy_resolv_list_disable(void)
{
  BLE_PHY_OP(resolv_list_disable);
}

static inline int ble_ll_phy_to_phy_mode(int phy, int phy_options)
{
  int phy_mode;

  /* 'phy' value can be used as 'phy_mode' value unless S=2 coding is
   * explicitly required. By default we'll use S=8 for Coded.
   */

  phy_mode = phy;

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_CODED_PHY
  if (phy == BLE_PHY_CODED && phy_options == BLE_HCI_LE_PHY_CODED_S2_PREF)
    {
      phy_mode = BLE_PHY_MODE_CODED_500KBPS;
    }
#else
  UNUSED(phy_options);
#endif

  return phy_mode;
}

#endif /* __WIRELESS_BLE_LL_BLE_PHY_H */
