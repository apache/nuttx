/****************************************************************************
 * include/nuttx/wireless/bluetooth/ble_ll.h
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
 * Derived from Apache Mynewt NimBLE (nimble/controller/include/controller/
 * ble_phy.h, ble_hw.h and nimble/include/nimble/ble.h).
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BLE_LL_H
#define __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BLE_LL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Channels */

#define BLE_PHY_NUM_CHANS           (40)
#define BLE_PHY_NUM_DATA_CHANS      (37)
#define BLE_PHY_CHAN0_FREQ_MHZ      (2402)
#define BLE_PHY_DATA_CHAN0_FREQ_MHZ (2404)
#define BLE_PHY_CHAN_SPACING_MHZ    (2)
#define BLE_PHY_NUM_ADV_CHANS       (3)
#define BLE_PHY_ADV_CHAN_START      (37)

#define BLE_IS_ADV_CHAN(chan)       ((chan) >= BLE_PHY_ADV_CHAN_START)
#define BLE_IS_DATA_CHAN(chan)      ((chan) < BLE_PHY_ADV_CHAN_START)

/* PHY states */

#define BLE_PHY_STATE_IDLE          (0)
#define BLE_PHY_STATE_RX            (1)
#define BLE_PHY_STATE_TX            (2)

/* PHY transitions */

#define BLE_PHY_TRANSITION_NONE     (0)
#define BLE_PHY_TRANSITION_RX_TX    (1)
#define BLE_PHY_TRANSITION_TX_RX    (2)
#define BLE_PHY_TRANSITION_TX_TX    (3)

/* PHY error codes */

#define BLE_PHY_ERR_RADIO_STATE     (1)
#define BLE_PHY_ERR_INIT            (2)
#define BLE_PHY_ERR_INV_PARAM       (3)
#define BLE_PHY_ERR_NO_BUFS         (4)
#define BLE_PHY_ERR_TX_LATE         (5)
#define BLE_PHY_ERR_RX_LATE         (6)

/* Maximum PDU length: 2 bytes LL header and 255 bytes payload */

#define BLE_PHY_MAX_PDU_LEN         (257)

/* Wait for response modes */

#define BLE_PHY_WFR_ENABLE_RX       (0)
#define BLE_PHY_WFR_ENABLE_TXRX     (1)

/* PHY modes.  Values for 1M, 2M and Coded S=8 are equal to the HCI PHY
 * values.
 */

#define BLE_PHY_MODE_CODED_500KBPS  (0)
#define BLE_PHY_MODE_1M             (1)
#define BLE_PHY_MODE_2M             (2)
#define BLE_PHY_MODE_CODED_125KBPS  (3)
#define BLE_PHY_NUM_MODE            (4)

/* PHY numbers (compatible with HCI) */

#define BLE_PHY_1M                  (1)
#define BLE_PHY_2M                  (2)
#define BLE_PHY_CODED               (3)

/* Link Layer timing and PDU format */

#define BLE_LL_IFS                  (150)
#define BLE_LL_PDU_HDR_LEN          (2)
#define BLE_LL_DATA_PDU_MAX_PYLD    (251)
#define BLE_LL_DATA_MIC_LEN         (4)
#define BLE_LL_PDU_HEADERMASK_DATA  (0xe3)

/* Encryption block size */

#define BLE_ENC_BLOCK_SIZE          (16)

/* PDU metadata flags.  Symbols with equal values are never set for the
 * same PDU.
 */

#define BLE_MBUF_HDR_F_CONNECT_IND_TXD  (0x4000)
#define BLE_MBUF_HDR_F_CONNECT_REQ_TXD  (0x4000)
#define BLE_MBUF_HDR_F_CONNECT_RSP_RXD  (0x0008)
#define BLE_MBUF_HDR_F_CONN_CREDIT      (0x8000)
#define BLE_MBUF_HDR_F_IGNORED          (0x8000)
#define BLE_MBUF_HDR_F_CONN_CREDIT_INT  (0x4000)
#define BLE_MBUF_HDR_F_SCAN_REQ_TXD     (0x4000)
#define BLE_MBUF_HDR_F_INITA_RESOLVED   (0x2000)
#define BLE_MBUF_HDR_F_TARGETA_RESOLVED (0x2000)
#define BLE_MBUF_HDR_F_EXT_ADV_SEC      (0x1000)
#define BLE_MBUF_HDR_F_EXT_ADV          (0x0800)
#define BLE_MBUF_HDR_F_RESOLVED         (0x0400)
#define BLE_MBUF_HDR_F_AUX_PTR_WAIT     (0x0200)
#define BLE_MBUF_HDR_F_AUX_PTR_FAILED   (0x0100)
#define BLE_MBUF_HDR_F_CRC_OK           (0x0080)
#define BLE_MBUF_HDR_F_DEVMATCH         (0x0040)
#define BLE_MBUF_HDR_F_MIC_FAILURE      (0x0020)
#define BLE_MBUF_HDR_F_SCAN_RSP_TXD     (0x0010)
#define BLE_MBUF_HDR_F_SCAN_RSP_RXD     (0x0008)
#define BLE_MBUF_HDR_F_RXSTATE_MASK     (0x0007)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* AES-128 ECB block */

struct ble_encryption_block
{
  uint8_t key[BLE_ENC_BLOCK_SIZE];
  uint8_t plain_text[BLE_ENC_BLOCK_SIZE];
  uint8_t cipher_text[BLE_ENC_BLOCK_SIZE];
};

/* Receive PDU metadata filled by the PHY */

struct ble_mbuf_hdr_rxinfo
{
  uint16_t flags;
  uint8_t  channel;
  uint8_t  handle;
  int8_t   rssi;
  int8_t   phy;
  uint8_t  phy_mode;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  int8_t   rpa_index;
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV
  FAR void *user_data;
#endif
};

/* Transmit PDU metadata */

struct ble_mbuf_hdr_txinfo
{
  uint8_t  num_data_pkt;
  uint8_t  hdr_byte;
  uint16_t offset;
  uint16_t pyld_len;
};

/* PDU metadata */

struct ble_mbuf_hdr
{
  union
  {
    struct ble_mbuf_hdr_rxinfo rxinfo;
    struct ble_mbuf_hdr_txinfo txinfo;
  };

  uint32_t beg_cputime;          /* Timer ticks when PDU started */
  uint32_t rem_usecs;            /* Microseconds remainder of beg_cputime */
};

/* Callbacks used by the PHY */

typedef CODE void (*ble_phy_tx_end_func)(FAR void *arg);
typedef CODE uint8_t (*ble_phy_tx_pducb_t)(FAR uint8_t *dptr,
                                           FAR void *pducb_arg,
                                           FAR uint8_t *hdr_byte);

/* Upper-half callbacks.  All are called from interrupt context. */

struct ble_ll_lowercb_s
{
  /* A PDU reception started.  rxbuf contains the PDU header. */

  CODE int (*rx_start)(FAR uint8_t *rxbuf, uint8_t chan,
                       FAR struct ble_mbuf_hdr *hdr);

  /* A PDU reception ended */

  CODE int (*rx_end)(FAR uint8_t *rxbuf, FAR struct ble_mbuf_hdr *hdr);

  /* Wait for response timer expired */

  CODE void (*wfr_exp)(void);

  /* Timer compare event */

  CODE void (*tmr_exp)(void);

  /* New random byte available */

  CODE void (*rng_sample)(uint8_t rnum);

  /* Current Link Layer state, stored in rxinfo.flags by the PHY */

  CODE uint8_t (*state_get)(void);
};

/* Lower-half operations.  Optional operations are marked; NULL selects
 * the generic implementation in the upper half.
 */

struct ble_ll_lower_s;
struct ble_ll_lower_ops_s
{
  /* Bind the upper-half callbacks.  Called once before any other
   * operation.
   */

  CODE void (*bind)(FAR struct ble_ll_lower_s *lower,
                    FAR const struct ble_ll_lowercb_s *cb);

  /* Initialize or reset the PHY */

  CODE int (*init)(FAR struct ble_ll_lower_s *lower);

  /* PHY */

  CODE int (*setchan)(FAR struct ble_ll_lower_s *lower, uint8_t chan,
                      uint32_t access_addr, uint32_t crcinit);
  CODE uint8_t (*chan_get)(FAR struct ble_ll_lower_s *lower);
  CODE uint32_t (*access_addr_get)(FAR struct ble_ll_lower_s *lower);
  CODE void (*tifs_set)(FAR struct ble_ll_lower_s *lower, uint16_t tifs);
  CODE void (*tifs_txtx_set)(FAR struct ble_ll_lower_s *lower,
                             uint16_t usecs, uint8_t anchor);
  CODE int (*tx_set_start_time)(FAR struct ble_ll_lower_s *lower,
                                uint32_t cputime, uint8_t rem_usecs);
  CODE int (*rx_set_start_time)(FAR struct ble_ll_lower_s *lower,
                                uint32_t cputime, uint8_t rem_usecs);
  CODE void (*set_txend_cb)(FAR struct ble_ll_lower_s *lower,
                            ble_phy_tx_end_func txend_cb, FAR void *arg);
  CODE int (*tx)(FAR struct ble_ll_lower_s *lower, ble_phy_tx_pducb_t pducb,
                 FAR void *pducb_arg, uint8_t end_trans);
  CODE void (*restart_rx)(FAR struct ble_ll_lower_s *lower);
  CODE void (*wfr_enable)(FAR struct ble_ll_lower_s *lower, int txrx,
                          uint8_t tx_phy_mode, uint32_t wfr_usecs);
  CODE void (*disable)(FAR struct ble_ll_lower_s *lower);
  CODE void (*rfclk_enable)(FAR struct ble_ll_lower_s *lower);
  CODE void (*rfclk_disable)(FAR struct ble_ll_lower_s *lower);
  CODE int (*state_get)(FAR struct ble_ll_lower_s *lower);
  CODE uint8_t (*xcvr_state_get)(FAR struct ble_ll_lower_s *lower);
  CODE int (*rx_started)(FAR struct ble_ll_lower_s *lower);
  CODE uint8_t (*max_data_pdu_pyld)(FAR struct ble_ll_lower_s *lower);
  CODE int (*tx_power_set)(FAR struct ble_ll_lower_s *lower, int dbm);
  CODE int (*tx_power_round)(FAR struct ble_ll_lower_s *lower, int dbm);
  CODE int (*tx_power_get)(FAR struct ble_ll_lower_s *lower);
  CODE void (*mode_set)(FAR struct ble_ll_lower_s *lower,
                        uint8_t tx_phy_mode, uint8_t rx_phy_mode);

  /* PHY encryption (AES-CCM) */

  CODE void (*encrypt_enable)(FAR struct ble_ll_lower_s *lower,
                              FAR const uint8_t *key);
  CODE void (*encrypt_header_mask_set)(FAR struct ble_ll_lower_s *lower,
                                       uint8_t mask);
  CODE void (*encrypt_iv_set)(FAR struct ble_ll_lower_s *lower,
                              FAR const uint8_t *iv);
  CODE void (*encrypt_counter_set)(FAR struct ble_ll_lower_s *lower,
                                   uint64_t counter, uint8_t dir_bit);
  CODE void (*encrypt_disable)(FAR struct ble_ll_lower_s *lower);

  /* PHY address resolution */

  CODE void (*resolv_list_enable)(FAR struct ble_ll_lower_s *lower);
  CODE void (*resolv_list_disable)(FAR struct ble_ll_lower_s *lower);

  /* Hardware filter accept list (CONFIG_BLE_LL_HW_WHITELIST) */

  CODE uint8_t (*whitelist_size)(FAR struct ble_ll_lower_s *lower);
  CODE void (*whitelist_clear)(FAR struct ble_ll_lower_s *lower);
  CODE int (*whitelist_add)(FAR struct ble_ll_lower_s *lower,
                            FAR const uint8_t *addr, uint8_t addr_type);
  CODE void (*whitelist_rmv)(FAR struct ble_ll_lower_s *lower,
                             FAR const uint8_t *addr, uint8_t addr_type);
  CODE void (*whitelist_enable)(FAR struct ble_ll_lower_s *lower);
  CODE void (*whitelist_disable)(FAR struct ble_ll_lower_s *lower);
  CODE int (*whitelist_match)(FAR struct ble_ll_lower_s *lower);

  /* Hardware resolving list */

  CODE void (*resolv_list_clear)(FAR struct ble_ll_lower_s *lower);
  CODE int (*resolv_list_add)(FAR struct ble_ll_lower_s *lower,
                              FAR const uint8_t *irk);
  CODE void (*resolv_list_rmv)(FAR struct ble_ll_lower_s *lower, int index);
  CODE uint8_t (*resolv_list_size)(FAR struct ble_ll_lower_s *lower);
  CODE int (*resolv_list_match)(FAR struct ble_ll_lower_s *lower);

  /* AES-128 ECB block encryption (optional) */

  CODE int (*encrypt_block)(FAR struct ble_ll_lower_s *lower,
                            FAR struct ble_encryption_block *ecb);

  /* Random number generator (optional) */

  CODE int (*rng_start)(FAR struct ble_ll_lower_s *lower);
  CODE int (*rng_stop)(FAR struct ble_ll_lower_s *lower);

  /* Device addresses (optional).  Return 0 if the address is present. */

  CODE int (*get_public_addr)(FAR struct ble_ll_lower_s *lower,
                              FAR uint8_t *addr);
  CODE int (*get_static_addr)(FAR struct ble_ll_lower_s *lower,
                              FAR uint8_t *addr);

  /* Free-running timer clocked at CONFIG_BLE_LL_TMR_FREQ with one compare
   * channel.  tmr_exp() is called when the counter reaches the target.
   */

  CODE uint32_t (*tmr_get)(FAR struct ble_ll_lower_s *lower);
  CODE void (*tmr_start)(FAR struct ble_ll_lower_s *lower, uint32_t target);
  CODE void (*tmr_stop)(FAR struct ble_ll_lower_s *lower);
};

/* Lower-half instance */

struct ble_ll_lower_s
{
  FAR const struct ble_ll_lower_ops_s *ops;

  /* Time from a scheduler timer event to the start of a transmission
   * (interrupt processing and radio ramp-up) in microseconds.
   */

  uint16_t sched_delay_us;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Name: ble_ll_register
 *
 * Description:
 *   Initialize the BLE Link Layer on top of a radio lower half and register
 *   it as a Bluetooth HCI driver.
 *
 * Input Parameters:
 *   lower - Radio lower half
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ble_ll_register(FAR struct ble_ll_lower_s *lower);

/****************************************************************************
 * Name: ble_ll_pdu_syncword_us
 *
 * Description:
 *   Duration of preamble and access address for a PHY mode.
 *
 ****************************************************************************/

uint32_t ble_ll_pdu_syncword_us(uint8_t phy_mode);

#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_BLUETOOTH_BLE_LL_H */
