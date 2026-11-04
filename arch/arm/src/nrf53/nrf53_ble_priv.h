/****************************************************************************
 * arch/arm/src/nrf53/nrf53_ble_priv.h
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
 * Derived from Apache Mynewt NimBLE (nimble/drivers/nrf5x/src/phy_priv.h,
 * nimble/drivers/nrf5x/src/nrf53/phy_ppi.h).
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF53_NRF53_BLE_PRIV_H
#define __ARCH_ARM_SRC_NRF53_NRF53_BLE_PRIV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/wireless/bluetooth/ble_ll.h>

#include "arm_internal.h"
#include "hardware/nrf53_aar.h"
#include "hardware/nrf53_ccm.h"
#include "hardware/nrf53_dppi.h"
#include "hardware/nrf53_radio.h"
#include "hardware/nrf53_rtc.h"
#include "hardware/nrf53_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTC0 and TIMER0 register addresses */

#define NRF53_BLE_RTC0(off)       (NRF53_RTC0_BASE + (off))
#define NRF53_BLE_TIMER0(off)     (NRF53_TIMER0_BASE + (off))

/* RTC0 compare channels: CC[0] starts TIMER0 via DPPI, CC[2] is the Link
 * Layer timer.
 */

#define NRF53_BLE_RTC_CC_START    0
#define NRF53_BLE_RTC_CC_TMR      2

/* DPPI channels published by RADIO, TIMER0 and RTC0 */

#define NRF53_BLE_DPPI_TIMER0_COMPARE0 0
#define NRF53_BLE_DPPI_TIMER0_COMPARE3 1
#define NRF53_BLE_DPPI_RADIO_END       2
#define NRF53_BLE_DPPI_RADIO_BCMATCH   3
#define NRF53_BLE_DPPI_RADIO_ADDRESS   4
#define NRF53_BLE_DPPI_RTC0_COMPARE0   5

#define NRF53_BLE_DPPI_MASK_ALL        0x3f

/* DPPI subscriptions switched at runtime.  RADIO ADDRESS -> TIMER0
 * CAPTURE[1] and RADIO END -> TIMER0 CAPTURE[2] are always enabled.
 *
 *   TIMER0_TXEN: TIMER0 COMPARE[0] -> RADIO TXEN
 *   TIMER0_RXEN: TIMER0 COMPARE[0] -> RADIO RXEN
 *   AAR_START:   RADIO BCMATCH     -> AAR START
 *   CCM_CRYPT:   RADIO ADDRESS     -> CCM CRYPT
 *   RTC0_START:  RTC0 COMPARE[0]   -> TIMER0 START
 *   WFR0:        RADIO ADDRESS     -> TIMER0 CAPTURE[3] (cancel wfr)
 *   WFR1:        TIMER0 COMPARE[3] -> RADIO DISABLE
 */

#define NRF53_BLE_PPI_TIMER0_TXEN  (1 << 0)
#define NRF53_BLE_PPI_TIMER0_RXEN  (1 << 1)
#define NRF53_BLE_PPI_AAR_START    (1 << 2)
#define NRF53_BLE_PPI_CCM_CRYPT    (1 << 3)
#define NRF53_BLE_PPI_RTC0_START   (1 << 4)
#define NRF53_BLE_PPI_WFR0         (1 << 5)
#define NRF53_BLE_PPI_WFR1         (1 << 6)

#define NRF53_BLE_PPI_MASK_ALL     (0x7f)

/* Size of the hardware filter accept list and resolving list */

#define NRF53_BLE_WHITELIST_SIZE  8

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
#  if CONFIG_BLE_LL_RESOLV_LIST_SIZE < 16
#    define NRF53_BLE_IRK_ENTRIES CONFIG_BLE_LL_RESOLV_LIST_SIZE
#  else
#    define NRF53_BLE_IRK_ENTRIES 16
#  endif
#endif

/* Scratch area for CCM: 16 bytes plus the maximum packet size */

#define NRF_ENC_SCRATCH_WORDS   (67)

/* Statistics */

#ifdef CONFIG_BLE_LL_STATS
#  define NRF53_BLE_STATS_INC(dev, f)     ((dev)->stats.f++)
#  define NRF53_BLE_STATS_INCN(dev, f, n) ((dev)->stats.f += (n))
#else
#  define NRF53_BLE_STATS_INC(dev, f)
#  define NRF53_BLE_STATS_INCN(dev, f, n)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Radio statistics */

struct nrf53_ble_stats_s
{
  uint32_t phy_isrs;
  uint32_t tx_good;
  uint32_t tx_fail;
  uint32_t tx_late;
  uint32_t tx_bytes;
  uint32_t rx_starts;
  uint32_t rx_aborts;
  uint32_t rx_valid;
  uint32_t rx_crc_err;
  uint32_t rx_late;
  uint32_t radio_state_errs;
  uint32_t rx_hw_err;
  uint32_t tx_hw_err;
};

/* PHY state */

struct nrf53_ble_phy_s
{
  int8_t   phy_txpwr_dbm;
  uint8_t  phy_chan;
  uint8_t  phy_state;
  uint8_t  phy_transition;
  uint8_t  phy_transition_late;
  uint8_t  phy_rx_started;
  uint8_t  phy_encrypted;
  uint8_t  phy_privacy;
  uint8_t  phy_tx_pyld_len;
  uint8_t  phy_cur_phy_mode;
  uint8_t  phy_tx_phy_mode;
  uint8_t  phy_rx_phy_mode;
  uint8_t  phy_bcc_offset;
  uint32_t phy_aar_scratch;
  uint32_t phy_access_address;
  struct ble_mbuf_hdr rxhdr;
  FAR void *txend_arg;
  ble_phy_tx_end_func txend_cb;
  uint32_t phy_start_cputime;
  uint16_t txtx_time_us;
  uint8_t  txtx_time_anchor;
};

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
/* CCM configuration data */

begin_packed_struct struct nrf_ccm_data_s
{
  uint8_t  key[16];
  uint64_t pkt_counter;
  uint8_t  dir_bit;
  uint8_t  iv[8];
} end_packed_struct;
#endif

/* nRF53 BLE radio device.  One RADIO peripheral per chip. */

struct nrf53_ble_dev_s
{
  struct ble_ll_lower_s lower;              /* Must be first */
  FAR const struct ble_ll_lowercb_s *cb;    /* Bound upper half */
  uint32_t rtc_cntr;                        /* Upper 8 bits of RTC0 */
  struct nrf53_ble_phy_s phy;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
  struct nrf_ccm_data_s ccm;
  uint32_t enc_scratch[NRF_ENC_SCRATCH_WORDS];
  uint32_t enc_buf[(BLE_PHY_MAX_PDU_LEN + 3) / 4];
#endif
  uint32_t tx_buf[(BLE_PHY_MAX_PDU_LEN + 3) / 4];
  uint32_t rx_buf[(BLE_PHY_MAX_PDU_LEN + 3) / 4];
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  uint32_t irk_list[NRF53_BLE_IRK_ENTRIES * 4];  /* IRKs used by AAR */
  uint8_t  num_irks;
#endif
  uint8_t  whitelist_mask;
#ifdef CONFIG_BLE_LL_STATS
  struct nrf53_ble_stats_s stats;
#endif
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline void nrf53_ble_ppi_set(uint32_t mask, bool enable)
{
  uint32_t en = enable ? DPPI_SUBSCRIBE_EN : 0;

  if (mask & NRF53_BLE_PPI_TIMER0_TXEN)
    {
      putreg32(en | NRF53_BLE_DPPI_TIMER0_COMPARE0,
               NRF53_RADIO_SUBSCRIBE_TXEN);
    }

  if (mask & NRF53_BLE_PPI_TIMER0_RXEN)
    {
      putreg32(en | NRF53_BLE_DPPI_TIMER0_COMPARE0,
               NRF53_RADIO_SUBSCRIBE_RXEN);
    }

  if (mask & NRF53_BLE_PPI_AAR_START)
    {
      putreg32(en | NRF53_BLE_DPPI_RADIO_BCMATCH,
               NRF53_AAR_SUBSCRIBE_START);
    }

  if (mask & NRF53_BLE_PPI_CCM_CRYPT)
    {
      putreg32(en | NRF53_BLE_DPPI_RADIO_ADDRESS,
               NRF53_CCM_SUBSCRIBE_CRYPT);
    }

  if (mask & NRF53_BLE_PPI_RTC0_START)
    {
      putreg32(en | NRF53_BLE_DPPI_RTC0_COMPARE0,
               NRF53_BLE_TIMER0(NRF53_TIM_SUBSCRIBE_START_OFFSET));
    }

  if (mask & NRF53_BLE_PPI_WFR0)
    {
      putreg32(en | NRF53_BLE_DPPI_RADIO_ADDRESS,
               NRF53_BLE_TIMER0(NRF53_TIM_SUBSCRIBE_CAPTURE_OFFSET(3)));
    }

  if (mask & NRF53_BLE_PPI_WFR1)
    {
      putreg32(en | NRF53_BLE_DPPI_TIMER0_COMPARE3,
               NRF53_RADIO_SUBSCRIBE_DISABLE);
    }
}

static inline void nrf53_ble_ppi_enable(uint32_t mask)
{
  nrf53_ble_ppi_set(mask, true);
}

static inline void nrf53_ble_ppi_disable(uint32_t mask)
{
  nrf53_ble_ppi_set(mask, false);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Timer (nrf53_ble.c) */

uint32_t nrf53_ble_tmr_get(FAR struct ble_ll_lower_s *lower);

/* PHY (nrf53_ble_phy.c) */

int nrf53_ble_phy_init(FAR struct ble_ll_lower_s *lower);
int nrf53_ble_phy_isr(int irq, FAR void *context, FAR void *arg);
int nrf53_ble_phy_setchan(FAR struct ble_ll_lower_s *lower, uint8_t chan,
                          uint32_t access_addr, uint32_t crcinit);
uint8_t nrf53_ble_phy_chan_get(FAR struct ble_ll_lower_s *lower);
uint32_t nrf53_ble_phy_access_addr_get(FAR struct ble_ll_lower_s *lower);
void nrf53_ble_phy_tifs_txtx_set(FAR struct ble_ll_lower_s *lower,
                                 uint16_t usecs, uint8_t anchor);
int nrf53_ble_phy_tx_set_start_time(FAR struct ble_ll_lower_s *lower,
                                    uint32_t cputime, uint8_t rem_usecs);
int nrf53_ble_phy_rx_set_start_time(FAR struct ble_ll_lower_s *lower,
                                    uint32_t cputime, uint8_t rem_usecs);
void nrf53_ble_phy_set_txend_cb(FAR struct ble_ll_lower_s *lower,
                                ble_phy_tx_end_func txend_cb,
                                FAR void *arg);
int nrf53_ble_phy_tx(FAR struct ble_ll_lower_s *lower,
                     ble_phy_tx_pducb_t pducb, FAR void *pducb_arg,
                     uint8_t end_trans);
void nrf53_ble_phy_restart_rx(FAR struct ble_ll_lower_s *lower);
void nrf53_ble_phy_wfr_enable(FAR struct ble_ll_lower_s *lower, int txrx,
                              uint8_t tx_phy_mode, uint32_t wfr_usecs);
void nrf53_ble_phy_disable(FAR struct ble_ll_lower_s *lower);
void nrf53_ble_phy_rfclk_enable(FAR struct ble_ll_lower_s *lower);
void nrf53_ble_phy_rfclk_disable(FAR struct ble_ll_lower_s *lower);
int nrf53_ble_phy_state_get(FAR struct ble_ll_lower_s *lower);
uint8_t nrf53_ble_phy_xcvr_state_get(FAR struct ble_ll_lower_s *lower);
int nrf53_ble_phy_rx_started(FAR struct ble_ll_lower_s *lower);
uint8_t nrf53_ble_phy_max_data_pdu_pyld(FAR struct ble_ll_lower_s *lower);
int nrf53_ble_phy_tx_power_set(FAR struct ble_ll_lower_s *lower, int dbm);
int nrf53_ble_phy_tx_power_round(FAR struct ble_ll_lower_s *lower,
                                 int dbm);
int nrf53_ble_phy_tx_power_get(FAR struct ble_ll_lower_s *lower);
void nrf53_ble_phy_mode_set(FAR struct ble_ll_lower_s *lower,
                            uint8_t tx_phy_mode, uint8_t rx_phy_mode);
void nrf53_ble_phy_encrypt_enable(FAR struct ble_ll_lower_s *lower,
                                  FAR const uint8_t *key);
void nrf53_ble_phy_encrypt_header_mask_set(FAR struct ble_ll_lower_s *lower,
                                           uint8_t mask);
void nrf53_ble_phy_encrypt_iv_set(FAR struct ble_ll_lower_s *lower,
                                  FAR const uint8_t *iv);
void nrf53_ble_phy_encrypt_counter_set(FAR struct ble_ll_lower_s *lower,
                                       uint64_t counter, uint8_t dir_bit);
void nrf53_ble_phy_encrypt_disable(FAR struct ble_ll_lower_s *lower);
void nrf53_ble_phy_resolv_list_enable(FAR struct ble_ll_lower_s *lower);
void nrf53_ble_phy_resolv_list_disable(FAR struct ble_ll_lower_s *lower);

/* Hardware support (nrf53_ble_hw.c) */

uint8_t nrf53_ble_hw_whitelist_size(FAR struct ble_ll_lower_s *lower);
void nrf53_ble_hw_whitelist_clear(FAR struct ble_ll_lower_s *lower);
int nrf53_ble_hw_whitelist_add(FAR struct ble_ll_lower_s *lower,
                               FAR const uint8_t *addr, uint8_t addr_type);
void nrf53_ble_hw_whitelist_rmv(FAR struct ble_ll_lower_s *lower,
                                FAR const uint8_t *addr, uint8_t addr_type);
void nrf53_ble_hw_whitelist_enable(FAR struct ble_ll_lower_s *lower);
void nrf53_ble_hw_whitelist_disable(FAR struct ble_ll_lower_s *lower);
int nrf53_ble_hw_whitelist_match(FAR struct ble_ll_lower_s *lower);
void nrf53_ble_hw_resolv_list_clear(FAR struct ble_ll_lower_s *lower);
int nrf53_ble_hw_resolv_list_add(FAR struct ble_ll_lower_s *lower,
                                 FAR const uint8_t *irk);
void nrf53_ble_hw_resolv_list_rmv(FAR struct ble_ll_lower_s *lower,
                                  int index);
uint8_t nrf53_ble_hw_resolv_list_size(FAR struct ble_ll_lower_s *lower);
int nrf53_ble_hw_resolv_list_match(FAR struct ble_ll_lower_s *lower);
int nrf53_ble_hw_encrypt_block(FAR struct ble_ll_lower_s *lower,
                               FAR struct ble_encryption_block *ecb);
int nrf53_ble_hw_rng_isr(int irq, FAR void *context, FAR void *arg);
int nrf53_ble_hw_rng_start(FAR struct ble_ll_lower_s *lower);
int nrf53_ble_hw_rng_stop(FAR struct ble_ll_lower_s *lower);
int nrf53_ble_hw_get_public_addr(FAR struct ble_ll_lower_s *lower,
                                 FAR uint8_t *addr);
int nrf53_ble_hw_get_static_addr(FAR struct ble_ll_lower_s *lower,
                                 FAR uint8_t *addr);

#endif /* __ARCH_ARM_SRC_NRF53_NRF53_BLE_PRIV_H */
