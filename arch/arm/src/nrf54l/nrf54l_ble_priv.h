/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_ble_priv.h
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
 * Derived from Apache Mynewt NimBLE (nimble/drivers/nrf5x/src/phy_priv.h)
 * and the Zephyr Bluetooth controller
 * (subsys/bluetooth/controller/ll_sw/nordic/hal/nrf5/radio/
 * radio_nrf5_dppi_resources.h, Copyright (c) 2021-2024 Nordic
 * Semiconductor ASA).
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_NRF54L_NRF54L_BLE_PRIV_H
#define __ARCH_ARM_SRC_NRF54L_NRF54L_BLE_PRIV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/wireless/bluetooth/ble_ll.h>

#include "arm_internal.h"
#include "nrf54l_grtc.h"
#include "hardware/nrf54l_dppi.h"
#include "hardware/nrf54l_radio.h"
#include "hardware/nrf54l_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TIMER10 register addresses */

#define NRF54L_BLE_TIMER(off)     (NRF54L_TIMER10_BASE + (off))

/* GRTC interrupt group and compare channels.  CC[2] is the Link Layer
 * timer, CC[3] starts TIMER10.
 */

#define NRF54L_BLE_GRTC           2
#define NRF54L_BLE_GRTC_CC_TMR    2
#define NRF54L_BLE_GRTC_CC_START  3

/* A GRTC compare value closer than this to the counter may not match */

#define NRF54L_BLE_GRTC_MARGIN    10

/* DPPI channels.  A PPIB bridge channel has the index of the DPPI channel
 * on both sides.
 *
 *   6:  RADIO BCMATCH      -> PPIB10/PPIB00 -> AAR START
 *   7:  RADIO PAYLOAD      -> PPIB10/PPIB00 -> CCM START
 *   8:  GRTC COMPARE[3]    -> PPIB21/PPIB11 -> TIMER10 START
 *   9:  TIMER10 COMPARE[0] -> RADIO TXEN or RXEN
 *   10: RADIO ADDRESS      -> TIMER10 CAPTURE[1], CAPTURE[3] (wfr)
 *   11: TIMER10 COMPARE[3] -> RADIO DISABLE (wfr)
 *   12: RADIO END          -> TIMER10 CAPTURE[2]
 */

#define NRF54L_BLE_DPPI_AAR       6
#define NRF54L_BLE_DPPI_CCM       7
#define NRF54L_BLE_DPPI_START     8
#define NRF54L_BLE_DPPI_TRX       9
#define NRF54L_BLE_DPPI_ADDRESS   10
#define NRF54L_BLE_DPPI_WFR       11
#define NRF54L_BLE_DPPI_END       12

/* Switchable connections, the equivalent of the nRF52 PPI channels.  TXEN
 * and RXEN share the TRX channel and are switched by the RADIO
 * subscription.
 */

#define NRF54L_BLE_PPI_AAR        DPPIC_CHEN_CH(NRF54L_BLE_DPPI_AAR)
#define NRF54L_BLE_PPI_CCM        DPPIC_CHEN_CH(NRF54L_BLE_DPPI_CCM)
#define NRF54L_BLE_PPI_START      DPPIC_CHEN_CH(NRF54L_BLE_DPPI_START)
#define NRF54L_BLE_PPI_WFR        DPPIC_CHEN_CH(NRF54L_BLE_DPPI_WFR)
#define NRF54L_BLE_PPI_TXEN       (1 << 30)
#define NRF54L_BLE_PPI_RXEN       (1 << 31)

#define NRF54L_BLE_PPI_DPPI_MASK \
  (NRF54L_BLE_PPI_AAR | NRF54L_BLE_PPI_CCM | NRF54L_BLE_PPI_START | \
   NRF54L_BLE_PPI_WFR)

#define NRF54L_BLE_PPI_MASK_ALL \
  (NRF54L_BLE_PPI_DPPI_MASK | NRF54L_BLE_PPI_TXEN | NRF54L_BLE_PPI_RXEN)

/* Size of the resolving list in AAR */

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
#  if CONFIG_BLE_LL_RESOLV_LIST_SIZE < 16
#    define NRF54L_BLE_IRK_ENTRIES CONFIG_BLE_LL_RESOLV_LIST_SIZE
#  else
#    define NRF54L_BLE_IRK_ENTRIES 16
#  endif
#endif

/* RADIO PCNF1.MAXLEN */

#define NRF54L_BLE_MAXLEN         255

/* Encrypted packet buffer: S0, LENGTH, S1, MAXLEN bytes and two extra
 * bytes read by CCM.
 */

#define NRF54L_BLE_ENC_BUF_LEN    (3 + NRF54L_BLE_MAXLEN + 2)

/* Receive buffer: 3 bytes of alignment padding, S0, LENGTH, S1 and
 * MAXLEN bytes.
 */

#define NRF54L_BLE_RX_BUF_LEN     (3 + 3 + NRF54L_BLE_MAXLEN)

/* Statistics */

#ifdef CONFIG_BLE_LL_STATS
#  define NRF54L_BLE_STATS_INC(dev, f)     ((dev)->stats.f++)
#  define NRF54L_BLE_STATS_INCN(dev, f, n) ((dev)->stats.f += (n))
#else
#  define NRF54L_BLE_STATS_INC(dev, f)
#  define NRF54L_BLE_STATS_INCN(dev, f, n)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Radio statistics */

struct nrf54l_ble_stats_s
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

struct nrf54l_ble_phy_s
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
  uint32_t phy_access_address;
  struct ble_mbuf_hdr rxhdr;
  FAR void *txend_arg;
  ble_phy_tx_end_func txend_cb;
  uint64_t phy_start_us;           /* GRTC time at which TIMER10 starts */
  uint16_t txtx_time_us;
  uint8_t  txtx_time_anchor;
};

/* EasyDMA job list entry of CCM and AAR */

struct nrf54l_ble_job_s
{
  uint32_t ptr;
  uint32_t attr;                   /* Length [23:0], attribute [31:24] */
};

#define NRF54L_BLE_JOB_ATTR(len, attr) \
  (((uint32_t)(len) & 0x00ffffff) | ((uint32_t)(attr) << 24))

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
/* CCM session configuration */

struct nrf54l_ble_ccm_s
{
  uint8_t  key[16];
  uint64_t pkt_counter;
  uint8_t  dir_bit;
  uint8_t  iv[8];
  uint8_t  hdr_mask;
};

/* CCM job lists */

struct nrf54l_ble_ccm_job_s
{
  uint16_t in_alen;
  uint16_t in_mlen;
  uint8_t  in_mlen_msb;
  uint8_t  out_mlen_msb;
  uint16_t out_alen;
  struct nrf54l_ble_job_s in[6];
  struct nrf54l_ble_job_s out[6];
};
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
/* AAR job lists: hash, prand, IRKs and a terminator in, IRK index out */

struct nrf54l_ble_aar_job_s
{
  uint32_t status;
  struct nrf54l_ble_job_s in[NRF54L_BLE_IRK_ENTRIES + 3];
  struct nrf54l_ble_job_s out[1];
};
#endif

/* nRF54L BLE radio device.  One RADIO peripheral per chip. */

struct nrf54l_ble_dev_s
{
  struct ble_ll_lower_s lower;              /* Must be first */
  FAR const struct ble_ll_lowercb_s *cb;    /* Bound upper half */
  FAR struct nrf54l_grtc_dev_s *grtc;       /* GRTC interrupt group */
  struct nrf54l_ble_phy_s phy;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
  struct nrf54l_ble_ccm_s ccm;
  struct nrf54l_ble_ccm_job_s ccm_job;
  uint32_t enc_buf[(NRF54L_BLE_ENC_BUF_LEN + 3) / 4];
#endif
  uint32_t tx_buf[(BLE_PHY_MAX_PDU_LEN + 3) / 4];
  uint32_t rx_buf[(NRF54L_BLE_RX_BUF_LEN + 3) / 4];
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  struct nrf54l_ble_aar_job_s aar_job;
  uint32_t irk_list[NRF54L_BLE_IRK_ENTRIES * 4];  /* IRKs used by AAR */
  uint8_t  num_irks;
#endif
#ifdef CONFIG_BLE_LL_STATS
  struct nrf54l_ble_stats_s stats;
#endif
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline void nrf54l_ble_ppi_enable(uint32_t mask)
{
  if (mask & NRF54L_BLE_PPI_TXEN)
    {
      putreg32(NRF54L_BLE_DPPI_TRX | RADIO_SUBSCRIBE_EN,
               NRF54L_RADIO_SUBSCRIBE_TXEN);
    }

  if (mask & NRF54L_BLE_PPI_RXEN)
    {
      putreg32(NRF54L_BLE_DPPI_TRX | RADIO_SUBSCRIBE_EN,
               NRF54L_RADIO_SUBSCRIBE_RXEN);
    }

  if (mask & NRF54L_BLE_PPI_WFR)
    {
      putreg32(NRF54L_BLE_DPPI_ADDRESS | TIM_SUBSCRIBE_EN,
               NRF54L_BLE_TIMER(NRF54L_TIM_SUBSCRIBE_CAPTURE_OFFSET(3)));
    }

  putreg32(mask & NRF54L_BLE_PPI_DPPI_MASK,
           NRF54L_DPPIC10_BASE + NRF54L_DPPIC_CHENSET_OFFSET);
}

static inline void nrf54l_ble_ppi_disable(uint32_t mask)
{
  putreg32(mask & NRF54L_BLE_PPI_DPPI_MASK,
           NRF54L_DPPIC10_BASE + NRF54L_DPPIC_CHENCLR_OFFSET);

  if (mask & NRF54L_BLE_PPI_TXEN)
    {
      putreg32(0, NRF54L_RADIO_SUBSCRIBE_TXEN);
    }

  if (mask & NRF54L_BLE_PPI_RXEN)
    {
      putreg32(0, NRF54L_RADIO_SUBSCRIBE_RXEN);
    }

  if (mask & NRF54L_BLE_PPI_WFR)
    {
      putreg32(0, NRF54L_BLE_TIMER(NRF54L_TIM_SUBSCRIBE_CAPTURE_OFFSET(3)));
    }
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Timer (nrf54l_ble.c) */

uint64_t nrf54l_ble_grtc_now(FAR struct nrf54l_ble_dev_s *dev);
uint64_t nrf54l_ble_tmr_t2us(FAR struct nrf54l_ble_dev_s *dev,
                             uint32_t ticks);
void nrf54l_ble_tmr_us2t(uint64_t usecs, FAR uint32_t *ticks,
                         FAR uint32_t *rem_us);
uint32_t nrf54l_ble_tmr_get(FAR struct ble_ll_lower_s *lower);

/* PHY (nrf54l_ble_phy.c) */

int nrf54l_ble_phy_init(FAR struct ble_ll_lower_s *lower);
int nrf54l_ble_phy_isr(int irq, FAR void *context, FAR void *arg);
int nrf54l_ble_phy_setchan(FAR struct ble_ll_lower_s *lower, uint8_t chan,
                           uint32_t access_addr, uint32_t crcinit);
uint8_t nrf54l_ble_phy_chan_get(FAR struct ble_ll_lower_s *lower);
uint32_t nrf54l_ble_phy_access_addr_get(FAR struct ble_ll_lower_s *lower);
void nrf54l_ble_phy_tifs_txtx_set(FAR struct ble_ll_lower_s *lower,
                                  uint16_t usecs, uint8_t anchor);
int nrf54l_ble_phy_tx_set_start_time(FAR struct ble_ll_lower_s *lower,
                                     uint32_t cputime, uint8_t rem_usecs);
int nrf54l_ble_phy_rx_set_start_time(FAR struct ble_ll_lower_s *lower,
                                     uint32_t cputime, uint8_t rem_usecs);
void nrf54l_ble_phy_set_txend_cb(FAR struct ble_ll_lower_s *lower,
                                 ble_phy_tx_end_func txend_cb,
                                 FAR void *arg);
int nrf54l_ble_phy_tx(FAR struct ble_ll_lower_s *lower,
                      ble_phy_tx_pducb_t pducb, FAR void *pducb_arg,
                      uint8_t end_trans);
void nrf54l_ble_phy_restart_rx(FAR struct ble_ll_lower_s *lower);
void nrf54l_ble_phy_wfr_enable(FAR struct ble_ll_lower_s *lower, int txrx,
                               uint8_t tx_phy_mode, uint32_t wfr_usecs);
void nrf54l_ble_phy_disable(FAR struct ble_ll_lower_s *lower);
void nrf54l_ble_phy_rfclk_enable(FAR struct ble_ll_lower_s *lower);
void nrf54l_ble_phy_rfclk_disable(FAR struct ble_ll_lower_s *lower);
int nrf54l_ble_phy_state_get(FAR struct ble_ll_lower_s *lower);
uint8_t nrf54l_ble_phy_xcvr_state_get(FAR struct ble_ll_lower_s *lower);
int nrf54l_ble_phy_rx_started(FAR struct ble_ll_lower_s *lower);
uint8_t nrf54l_ble_phy_max_data_pdu_pyld(FAR struct ble_ll_lower_s *lower);
int nrf54l_ble_phy_tx_power_set(FAR struct ble_ll_lower_s *lower, int dbm);
int nrf54l_ble_phy_tx_power_round(FAR struct ble_ll_lower_s *lower,
                                  int dbm);
int nrf54l_ble_phy_tx_power_get(FAR struct ble_ll_lower_s *lower);
void nrf54l_ble_phy_mode_set(FAR struct ble_ll_lower_s *lower,
                             uint8_t tx_phy_mode, uint8_t rx_phy_mode);
void nrf54l_ble_phy_encrypt_enable(FAR struct ble_ll_lower_s *lower,
                                   FAR const uint8_t *key);
void
nrf54l_ble_phy_encrypt_header_mask_set(FAR struct ble_ll_lower_s *lower,
                                       uint8_t mask);
void nrf54l_ble_phy_encrypt_iv_set(FAR struct ble_ll_lower_s *lower,
                                   FAR const uint8_t *iv);
void nrf54l_ble_phy_encrypt_counter_set(FAR struct ble_ll_lower_s *lower,
                                        uint64_t counter, uint8_t dir_bit);
void nrf54l_ble_phy_encrypt_disable(FAR struct ble_ll_lower_s *lower);
void nrf54l_ble_phy_resolv_list_enable(FAR struct ble_ll_lower_s *lower);
void nrf54l_ble_phy_resolv_list_disable(FAR struct ble_ll_lower_s *lower);

/* Hardware support (nrf54l_ble_hw.c) */

void nrf54l_ble_hw_resolv_list_clear(FAR struct ble_ll_lower_s *lower);
int nrf54l_ble_hw_resolv_list_add(FAR struct ble_ll_lower_s *lower,
                                  FAR const uint8_t *irk);
void nrf54l_ble_hw_resolv_list_rmv(FAR struct ble_ll_lower_s *lower,
                                   int index);
uint8_t nrf54l_ble_hw_resolv_list_size(FAR struct ble_ll_lower_s *lower);
int nrf54l_ble_hw_resolv_list_match(FAR struct ble_ll_lower_s *lower);
int nrf54l_ble_hw_get_public_addr(FAR struct ble_ll_lower_s *lower,
                                  FAR uint8_t *addr);
int nrf54l_ble_hw_get_static_addr(FAR struct ble_ll_lower_s *lower,
                                  FAR uint8_t *addr);

#endif /* __ARCH_ARM_SRC_NRF54L_NRF54L_BLE_PRIV_H */
