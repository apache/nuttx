/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_ble_phy.c
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
 * Derived from Apache Mynewt NimBLE (nimble/drivers/nrf5x/src/ble_phy.c,
 * nimble/drivers/nrf5x/src/nrf52/phy.c) and the Zephyr Bluetooth
 * controller (subsys/bluetooth/controller/ll_sw/nordic/hal/nrf5/radio/
 * radio.c, radio_nrf54lx.h, Copyright (c) 2016-2024 Nordic Semiconductor
 * ASA).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <sys/param.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/bluetooth/ble_ll.h>

#include "arm_internal.h"
#include "nvic.h"
#include "nrf54l_ble_priv.h"
#include "hardware/nrf54l_aar.h"
#include "hardware/nrf54l_ccm.h"
#include "hardware/nrf54l_grtc.h"
#include "hardware/nrf54l_radio.h"
#include "hardware/nrf54l_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* To disable all radio interrupts */

#define NRF_RADIO_IRQ_MASK_ALL  (0xffffffff)

/* Radio ramp-up times in usecs (fast mode) */

#define BLE_PHY_T_TXENFAST      (40)
#define BLE_PHY_T_RXENFAST      (40)

/* TIMER10 runs from TIMER10 START to the radio enable for this time */

#define NRF54L_BLE_TIMER_LEAD   (4)

/* TIMER10 prescaler: 32 MHz / 2^5 = 1 MHz */

#define NRF54L_BLE_TIMER_PRESCALER (5)

/* We configure the radio with a 1 byte S0 field, 8 bit length field and
 * zero bit S1 field.
 */

#define NRF_LFLEN_BITS          (8)
#define NRF_S0LEN               (1)
#define NRF_S1LEN_BITS          (0)
#define NRF_CILEN_BITS          (2)
#define NRF_TERMLEN_BITS        (3)

/* Maximum length of frames */

#define NRF_MAXLEN              NRF54L_BLE_MAXLEN
#define NRF_BALEN               (3)

/* PCNF0 configuration values */

#define NRF_PCNF0               ((NRF_LFLEN_BITS << RADIO_PCNF0_LFLEN_SHIFT) | \
                                 RADIO_PCNF0_S1INCL |                         \
                                 (NRF_S0LEN << RADIO_PCNF0_S0LEN_SHIFT) |     \
                                 (NRF_S1LEN_BITS << RADIO_PCNF0_S1LEN_SHIFT))
#define NRF_PCNF0_1M            (NRF_PCNF0 | RADIO_PCNF0_PLEN_8BIT)
#define NRF_PCNF0_2M            (NRF_PCNF0 | RADIO_PCNF0_PLEN_16BIT)
#define NRF_PCNF0_CODED         (NRF_PCNF0 | RADIO_PCNF0_PLEN_LONGRANGE |      \
                                 (NRF_CILEN_BITS << RADIO_PCNF0_CILEN_SHIFT) | \
                                 (NRF_TERMLEN_BITS << RADIO_PCNF0_TERMLEN_SHIFT))

/* Data whitening: the IV is the channel index with bit 6 set */

#define NRF_DATAWHITE(chan)     (RADIO_DATAWHITE_POLY_BLE | 0x40 | (chan))

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* RF center frequency for each channel index (offset from 2400 MHz) */

static const uint8_t g_ble_phy_chan_freq[BLE_PHY_NUM_CHANS] =
{
  4,  6,  8,  10, 12, 14, 16, 18, 20, 22, /* 0-9 */
  24, 28, 30, 32, 34, 36, 38, 40, 42, 44, /* 10-19 */
  46, 48, 50, 52, 54, 56, 58, 60, 62, 64, /* 20-29 */
  66, 68, 70, 72, 74, 76, 78, 2,  26, 80, /* 30-39 */
};

/* TX power levels in dBm, highest first, and their TXPOWER values */

static const int8_t g_ble_phy_txpower_dbm[] =
{
  8, 7, 6, 5, 4, 3, 2, 1, 0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -12,
  -14, -16, -18, -20, -28, -40, -46
};

static const uint16_t g_ble_phy_txpower_reg[] =
{
  RADIO_TXPOWER_POS8DBM, RADIO_TXPOWER_POS7DBM, RADIO_TXPOWER_POS6DBM,
  RADIO_TXPOWER_POS5DBM, RADIO_TXPOWER_POS4DBM, RADIO_TXPOWER_POS3DBM,
  RADIO_TXPOWER_POS2DBM, RADIO_TXPOWER_POS1DBM, RADIO_TXPOWER_0DBM,
  RADIO_TXPOWER_NEG1DBM, RADIO_TXPOWER_NEG2DBM, RADIO_TXPOWER_NEG3DBM,
  RADIO_TXPOWER_NEG4DBM, RADIO_TXPOWER_NEG5DBM, RADIO_TXPOWER_NEG6DBM,
  RADIO_TXPOWER_NEG7DBM, RADIO_TXPOWER_NEG8DBM, RADIO_TXPOWER_NEG9DBM,
  RADIO_TXPOWER_NEG10DBM, RADIO_TXPOWER_NEG12DBM, RADIO_TXPOWER_NEG14DBM,
  RADIO_TXPOWER_NEG16DBM, RADIO_TXPOWER_NEG18DBM, RADIO_TXPOWER_NEG20DBM,
  RADIO_TXPOWER_NEG28DBM, RADIO_TXPOWER_NEG40DBM, RADIO_TXPOWER_NEG46DBM
};

#ifdef CONFIG_BLE_LL_PHY
/* Packet start offsets (in usecs) */

static const uint16_t g_ble_phy_mode_pkt_start_off[BLE_PHY_NUM_MODE] =
{
  [BLE_PHY_MODE_1M]            = 40,
  [BLE_PHY_MODE_2M]            = 24,
  [BLE_PHY_MODE_CODED_125KBPS] = 376,
  [BLE_PHY_MODE_CODED_500KBPS] = 376
};
#endif

/* Radio event delays, nRF52 values.  The fast ramp-up and chain delays of
 * the Zephyr controller are equal for nRF52840 and nRF54L.
 */

/* Delay between EVENTS_READY and start of tx */

static const uint8_t g_ble_phy_t_txdelay[BLE_PHY_NUM_MODE] =
{
  [BLE_PHY_MODE_1M]            = 6,
  [BLE_PHY_MODE_2M]            = 5,
  [BLE_PHY_MODE_CODED_125KBPS] = 5,
  [BLE_PHY_MODE_CODED_500KBPS] = 5
};

/* Delay between EVENTS_ADDRESS and txd access address */

static const uint8_t g_ble_phy_t_txaddrdelay[BLE_PHY_NUM_MODE] =
{
  [BLE_PHY_MODE_1M]            = 7,
  [BLE_PHY_MODE_2M]            = 5,
  [BLE_PHY_MODE_CODED_125KBPS] = 17,
  [BLE_PHY_MODE_CODED_500KBPS] = 17
};

/* Delay between EVENTS_END and end of txd packet */

static const uint8_t g_ble_phy_t_txenddelay[BLE_PHY_NUM_MODE] =
{
  [BLE_PHY_MODE_1M]            = 6,
  [BLE_PHY_MODE_2M]            = 4,
  [BLE_PHY_MODE_CODED_125KBPS] = 9,
  [BLE_PHY_MODE_CODED_500KBPS] = 3
};

/* Delay between rxd access address (w/ TERM1 for coded) and EVENTS_ADDRESS */

static const uint8_t g_ble_phy_t_rxaddrdelay[BLE_PHY_NUM_MODE] =
{
  [BLE_PHY_MODE_1M]            = 6,
  [BLE_PHY_MODE_2M]            = 2,
  [BLE_PHY_MODE_CODED_125KBPS] = 17,
  [BLE_PHY_MODE_CODED_500KBPS] = 17
};

/* Delay between end of rxd packet and EVENTS_END */

static const uint8_t g_ble_phy_t_rxenddelay[BLE_PHY_NUM_MODE] =
{
  [BLE_PHY_MODE_1M]            = 4,
  [BLE_PHY_MODE_2M]            = 1,
  [BLE_PHY_MODE_CODED_125KBPS] = 27,
  [BLE_PHY_MODE_CODED_500KBPS] = 22
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: timer_reset
 ****************************************************************************/

static void timer_reset(void)
{
  /* Reset CC registers to avoid triggering spurious COMPARE events */

  putreg32(0, NRF54L_BLE_TIMER(NRF54L_TIM_CC_OFFSET(0)));
  putreg32(0, NRF54L_BLE_TIMER(NRF54L_TIM_CC_OFFSET(1)));
  putreg32(0, NRF54L_BLE_TIMER(NRF54L_TIM_CC_OFFSET(2)));
  putreg32(0, NRF54L_BLE_TIMER(NRF54L_TIM_CC_OFFSET(3)));
}

/****************************************************************************
 * Name: timer_cc_set
 ****************************************************************************/

static inline void timer_cc_set(int cc, uint32_t value)
{
  putreg32(value, NRF54L_BLE_TIMER(NRF54L_TIM_CC_OFFSET(cc)));
}

/****************************************************************************
 * Name: timer_cc_get
 ****************************************************************************/

static inline uint32_t timer_cc_get(int cc)
{
  return getreg32(NRF54L_BLE_TIMER(NRF54L_TIM_CC_OFFSET(cc)));
}

/****************************************************************************
 * Name: timer_event_clear
 ****************************************************************************/

static inline void timer_event_clear(int cc)
{
  putreg32(0, NRF54L_BLE_TIMER(NRF54L_TIM_EVENTS_COMPARE_OFFSET(cc)));
}

/****************************************************************************
 * Name: timer_did_miss
 *
 * Description:
 *   Check if TIMER10 already counted past CC[cc] without generating the
 *   COMPARE event.  CC[cc_scratch] is used to capture the current value.
 *
 ****************************************************************************/

static bool timer_did_miss(int cc, int cc_scratch)
{
  uint32_t target;
  uint32_t now;

  putreg32(1, NRF54L_BLE_TIMER(NRF54L_TIM_TASKS_CAPTURE_OFFSET(cc_scratch)));
  target = timer_cc_get(cc);
  now    = timer_cc_get(cc_scratch);

  /* COMPARE event is not triggered when CC is set to the current value of
   * the timer, so equal CC values without a COMPARE event are a miss too.
   */

  return (now >= target) &&
         !getreg32(NRF54L_BLE_TIMER(NRF54L_TIM_EVENTS_COMPARE_OFFSET(cc)));
}

/****************************************************************************
 * Name: timer_stop
 *
 * Description:
 *   Stop TIMER10 and the GRTC compare that starts it.
 *
 ****************************************************************************/

static void timer_stop(FAR struct nrf54l_ble_dev_s *dev)
{
  NRF54L_GRTC_DISABLECC(dev->grtc, NRF54L_BLE_GRTC_CC_START);
  putreg32(1, NRF54L_BLE_TIMER(NRF54L_TIM_TASKS_STOP_OFFSET));
}

/****************************************************************************
 * Name: phy_ppi_init
 ****************************************************************************/

static void phy_ppi_init(void)
{
  uintptr_t dppic = NRF54L_DPPIC10_BASE;

  /* TIMER10 COMPARE[0] -> RADIO TXEN/RXEN, switched by the subscription */

  putreg32(NRF54L_BLE_DPPI_TRX | TIM_PUBLISH_EN,
           NRF54L_BLE_TIMER(NRF54L_TIM_PUBLISH_COMPARE_OFFSET(0)));

  /* RADIO ADDRESS -> TIMER10 CAPTURE[1] and RADIO END -> TIMER10
   * CAPTURE[2] are always enabled.
   */

  putreg32(NRF54L_BLE_DPPI_ADDRESS | RADIO_PUBLISH_EN,
           NRF54L_RADIO_PUBLISH_ADDRESS);
  putreg32(NRF54L_BLE_DPPI_ADDRESS | TIM_SUBSCRIBE_EN,
           NRF54L_BLE_TIMER(NRF54L_TIM_SUBSCRIBE_CAPTURE_OFFSET(1)));
  putreg32(NRF54L_BLE_DPPI_END | RADIO_PUBLISH_EN,
           NRF54L_RADIO_PUBLISH_END);
  putreg32(NRF54L_BLE_DPPI_END | TIM_SUBSCRIBE_EN,
           NRF54L_BLE_TIMER(NRF54L_TIM_SUBSCRIBE_CAPTURE_OFFSET(2)));

  /* Wait for response timer: RADIO ADDRESS -> TIMER10 CAPTURE[3] cancels
   * the timer, TIMER10 COMPARE[3] -> RADIO DISABLE.
   */

  putreg32(NRF54L_BLE_DPPI_WFR | TIM_PUBLISH_EN,
           NRF54L_BLE_TIMER(NRF54L_TIM_PUBLISH_COMPARE_OFFSET(3)));
  putreg32(NRF54L_BLE_DPPI_WFR | RADIO_SUBSCRIBE_EN,
           NRF54L_RADIO_SUBSCRIBE_DISABLE);

  putreg32(DPPIC_CHEN_CH(NRF54L_BLE_DPPI_TRX) |
           DPPIC_CHEN_CH(NRF54L_BLE_DPPI_ADDRESS) |
           DPPIC_CHEN_CH(NRF54L_BLE_DPPI_END),
           dppic + NRF54L_DPPIC_CHENSET_OFFSET);

  nrf54l_ble_ppi_disable(NRF54L_BLE_PPI_MASK_ALL);
}

/****************************************************************************
 * Name: phy_txpower_round
 ****************************************************************************/

static int phy_txpower_round(int dbm)
{
  int i;

  for (i = 0; i < nitems(g_ble_phy_txpower_dbm) - 1; i++)
    {
      if (dbm >= g_ble_phy_txpower_dbm[i])
        {
          break;
        }
    }

  return i;
}

#ifdef CONFIG_BLE_LL_PHY
/****************************************************************************
 * Name: ble_phy_mode_pdu_start_off
 *
 * Description:
 *   Packet start offset (in usecs).  This is the preamble plus access
 *   address.  For LE Coded PHY this also includes CI and TERM1.
 *
 ****************************************************************************/

static uint32_t ble_phy_mode_pdu_start_off(int phy_mode)
{
  return g_ble_phy_mode_pkt_start_off[phy_mode];
}

/****************************************************************************
 * Name: ble_phy_mode_apply
 ****************************************************************************/

static void ble_phy_mode_apply(FAR struct nrf54l_ble_dev_s *dev,
                               uint8_t phy_mode)
{
  if (phy_mode == dev->phy.phy_cur_phy_mode)
    {
      return;
    }

  switch (phy_mode)
    {
      case BLE_PHY_MODE_1M:
        putreg32(RADIO_MODE_BLE1MBIT, NRF54L_RADIO_MODE);
        putreg32(NRF_PCNF0_1M, NRF54L_RADIO_PCNF0);
        break;

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_2M_PHY
      case BLE_PHY_MODE_2M:
        putreg32(RADIO_MODE_BLE2MBIT, NRF54L_RADIO_MODE);
        putreg32(NRF_PCNF0_2M, NRF54L_RADIO_PCNF0);
        break;
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_CODED_PHY
      case BLE_PHY_MODE_CODED_125KBPS:
        putreg32(RADIO_MODE_BLELR125KBIT, NRF54L_RADIO_MODE);
        putreg32(NRF_PCNF0_CODED, NRF54L_RADIO_PCNF0);
        break;

      case BLE_PHY_MODE_CODED_500KBPS:
        putreg32(RADIO_MODE_BLELR500KBIT, NRF54L_RADIO_MODE);
        putreg32(NRF_PCNF0_CODED, NRF54L_RADIO_PCNF0);
        break;
#endif

      default:
        DEBUGPANIC();
    }

  dev->phy.phy_cur_phy_mode = phy_mode;
}
#else
/****************************************************************************
 * Name: ble_phy_mode_pdu_start_off
 ****************************************************************************/

static uint32_t ble_phy_mode_pdu_start_off(int phy_mode)
{
  return 40;
}
#endif

/****************************************************************************
 * Name: ble_phy_get_cur_phy
 ****************************************************************************/

static int ble_phy_get_cur_phy(FAR struct nrf54l_ble_dev_s *dev)
{
#ifdef CONFIG_BLE_LL_PHY
  switch (dev->phy.phy_cur_phy_mode)
    {
      case BLE_PHY_MODE_1M:
        return BLE_PHY_1M;

      case BLE_PHY_MODE_2M:
        return BLE_PHY_2M;

      case BLE_PHY_MODE_CODED_125KBPS:
      case BLE_PHY_MODE_CODED_500KBPS:
        return BLE_PHY_CODED;

      default:
        DEBUGPANIC();
        return -1;
    }
#else
  return BLE_PHY_1M;
#endif
}

/****************************************************************************
 * Name: nrf_wait_disabled
 *
 * Description:
 *   Wait until the radio leaves the RX or TX disable state.  This ends
 *   within a few microseconds.
 *
 ****************************************************************************/

static void nrf_wait_disabled(void)
{
  uint32_t state;

  state = getreg32(NRF54L_RADIO_STATE);
  if (state == RADIO_STATE_RXDISABLE || state == RADIO_STATE_TXDISABLE)
    {
      while (getreg32(NRF54L_RADIO_STATE) == state)
        {
        }
    }
}

/****************************************************************************
 * Name: ble_phy_timer_arm
 *
 * Description:
 *   Start TIMER10 from GRTC CC[3] at the given GRTC time, TIMER10 CC[0]
 *   enables the radio after radio_us.
 *
 * Returned Value:
 *   0 on success, -1 if the start time is too close or in the past.
 *
 ****************************************************************************/

static int ble_phy_timer_arm(FAR struct nrf54l_ble_dev_s *dev,
                             uint64_t start_us, uint32_t radio_us)
{
  uintptr_t evt;

  evt = NRF54L_GRTC_BASE +
        NRF54L_GRTC_EVENTS_COMPARE_OFFSET(NRF54L_BLE_GRTC_CC_START);

  NRF54L_GRTC_DISABLECC(dev->grtc, NRF54L_BLE_GRTC_CC_START);
  putreg32(0, evt);

  if (start_us <= nrf54l_ble_grtc_now(dev) + NRF54L_BLE_GRTC_MARGIN)
    {
      return -1;
    }

  /* Clear and set TIMER10 to fire off at proper time */

  putreg32(1, NRF54L_BLE_TIMER(NRF54L_TIM_TASKS_STOP_OFFSET));
  timer_reset();
  putreg32(1, NRF54L_BLE_TIMER(NRF54L_TIM_TASKS_CLEAR_OFFSET));
  timer_cc_set(0, radio_us);
  timer_event_clear(0);

  /* Set GRTC compare to start TIMER10 */

  NRF54L_GRTC_SETCC(dev->grtc, NRF54L_BLE_GRTC_CC_START, start_us);
  nrf54l_ble_ppi_enable(NRF54L_BLE_PPI_START);

  if (nrf54l_ble_grtc_now(dev) >= start_us && getreg32(evt) == 0)
    {
      nrf54l_ble_ppi_disable(NRF54L_BLE_PPI_START);
      NRF54L_GRTC_DISABLECC(dev->grtc, NRF54L_BLE_GRTC_CC_START);
      return -1;
    }

  dev->phy.phy_start_us = start_us;
  return 0;
}

/****************************************************************************
 * Name: ble_phy_set_start_time
 *
 * Description:
 *   Program GRTC CC[3] and TIMER10 CC[0] so that the radio is enabled at the
 *   requested time.
 *
 * Returned Value:
 *   0 on success, -1 if the start time is too close or in the past.
 *
 ****************************************************************************/

static int ble_phy_set_start_time(FAR struct nrf54l_ble_dev_s *dev,
                                  uint32_t cputime, uint8_t rem_us, bool tx)
{
  uint64_t radio_us;

  /* GRTC time for radio enable */

  radio_us = nrf54l_ble_tmr_t2us(dev, cputime) + rem_us;
  if (tx)
    {
      radio_us -= BLE_PHY_T_TXENFAST +
                  g_ble_phy_t_txdelay[dev->phy.phy_cur_phy_mode];
    }
  else
    {
      radio_us -= BLE_PHY_T_RXENFAST;
    }

  return ble_phy_timer_arm(dev, radio_us - NRF54L_BLE_TIMER_LEAD,
                           NRF54L_BLE_TIMER_LEAD);
}

/****************************************************************************
 * Name: ble_phy_set_start_now
 *
 * Description:
 *   Start the radio as soon as possible.  Only used for RX.
 *
 ****************************************************************************/

static void ble_phy_set_start_now(FAR struct nrf54l_ble_dev_s *dev)
{
  irqstate_t flags;

  flags = enter_critical_section();

  while (ble_phy_timer_arm(dev, nrf54l_ble_grtc_now(dev) +
                           3 * NRF54L_BLE_GRTC_MARGIN,
                           NRF54L_BLE_TIMER_LEAD) != 0)
    {
    }

  leave_critical_section(flags);
}

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
/****************************************************************************
 * Name: get_be32
 ****************************************************************************/

static inline uint32_t get_be32(FAR const uint8_t *buf)
{
  return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
         ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
}

/****************************************************************************
 * Name: ble_phy_ccm_job
 ****************************************************************************/

static inline void ble_phy_ccm_job(FAR struct nrf54l_ble_job_s *job,
                                   FAR const void *ptr, uint32_t len,
                                   uint8_t attr)
{
  job->ptr  = (uint32_t)(uintptr_t)ptr;
  job->attr = NRF54L_BLE_JOB_ATTR(len, attr);
}

/****************************************************************************
 * Name: ble_phy_ccm_config
 *
 * Description:
 *   Enable CCM and load the mode, key, nonce and header mask.
 *
 ****************************************************************************/

static void ble_phy_ccm_config(FAR struct nrf54l_ble_dev_s *dev,
                               uint32_t mode)
{
  FAR struct nrf54l_ble_ccm_s *ccm = &dev->ccm;
  uint8_t ctr[5];
  int i;

  for (i = 0; i < 5; i++)
    {
      ctr[i] = (uint8_t)(ccm->pkt_counter >> (8 * i));
    }

  putreg32(CCM_ENABLE_DISABLED, NRF54L_CCM_ENABLE);
  putreg32(CCM_ENABLE_ENABLED, NRF54L_CCM_ENABLE);
  putreg32(mode | CCM_MODE_PROTOCOL_BLE | CCM_MODE_MACLEN_M4,
           NRF54L_CCM_MODE);
  putreg32(ccm->hdr_mask, NRF54L_CCM_ADATAMASK);

  putreg32(get_be32(&ccm->key[0]), NRF54L_CCM_KEY(3));
  putreg32(get_be32(&ccm->key[4]), NRF54L_CCM_KEY(2));
  putreg32(get_be32(&ccm->key[8]), NRF54L_CCM_KEY(1));
  putreg32(get_be32(&ccm->key[12]), NRF54L_CCM_KEY(0));

  putreg32(ctr[0], NRF54L_CCM_NONCE(3));
  putreg32(get_be32(&ctr[1]) | ((uint32_t)ccm->dir_bit << 7),
           NRF54L_CCM_NONCE(2));
  putreg32(get_be32(&ccm->iv[0]), NRF54L_CCM_NONCE(1));
  putreg32(get_be32(&ccm->iv[4]), NRF54L_CCM_NONCE(0));

  putreg32(0, NRF54L_CCM_EVENTS_END);
  putreg32(0, NRF54L_CCM_EVENTS_ERROR);
}

/****************************************************************************
 * Name: ble_phy_ccm_rx_setup
 *
 * Description:
 *   Decrypt the received PDU from enc_buf to dptr.  CCM is started by
 *   RADIO PAYLOAD.
 *
 ****************************************************************************/

static void ble_phy_ccm_rx_setup(FAR struct nrf54l_ble_dev_s *dev,
                                 FAR uint8_t *dptr)
{
  FAR struct nrf54l_ble_ccm_job_s *job = &dev->ccm_job;
  FAR uint8_t *src = (FAR uint8_t *)dev->enc_buf;

  ble_phy_ccm_config(dev, CCM_MODE_FASTDECRYPTION);

  job->in_alen     = 1;
  job->in_mlen_msb = 0;
  ble_phy_ccm_job(&job->in[0], &job->in_alen, 2, CCM_JOB_ATTR_ALEN);
  ble_phy_ccm_job(&job->in[1], src + 1, 1, CCM_JOB_ATTR_MLEN);
  ble_phy_ccm_job(&job->in[2], &job->in_mlen_msb, 1, CCM_JOB_ATTR_MLEN);
  ble_phy_ccm_job(&job->in[3], src, 1, CCM_JOB_ATTR_ADATA);
  ble_phy_ccm_job(&job->in[4], src + 3, NRF_MAXLEN + 2, CCM_JOB_ATTR_MDATA);
  ble_phy_ccm_job(&job->in[5], NULL, 0, 0);

  ble_phy_ccm_job(&job->out[0], &job->out_alen, 2, CCM_JOB_ATTR_ALEN);
  ble_phy_ccm_job(&job->out[1], dptr + 1, 1, CCM_JOB_ATTR_MLEN);
  ble_phy_ccm_job(&job->out[2], &job->out_mlen_msb, 1, CCM_JOB_ATTR_MLEN);
  ble_phy_ccm_job(&job->out[3], dptr, 1, CCM_JOB_ATTR_ADATA);
  ble_phy_ccm_job(&job->out[4], dptr + 3, NRF_MAXLEN - BLE_LL_DATA_MIC_LEN,
                  CCM_JOB_ATTR_MDATA);
  ble_phy_ccm_job(&job->out[5], NULL, 0, 0);

  putreg32((uint32_t)(uintptr_t)job->in, NRF54L_CCM_IN_PTR);
  putreg32((uint32_t)(uintptr_t)job->out, NRF54L_CCM_OUT_PTR);
  putreg32(NRF54L_BLE_DPPI_CCM | CCM_SUBSCRIBE_EN,
           NRF54L_CCM_SUBSCRIBE_START);
}

/****************************************************************************
 * Name: ble_phy_ccm_tx_start
 *
 * Description:
 *   Encrypt the PDU in src to dst.  dst is read by the radio while CCM
 *   writes it.
 *
 ****************************************************************************/

static void ble_phy_ccm_tx_start(FAR struct nrf54l_ble_dev_s *dev,
                                 FAR const uint8_t *src, FAR uint8_t *dst)
{
  FAR struct nrf54l_ble_ccm_job_s *job = &dev->ccm_job;
  uint8_t mlen = src[1];

  ble_phy_ccm_config(dev, CCM_MODE_ENCRYPTION | CCM_MODE_DATARATE_4MBIT);

  job->in_alen = 1;
  job->in_mlen = mlen;
  ble_phy_ccm_job(&job->in[0], &job->in_alen, 2, CCM_JOB_ATTR_ALEN);
  ble_phy_ccm_job(&job->in[1], &job->in_mlen, 2, CCM_JOB_ATTR_MLEN);
  ble_phy_ccm_job(&job->in[2], src, 1, CCM_JOB_ATTR_ADATA);
  ble_phy_ccm_job(&job->in[3], src + 3, mlen, CCM_JOB_ATTR_MDATA);
  ble_phy_ccm_job(&job->in[4], NULL, 0, 0);

  ble_phy_ccm_job(&job->out[0], &job->out_alen, 2, CCM_JOB_ATTR_ALEN);
  ble_phy_ccm_job(&job->out[1], dst + 1, 1, CCM_JOB_ATTR_MLEN);
  ble_phy_ccm_job(&job->out[2], &job->out_mlen_msb, 1, CCM_JOB_ATTR_MLEN);
  ble_phy_ccm_job(&job->out[3], dst, 1, CCM_JOB_ATTR_ADATA);
  ble_phy_ccm_job(&job->out[4], dst + 3, mlen + BLE_LL_DATA_MIC_LEN,
                  CCM_JOB_ATTR_MDATA);
  ble_phy_ccm_job(&job->out[5], NULL, 0, 0);

  putreg32((uint32_t)(uintptr_t)job->in, NRF54L_CCM_IN_PTR);
  putreg32((uint32_t)(uintptr_t)job->out, NRF54L_CCM_OUT_PTR);
  putreg32(0, NRF54L_CCM_SUBSCRIBE_START);
  putreg32(1, NRF54L_CCM_TASKS_START);
}
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
/****************************************************************************
 * Name: ble_phy_aar_setup
 *
 * Description:
 *   Enable AAR with the current IRK list.  The address is set and AAR is
 *   started by RADIO BCMATCH once the address is received.
 *
 ****************************************************************************/

static void ble_phy_aar_setup(FAR struct nrf54l_ble_dev_s *dev)
{
  FAR struct nrf54l_ble_aar_job_s *job = &dev->aar_job;
  int i;

  for (i = 0; i < dev->num_irks; i++)
    {
      job->in[2 + i].ptr  = (uint32_t)(uintptr_t)&dev->irk_list[4 * i];
      job->in[2 + i].attr = NRF54L_BLE_JOB_ATTR(16, AAR_JOB_ATTR_IRK);
    }

  job->in[2 + i].ptr  = 0;
  job->in[2 + i].attr = 0;

  job->status      = UINT32_MAX;
  job->out[0].ptr  = (uint32_t)(uintptr_t)&job->status;
  job->out[0].attr = NRF54L_BLE_JOB_ATTR(sizeof(job->status),
                                         AAR_JOB_ATTR_INDEX);

  putreg32(AAR_ENABLE_ENABLED, NRF54L_AAR_ENABLE);
  putreg32(1, NRF54L_AAR_MAXRESOLVED);
  putreg32((uint32_t)(uintptr_t)job->in, NRF54L_AAR_IN_PTR);
  putreg32((uint32_t)(uintptr_t)job->out, NRF54L_AAR_OUT_PTR);
  putreg32(NRF54L_BLE_DPPI_AAR | AAR_SUBSCRIBE_EN,
           NRF54L_AAR_SUBSCRIBE_START);
  putreg32(0, NRF54L_AAR_EVENTS_END);
  putreg32(0, NRF54L_AAR_EVENTS_RESOLVED);
  putreg32(0, NRF54L_AAR_EVENTS_NOTRESOLVED);
}
#endif

/****************************************************************************
 * Name: ble_phy_rx_xcvr_setup
 *
 * Description:
 *   Setup transceiver for receive.
 *
 ****************************************************************************/

static void ble_phy_rx_xcvr_setup(FAR struct nrf54l_ble_dev_s *dev)
{
  FAR uint8_t *dptr;

  dptr  = (FAR uint8_t *)&dev->rx_buf[0];
  dptr += 3;

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
  if (dev->phy.phy_encrypted)
    {
      putreg32((uint32_t)&dev->enc_buf[0], NRF54L_RADIO_PACKETPTR);
      ble_phy_ccm_rx_setup(dev, dptr);
      nrf54l_ble_ppi_enable(NRF54L_BLE_PPI_CCM);
    }
  else
#endif
    {
      putreg32((uint32_t)dptr, NRF54L_RADIO_PACKETPTR);
    }

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  if (dev->phy.phy_privacy)
    {
      ble_phy_aar_setup(dev);
    }
  else if (dev->phy.phy_encrypted == 0)
    {
      putreg32(AAR_ENABLE_DISABLED, NRF54L_AAR_ENABLE);
    }
#endif

  /* Turn off trigger TXEN on output compare match and AAR on bcmatch */

  nrf54l_ble_ppi_disable(NRF54L_BLE_PPI_TXEN | NRF54L_BLE_PPI_AAR);

  /* Reset the rx started flag.  Used for the wait for response */

  dev->phy.phy_rx_started = 0;
  dev->phy.phy_state      = BLE_PHY_STATE_RX;

#ifdef CONFIG_BLE_LL_PHY
  /* On Coded PHY there are CI and TERM1 fields before the PDU */

  if (dev->phy.phy_cur_phy_mode == BLE_PHY_MODE_CODED_125KBPS ||
      dev->phy.phy_cur_phy_mode == BLE_PHY_MODE_CODED_500KBPS)
    {
      dev->phy.phy_bcc_offset = 5;
    }
  else
#endif
    {
      dev->phy.phy_bcc_offset = 0;
    }

  /* We want to know when the 1st byte is received (after address) */

  putreg32(8 + dev->phy.phy_bcc_offset, NRF54L_RADIO_BCC);
  putreg32(0, NRF54L_RADIO_EVENTS_ADDRESS);
  putreg32(0, NRF54L_RADIO_EVENTS_DEVMATCH);
  putreg32(0, NRF54L_RADIO_EVENTS_BCMATCH);
  putreg32(0, NRF54L_RADIO_EVENTS_CRCOK);
  putreg32(RADIO_SHORTS_PHYEND_DISABLE | RADIO_SHORTS_READY_START |
           RADIO_SHORTS_ADDRESS_BCSTART | RADIO_SHORTS_ADDRESS_RSSISTART,
           NRF54L_RADIO_SHORTS);

  putreg32(RADIO_INT_ADDRESS | RADIO_INT_DISABLED, NRF54L_RADIO_INTENSET00);
}

/****************************************************************************
 * Name: ble_phy_tx_end_isr
 *
 * Description:
 *   Called from interrupt context when the transmit ends.
 *
 ****************************************************************************/

static void ble_phy_tx_end_isr(FAR struct nrf54l_ble_dev_s *dev)
{
  uint8_t tx_phy_mode;
  uint8_t transition;
  uint32_t rx_time;
  uint32_t tx_time;
  uint32_t radio_time;
  uint16_t tifs;

  /* Store PHY on which we've just transmitted */

  tx_phy_mode = dev->phy.phy_cur_phy_mode;

  DEBUGASSERT(dev->phy.phy_state == BLE_PHY_STATE_TX);

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
  /* HW error during an encrypted transmission is only counted */

  if (dev->phy.phy_encrypted && getreg32(NRF54L_CCM_EVENTS_ERROR))
    {
      NRF54L_BLE_STATS_INC(dev, tx_hw_err);
      putreg32(0, NRF54L_CCM_EVENTS_ERROR);
    }
#endif

  tifs       = BLE_LL_IFS;
  transition = dev->phy.phy_transition;

  if (dev->phy.txend_cb)
    {
      dev->phy.txend_cb(dev->phy.txend_arg);
    }

  if (transition == BLE_PHY_TRANSITION_TX_RX)
    {
#ifdef CONFIG_BLE_LL_PHY
      ble_phy_mode_apply(dev, dev->phy.phy_rx_phy_mode);
#endif

      /* Packet pointer needs to be reset */

      ble_phy_rx_xcvr_setup(dev);

      nrf54l_ble_phy_wfr_enable(&dev->lower, BLE_PHY_WFR_ENABLE_TXRX,
                                tx_phy_mode, 0);

      /* Schedule RX exactly T_IFS after TX end captured in CC[2], adjusted
       * for the delay between EVENT_END and actual TX end and started a bit
       * earlier due to allowed active clock accuracy.
       */

      rx_time  = timer_cc_get(2) + tifs;
      rx_time += g_ble_phy_t_txenddelay[tx_phy_mode];
      rx_time -= 2;

      radio_time = rx_time - BLE_PHY_T_RXENFAST;
      timer_event_clear(0);
      nrf54l_ble_ppi_enable(NRF54L_BLE_PPI_RXEN);
      timer_cc_set(0, radio_time);

      /* If TIMER10 already counted past CC[0], the wfr timer cancels the
       * reception.
       */
    }
  else if (transition == BLE_PHY_TRANSITION_TX_TX)
    {
      if (dev->phy.txtx_time_anchor)
        {
          /* TX anchor relative to current TX end captured in CC[2] */

          tx_time  = timer_cc_get(2);
          tx_time += g_ble_phy_t_txenddelay[tx_phy_mode];
        }
      else
        {
          /* TX anchor relative to current TX start: access address is
           * captured in CC[1].
           */

          tx_time  = timer_cc_get(1);
          tx_time += g_ble_phy_t_txaddrdelay[tx_phy_mode];
          tx_time -= ble_ll_pdu_syncword_us(tx_phy_mode);
        }

      tx_time += dev->phy.txtx_time_us;

      /* Adjust for delay between EVENT_READY and actual TX start time */

      tx_time -= g_ble_phy_t_txdelay[dev->phy.phy_cur_phy_mode];

      radio_time = tx_time - BLE_PHY_T_TXENFAST;
      timer_event_clear(0);
      nrf54l_ble_ppi_enable(NRF54L_BLE_PPI_TXEN);
      timer_cc_set(0, radio_time);

      if (timer_did_miss(0, 3))
        {
          nrf54l_ble_ppi_disable(NRF54L_BLE_PPI_TXEN);
          dev->phy.phy_transition_late = 1;
        }
    }
  else
    {
      timer_stop(dev);
      nrf54l_ble_ppi_disable(NRF54L_BLE_PPI_WFR | NRF54L_BLE_PPI_TXEN |
                             NRF54L_BLE_PPI_START);
      DEBUGASSERT(transition == BLE_PHY_TRANSITION_NONE);
    }
}

/****************************************************************************
 * Name: ble_phy_get_cur_rx_phy_mode
 ****************************************************************************/

static inline uint8_t
ble_phy_get_cur_rx_phy_mode(FAR struct nrf54l_ble_dev_s *dev)
{
  uint8_t phy;

  phy = dev->phy.phy_cur_phy_mode;

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_CODED_PHY
  /* For Coded PHY the actual coding of the received packet is given by
   * the CI field.
   */

  if (phy == BLE_PHY_MODE_CODED_125KBPS ||
      phy == BLE_PHY_MODE_CODED_500KBPS)
    {
      phy = (getreg32(NRF54L_RADIO_PDUSTAT) & RADIO_PDUSTAT_CISTAT_MASK) ?
            BLE_PHY_MODE_CODED_500KBPS : BLE_PHY_MODE_CODED_125KBPS;
    }
#endif

  return phy;
}

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
/****************************************************************************
 * Name: ble_phy_rx_decrypt
 *
 * Description:
 *   Finish decryption of the received PDU.  An empty PDU is not encrypted
 *   and has no MIC.
 *
 ****************************************************************************/

static void ble_phy_rx_decrypt(FAR struct nrf54l_ble_dev_s *dev,
                               FAR uint8_t *dptr,
                               FAR struct ble_mbuf_hdr *ble_hdr)
{
  FAR uint8_t *src = (FAR uint8_t *)dev->enc_buf;

  if (src[1] == 0)
    {
      putreg32(1, NRF54L_CCM_TASKS_STOP);
      dptr[0] = src[0];
      dptr[1] = 0;
      return;
    }

  /* Make sure CCM finished */

  while (getreg32(NRF54L_CCM_EVENTS_END) == 0)
    {
    }

  if ((getreg32(NRF54L_CCM_MACSTATUS) & CCM_MACSTATUS_PASSED) == 0)
    {
      ble_hdr->rxinfo.flags |= BLE_MBUF_HDR_F_MIC_FAILURE;
    }

  /* CCM error: treat as a CRC error so the PDU is not used */

  if (getreg32(NRF54L_CCM_EVENTS_ERROR))
    {
      NRF54L_BLE_STATS_INC(dev, rx_hw_err);
      ble_hdr->rxinfo.flags &= ~BLE_MBUF_HDR_F_CRC_OK;
    }
}
#endif

/****************************************************************************
 * Name: ble_phy_rx_end_isr
 ****************************************************************************/

static void ble_phy_rx_end_isr(FAR struct nrf54l_ble_dev_s *dev)
{
  FAR struct ble_mbuf_hdr *ble_hdr;
  FAR uint8_t *dptr;
  uint32_t radio_time;
  uint32_t tx_time;
  uint16_t tifs;
  uint8_t crcok;
  int rc;

  /* Disable automatic RXEN */

  nrf54l_ble_ppi_disable(NRF54L_BLE_PPI_RXEN);

  /* Set RSSI and CRC status flag in header */

  ble_hdr = &dev->phy.rxhdr;
  ble_hdr->rxinfo.rssi = -1 * (int)(getreg32(NRF54L_RADIO_RSSISAMPLE) &
                                    RADIO_RSSISAMPLE_MASK);

  dptr  = (FAR uint8_t *)&dev->rx_buf[0];
  dptr += 3;

  /* Count PHY crc errors and valid packets */

  crcok = getreg32(NRF54L_RADIO_EVENTS_CRCOK);
  if (!crcok)
    {
      NRF54L_BLE_STATS_INC(dev, rx_crc_err);
    }
  else
    {
      NRF54L_BLE_STATS_INC(dev, rx_valid);
      ble_hdr->rxinfo.flags |= BLE_MBUF_HDR_F_CRC_OK;

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
      if (dev->phy.phy_encrypted)
        {
          ble_phy_rx_decrypt(dev, dptr, ble_hdr);
        }
#endif
    }

#ifdef CONFIG_BLE_LL_PHY
  ble_phy_mode_apply(dev, dev->phy.phy_tx_phy_mode);
#endif

  /* Schedule TX now; it is cancelled after the PDU is processed if not
   * needed.  This leaves the radio ramp-up time for processing.
   */

  tifs = BLE_LL_IFS;

  /* Schedule TX exactly T_IFS after RX end captured in CC[2] */

  tx_time  = timer_cc_get(2) + tifs;
  tx_time -= g_ble_phy_t_rxenddelay[ble_hdr->rxinfo.phy_mode];

  /* Adjust for delay between EVENT_READY and actual TX start time */

  tx_time -= g_ble_phy_t_txdelay[dev->phy.phy_cur_phy_mode];

  radio_time = tx_time - BLE_PHY_T_TXENFAST;
  timer_event_clear(0);
  nrf54l_ble_ppi_enable(NRF54L_BLE_PPI_TXEN);
  timer_cc_set(0, radio_time);

  /* Check if TIMER10 did not already count past CC[0] */

  if (timer_did_miss(0, 3))
    {
      nrf54l_ble_ppi_disable(NRF54L_BLE_PPI_TXEN);
      dev->phy.phy_transition_late = 1;
    }

  /* The RAM S1 byte is not sent over the air but is present in the
   * buffer.  Move the header to remove it.
   */

  dptr[2] = dptr[1];
  dptr[1] = dptr[0];
  rc = dev->cb->rx_end(dev->cb, dptr + 1, ble_hdr);
  if (rc < 0)
    {
      nrf54l_ble_phy_disable(&dev->lower);
    }
}

/****************************************************************************
 * Name: ble_phy_rx_start_isr
 ****************************************************************************/

static bool ble_phy_rx_start_isr(FAR struct nrf54l_ble_dev_s *dev)
{
  FAR struct ble_mbuf_hdr *ble_hdr;
  FAR uint8_t *dptr;
  uint32_t usecs;
  uint32_t ticks;
  uint32_t state;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  FAR uint8_t *addr;
  int adva_offset;
#endif
  int rc;

  dptr = (FAR uint8_t *)&dev->rx_buf[0];

  /* Clear events and clear interrupt */

  putreg32(0, NRF54L_RADIO_EVENTS_ADDRESS);
  putreg32(RADIO_INT_ADDRESS, NRF54L_RADIO_INTENCLR00);

  /* Clear wfr timer channels */

  nrf54l_ble_ppi_disable(NRF54L_BLE_PPI_WFR);

  /* Initialize the PDU metadata */

  ble_hdr                   = &dev->phy.rxhdr;
  ble_hdr->rxinfo.flags     = dev->cb->state_get(dev->cb);
  ble_hdr->rxinfo.channel   = dev->phy.phy_chan;
  ble_hdr->rxinfo.handle    = 0;
  ble_hdr->rxinfo.phy       = ble_phy_get_cur_phy(dev);
  ble_hdr->rxinfo.phy_mode  = ble_phy_get_cur_rx_phy_mode(dev);
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV
  ble_hdr->rxinfo.user_data = NULL;
#endif

  /* Packet start time: TIMER10 started at phy_start_us and captured the
   * address event in CC[1].
   */

  nrf54l_ble_tmr_us2t(dev->phy.phy_start_us + timer_cc_get(1) -
                      ble_phy_mode_pdu_start_off(ble_hdr->rxinfo.phy_mode) -
                      g_ble_phy_t_rxaddrdelay[ble_hdr->rxinfo.phy_mode],
                      &ticks, &usecs);

  ble_hdr->beg_cputime = ticks;
  ble_hdr->rem_usecs   = usecs;

  /* Wait to get 1st byte of frame */

  while (1)
    {
      state = getreg32(NRF54L_RADIO_STATE);
      if (getreg32(NRF54L_RADIO_EVENTS_BCMATCH) != 0)
        {
          break;
        }

      /* If state is disabled, we should have the BCMATCH */

      if (state == RADIO_STATE_DISABLED)
        {
          putreg32(NRF_RADIO_IRQ_MASK_ALL, NRF54L_RADIO_INTENCLR00);
          putreg32(0, NRF54L_RADIO_SHORTS);
          return false;
        }
    }

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  /* If privacy is enabled and received PDU has TxAdd bit set (random
   * address) resolve the address using AAR.
   */

  if (dev->phy.phy_privacy && (dptr[3] & 0x40))
    {
      /* AdvA is located after the S0, length and S1 fields.  Extended
       * advertising PDUs have 2 more octets.
       */

      adva_offset = (dptr[3] & 0x0f) == 0x07 ? 2 : 0;
      addr        = dptr + 6 + adva_offset;

      dev->aar_job.in[0].ptr  = (uint32_t)(uintptr_t)addr;
      dev->aar_job.in[0].attr = NRF54L_BLE_JOB_ATTR(3, AAR_JOB_ATTR_HASH);
      dev->aar_job.in[1].ptr  = (uint32_t)(uintptr_t)(addr + 3);
      dev->aar_job.in[1].attr = NRF54L_BLE_JOB_ATTR(3, AAR_JOB_ATTR_PRAND);

      /* Trigger AAR after last bit of AdvA is received */

      putreg32(0, NRF54L_RADIO_EVENTS_BCMATCH);
      nrf54l_ble_ppi_enable(NRF54L_BLE_PPI_AAR);
      putreg32((BLE_LL_PDU_HDR_LEN + adva_offset + 6) * 8 +
               dev->phy.phy_bcc_offset, NRF54L_RADIO_BCC);
    }
#endif

  /* Call Link Layer receive start function */

  rc = dev->cb->rx_start(dev->cb, dptr + 3, dev->phy.phy_chan,
                         &dev->phy.rxhdr);
  if (rc >= 0)
    {
      /* Set rx started flag and enable rx end ISR */

      dev->phy.phy_rx_started = 1;
    }
  else
    {
      nrf54l_ble_phy_disable(&dev->lower);
      NRF54L_BLE_STATS_INC(dev, rx_aborts);
    }

  NRF54L_BLE_STATS_INC(dev, rx_starts);

  return true;
}

/****************************************************************************
 * Name: ble_phy_set_access_addr
 ****************************************************************************/

static int ble_phy_set_access_addr(FAR struct nrf54l_ble_dev_s *dev,
                                   uint32_t access_addr)
{
  putreg32(access_addr << 8, NRF54L_RADIO_BASE0);
  putreg32((getreg32(NRF54L_RADIO_PREFIX0) & 0xffffff00) |
           (access_addr >> 24), NRF54L_RADIO_PREFIX0);

  dev->phy.phy_access_address = access_addr;

  return 0;
}

/****************************************************************************
 * Name: ble_phy_disable_irq_and_ppi
 *
 * Description:
 *   Called when reception was stopped and the PHY is to be restarted or
 *   disabled.
 *
 ****************************************************************************/

static void ble_phy_disable_irq_and_ppi(FAR struct nrf54l_ble_dev_s *dev)
{
  int irq = NRF54L_IRQ_RADIO_0 - NRF54L_IRQ_EXTINT;

  putreg32(NRF_RADIO_IRQ_MASK_ALL, NRF54L_RADIO_INTENCLR00);
  putreg32(0, NRF54L_RADIO_SHORTS);
  putreg32(1, NRF54L_RADIO_TASKS_DISABLE);
  nrf54l_ble_ppi_disable(NRF54L_BLE_PPI_MASK_ALL);
  putreg32(1 << (irq & 0x1f), NVIC_IRQ_CLRPEND(irq));
  dev->phy.phy_state = BLE_PHY_STATE_IDLE;
}

/****************************************************************************
 * Name: ble_phy_rx
 *
 * Description:
 *   Puts the PHY into receive mode.
 *
 ****************************************************************************/

static int ble_phy_rx(FAR struct nrf54l_ble_dev_s *dev)
{
  uint32_t state;

  /* Wait if the radio is disabling.  The radio can ramp-up from the idle
   * state too.  TX and RX state values differ only in the 3rd bit.
   */

  nrf_wait_disabled();

  state = getreg32(NRF54L_RADIO_STATE);
  if ((state != RADIO_STATE_DISABLED) &&
      ((state & 0x07) != RADIO_STATE_RXIDLE))
    {
      nrf54l_ble_phy_disable(&dev->lower);
      NRF54L_BLE_STATS_INC(dev, radio_state_errs);
      return BLE_PHY_ERR_RADIO_STATE;
    }

  /* Make sure all interrupts are disabled */

  putreg32(NRF_RADIO_IRQ_MASK_ALL, NRF54L_RADIO_INTENCLR00);

  /* Clear events prior to enabling receive */

  putreg32(0, NRF54L_RADIO_EVENTS_END);
  putreg32(0, NRF54L_RADIO_EVENTS_DISABLED);

  ble_phy_rx_xcvr_setup(dev);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_ble_phy_isr
 ****************************************************************************/

int nrf54l_ble_phy_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)arg;
  uint32_t irq_en;

  /* Read irq register to determine which interrupts are enabled */

  irq_en = getreg32(NRF54L_RADIO_INTENSET00);

  /* NOTE: order of checking is important!  If things get delayed, we may
   * have both an ADDRESS and DISABLED interrupt in rx state.  If we get an
   * address, we disable the DISABLED interrupt.
   */

  if ((irq_en & RADIO_INT_ADDRESS) &&
      getreg32(NRF54L_RADIO_EVENTS_ADDRESS))
    {
      /* The wfr timer may fire together with EVENT_ADDRESS.  The radio is
       * then disabled while waiting for BCMATCH and ble_phy_rx_start_isr()
       * fails, so keep the DISABLED irq to handle it below.
       */

      if (ble_phy_rx_start_isr(dev))
        {
          irq_en &= ~RADIO_INT_DISABLED;
        }
    }

  /* Handle disabled event.  On RX, phy_rx_started tells if we were
   * receiving a PDU, otherwise this is due to wfr.
   */

  if ((irq_en & RADIO_INT_DISABLED) &&
      getreg32(NRF54L_RADIO_EVENTS_DISABLED))
    {
      DEBUGASSERT(getreg32(NRF54L_RADIO_EVENTS_END) ||
                  ((dev->phy.phy_state == BLE_PHY_STATE_RX) &&
                   !dev->phy.phy_rx_started));

      putreg32(0, NRF54L_RADIO_EVENTS_END);
      putreg32(0, NRF54L_RADIO_EVENTS_DISABLED);
      putreg32(RADIO_INT_DISABLED, NRF54L_RADIO_INTENCLR00);

      switch (dev->phy.phy_state)
        {
          case BLE_PHY_STATE_RX:
            if (dev->phy.phy_rx_started)
              {
                ble_phy_rx_end_isr(dev);
              }
            else
              {
                dev->cb->wfr_exp(dev->cb);
              }
            break;

          case BLE_PHY_STATE_TX:
            ble_phy_tx_end_isr(dev);
            break;

          default:
            DEBUGPANIC();
        }
    }

  dev->phy.phy_transition_late = 0;

  NRF54L_BLE_STATS_INC(dev, phy_isrs);

  return OK;
}

/****************************************************************************
 * Name: nrf54l_ble_phy_init
 *
 * Description:
 *   Initialize the PHY.
 *
 ****************************************************************************/

int nrf54l_ble_phy_init(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  /* Default phy to use is 1M */

  dev->phy.phy_cur_phy_mode = BLE_PHY_MODE_1M;
  dev->phy.phy_tx_phy_mode  = BLE_PHY_MODE_1M;
  dev->phy.phy_rx_phy_mode  = BLE_PHY_MODE_1M;

  /* Set phy channel to an invalid channel so first set channel works */

  dev->phy.phy_chan = BLE_PHY_NUM_CHANS;

  /* Disable all interrupts and reset the radio.  SOFTRESET keeps the
   * PUBLISH and SUBSCRIBE registers and must be used in DISABLED state.
   */

  putreg32(NRF_RADIO_IRQ_MASK_ALL, NRF54L_RADIO_INTENCLR00);
  putreg32(0, NRF54L_RADIO_SHORTS);
  putreg32(1, NRF54L_RADIO_TASKS_DISABLE);
  while (getreg32(NRF54L_RADIO_STATE) != RADIO_STATE_DISABLED)
    {
    }

  putreg32(1, NRF54L_RADIO_TASKS_SOFTRESET);

  /* Set configuration registers */

  putreg32(RADIO_MODE_BLE1MBIT, NRF54L_RADIO_MODE);
  putreg32(NRF_PCNF0_1M, NRF54L_RADIO_PCNF0);
  putreg32(NRF_MAXLEN | RADIO_PCNF1_ENDIAN_LITTLE |
           (NRF_BALEN << RADIO_PCNF1_BALEN_SHIFT) | RADIO_PCNF1_WHITEEN,
           NRF54L_RADIO_PCNF1);

  /* Enable radio fast ramp-up */

  putreg32(RADIO_TIMING_RU, NRF54L_RADIO_TIMING);

  /* Set logical address 0 for TX and RX */

  putreg32(0, NRF54L_RADIO_TXADDRESS);
  putreg32(RADIO_RXADDRESSES_ADDR(0), NRF54L_RADIO_RXADDRESSES);

  /* Configure the CRC registers */

  putreg32(RADIO_CRCCNF_SKIPADDR_SKIP | RADIO_CRCCNF_LEN_3,
           NRF54L_RADIO_CRCCNF);
  putreg32(0x0000065b, NRF54L_RADIO_CRCPOLY);

  /* Configure IFS */

  putreg32(BLE_LL_IFS, NRF54L_RADIO_TIFS);

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
  putreg32(0xffffffff, NRF54L_CCM_INTENCLR);
  putreg32(0, NRF54L_CCM_EVENTS_ERROR);
  dev->ccm.hdr_mask = BLE_LL_PDU_HEADERMASK_DATA;
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  putreg32(0xffffffff, NRF54L_AAR_INTENCLR);
  putreg32(0, NRF54L_AAR_EVENTS_END);
  putreg32(0, NRF54L_AAR_EVENTS_RESOLVED);
  putreg32(0, NRF54L_AAR_EVENTS_NOTRESOLVED);
#endif

  /* TIMER10: 32-bit timer at 1 MHz */

  putreg32(1, NRF54L_BLE_TIMER(NRF54L_TIM_TASKS_STOP_OFFSET));
  putreg32(TIM_BITMODE_32B, NRF54L_BLE_TIMER(NRF54L_TIM_BITMODE_OFFSET));
  putreg32(TIM_MODE_TIMER, NRF54L_BLE_TIMER(NRF54L_TIM_MODE_OFFSET));
  putreg32(NRF54L_BLE_TIMER_PRESCALER,
           NRF54L_BLE_TIMER(NRF54L_TIM_PRESCALER_OFFSET));

  phy_ppi_init();

  up_enable_irq(NRF54L_IRQ_RADIO_0);

  return 0;
}

/****************************************************************************
 * Name: nrf54l_ble_phy_wfr_enable
 *
 * Description:
 *   Set DPPI so that we can time out waiting for a reception.  This
 *   happens after a transmission when waiting for a response (txrx is
 *   BLE_PHY_WFR_ENABLE_TXRX) or when waiting for the first packet of a
 *   connection event as a peripheral (txrx is BLE_PHY_WFR_ENABLE_RX).
 *
 *   wfr_usecs is not used for a txrx turn-around since we know when the
 *   address of the received frame is expected.
 *
 ****************************************************************************/

void nrf54l_ble_phy_wfr_enable(FAR struct ble_ll_lower_s *lower, int txrx,
                               uint8_t tx_phy_mode, uint32_t wfr_usecs)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;
  uint32_t end_time;
  uint8_t phy;

  phy = dev->phy.phy_cur_phy_mode;

  if (txrx == BLE_PHY_WFR_ENABLE_TXRX)
    {
      /* RX shall start exactly T_IFS after TX end captured in CC[2].  Wait
       * 2 usec longer due to allowed active clock accuracy and 1 usec more
       * since the PDU start may be captured at the end of a timer cycle.
       */

      end_time  = timer_cc_get(2) + BLE_LL_IFS;
      end_time += g_ble_phy_t_txenddelay[tx_phy_mode];
      end_time += 2;
      end_time += 1;
    }
  else
    {
      /* RX shall start no later than wfr_usecs after RX enabled.  CC[0] is
       * the time of RXEN so adjust for radio ramp-up.
       */

      end_time = timer_cc_get(0) + BLE_PHY_T_RXENFAST + wfr_usecs;
    }

  /* Adjust for receiving access address since this triggers EVENT_ADDRESS
   * and for the delay between actual access address RX and EVENT_ADDRESS.
   */

  end_time += ble_phy_mode_pdu_start_off(phy);
  end_time += g_ble_phy_t_rxaddrdelay[phy];

  /* Enable wait for response DPPI */

  timer_event_clear(3);
  nrf54l_ble_ppi_enable(NRF54L_BLE_PPI_WFR);
  timer_cc_set(3, end_time);

  /* If the CPU was halted for a moment, TIMER10 may already have counted
   * past CC[3].  Disable the radio manually in that case.  CC[1] is only
   * used as a reference on RX start so it is used as scratch here.
   */

  if (timer_did_miss(3, 1))
    {
      nrf54l_ble_ppi_disable(NRF54L_BLE_PPI_WFR);
      putreg32(1, NRF54L_RADIO_TASKS_DISABLE);
    }
}

#ifdef CONFIG_BLE_LL_PHY
/****************************************************************************
 * Name: nrf54l_ble_phy_mode_set
 ****************************************************************************/

void nrf54l_ble_phy_mode_set(FAR struct ble_ll_lower_s *lower,
                             uint8_t tx_phy_mode, uint8_t rx_phy_mode)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  dev->phy.phy_tx_phy_mode = tx_phy_mode;
  dev->phy.phy_rx_phy_mode = rx_phy_mode;
}
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
/****************************************************************************
 * Name: nrf54l_ble_phy_encrypt_enable
 ****************************************************************************/

void nrf54l_ble_phy_encrypt_enable(FAR struct ble_ll_lower_s *lower,
                                   FAR const uint8_t *key)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  memcpy(dev->ccm.key, key, 16);
  dev->phy.phy_encrypted = 1;
  putreg32(AAR_ENABLE_DISABLED, NRF54L_AAR_ENABLE);
  putreg32(CCM_ENABLE_ENABLED, NRF54L_CCM_ENABLE);
}

/****************************************************************************
 * Name: nrf54l_ble_phy_encrypt_header_mask_set
 ****************************************************************************/

void
nrf54l_ble_phy_encrypt_header_mask_set(FAR struct ble_ll_lower_s *lower,
                                       uint8_t mask)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  dev->ccm.hdr_mask = mask;
}

/****************************************************************************
 * Name: nrf54l_ble_phy_encrypt_iv_set
 ****************************************************************************/

void nrf54l_ble_phy_encrypt_iv_set(FAR struct ble_ll_lower_s *lower,
                                   FAR const uint8_t *iv)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  memcpy(dev->ccm.iv, iv, 8);
}

/****************************************************************************
 * Name: nrf54l_ble_phy_encrypt_counter_set
 ****************************************************************************/

void nrf54l_ble_phy_encrypt_counter_set(FAR struct ble_ll_lower_s *lower,
                                        uint64_t counter, uint8_t dir_bit)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  dev->ccm.pkt_counter = counter;
  dev->ccm.dir_bit     = dir_bit;
}

/****************************************************************************
 * Name: nrf54l_ble_phy_encrypt_disable
 ****************************************************************************/

void nrf54l_ble_phy_encrypt_disable(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  nrf54l_ble_ppi_disable(NRF54L_BLE_PPI_CCM);
  putreg32(1, NRF54L_CCM_TASKS_STOP);
  putreg32(0, NRF54L_CCM_EVENTS_ERROR);
  putreg32(CCM_ENABLE_DISABLED, NRF54L_CCM_ENABLE);

  dev->phy.phy_encrypted = 0;
}
#endif

/****************************************************************************
 * Name: nrf54l_ble_phy_set_txend_cb
 ****************************************************************************/

void nrf54l_ble_phy_set_txend_cb(FAR struct ble_ll_lower_s *lower,
                                 ble_phy_tx_end_func txend_cb,
                                 FAR void *arg)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  dev->phy.txend_cb  = txend_cb;
  dev->phy.txend_arg = arg;
}

/****************************************************************************
 * Name: nrf54l_ble_phy_tx_set_start_time
 *
 * Description:
 *   Set the start time of a transmission when we are not going from rx to
 *   tx automatically.  The channel must already be set.
 *
 * Input Parameters:
 *   cputime   - Tick at which the 1st bit of the preamble is transmitted
 *   rem_usecs - Microseconds from the cputime tick
 *
 ****************************************************************************/

int nrf54l_ble_phy_tx_set_start_time(FAR struct ble_ll_lower_s *lower,
                                     uint32_t cputime, uint8_t rem_usecs)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;
  int rc;

#ifdef CONFIG_BLE_LL_PHY
  ble_phy_mode_apply(dev, dev->phy.phy_tx_phy_mode);
#endif

  /* Clear timer compare to RXEN since we are transmitting */

  nrf54l_ble_ppi_disable(NRF54L_BLE_PPI_RXEN);

  if (ble_phy_set_start_time(dev, cputime, rem_usecs, true) != 0)
    {
      NRF54L_BLE_STATS_INC(dev, tx_late);
      nrf54l_ble_phy_disable(lower);
      rc = BLE_PHY_ERR_TX_LATE;
    }
  else
    {
      /* Enable DPPI to automatically start TXEN */

      nrf54l_ble_ppi_enable(NRF54L_BLE_PPI_TXEN);
      rc = 0;
    }

  return rc;
}

/****************************************************************************
 * Name: nrf54l_ble_phy_rx_set_start_time
 *
 * Description:
 *   Set the start time of a reception.  If we are late, we still attempt
 *   to receive.  The channel must already be set.
 *
 ****************************************************************************/

int nrf54l_ble_phy_rx_set_start_time(FAR struct ble_ll_lower_s *lower,
                                     uint32_t cputime, uint8_t rem_usecs)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;
  bool late = false;
  int rc;

#ifdef CONFIG_BLE_LL_PHY
  ble_phy_mode_apply(dev, dev->phy.phy_rx_phy_mode);
#endif

  /* Clear timer compare to TXEN since we are receiving */

  nrf54l_ble_ppi_disable(NRF54L_BLE_PPI_TXEN);

  if (ble_phy_set_start_time(dev, cputime, rem_usecs, false) != 0)
    {
      NRF54L_BLE_STATS_INC(dev, rx_late);

      /* We're late so let's just try to start RX as soon as possible */

      ble_phy_set_start_now(dev);
      late = true;
    }

  /* Enable DPPI to automatically start RXEN */

  nrf54l_ble_ppi_enable(NRF54L_BLE_PPI_RXEN);

  rc = ble_phy_rx(dev);

  /* If we enabled the receiver but were late, return a proper error code
   * so the caller can handle this.
   */

  if (!rc && late)
    {
      rc = BLE_PHY_ERR_RX_LATE;
    }

  return rc;
}

/****************************************************************************
 * Name: nrf54l_ble_phy_tx
 ****************************************************************************/

int nrf54l_ble_phy_tx(FAR struct ble_ll_lower_s *lower,
                      ble_phy_tx_pducb_t pducb, FAR void *pducb_arg,
                      uint8_t end_trans)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;
  FAR uint8_t *pktptr;
  FAR uint8_t *dptr;
  uint8_t payload_len;
  uint8_t hdr_byte;
  uint32_t state;
  int rc;

  if (dev->phy.phy_transition_late)
    {
      nrf54l_ble_phy_disable(lower);
      NRF54L_BLE_STATS_INC(dev, tx_late);
      return BLE_PHY_ERR_TX_LATE;
    }

  /* Make sure the radio is not moving to the disabled state */

  nrf_wait_disabled();

  /* Clear all DPPI connections that should not be used when transmitting */

  nrf54l_ble_ppi_disable(NRF54L_BLE_PPI_WFR | NRF54L_BLE_PPI_AAR |
                         NRF54L_BLE_PPI_CCM);

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
  if (dev->phy.phy_encrypted)
    {
      dptr   = (FAR uint8_t *)&dev->enc_buf[0];
      pktptr = (FAR uint8_t *)&dev->tx_buf[0];
    }
  else
#endif
    {
      dptr   = (FAR uint8_t *)&dev->tx_buf[0];
      pktptr = dptr;
    }

  /* Set PDU payload */

  payload_len = pducb(&dptr[3], pducb_arg, &hdr_byte);

  /* RAM representation has S0, LENGTH and S1 fields (3 bytes) */

  dptr[0] = hdr_byte;
  dptr[1] = payload_len;
  dptr[2] = 0;

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
  /* Encrypt into the radio buffer.  An empty PDU is sent as is. */

  if (dev->phy.phy_encrypted)
    {
      pktptr[2] = 0;
      if (payload_len != 0)
        {
          ble_phy_ccm_tx_start(dev, dptr, pktptr);
        }
      else
        {
          pktptr[0] = dptr[0];
          pktptr[1] = 0;
        }
    }
#endif

  putreg32((uint32_t)pktptr, NRF54L_RADIO_PACKETPTR);

  /* Clear the ready, end and disabled events */

  putreg32(0, NRF54L_RADIO_EVENTS_READY);
  putreg32(0, NRF54L_RADIO_EVENTS_END);
  putreg32(0, NRF54L_RADIO_EVENTS_DISABLED);

  /* Enable shortcuts for transmit start/end */

  putreg32(RADIO_SHORTS_PHYEND_DISABLE | RADIO_SHORTS_READY_START,
           NRF54L_RADIO_SHORTS);
  putreg32(RADIO_INT_DISABLED, NRF54L_RADIO_INTENSET00);

  /* Set the PHY transition and transmitted payload length */

  dev->phy.phy_transition  = end_trans;
  dev->phy.phy_tx_pyld_len = payload_len;

  /* If we already started transmitting, abort it! */

  state = getreg32(NRF54L_RADIO_STATE);
  if (state != RADIO_STATE_TX)
    {
      dev->phy.phy_state = BLE_PHY_STATE_TX;
      NRF54L_BLE_STATS_INC(dev, tx_good);
      NRF54L_BLE_STATS_INCN(dev, tx_bytes,
                            payload_len + BLE_LL_PDU_HDR_LEN);
      rc = 0;
    }
  else
    {
      nrf54l_ble_phy_disable(lower);
      NRF54L_BLE_STATS_INC(dev, tx_late);
      rc = BLE_PHY_ERR_RADIO_STATE;
    }

  return rc;
}

/****************************************************************************
 * Name: nrf54l_ble_phy_tx_power_set
 ****************************************************************************/

int nrf54l_ble_phy_tx_power_set(FAR struct ble_ll_lower_s *lower, int dbm)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;
  int i;

  /* Get actual TX power supported by radio */

  i = phy_txpower_round(dbm);

  putreg32(g_ble_phy_txpower_reg[i], NRF54L_RADIO_TXPOWER);
  dev->phy.phy_txpwr_dbm = g_ble_phy_txpower_dbm[i];

  return 0;
}

/****************************************************************************
 * Name: nrf54l_ble_phy_tx_power_round
 ****************************************************************************/

int nrf54l_ble_phy_tx_power_round(FAR struct ble_ll_lower_s *lower, int dbm)
{
  return g_ble_phy_txpower_dbm[phy_txpower_round(dbm)];
}

/****************************************************************************
 * Name: nrf54l_ble_phy_tx_power_get
 ****************************************************************************/

int nrf54l_ble_phy_tx_power_get(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  return dev->phy.phy_txpwr_dbm;
}

/****************************************************************************
 * Name: nrf54l_ble_phy_setchan
 *
 * Description:
 *   Set the logical frequency of the transceiver.  The FREQUENCY register
 *   gives the frequency as 2400 + FREQ MHz.
 *
 * Input Parameters:
 *   chan - Data Channel Index or Advertising Channel index
 *
 ****************************************************************************/

int nrf54l_ble_phy_setchan(FAR struct ble_ll_lower_s *lower, uint8_t chan,
                           uint32_t access_addr, uint32_t crcinit)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  DEBUGASSERT(chan < BLE_PHY_NUM_CHANS);

  if (chan >= BLE_PHY_NUM_CHANS)
    {
      return BLE_PHY_ERR_INV_PARAM;
    }

  /* Set current access address */

  ble_phy_set_access_addr(dev, access_addr);

  /* Configure crcinit */

  putreg32(crcinit, NRF54L_RADIO_CRCINIT);

  /* Set the frequency and the data whitening initial value */

  dev->phy.phy_chan = chan;
  putreg32(g_ble_phy_chan_freq[chan], NRF54L_RADIO_FREQUENCY);
  putreg32(NRF_DATAWHITE(chan), NRF54L_RADIO_DATAWHITE);

  return 0;
}

/****************************************************************************
 * Name: nrf54l_ble_phy_chan_get
 ****************************************************************************/

uint8_t nrf54l_ble_phy_chan_get(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  return dev->phy.phy_chan;
}

/****************************************************************************
 * Name: nrf54l_ble_phy_restart_rx
 ****************************************************************************/

void nrf54l_ble_phy_restart_rx(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  timer_stop(dev);
  ble_phy_disable_irq_and_ppi(dev);

  ble_phy_set_start_now(dev);

  /* Enable DPPI to automatically start RXEN */

  nrf54l_ble_ppi_enable(NRF54L_BLE_PPI_RXEN);

  ble_phy_rx(dev);
}

/****************************************************************************
 * Name: nrf54l_ble_phy_disable
 *
 * Description:
 *   Disable the PHY.  This stops the usec timer, disables interrupts, the
 *   RADIO and DPPI and sets state to idle.
 *
 ****************************************************************************/

void nrf54l_ble_phy_disable(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  timer_stop(dev);
  ble_phy_disable_irq_and_ppi(dev);

  dev->phy.phy_transition_late = 0;
}

/****************************************************************************
 * Name: nrf54l_ble_phy_access_addr_get
 ****************************************************************************/

uint32_t nrf54l_ble_phy_access_addr_get(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  return dev->phy.phy_access_address;
}

/****************************************************************************
 * Name: nrf54l_ble_phy_state_get
 ****************************************************************************/

int nrf54l_ble_phy_state_get(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  return dev->phy.phy_state;
}

/****************************************************************************
 * Name: nrf54l_ble_phy_rx_started
 ****************************************************************************/

int nrf54l_ble_phy_rx_started(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  return dev->phy.phy_rx_started;
}

/****************************************************************************
 * Name: nrf54l_ble_phy_xcvr_state_get
 ****************************************************************************/

uint8_t nrf54l_ble_phy_xcvr_state_get(FAR struct ble_ll_lower_s *lower)
{
  return (uint8_t)getreg32(NRF54L_RADIO_STATE);
}

/****************************************************************************
 * Name: nrf54l_ble_phy_max_data_pdu_pyld
 ****************************************************************************/

uint8_t nrf54l_ble_phy_max_data_pdu_pyld(FAR struct ble_ll_lower_s *lower)
{
  return BLE_LL_DATA_PDU_MAX_PYLD;
}

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
/****************************************************************************
 * Name: nrf54l_ble_phy_resolv_list_enable
 ****************************************************************************/

void nrf54l_ble_phy_resolv_list_enable(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  dev->phy.phy_privacy = 1;
}

/****************************************************************************
 * Name: nrf54l_ble_phy_resolv_list_disable
 ****************************************************************************/

void nrf54l_ble_phy_resolv_list_disable(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  dev->phy.phy_privacy = 0;
}
#endif

/****************************************************************************
 * Name: nrf54l_ble_phy_rfclk_enable
 *
 * Description:
 *   HFXO is started at boot and kept running.
 *
 ****************************************************************************/

void nrf54l_ble_phy_rfclk_enable(FAR struct ble_ll_lower_s *lower)
{
}

/****************************************************************************
 * Name: nrf54l_ble_phy_rfclk_disable
 ****************************************************************************/

void nrf54l_ble_phy_rfclk_disable(FAR struct ble_ll_lower_s *lower)
{
}

/****************************************************************************
 * Name: nrf54l_ble_phy_tifs_txtx_set
 ****************************************************************************/

void nrf54l_ble_phy_tifs_txtx_set(FAR struct ble_ll_lower_s *lower,
                                  uint16_t usecs, uint8_t anchor)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  dev->phy.txtx_time_us     = usecs;
  dev->phy.txtx_time_anchor = anchor;
}
