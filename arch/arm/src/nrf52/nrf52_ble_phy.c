/****************************************************************************
 * arch/arm/src/nrf52/nrf52_ble_phy.c
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
 * nimble/drivers/nrf5x/src/nrf52/phy.c).
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
#include "nrf52_clockconfig.h"
#include "nrf52_ble_priv.h"
#include "hardware/nrf52_aar.h"
#include "hardware/nrf52_ccm.h"
#include "hardware/nrf52_radio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* To disable all radio interrupts */

#define NRF_RADIO_IRQ_MASK_ALL  (0x34ff)

/* Radio ramp-up times in usecs (fast mode) */

#define BLE_PHY_T_TXENFAST      (40)
#define BLE_PHY_T_RXENFAST      (40)

/* We configure the radio with a 1 byte S0 field, 8 bit length field and
 * zero bit S1 field.
 */

#define NRF_LFLEN_BITS          (8)
#define NRF_S0LEN               (1)
#define NRF_S1LEN_BITS          (0)
#define NRF_CILEN_BITS          (2)
#define NRF_TERMLEN_BITS        (3)

/* Maximum length of frames */

#define NRF_MAXLEN              (255)
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

/* Radio TX power settings */

#define NRF_TXPOWER_POS4DBM     (4)
#define NRF_TXPOWER_POS3DBM     (3)
#define NRF_TXPOWER_0DBM        (0)
#define NRF_TXPOWER_NEG4DBM     (-4)
#define NRF_TXPOWER_NEG8DBM     (-8)
#define NRF_TXPOWER_NEG12DBM    (-12)
#define NRF_TXPOWER_NEG16DBM    (-16)
#define NRF_TXPOWER_NEG20DBM    (-20)
#define NRF_TXPOWER_NEG40DBM    (-40)

/* Chip variant registers used for errata detection */

#define NRF52_ERRATA_VAR1       (0x10000130)
#define NRF52_ERRATA_VAR2       (0x10000134)
#define NRF52_ERRATA_VAR1_ALT   (0xf0000fe0)
#define NRF52_ERRATA_VAR2_ALT   (0xf0000fe8)

/* Undocumented RADIO register used by errata 191 workaround */

#define NRF52_RADIO_ERRATA191   (NRF52_RADIO_BASE + 0x740)

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
 * Name: nrf52_ble_errata_102_106_107
 *
 * Description:
 *   RADIO errata 102, 106 and 107 (nRF52832): PAYLOAD/END events delayed,
 *   higher CRC error rates and immediate address match for some access
 *   addresses.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_NRF52832
static bool nrf52_ble_errata_102_106_107(void)
{
  uint32_t var1;
  uint32_t var2;

  if (getreg32(NRF52_ERRATA_VAR1) == 0xffffffff)
    {
      var1 = getreg32(NRF52_ERRATA_VAR1_ALT) & 0xff;
      var2 = (getreg32(NRF52_ERRATA_VAR2_ALT) & 0xf0) >> 4;
    }
  else
    {
      var1 = getreg32(NRF52_ERRATA_VAR1);
      var2 = getreg32(NRF52_ERRATA_VAR2);
    }

  return var1 == 0x06 && (var2 == 0x04 || var2 == 0x05);
}
#endif

/****************************************************************************
 * Name: nrf52_ble_errata_191
 *
 * Description:
 *   RADIO errata 191 (nRF52840): high packet error rate in BLE Long Range
 *   mode.
 *
 ****************************************************************************/

#if defined(CONFIG_BLE_LL_CFG_FEAT_LE_CODED_PHY) && \
    defined(CONFIG_ARCH_CHIP_NRF52840)
static bool nrf52_ble_errata_191(void)
{
  return getreg32(NRF52_ERRATA_VAR1) == 0x08 &&
         getreg32(NRF52_ERRATA_VAR2) != 0x00;
}

static bool ble_phy_mode_is_coded(uint8_t phy_mode)
{
  return phy_mode == BLE_PHY_MODE_CODED_125KBPS ||
         phy_mode == BLE_PHY_MODE_CODED_500KBPS;
}

static void nrf52_ble_errata_191_apply(FAR struct nrf52_ble_dev_s *dev,
                                       uint8_t new_phy_mode)
{
  bool from_coded = ble_phy_mode_is_coded(dev->phy.phy_cur_phy_mode);
  bool to_coded = ble_phy_mode_is_coded(new_phy_mode);

  /* Apply only when switching to or from LE Coded */

  if (from_coded == to_coded)
    {
      return;
    }

  if (to_coded)
    {
      modifyreg32(NRF52_RADIO_ERRATA191, 0x8000ff00,
                  0x80000000 | (196 << 8));
    }
  else
    {
      modifyreg32(NRF52_RADIO_ERRATA191, 0x80000000, 0);
    }
}
#endif

/****************************************************************************
 * Name: timer0_reset
 ****************************************************************************/

static void timer0_reset(void)
{
  /* Reset CC registers to avoid triggering spurious COMPARE events */

  putreg32(0, NRF52_BLE_TIMER0(NRF52_TIM_CC_OFFSET(0)));
  putreg32(0, NRF52_BLE_TIMER0(NRF52_TIM_CC_OFFSET(1)));
  putreg32(0, NRF52_BLE_TIMER0(NRF52_TIM_CC_OFFSET(2)));
  putreg32(0, NRF52_BLE_TIMER0(NRF52_TIM_CC_OFFSET(3)));
}

/****************************************************************************
 * Name: timer0_cc_set
 ****************************************************************************/

static inline void timer0_cc_set(int cc, uint32_t value)
{
  putreg32(value, NRF52_BLE_TIMER0(NRF52_TIM_CC_OFFSET(cc)));
}

/****************************************************************************
 * Name: timer0_cc_get
 ****************************************************************************/

static inline uint32_t timer0_cc_get(int cc)
{
  return getreg32(NRF52_BLE_TIMER0(NRF52_TIM_CC_OFFSET(cc)));
}

/****************************************************************************
 * Name: timer0_event_clear
 ****************************************************************************/

static inline void timer0_event_clear(int cc)
{
  putreg32(0, NRF52_BLE_TIMER0(NRF52_TIM_EVENTS_COMPARE_OFFSET(cc)));
}

/****************************************************************************
 * Name: timer0_did_miss
 *
 * Description:
 *   Check if TIMER0 already counted past CC[cc] without generating the
 *   COMPARE event.  CC[cc_scratch] is used to capture the current value.
 *
 ****************************************************************************/

static bool timer0_did_miss(int cc, int cc_scratch)
{
  uint32_t target;
  uint32_t now;

  putreg32(1, NRF52_BLE_TIMER0(NRF52_TIM_TASKS_CAPTURE_OFFSET(cc_scratch)));
  target = timer0_cc_get(cc);
  now    = timer0_cc_get(cc_scratch);

  /* COMPARE event is not triggered when CC is set to the current value of
   * TIMER0 (at the time it was set), so the case of equal CC values without
   * a COMPARE event should be also considered a miss here.
   */

  return (now >= target) &&
         !getreg32(NRF52_BLE_TIMER0(NRF52_TIM_EVENTS_COMPARE_OFFSET(cc)));
}

/****************************************************************************
 * Name: phy_ppi_init
 ****************************************************************************/

static void phy_ppi_init(void)
{
  /* RADIO ADDRESS -> TIMER0 CAPTURE[1] and RADIO END -> TIMER0 CAPTURE[2]
   * are always enabled.
   */

  nrf52_ble_ppi_enable(PPI_CHEN_CH(NRF52_BLE_PPI_ADDR_CAPTURE) |
                       PPI_CHEN_CH(NRF52_BLE_PPI_END_CAPTURE));

  /* Wait for response timer: capture TIMER0 in CC[3] on ADDRESS to cancel
   * the timer, disable the radio on TIMER0 COMPARE[3].
   */

  putreg32(NRF52_RADIO_EVENTS_ADDRESS, NRF52_PPI_CHEEP(NRF52_BLE_PPI_WFR0));
  putreg32(NRF52_BLE_TIMER0(NRF52_TIM_TASKS_CAPTURE_OFFSET(3)),
           NRF52_PPI_CHTEP(NRF52_BLE_PPI_WFR0));
  putreg32(NRF52_BLE_TIMER0(NRF52_TIM_EVENTS_COMPARE_OFFSET(3)),
           NRF52_PPI_CHEEP(NRF52_BLE_PPI_WFR1));
  putreg32(NRF52_RADIO_TASKS_DISABLE, NRF52_PPI_CHTEP(NRF52_BLE_PPI_WFR1));
}

/****************************************************************************
 * Name: phy_ppi_wfr_enable/disable
 ****************************************************************************/

static inline void phy_ppi_wfr_enable(void)
{
  nrf52_ble_ppi_enable(PPI_CHEN_CH(NRF52_BLE_PPI_WFR0) |
                       PPI_CHEN_CH(NRF52_BLE_PPI_WFR1));
}

static inline void phy_ppi_wfr_disable(void)
{
  nrf52_ble_ppi_disable(PPI_CHEN_CH(NRF52_BLE_PPI_WFR0) |
                        PPI_CHEN_CH(NRF52_BLE_PPI_WFR1));
}

/****************************************************************************
 * Name: phy_txpower_round
 ****************************************************************************/

static int8_t phy_txpower_round(int8_t dbm)
{
  static const int8_t levels[] =
  {
    NRF_TXPOWER_POS4DBM, NRF_TXPOWER_POS3DBM, NRF_TXPOWER_0DBM,
    NRF_TXPOWER_NEG4DBM, NRF_TXPOWER_NEG8DBM, NRF_TXPOWER_NEG12DBM,
    NRF_TXPOWER_NEG16DBM, NRF_TXPOWER_NEG20DBM
  };

  int i;

  for (i = 0; i < nitems(levels); i++)
    {
      if (dbm >= levels[i])
        {
          return levels[i];
        }
    }

  return NRF_TXPOWER_NEG40DBM;
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

static void ble_phy_mode_apply(FAR struct nrf52_ble_dev_s *dev,
                               uint8_t phy_mode)
{
  if (phy_mode == dev->phy.phy_cur_phy_mode)
    {
      return;
    }

#if defined(CONFIG_BLE_LL_CFG_FEAT_LE_CODED_PHY) && \
    defined(CONFIG_ARCH_CHIP_NRF52840)
  if (nrf52_ble_errata_191())
    {
      nrf52_ble_errata_191_apply(dev, phy_mode);
    }
#endif

  switch (phy_mode)
    {
      case BLE_PHY_MODE_1M:
        putreg32(RADIO_MODE_BLE1MBIT, NRF52_RADIO_MODE);
        putreg32(NRF_PCNF0_1M, NRF52_RADIO_PCNF0);
        break;

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_2M_PHY
      case BLE_PHY_MODE_2M:
        putreg32(RADIO_MODE_BLE2MBIT, NRF52_RADIO_MODE);
        putreg32(NRF_PCNF0_2M, NRF52_RADIO_PCNF0);
        break;
#endif

#if defined(CONFIG_BLE_LL_CFG_FEAT_LE_CODED_PHY) && defined(HAVE_RADIO_BLELR)
      case BLE_PHY_MODE_CODED_125KBPS:
        putreg32(RADIO_MODE_BLELR125KBIT, NRF52_RADIO_MODE);
        putreg32(NRF_PCNF0_CODED, NRF52_RADIO_PCNF0);
        break;

      case BLE_PHY_MODE_CODED_500KBPS:
        putreg32(RADIO_MODE_BLELR500KBIT, NRF52_RADIO_MODE);
        putreg32(NRF_PCNF0_CODED, NRF52_RADIO_PCNF0);
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

static int ble_phy_get_cur_phy(FAR struct nrf52_ble_dev_s *dev)
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

  state = getreg32(NRF52_RADIO_STATE);
  if (state == RADIO_STATE_RXDISABLE || state == RADIO_STATE_TXDISABLE)
    {
      while (getreg32(NRF52_RADIO_STATE) == state)
        {
        }
    }
}

/****************************************************************************
 * Name: ble_phy_set_start_time
 *
 * Description:
 *   Program RTC0 CC[0] and TIMER0 CC[0] so that the radio is enabled at the
 *   requested time.
 *
 * Returned Value:
 *   0 on success, -1 if the start time is too close or in the past.
 *
 ****************************************************************************/

static int ble_phy_set_start_time(FAR struct nrf52_ble_dev_s *dev,
                                  uint32_t cputime, uint8_t rem_us, bool tx)
{
  uint32_t next_cc;
  uint32_t cur_cc;
  uint32_t cntr;
  uint32_t delta;
  int radio_rem_us;
  int rem_us_corr;

  /* Calculate rem_us for radio enable.  The result may be negative but it
   * is adjusted below.
   */

  if (tx)
    {
      radio_rem_us = rem_us - BLE_PHY_T_TXENFAST -
                     g_ble_phy_t_txdelay[dev->phy.phy_cur_phy_mode];
    }
  else
    {
      radio_rem_us = rem_us - BLE_PHY_T_TXENFAST;
    }

  /* rem_us must be >= 1 for the TIMER0 compare event to be triggered */

  if (radio_rem_us <= -30)
    {
      /* rem_us is -60..-30 */

      cputime    -= 2;
      rem_us_corr = 61;
    }
  else
    {
      /* rem_us is -29..0 */

      cputime    -= 1;
      rem_us_corr = 30;
    }

  /* We can set the RTC compare to start TIMER0 if:
   *   a) Current compare value is not N+1 or N+2 ticks from current
   *      counter.
   *   b) The value we want to set is at least N+2 from current counter.
   *
   * The counter may tick while we do these calculations.
   */

  next_cc = cputime & 0xffffff;
  cur_cc  = getreg32(NRF52_BLE_RTC0(NRF52_RTC_CC_OFFSET(
                     NRF52_BLE_RTC_CC_START)));
  cntr    = getreg32(NRF52_BLE_RTC0(NRF52_RTC_COUNTER_OFFSET));

  delta = (cur_cc - cntr) & 0xffffff;
  if ((delta <= 3) && (delta != 0))
    {
      return -1;
    }

  delta = (next_cc - cntr) & 0xffffff;
  if ((delta & 0x800000) || (delta < 3))
    {
      return -1;
    }

  timer0_reset();

  /* Clear and set TIMER0 to fire off at proper time */

  putreg32(1, NRF52_BLE_TIMER0(NRF52_TIM_TASKS_CLEAR_OFFSET));
  timer0_cc_set(0, radio_rem_us + rem_us_corr);
  timer0_event_clear(0);

  /* Set RTC compare to start TIMER0 */

  putreg32(0, NRF52_BLE_RTC0(NRF52_RTC_EVENTS_COMPARE_OFFSET(
              NRF52_BLE_RTC_CC_START)));
  putreg32(next_cc, NRF52_BLE_RTC0(NRF52_RTC_CC_OFFSET(
                    NRF52_BLE_RTC_CC_START)));
  putreg32(RTC_EVTEN_COMPARE(NRF52_BLE_RTC_CC_START),
           NRF52_BLE_RTC0(NRF52_RTC_EVTENSET_OFFSET));

  /* Enable PPI */

  nrf52_ble_ppi_enable(PPI_CHEN_CH(NRF52_BLE_PPI_RTC0_START));

  /* Store the cputime at which we set the RTC */

  dev->phy.phy_start_cputime = cputime;

  return 0;
}

/****************************************************************************
 * Name: ble_phy_set_start_now
 *
 * Description:
 *   Start the radio as soon as possible.  Only used for RX.
 *
 ****************************************************************************/

static int ble_phy_set_start_now(FAR struct nrf52_ble_dev_s *dev)
{
  irqstate_t flags;
  uint32_t now;

  flags = enter_critical_section();

  /* TIMER0 compare register must be at least 1 or the compare event is not
   * triggered.
   */

  putreg32(1, NRF52_BLE_TIMER0(NRF52_TIM_TASKS_CLEAR_OFFSET));
  timer0_cc_set(0, 1);
  timer0_event_clear(0);

  /* Set RTC compare to start TIMER0.  It must be at least N+2 ticks from
   * the current value; use N+3 to account for an extra tick during these
   * operations.
   */

  now = nrf52_ble_tmr_get(&dev->lower);
  putreg32(0, NRF52_BLE_RTC0(NRF52_RTC_EVENTS_COMPARE_OFFSET(
              NRF52_BLE_RTC_CC_START)));
  putreg32((now + 3) & 0xffffff, NRF52_BLE_RTC0(NRF52_RTC_CC_OFFSET(
                                 NRF52_BLE_RTC_CC_START)));
  putreg32(RTC_EVTEN_COMPARE(NRF52_BLE_RTC_CC_START),
           NRF52_BLE_RTC0(NRF52_RTC_EVTENSET_OFFSET));

  /* Enable PPI */

  nrf52_ble_ppi_enable(PPI_CHEN_CH(NRF52_BLE_PPI_RTC0_START));

  /* The compare event may be triggered on the previous CC value so in rare
   * cases the actual start time may be up to 2 ticks earlier.
   */

  dev->phy.phy_start_cputime = now + 3;

  leave_critical_section(flags);

  return 0;
}

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
/****************************************************************************
 * Name: ble_phy_get_ccm_datarate
 ****************************************************************************/

static uint32_t ble_phy_get_ccm_datarate(FAR struct nrf52_ble_dev_s *dev)
{
#ifdef CONFIG_BLE_LL_PHY
  switch (dev->phy.phy_cur_phy_mode)
    {
      case BLE_PHY_MODE_1M:
        return CCM_MODE_DATARATE_1MBIT;

      case BLE_PHY_MODE_2M:
        return CCM_MODE_DATARATE_2MBIT;

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_CODED_PHY
      case BLE_PHY_MODE_CODED_125KBPS:
        return CCM_MODE_DATARATE_125KBPS;

      case BLE_PHY_MODE_CODED_500KBPS:
        return CCM_MODE_DATARATE_500KBPS;
#endif
    }

  DEBUGPANIC();
  return 0;
#else
  return CCM_MODE_DATARATE_1MBIT;
#endif
}
#endif

/****************************************************************************
 * Name: ble_phy_rx_xcvr_setup
 *
 * Description:
 *   Setup transceiver for receive.
 *
 ****************************************************************************/

static void ble_phy_rx_xcvr_setup(FAR struct nrf52_ble_dev_s *dev)
{
  FAR uint8_t *dptr;

  dptr  = (FAR uint8_t *)&dev->rx_buf[0];
  dptr += 3;

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
  if (dev->phy.phy_encrypted)
    {
      putreg32((uint32_t)&dev->enc_buf[0], NRF52_RADIO_PACKETPTR);
      putreg32((uint32_t)&dev->enc_buf[0], NRF52_CCM_INPTR);
      putreg32((uint32_t)dptr, NRF52_CCM_OUTPTR);
      putreg32((uint32_t)&dev->enc_scratch[0], NRF52_CCM_SCRATCHPTR);
      putreg32(CCM_MODE_LENGTH_EXTENDED | CCM_MODE_DECRYPTION |
               ble_phy_get_ccm_datarate(dev), NRF52_CCM_MODE);
      putreg32((uint32_t)&dev->ccm, NRF52_CCM_CNFPTR);
      putreg32(0, NRF52_CCM_SHORTS);
      putreg32(0, NRF52_CCM_EVENTS_ERROR);
      putreg32(0, NRF52_CCM_EVENTS_ENDCRYPT);
      putreg32(1, NRF52_CCM_TASKS_KSGEN);
      nrf52_ble_ppi_enable(PPI_CHEN_CH(NRF52_BLE_PPI_CCM_CRYPT));
    }
  else
#endif
    {
      putreg32((uint32_t)dptr, NRF52_RADIO_PACKETPTR);
    }

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  if (dev->phy.phy_privacy)
    {
      putreg32(AAR_ENABLE_ENABLED, NRF52_AAR_ENABLE);
      putreg32((uint32_t)&dev->irk_list[0], NRF52_AAR_IRKPTR);
      putreg32((uint32_t)&dev->phy.phy_aar_scratch,
               NRF52_AAR_SCRATCHPTR);
      putreg32(0, NRF52_AAR_EVENTS_END);
      putreg32(0, NRF52_AAR_EVENTS_RESOLVED);
      putreg32(0, NRF52_AAR_EVENTS_NOTRESOLVED);
    }
  else if (dev->phy.phy_encrypted == 0)
    {
      putreg32(AAR_ENABLE_DISABLED, NRF52_AAR_ENABLE);
    }
#endif

  /* Turn off trigger TXEN on output compare match and AAR on bcmatch */

  nrf52_ble_ppi_disable(PPI_CHEN_CH(NRF52_BLE_PPI_TIMER0_TXEN) |
                        PPI_CHEN_CH(NRF52_BLE_PPI_AAR_START));

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

  putreg32(8 + dev->phy.phy_bcc_offset, NRF52_RADIO_BCC);
  putreg32(0, NRF52_RADIO_EVENTS_ADDRESS);
  putreg32(0, NRF52_RADIO_EVENTS_DEVMATCH);
  putreg32(0, NRF52_RADIO_EVENTS_BCMATCH);
  putreg32(0, NRF52_RADIO_EVENTS_RSSIEND);
  putreg32(0, NRF52_RADIO_EVENTS_CRCOK);
  putreg32(RADIO_SHORTS_END_DISABLE | RADIO_SHORTS_READY_START |
           RADIO_SHORTS_ADDRESS_BCSTART | RADIO_SHORTS_ADDRESS_RSSISTART |
           RADIO_SHORTS_DISABLED_RSSISTOP, NRF52_RADIO_SHORTS);

  putreg32(RADIO_INT_ADDRESS | RADIO_INT_DISABLED, NRF52_RADIO_INTENSET);
}

/****************************************************************************
 * Name: ble_phy_tx_end_isr
 *
 * Description:
 *   Called from interrupt context when the transmit ends.
 *
 ****************************************************************************/

static void ble_phy_tx_end_isr(FAR struct nrf52_ble_dev_s *dev)
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

  if (dev->phy.phy_encrypted && getreg32(NRF52_CCM_EVENTS_ERROR))
    {
      NRF52_BLE_STATS_INC(dev, tx_hw_err);
      putreg32(0, NRF52_CCM_EVENTS_ERROR);
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

      nrf52_ble_phy_wfr_enable(&dev->lower, BLE_PHY_WFR_ENABLE_TXRX,
                               tx_phy_mode, 0);

      /* Schedule RX exactly T_IFS after TX end captured in CC[2], adjusted
       * for the delay between EVENT_END and actual TX end and started a bit
       * earlier due to allowed active clock accuracy.
       */

      rx_time  = timer0_cc_get(2) + tifs;
      rx_time += g_ble_phy_t_txenddelay[tx_phy_mode];
      rx_time -= 2;

      radio_time = rx_time - BLE_PHY_T_RXENFAST;
      timer0_event_clear(0);
      nrf52_ble_ppi_enable(PPI_CHEN_CH(NRF52_BLE_PPI_TIMER0_RXEN));
      timer0_cc_set(0, radio_time);

      /* If TIMER0 already counted past CC[0], the wfr timer cancels the
       * reception.
       */
    }
  else if (transition == BLE_PHY_TRANSITION_TX_TX)
    {
      if (dev->phy.txtx_time_anchor)
        {
          /* TX anchor relative to current TX end captured in CC[2] */

          tx_time  = timer0_cc_get(2);
          tx_time += g_ble_phy_t_txenddelay[tx_phy_mode];
        }
      else
        {
          /* TX anchor relative to current TX start: access address is
           * captured in CC[1].
           */

          tx_time  = timer0_cc_get(1);
          tx_time += g_ble_phy_t_txaddrdelay[tx_phy_mode];
          tx_time -= ble_ll_pdu_syncword_us(tx_phy_mode);
        }

      tx_time += dev->phy.txtx_time_us;

      /* Adjust for delay between EVENT_READY and actual TX start time */

      tx_time -= g_ble_phy_t_txdelay[dev->phy.phy_cur_phy_mode];

      radio_time = tx_time - BLE_PHY_T_TXENFAST;
      timer0_event_clear(0);
      nrf52_ble_ppi_enable(PPI_CHEN_CH(NRF52_BLE_PPI_TIMER0_TXEN));
      timer0_cc_set(0, radio_time);

      if (timer0_did_miss(0, 3))
        {
          nrf52_ble_ppi_disable(PPI_CHEN_CH(NRF52_BLE_PPI_TIMER0_TXEN));
          dev->phy.phy_transition_late = 1;
        }
    }
  else
    {
      putreg32(1, NRF52_BLE_TIMER0(NRF52_TIM_TASKS_STOP_OFFSET));
      putreg32(1, NRF52_BLE_TIMER0(NRF52_TIM_TASKS_SHUTDOWN_OFFSET));
      phy_ppi_wfr_disable();
      nrf52_ble_ppi_disable(PPI_CHEN_CH(NRF52_BLE_PPI_TIMER0_TXEN) |
                            PPI_CHEN_CH(NRF52_BLE_PPI_RTC0_START));
      DEBUGASSERT(transition == BLE_PHY_TRANSITION_NONE);
    }
}

/****************************************************************************
 * Name: ble_phy_get_cur_rx_phy_mode
 ****************************************************************************/

static inline uint8_t
ble_phy_get_cur_rx_phy_mode(FAR struct nrf52_ble_dev_s *dev)
{
  uint8_t phy;

  phy = dev->phy.phy_cur_phy_mode;

#if defined(CONFIG_BLE_LL_CFG_FEAT_LE_CODED_PHY) && defined(HAVE_RADIO_BLELR)
  /* For Coded PHY the actual coding of the received packet is given by
   * the CI field.
   */

  if (phy == BLE_PHY_MODE_CODED_125KBPS ||
      phy == BLE_PHY_MODE_CODED_500KBPS)
    {
      phy = (getreg32(NRF52_RADIO_PDUSTAT) & RADIO_PDUSTAT_CISTAT_MASK) ?
            BLE_PHY_MODE_CODED_500KBPS : BLE_PHY_MODE_CODED_125KBPS;
    }
#endif

  return phy;
}

/****************************************************************************
 * Name: ble_phy_rx_end_isr
 ****************************************************************************/

static void ble_phy_rx_end_isr(FAR struct nrf52_ble_dev_s *dev)
{
  FAR struct ble_mbuf_hdr *ble_hdr;
  FAR uint8_t *dptr;
  uint32_t radio_time;
  uint32_t tx_time;
  uint16_t tifs;
  uint8_t crcok;
  int rc;

  /* Disable automatic RXEN */

  nrf52_ble_ppi_disable(PPI_CHEN_CH(NRF52_BLE_PPI_TIMER0_RXEN));

  /* Set RSSI and CRC status flag in header */

  ble_hdr = &dev->phy.rxhdr;
  DEBUGASSERT(getreg32(NRF52_RADIO_EVENTS_RSSIEND) != 0);
  ble_hdr->rxinfo.rssi = -1 * (int)getreg32(NRF52_RADIO_RSSISAMPLE);

  dptr  = (FAR uint8_t *)&dev->rx_buf[0];
  dptr += 3;

  /* Count PHY crc errors and valid packets */

  crcok = getreg32(NRF52_RADIO_EVENTS_CRCOK);
  if (!crcok)
    {
      NRF52_BLE_STATS_INC(dev, rx_crc_err);
    }
  else
    {
      NRF52_BLE_STATS_INC(dev, rx_valid);
      ble_hdr->rxinfo.flags |= BLE_MBUF_HDR_F_CRC_OK;

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
      if (dev->phy.phy_encrypted)
        {
          /* Make sure CCM finished */

          while (getreg32(NRF52_CCM_EVENTS_ENDCRYPT) == 0)
            {
            }

          /* Only set MIC failure flag if frame is not zero length */

          if ((dptr[1] != 0) && (getreg32(NRF52_CCM_MICSTATUS) == 0))
            {
              ble_hdr->rxinfo.flags |= BLE_MBUF_HDR_F_MIC_FAILURE;
            }

          /* CCM error: treat as a CRC error so the PDU is not used */

          if (getreg32(NRF52_CCM_EVENTS_ERROR))
            {
              NRF52_BLE_STATS_INC(dev, rx_hw_err);
              ble_hdr->rxinfo.flags &= ~BLE_MBUF_HDR_F_CRC_OK;
            }
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

  tx_time  = timer0_cc_get(2) + tifs;
  tx_time -= g_ble_phy_t_rxenddelay[ble_hdr->rxinfo.phy_mode];

  /* Adjust for delay between EVENT_READY and actual TX start time */

  tx_time -= g_ble_phy_t_txdelay[dev->phy.phy_cur_phy_mode];

  radio_time = tx_time - BLE_PHY_T_TXENFAST;
  timer0_event_clear(0);
  nrf52_ble_ppi_enable(PPI_CHEN_CH(NRF52_BLE_PPI_TIMER0_TXEN));
  timer0_cc_set(0, radio_time);

  /* Check if TIMER0 did not already count past CC[0] */

  if (timer0_did_miss(0, 3))
    {
      nrf52_ble_ppi_disable(PPI_CHEN_CH(NRF52_BLE_PPI_TIMER0_TXEN));
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
      nrf52_ble_phy_disable(&dev->lower);
    }
}

/****************************************************************************
 * Name: ble_phy_rx_start_isr
 ****************************************************************************/

static bool ble_phy_rx_start_isr(FAR struct nrf52_ble_dev_s *dev)
{
  FAR struct ble_mbuf_hdr *ble_hdr;
  FAR uint8_t *dptr;
  uint32_t pdu_usecs;
  uint32_t state;
  uint32_t usecs;
  uint32_t ticks;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  int adva_offset;
#endif
  int rc;

  dptr = (FAR uint8_t *)&dev->rx_buf[0];

  /* Clear events and clear interrupt */

  putreg32(0, NRF52_RADIO_EVENTS_ADDRESS);
  putreg32(RADIO_INT_ADDRESS, NRF52_RADIO_INTENCLR);

  /* Clear wfr timer channels */

  phy_ppi_wfr_disable();

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

  /* Calculate accurate packet start time (with remainder).  The reception
   * may start during the preamble, before TIMER0 was running.
   */

  ble_hdr->beg_cputime = dev->phy.phy_start_cputime;

  usecs     = timer0_cc_get(1);
  pdu_usecs = ble_phy_mode_pdu_start_off(ble_hdr->rxinfo.phy_mode) +
              g_ble_phy_t_rxaddrdelay[ble_hdr->rxinfo.phy_mode];
  if (usecs < pdu_usecs)
    {
      dev->phy.phy_start_cputime--;
      usecs += 30;
    }

  usecs -= pdu_usecs;

  ticks  = ((uint64_t)usecs * 32768) / 1000000;
  usecs -= ((uint64_t)ticks * 1000000) / 32768;
  if (usecs == 31)
    {
      usecs = 0;
      ++ticks;
    }

  ble_hdr->beg_cputime += ticks;
  ble_hdr->rem_usecs    = usecs;

  /* Wait to get 1st byte of frame */

  while (1)
    {
      state = getreg32(NRF52_RADIO_STATE);
      if (getreg32(NRF52_RADIO_EVENTS_BCMATCH) != 0)
        {
          break;
        }

      /* If state is disabled, we should have the BCMATCH */

      if (state == RADIO_STATE_DISABLED)
        {
          putreg32(NRF_RADIO_IRQ_MASK_ALL, NRF52_RADIO_INTENCLR);
          putreg32(0, NRF52_RADIO_SHORTS);
          return false;
        }
    }

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  /* If privacy is enabled and received PDU has TxAdd bit set (random
   * address) resolve the address using AAR.
   */

  if (dev->phy.phy_privacy && (dptr[3] & 0x40))
    {
      /* AdvA is located at 4th octet in RX buffer (after S0, length and S1
       * fields).  Extended advertising PDUs have 2 more octets.
       */

      adva_offset = (dptr[3] & 0x0f) == 0x07 ? 2 : 0;
      putreg32((uint32_t)(dptr + 3 + adva_offset), NRF52_AAR_ADDRPTR);

      /* Trigger AAR after last bit of AdvA is received */

      putreg32(0, NRF52_RADIO_EVENTS_BCMATCH);
      nrf52_ble_ppi_enable(PPI_CHEN_CH(NRF52_BLE_PPI_AAR_START));
      putreg32((BLE_LL_PDU_HDR_LEN + adva_offset + 6) * 8 +
               dev->phy.phy_bcc_offset, NRF52_RADIO_BCC);
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
      nrf52_ble_phy_disable(&dev->lower);
      NRF52_BLE_STATS_INC(dev, rx_aborts);
    }

  NRF52_BLE_STATS_INC(dev, rx_starts);

  return true;
}

/****************************************************************************
 * Name: ble_phy_set_access_addr
 ****************************************************************************/

static int ble_phy_set_access_addr(FAR struct nrf52_ble_dev_s *dev,
                                   uint32_t access_addr)
{
  putreg32(access_addr << 8, NRF52_RADIO_BASE0);
  putreg32((getreg32(NRF52_RADIO_PREFIX0) & 0xffffff00) |
           (access_addr >> 24), NRF52_RADIO_PREFIX0);

  dev->phy.phy_access_address = access_addr;

#ifdef CONFIG_ARCH_CHIP_NRF52832
  if (nrf52_ble_errata_102_106_107())
    {
      modifyreg32(NRF52_RADIO_BASE + 0x774, 0x00000001, 0x01000000);
    }
#endif

  return 0;
}

/****************************************************************************
 * Name: ble_phy_stop_usec_timer
 *
 * Description:
 *   Stop the timer used to count microseconds.
 *
 ****************************************************************************/

static void ble_phy_stop_usec_timer(void)
{
  putreg32(1, NRF52_BLE_TIMER0(NRF52_TIM_TASKS_STOP_OFFSET));
  putreg32(1, NRF52_BLE_TIMER0(NRF52_TIM_TASKS_SHUTDOWN_OFFSET));
  putreg32(RTC_EVTEN_COMPARE(NRF52_BLE_RTC_CC_START),
           NRF52_BLE_RTC0(NRF52_RTC_EVTENCLR_OFFSET));
}

/****************************************************************************
 * Name: ble_phy_disable_irq_and_ppi
 *
 * Description:
 *   Called when reception was stopped and the PHY is to be restarted or
 *   disabled.
 *
 ****************************************************************************/

static void ble_phy_disable_irq_and_ppi(FAR struct nrf52_ble_dev_s *dev)
{
  int irq = NRF52_IRQ_RADIO - NRF52_IRQ_EXTINT;

  putreg32(NRF_RADIO_IRQ_MASK_ALL, NRF52_RADIO_INTENCLR);
  putreg32(0, NRF52_RADIO_SHORTS);
  putreg32(1, NRF52_RADIO_TASKS_DISABLE);
  nrf52_ble_ppi_disable(NRF52_BLE_PPI_MASK_ALL);
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

static int ble_phy_rx(FAR struct nrf52_ble_dev_s *dev)
{
  uint32_t state;

  /* Wait if the radio is disabling.  The radio can ramp-up from the idle
   * state too.  TX and RX state values differ only in the 3rd bit.
   */

  nrf_wait_disabled();

  state = getreg32(NRF52_RADIO_STATE);
  if ((state != RADIO_STATE_DISABLED) &&
      ((state & 0x07) != RADIO_STATE_RXIDLE))
    {
      nrf52_ble_phy_disable(&dev->lower);
      NRF52_BLE_STATS_INC(dev, radio_state_errs);
      return BLE_PHY_ERR_RADIO_STATE;
    }

  /* Make sure all interrupts are disabled */

  putreg32(NRF_RADIO_IRQ_MASK_ALL, NRF52_RADIO_INTENCLR);

  /* Clear events prior to enabling receive */

  putreg32(0, NRF52_RADIO_EVENTS_END);
  putreg32(0, NRF52_RADIO_EVENTS_DISABLED);

  ble_phy_rx_xcvr_setup(dev);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_ble_phy_isr
 ****************************************************************************/

int nrf52_ble_phy_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)arg;
  uint32_t irq_en;

  /* Read irq register to determine which interrupts are enabled */

  irq_en = getreg32(NRF52_RADIO_INTENSET);

  /* NOTE: order of checking is important!  If things get delayed, we may
   * have both an ADDRESS and DISABLED interrupt in rx state.  If we get an
   * address, we disable the DISABLED interrupt.
   */

  if ((irq_en & RADIO_INT_ADDRESS) && getreg32(NRF52_RADIO_EVENTS_ADDRESS))
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

  if ((irq_en & RADIO_INT_DISABLED) && getreg32(NRF52_RADIO_EVENTS_DISABLED))
    {
      DEBUGASSERT(getreg32(NRF52_RADIO_EVENTS_END) ||
                  ((dev->phy.phy_state == BLE_PHY_STATE_RX) &&
                   !dev->phy.phy_rx_started));

      putreg32(0, NRF52_RADIO_EVENTS_END);
      putreg32(0, NRF52_RADIO_EVENTS_DISABLED);
      putreg32(RADIO_INT_DISABLED, NRF52_RADIO_INTENCLR);

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

  NRF52_BLE_STATS_INC(dev, phy_isrs);

  return OK;
}

/****************************************************************************
 * Name: nrf52_ble_phy_init
 *
 * Description:
 *   Initialize the PHY.
 *
 ****************************************************************************/

int nrf52_ble_phy_init(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  /* Default phy to use is 1M */

  dev->phy.phy_cur_phy_mode = BLE_PHY_MODE_1M;
  dev->phy.phy_tx_phy_mode  = BLE_PHY_MODE_1M;
  dev->phy.phy_rx_phy_mode  = BLE_PHY_MODE_1M;

  /* Set phy channel to an invalid channel so first set channel works */

  dev->phy.phy_chan = BLE_PHY_NUM_CHANS;

  /* Toggle peripheral power to reset */

  putreg32(RADIO_POWER_DISABLE, NRF52_RADIO_POWER);
  putreg32(RADIO_POWER_ENABLE, NRF52_RADIO_POWER);

  /* Disable all interrupts */

  putreg32(NRF_RADIO_IRQ_MASK_ALL, NRF52_RADIO_INTENCLR);

  /* Set configuration registers */

  putreg32(RADIO_MODE_BLE1MBIT, NRF52_RADIO_MODE);
  putreg32(NRF_PCNF0, NRF52_RADIO_PCNF0);
  putreg32(NRF_MAXLEN | RADIO_PCNF1_ENDIAN_LITTLE |
           (NRF_BALEN << RADIO_PCNF1_BALEN_SHIFT) | RADIO_PCNF1_WHITEEN,
           NRF52_RADIO_PCNF1);

  /* Enable radio fast ramp-up */

  modifyreg32(NRF52_RADIO_MODECNF0, 0, RADIO_MODECNF0_RU);

  /* Set logical address 0 for TX and RX */

  putreg32(0, NRF52_RADIO_TXADDRESS);
  putreg32(RADIO_RXADDRESSES_ADDR(0), NRF52_RADIO_RXADDRESSES);

  /* Configure the CRC registers */

  putreg32(RADIO_CRCCNF_SKIPADDR_SKIP | RADIO_CRCCNF_LEN_3,
           NRF52_RADIO_CRCCNF);
  putreg32(0x0000065b, NRF52_RADIO_CRCPOLY);

  /* Configure IFS */

  putreg32(BLE_LL_IFS, NRF52_RADIO_TIFS);

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
  putreg32(0xffffffff, NRF52_CCM_INTENCLR);
  putreg32(CCM_SHORTS_ENDKSGEN_CRYPT, NRF52_CCM_SHORTS);
  putreg32(0, NRF52_CCM_EVENTS_ERROR);
  memset(dev->enc_scratch, 0, sizeof(dev->enc_scratch));
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  dev->phy.phy_aar_scratch = 0;
  putreg32((uint32_t)&dev->irk_list[0], NRF52_AAR_IRKPTR);
  putreg32(0xffffffff, NRF52_AAR_INTENCLR);
  putreg32(0, NRF52_AAR_EVENTS_END);
  putreg32(0, NRF52_AAR_EVENTS_RESOLVED);
  putreg32(0, NRF52_AAR_EVENTS_NOTRESOLVED);
  putreg32(0, NRF52_AAR_NIRK);
#endif

  /* TIMER0: 32-bit timer at 1 MHz */

  putreg32(1, NRF52_BLE_TIMER0(NRF52_TIM_TASKS_STOP_OFFSET));
  putreg32(1, NRF52_BLE_TIMER0(NRF52_TIM_TASKS_SHUTDOWN_OFFSET));
  putreg32(3, NRF52_BLE_TIMER0(NRF52_TIM_BITMODE_OFFSET));
  putreg32(0, NRF52_BLE_TIMER0(NRF52_TIM_MODE_OFFSET));
  putreg32(4, NRF52_BLE_TIMER0(NRF52_TIM_PRESCALER_OFFSET));

  phy_ppi_init();

  up_enable_irq(NRF52_IRQ_RADIO);

  return 0;
}

/****************************************************************************
 * Name: nrf52_ble_phy_wfr_enable
 *
 * Description:
 *   Set PPI so that we can time out waiting for a reception.  This happens
 *   after a transmission when waiting for a response (txrx is
 *   BLE_PHY_WFR_ENABLE_TXRX) or when waiting for the first packet of a
 *   connection event as a peripheral (txrx is BLE_PHY_WFR_ENABLE_RX).
 *
 *   wfr_usecs is not used for a txrx turn-around since we know when the
 *   address of the received frame is expected.
 *
 ****************************************************************************/

void nrf52_ble_phy_wfr_enable(FAR struct ble_ll_lower_s *lower, int txrx,
                              uint8_t tx_phy_mode, uint32_t wfr_usecs)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;
  uint32_t end_time;
  uint8_t phy;

  phy = dev->phy.phy_cur_phy_mode;

  if (txrx == BLE_PHY_WFR_ENABLE_TXRX)
    {
      /* RX shall start exactly T_IFS after TX end captured in CC[2].  Wait
       * 2 usec longer due to allowed active clock accuracy and 1 usec more
       * since the PDU start may be captured at the end of a timer cycle.
       */

      end_time  = timer0_cc_get(2) + BLE_LL_IFS;
      end_time += g_ble_phy_t_txenddelay[tx_phy_mode];
      end_time += 2;
      end_time += 1;
    }
  else
    {
      /* RX shall start no later than wfr_usecs after RX enabled.  CC[0] is
       * the time of RXEN so adjust for radio ramp-up.
       */

      end_time = timer0_cc_get(0) + BLE_PHY_T_RXENFAST + wfr_usecs;
    }

  /* Adjust for receiving access address since this triggers EVENT_ADDRESS
   * and for the delay between actual access address RX and EVENT_ADDRESS.
   */

  end_time += ble_phy_mode_pdu_start_off(phy);
  end_time += g_ble_phy_t_rxaddrdelay[phy];

  /* Enable wait for response PPI */

  timer0_event_clear(3);
  phy_ppi_wfr_enable();
  timer0_cc_set(3, end_time);

  /* If the CPU was halted for a moment, TIMER0 may already have counted
   * past CC[3].  Disable the radio manually in that case.  CC[1] is only
   * used as a reference on RX start so it is used as scratch here.
   */

  if (timer0_did_miss(3, 1))
    {
      phy_ppi_wfr_disable();
      putreg32(1, NRF52_RADIO_TASKS_DISABLE);
    }
}

#ifdef CONFIG_BLE_LL_PHY
/****************************************************************************
 * Name: nrf52_ble_phy_mode_set
 ****************************************************************************/

void nrf52_ble_phy_mode_set(FAR struct ble_ll_lower_s *lower,
                            uint8_t tx_phy_mode, uint8_t rx_phy_mode)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  dev->phy.phy_tx_phy_mode = tx_phy_mode;
  dev->phy.phy_rx_phy_mode = rx_phy_mode;
}
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
/****************************************************************************
 * Name: nrf52_ble_phy_encrypt_enable
 ****************************************************************************/

void nrf52_ble_phy_encrypt_enable(FAR struct ble_ll_lower_s *lower,
                                  FAR const uint8_t *key)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  memcpy(dev->ccm.key, key, 16);
  dev->phy.phy_encrypted = 1;
  putreg32(AAR_ENABLE_DISABLED, NRF52_AAR_ENABLE);
  putreg32(CCM_ENABLE_ENABLED, NRF52_CCM_ENABLE);
}

/****************************************************************************
 * Name: nrf52_ble_phy_encrypt_header_mask_set
 *
 * Description:
 *   The nRF52 CCM always uses the default data PDU header mask.
 *
 ****************************************************************************/

void nrf52_ble_phy_encrypt_header_mask_set(FAR struct ble_ll_lower_s *lower,
                                           uint8_t mask)
{
  UNUSED(mask);
}

/****************************************************************************
 * Name: nrf52_ble_phy_encrypt_iv_set
 ****************************************************************************/

void nrf52_ble_phy_encrypt_iv_set(FAR struct ble_ll_lower_s *lower,
                                  FAR const uint8_t *iv)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  memcpy(dev->ccm.iv, iv, 8);
}

/****************************************************************************
 * Name: nrf52_ble_phy_encrypt_counter_set
 ****************************************************************************/

void nrf52_ble_phy_encrypt_counter_set(FAR struct ble_ll_lower_s *lower,
                                       uint64_t counter, uint8_t dir_bit)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  dev->ccm.pkt_counter = counter;
  dev->ccm.dir_bit     = dir_bit;
}

/****************************************************************************
 * Name: nrf52_ble_phy_encrypt_disable
 ****************************************************************************/

void nrf52_ble_phy_encrypt_disable(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  nrf52_ble_ppi_disable(PPI_CHEN_CH(NRF52_BLE_PPI_CCM_CRYPT));
  putreg32(1, NRF52_CCM_TASKS_STOP);
  putreg32(0, NRF52_CCM_EVENTS_ERROR);
  putreg32(CCM_ENABLE_DISABLED, NRF52_CCM_ENABLE);

  dev->phy.phy_encrypted = 0;
}
#endif

/****************************************************************************
 * Name: nrf52_ble_phy_set_txend_cb
 ****************************************************************************/

void nrf52_ble_phy_set_txend_cb(FAR struct ble_ll_lower_s *lower,
                                ble_phy_tx_end_func txend_cb, FAR void *arg)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  dev->phy.txend_cb  = txend_cb;
  dev->phy.txend_arg = arg;
}

/****************************************************************************
 * Name: nrf52_ble_phy_tx_set_start_time
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

int nrf52_ble_phy_tx_set_start_time(FAR struct ble_ll_lower_s *lower,
                                    uint32_t cputime, uint8_t rem_usecs)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;
  int rc;

#ifdef CONFIG_BLE_LL_PHY
  ble_phy_mode_apply(dev, dev->phy.phy_tx_phy_mode);
#endif

  /* Clear timer0 compare to RXEN since we are transmitting */

  nrf52_ble_ppi_disable(PPI_CHEN_CH(NRF52_BLE_PPI_TIMER0_RXEN));

  if (ble_phy_set_start_time(dev, cputime, rem_usecs, true) != 0)
    {
      NRF52_BLE_STATS_INC(dev, tx_late);
      nrf52_ble_phy_disable(lower);
      rc = BLE_PHY_ERR_TX_LATE;
    }
  else
    {
      /* Enable PPI to automatically start TXEN */

      nrf52_ble_ppi_enable(PPI_CHEN_CH(NRF52_BLE_PPI_TIMER0_TXEN));
      rc = 0;
    }

  return rc;
}

/****************************************************************************
 * Name: nrf52_ble_phy_rx_set_start_time
 *
 * Description:
 *   Set the start time of a reception.  If we are late, we still attempt
 *   to receive.  The channel must already be set.
 *
 ****************************************************************************/

int nrf52_ble_phy_rx_set_start_time(FAR struct ble_ll_lower_s *lower,
                                    uint32_t cputime, uint8_t rem_usecs)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;
  bool late = false;
  int rc;

#ifdef CONFIG_BLE_LL_PHY
  ble_phy_mode_apply(dev, dev->phy.phy_rx_phy_mode);
#endif

  /* Clear timer0 compare to TXEN since we are receiving */

  nrf52_ble_ppi_disable(PPI_CHEN_CH(NRF52_BLE_PPI_TIMER0_TXEN));

  if (ble_phy_set_start_time(dev, cputime, rem_usecs, false) != 0)
    {
      NRF52_BLE_STATS_INC(dev, rx_late);

      /* We're late so let's just try to start RX as soon as possible */

      ble_phy_set_start_now(dev);
      late = true;
    }

  /* Enable PPI to automatically start RXEN */

  nrf52_ble_ppi_enable(PPI_CHEN_CH(NRF52_BLE_PPI_TIMER0_RXEN));

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
 * Name: nrf52_ble_phy_tx
 ****************************************************************************/

int nrf52_ble_phy_tx(FAR struct ble_ll_lower_s *lower,
                     ble_phy_tx_pducb_t pducb, FAR void *pducb_arg,
                     uint8_t end_trans)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;
  FAR uint8_t *pktptr;
  FAR uint8_t *dptr;
  uint8_t payload_len;
  uint8_t hdr_byte;
  uint32_t state;
  int rc;

  if (dev->phy.phy_transition_late)
    {
      nrf52_ble_phy_disable(lower);
      NRF52_BLE_STATS_INC(dev, tx_late);
      return BLE_PHY_ERR_TX_LATE;
    }

  /* Make sure the radio is not moving to the disabled state */

  nrf_wait_disabled();

  /* Clear all PPI channels that should not be used when transmitting */

  phy_ppi_wfr_disable();
  nrf52_ble_ppi_disable(PPI_CHEN_CH(NRF52_BLE_PPI_AAR_START) |
                        PPI_CHEN_CH(NRF52_BLE_PPI_CCM_CRYPT));

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
  if (dev->phy.phy_encrypted)
    {
      dptr   = (FAR uint8_t *)&dev->enc_buf[0];
      pktptr = (FAR uint8_t *)&dev->tx_buf[0];
      putreg32(CCM_SHORTS_ENDKSGEN_CRYPT, NRF52_CCM_SHORTS);
      putreg32((uint32_t)dptr, NRF52_CCM_INPTR);
      putreg32((uint32_t)pktptr, NRF52_CCM_OUTPTR);
      putreg32((uint32_t)&dev->enc_scratch[0], NRF52_CCM_SCRATCHPTR);
      putreg32(0, NRF52_CCM_EVENTS_ERROR);
      putreg32(CCM_MODE_LENGTH_EXTENDED | ble_phy_get_ccm_datarate(dev),
               NRF52_CCM_MODE);
      putreg32((uint32_t)&dev->ccm, NRF52_CCM_CNFPTR);
    }
  else
#endif
    {
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
      putreg32((uint32_t)&dev->irk_list[0], NRF52_AAR_IRKPTR);
#endif
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
  /* Start key-stream generation and encryption (via short) */

  if (dev->phy.phy_encrypted)
    {
      putreg32(1, NRF52_CCM_TASKS_KSGEN);
    }
#endif

  putreg32((uint32_t)pktptr, NRF52_RADIO_PACKETPTR);

  /* Clear the ready, end and disabled events */

  putreg32(0, NRF52_RADIO_EVENTS_READY);
  putreg32(0, NRF52_RADIO_EVENTS_END);
  putreg32(0, NRF52_RADIO_EVENTS_DISABLED);

  /* Enable shortcuts for transmit start/end */

  putreg32(RADIO_SHORTS_END_DISABLE | RADIO_SHORTS_READY_START,
           NRF52_RADIO_SHORTS);
  putreg32(RADIO_INT_DISABLED, NRF52_RADIO_INTENSET);

  /* Set the PHY transition and transmitted payload length */

  dev->phy.phy_transition  = end_trans;
  dev->phy.phy_tx_pyld_len = payload_len;

  /* If we already started transmitting, abort it! */

  state = getreg32(NRF52_RADIO_STATE);
  if (state != RADIO_STATE_TX)
    {
      dev->phy.phy_state = BLE_PHY_STATE_TX;
      NRF52_BLE_STATS_INC(dev, tx_good);
      NRF52_BLE_STATS_INCN(dev, tx_bytes, payload_len + BLE_LL_PDU_HDR_LEN);
      rc = 0;
    }
  else
    {
      nrf52_ble_phy_disable(lower);
      NRF52_BLE_STATS_INC(dev, tx_late);
      rc = BLE_PHY_ERR_RADIO_STATE;
    }

  return rc;
}

/****************************************************************************
 * Name: nrf52_ble_phy_tx_power_set
 ****************************************************************************/

int nrf52_ble_phy_tx_power_set(FAR struct ble_ll_lower_s *lower, int dbm)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  /* Get actual TX power supported by radio */

  dbm = phy_txpower_round(dbm);

  putreg32((uint32_t)dbm & RADIO_TXPOWER_MASK, NRF52_RADIO_TXPOWER);
  dev->phy.phy_txpwr_dbm = dbm;

  return 0;
}

/****************************************************************************
 * Name: nrf52_ble_phy_tx_power_round
 ****************************************************************************/

int nrf52_ble_phy_tx_power_round(FAR struct ble_ll_lower_s *lower, int dbm)
{
  return phy_txpower_round(dbm);
}

/****************************************************************************
 * Name: nrf52_ble_phy_tx_power_get
 ****************************************************************************/

int nrf52_ble_phy_tx_power_get(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  return dev->phy.phy_txpwr_dbm;
}

/****************************************************************************
 * Name: nrf52_ble_phy_setchan
 *
 * Description:
 *   Set the logical frequency of the transceiver.  The FREQUENCY register
 *   gives the frequency as 2400 + FREQ MHz.
 *
 * Input Parameters:
 *   chan - Data Channel Index or Advertising Channel index
 *
 ****************************************************************************/

int nrf52_ble_phy_setchan(FAR struct ble_ll_lower_s *lower, uint8_t chan,
                          uint32_t access_addr, uint32_t crcinit)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  DEBUGASSERT(chan < BLE_PHY_NUM_CHANS);

  if (chan >= BLE_PHY_NUM_CHANS)
    {
      return BLE_PHY_ERR_INV_PARAM;
    }

  /* Set current access address */

  ble_phy_set_access_addr(dev, access_addr);

  /* Configure crcinit */

  putreg32(crcinit, NRF52_RADIO_CRCINIT);

  /* Set the frequency and the data whitening initial value */

  dev->phy.phy_chan = chan;
  putreg32(g_ble_phy_chan_freq[chan], NRF52_RADIO_FREQUENCY);
  putreg32(chan, NRF52_RADIO_DATAWHITEIV);

  return 0;
}

/****************************************************************************
 * Name: nrf52_ble_phy_chan_get
 ****************************************************************************/

uint8_t nrf52_ble_phy_chan_get(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  return dev->phy.phy_chan;
}

/****************************************************************************
 * Name: nrf52_ble_phy_restart_rx
 ****************************************************************************/

void nrf52_ble_phy_restart_rx(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  ble_phy_stop_usec_timer();
  ble_phy_disable_irq_and_ppi(dev);

  ble_phy_set_start_now(dev);

  /* Enable PPI to automatically start RXEN */

  nrf52_ble_ppi_enable(PPI_CHEN_CH(NRF52_BLE_PPI_TIMER0_RXEN));

  ble_phy_rx(dev);
}

/****************************************************************************
 * Name: nrf52_ble_phy_disable
 *
 * Description:
 *   Disable the PHY.  This stops the usec timer, disables interrupts, the
 *   RADIO and PPI and sets state to idle.
 *
 ****************************************************************************/

void nrf52_ble_phy_disable(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  ble_phy_stop_usec_timer();
  ble_phy_disable_irq_and_ppi(dev);

  dev->phy.phy_transition_late = 0;
}

/****************************************************************************
 * Name: nrf52_ble_phy_access_addr_get
 ****************************************************************************/

uint32_t nrf52_ble_phy_access_addr_get(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  return dev->phy.phy_access_address;
}

/****************************************************************************
 * Name: nrf52_ble_phy_state_get
 ****************************************************************************/

int nrf52_ble_phy_state_get(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  return dev->phy.phy_state;
}

/****************************************************************************
 * Name: nrf52_ble_phy_rx_started
 ****************************************************************************/

int nrf52_ble_phy_rx_started(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  return dev->phy.phy_rx_started;
}

/****************************************************************************
 * Name: nrf52_ble_phy_xcvr_state_get
 ****************************************************************************/

uint8_t nrf52_ble_phy_xcvr_state_get(FAR struct ble_ll_lower_s *lower)
{
  return (uint8_t)getreg32(NRF52_RADIO_STATE);
}

/****************************************************************************
 * Name: nrf52_ble_phy_max_data_pdu_pyld
 ****************************************************************************/

uint8_t nrf52_ble_phy_max_data_pdu_pyld(FAR struct ble_ll_lower_s *lower)
{
  return BLE_LL_DATA_PDU_MAX_PYLD;
}

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
/****************************************************************************
 * Name: nrf52_ble_phy_resolv_list_enable
 ****************************************************************************/

void nrf52_ble_phy_resolv_list_enable(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  putreg32(dev->num_irks, NRF52_AAR_NIRK);
  dev->phy.phy_privacy = 1;
}

/****************************************************************************
 * Name: nrf52_ble_phy_resolv_list_disable
 ****************************************************************************/

void nrf52_ble_phy_resolv_list_disable(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  dev->phy.phy_privacy = 0;
}
#endif

/****************************************************************************
 * Name: nrf52_ble_phy_rfclk_enable
 ****************************************************************************/

void nrf52_ble_phy_rfclk_enable(FAR struct ble_ll_lower_s *lower)
{
  nrf52_clock_hfxo_request();
}

/****************************************************************************
 * Name: nrf52_ble_phy_rfclk_disable
 ****************************************************************************/

void nrf52_ble_phy_rfclk_disable(FAR struct ble_ll_lower_s *lower)
{
  nrf52_clock_hfxo_release();
}

/****************************************************************************
 * Name: nrf52_ble_phy_tifs_txtx_set
 ****************************************************************************/

void nrf52_ble_phy_tifs_txtx_set(FAR struct ble_ll_lower_s *lower,
                                 uint16_t usecs, uint8_t anchor)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  dev->phy.txtx_time_us     = usecs;
  dev->phy.txtx_time_anchor = anchor;
}
