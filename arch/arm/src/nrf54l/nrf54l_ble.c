/****************************************************************************
 * arch/arm/src/nrf54l/nrf54l_ble.c
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
 * Derived from Apache Mynewt NimBLE (porting/nimble/src/hal_timer.c) and
 * the Zephyr Bluetooth controller
 * (subsys/bluetooth/controller/ll_sw/nordic/hal/nrf5/radio/
 * radio_nrf5_dppi.h, Copyright (c) 2018-2020 Nordic Semiconductor ASA).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/bluetooth/ble_ll.h>

#include "arm_internal.h"
#include "nrf54l_ble.h"
#include "nrf54l_ble_priv.h"
#include "nrf54l_grtc.h"
#include "hardware/nrf54l_grtc.h"
#include "hardware/nrf54l_power.h"
#include "hardware/nrf54l_ppib.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_BLE_LL_TMR_FREQ != 32768
#  error "nRF54L BLE lower half requires CONFIG_BLE_LL_TMR_FREQ=32768"
#endif

/* The Link Layer timer runs at 32768 Hz, the GRTC counter at 1 MHz.
 * ticks = us * 512 / 15625.
 */

#define NRF54L_BLE_US2T(us)   (((uint64_t)(us) * 512) / 15625)
#define NRF54L_BLE_T2US(t)    (((uint64_t)(t) * 15625 + 511) / 512)

/* Time from scheduler timer event to TX start: radio ramp-up and
 * processing delay.
 */

#define NRF54L_BLE_SCHED_DELAY_US  (40 + 153)

/* GRTC interrupt of the Link Layer timer */

#define NRF54L_BLE_GRTC_IRQ   NRF54L_IRQ_GRTC_2

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nrf54l_ble_bind(FAR struct ble_ll_lower_s *lower,
                            FAR const struct ble_ll_lowercb_s *cb);
static void nrf54l_ble_tmr_start(FAR struct ble_ll_lower_s *lower,
                                 uint32_t target);
static void nrf54l_ble_tmr_stop(FAR struct ble_ll_lower_s *lower);
static int nrf54l_ble_grtc_isr(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ble_ll_lower_ops_s g_nrf54l_ble_ops =
{
  .bind                    = nrf54l_ble_bind,
  .init                    = nrf54l_ble_phy_init,
  .setchan                 = nrf54l_ble_phy_setchan,
  .chan_get                = nrf54l_ble_phy_chan_get,
  .access_addr_get         = nrf54l_ble_phy_access_addr_get,
  .tifs_txtx_set           = nrf54l_ble_phy_tifs_txtx_set,
  .tx_set_start_time       = nrf54l_ble_phy_tx_set_start_time,
  .rx_set_start_time       = nrf54l_ble_phy_rx_set_start_time,
  .set_txend_cb            = nrf54l_ble_phy_set_txend_cb,
  .tx                      = nrf54l_ble_phy_tx,
  .restart_rx              = nrf54l_ble_phy_restart_rx,
  .wfr_enable              = nrf54l_ble_phy_wfr_enable,
  .disable                 = nrf54l_ble_phy_disable,
  .rfclk_enable            = nrf54l_ble_phy_rfclk_enable,
  .rfclk_disable           = nrf54l_ble_phy_rfclk_disable,
  .state_get               = nrf54l_ble_phy_state_get,
  .xcvr_state_get          = nrf54l_ble_phy_xcvr_state_get,
  .rx_started              = nrf54l_ble_phy_rx_started,
  .max_data_pdu_pyld       = nrf54l_ble_phy_max_data_pdu_pyld,
  .tx_power_set            = nrf54l_ble_phy_tx_power_set,
  .tx_power_round          = nrf54l_ble_phy_tx_power_round,
  .tx_power_get            = nrf54l_ble_phy_tx_power_get,
#ifdef CONFIG_BLE_LL_PHY
  .mode_set                = nrf54l_ble_phy_mode_set,
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
  .encrypt_enable          = nrf54l_ble_phy_encrypt_enable,
  .encrypt_header_mask_set = nrf54l_ble_phy_encrypt_header_mask_set,
  .encrypt_iv_set          = nrf54l_ble_phy_encrypt_iv_set,
  .encrypt_counter_set     = nrf54l_ble_phy_encrypt_counter_set,
  .encrypt_disable         = nrf54l_ble_phy_encrypt_disable,
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  .resolv_list_enable      = nrf54l_ble_phy_resolv_list_enable,
  .resolv_list_disable     = nrf54l_ble_phy_resolv_list_disable,
  .resolv_list_clear       = nrf54l_ble_hw_resolv_list_clear,
  .resolv_list_add         = nrf54l_ble_hw_resolv_list_add,
  .resolv_list_rmv         = nrf54l_ble_hw_resolv_list_rmv,
  .resolv_list_size        = nrf54l_ble_hw_resolv_list_size,
  .resolv_list_match       = nrf54l_ble_hw_resolv_list_match,
#endif
  .get_public_addr         = nrf54l_ble_hw_get_public_addr,
  .get_static_addr         = nrf54l_ble_hw_get_static_addr,
  .tmr_get                 = nrf54l_ble_tmr_get,
  .tmr_start               = nrf54l_ble_tmr_start,
  .tmr_stop                = nrf54l_ble_tmr_stop,
};

/* One RADIO peripheral per chip */

static struct nrf54l_ble_dev_s g_nrf54l_ble_dev =
{
  .lower =
  {
    .ops            = &g_nrf54l_ble_ops,
    .sched_delay_us = NRF54L_BLE_SCHED_DELAY_US,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_ble_bind
 ****************************************************************************/

static void nrf54l_ble_bind(FAR struct ble_ll_lower_s *lower,
                            FAR const struct ble_ll_lowercb_s *cb)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  dev->cb = cb;
}

/****************************************************************************
 * Name: nrf54l_ble_tmr_start
 *
 * Description:
 *   Program GRTC CC[2] for the given 32-bit expiry time.  An expired target
 *   or a compare value that may not match pends the interrupt instead.
 *
 ****************************************************************************/

static void nrf54l_ble_tmr_start(FAR struct ble_ll_lower_s *lower,
                                 uint32_t target)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;
  irqstate_t flags;
  uint64_t now_t;
  uint64_t now;
  uint64_t cc;
  int32_t delta_t;

  flags = enter_critical_section();

  NRF54L_GRTC_DISABLEINT(dev->grtc, NRF54L_BLE_GRTC_CC_TMR);
  NRF54L_GRTC_ACKINT(dev->grtc, NRF54L_BLE_GRTC_CC_TMR);

  now     = nrf54l_ble_grtc_now(dev);
  now_t   = NRF54L_BLE_US2T(now);
  delta_t = (int32_t)(target - (uint32_t)now_t);

  if (delta_t < 1)
    {
      /* Already expired, fire immediately */

      up_trigger_irq(NRF54L_BLE_GRTC_IRQ, 0);
    }
  else
    {
      cc = NRF54L_BLE_T2US(now_t + delta_t);

      /* Move a due compare value ahead of the counter, repeat until it is
       * ahead or the event fired.
       */

      for (; ; )
        {
          if (cc <= now + NRF54L_BLE_GRTC_MARGIN)
            {
              cc = now + NRF54L_BLE_GRTC_MARGIN;
            }

          NRF54L_GRTC_SETCC(dev->grtc, NRF54L_BLE_GRTC_CC_TMR, cc);

          now = nrf54l_ble_grtc_now(dev);
          if (cc > now ||
              NRF54L_GRTC_CHECKINT(dev->grtc, NRF54L_BLE_GRTC_CC_TMR))
            {
              break;
            }
        }

      NRF54L_GRTC_ENABLEINT(dev->grtc, NRF54L_BLE_GRTC_CC_TMR);
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf54l_ble_tmr_stop
 ****************************************************************************/

static void nrf54l_ble_tmr_stop(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  NRF54L_GRTC_DISABLEINT(dev->grtc, NRF54L_BLE_GRTC_CC_TMR);
  NRF54L_GRTC_DISABLECC(dev->grtc, NRF54L_BLE_GRTC_CC_TMR);
}

/****************************************************************************
 * Name: nrf54l_ble_grtc_isr
 ****************************************************************************/

static int nrf54l_ble_grtc_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)arg;

  if (NRF54L_GRTC_CHECKINT(dev->grtc, NRF54L_BLE_GRTC_CC_TMR))
    {
      NRF54L_GRTC_ACKINT(dev->grtc, NRF54L_BLE_GRTC_CC_TMR);
    }

  /* A late timer is handled with a pending interrupt, always check the
   * timer queue.
   */

  if (dev->cb != NULL)
    {
      dev->cb->tmr_exp(dev->cb);
    }

  return OK;
}

/****************************************************************************
 * Name: nrf54l_ble_dppi_init
 *
 * Description:
 *   Connect the events and tasks that cross power domains: GRTC CC[3] to
 *   TIMER10 START and RADIO BCMATCH and PAYLOAD to the AAR and CCM START
 *   task.
 *
 ****************************************************************************/

static void nrf54l_ble_dppi_init(void)
{
  /* GRTC (PERI domain) -> PPIB21 -> PPIB11 -> TIMER10 (RADIO domain) */

  putreg32(NRF54L_BLE_DPPI_START | GRTC_PUBLISH_EN,
           NRF54L_GRTC_BASE +
           NRF54L_GRTC_PUBLISH_COMPARE_OFFSET(NRF54L_BLE_GRTC_CC_START));
  putreg32(NRF54L_BLE_DPPI_START | PPIB_SUBSCRIBE_EN,
           NRF54L_PPIB21_BASE +
           NRF54L_PPIB_SUBSCRIBE_SEND_OFFSET(NRF54L_BLE_DPPI_START));
  putreg32(NRF54L_BLE_DPPI_START | PPIB_PUBLISH_EN,
           NRF54L_PPIB11_BASE +
           NRF54L_PPIB_PUBLISH_RECEIVE_OFFSET(NRF54L_BLE_DPPI_START));
  putreg32(DPPIC_CHEN_CH(NRF54L_BLE_DPPI_START),
           NRF54L_DPPIC20_BASE + NRF54L_DPPIC_CHENSET_OFFSET);
  putreg32(NRF54L_BLE_DPPI_START | TIM_SUBSCRIBE_EN,
           NRF54L_BLE_TIMER(NRF54L_TIM_SUBSCRIBE_START_OFFSET));

  /* RADIO (RADIO domain) -> PPIB10 -> PPIB00 -> AAR/CCM (MCU domain) */

  putreg32(NRF54L_BLE_DPPI_AAR | RADIO_PUBLISH_EN,
           NRF54L_RADIO_PUBLISH_BCMATCH);
  putreg32(NRF54L_BLE_DPPI_CCM | RADIO_PUBLISH_EN,
           NRF54L_RADIO_PUBLISH_PAYLOAD);
  putreg32(NRF54L_BLE_DPPI_AAR | PPIB_SUBSCRIBE_EN,
           NRF54L_PPIB10_BASE +
           NRF54L_PPIB_SUBSCRIBE_SEND_OFFSET(NRF54L_BLE_DPPI_AAR));
  putreg32(NRF54L_BLE_DPPI_CCM | PPIB_SUBSCRIBE_EN,
           NRF54L_PPIB10_BASE +
           NRF54L_PPIB_SUBSCRIBE_SEND_OFFSET(NRF54L_BLE_DPPI_CCM));
  putreg32(NRF54L_BLE_DPPI_AAR | PPIB_PUBLISH_EN,
           NRF54L_PPIB00_BASE +
           NRF54L_PPIB_PUBLISH_RECEIVE_OFFSET(NRF54L_BLE_DPPI_AAR));
  putreg32(NRF54L_BLE_DPPI_CCM | PPIB_PUBLISH_EN,
           NRF54L_PPIB00_BASE +
           NRF54L_PPIB_PUBLISH_RECEIVE_OFFSET(NRF54L_BLE_DPPI_CCM));
  putreg32(DPPIC_CHEN_CH(NRF54L_BLE_DPPI_AAR) |
           DPPIC_CHEN_CH(NRF54L_BLE_DPPI_CCM),
           NRF54L_DPPIC00_BASE + NRF54L_DPPIC_CHENSET_OFFSET);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf54l_ble_grtc_now
 *
 * Description:
 *   Return the GRTC counter in microseconds.
 *
 ****************************************************************************/

uint64_t nrf54l_ble_grtc_now(FAR struct nrf54l_ble_dev_s *dev)
{
  uint64_t now = 0;

  NRF54L_GRTC_GETCOUNTER(dev->grtc, &now);
  return now;
}

/****************************************************************************
 * Name: nrf54l_ble_tmr_t2us
 *
 * Description:
 *   Return the GRTC time of the start of a Link Layer timer tick.  The tick
 *   must be within 2^31 ticks of the current time.
 *
 ****************************************************************************/

uint64_t nrf54l_ble_tmr_t2us(FAR struct nrf54l_ble_dev_s *dev,
                             uint32_t ticks)
{
  uint64_t now_t;

  now_t = NRF54L_BLE_US2T(nrf54l_ble_grtc_now(dev));
  return NRF54L_BLE_T2US(now_t + (int32_t)(ticks - (uint32_t)now_t));
}

/****************************************************************************
 * Name: nrf54l_ble_tmr_us2t
 *
 * Description:
 *   Convert a GRTC time to Link Layer timer ticks and the microseconds
 *   from the start of that tick.
 *
 ****************************************************************************/

void nrf54l_ble_tmr_us2t(uint64_t usecs, FAR uint32_t *ticks,
                         FAR uint32_t *rem_us)
{
  uint64_t t;

  t       = NRF54L_BLE_US2T(usecs);
  *ticks  = (uint32_t)t;
  *rem_us = (uint32_t)(usecs - NRF54L_BLE_T2US(t));
}

/****************************************************************************
 * Name: nrf54l_ble_tmr_get
 *
 * Description:
 *   Return the 32-bit Link Layer timer derived from the GRTC counter.
 *
 ****************************************************************************/

uint32_t nrf54l_ble_tmr_get(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf54l_ble_dev_s *dev = (FAR struct nrf54l_ble_dev_s *)lower;

  return (uint32_t)NRF54L_BLE_US2T(nrf54l_ble_grtc_now(dev));
}

/****************************************************************************
 * Name: nrf54l_ble_initialize
 *
 * Description:
 *   Initialize the nRF54L radio lower half and register the BLE Link Layer.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nrf54l_ble_initialize(void)
{
  FAR struct nrf54l_ble_dev_s *dev = &g_nrf54l_ble_dev;
  int ret;

  dev->grtc = nrf54l_grtc_init(NRF54L_BLE_GRTC);
  if (dev->grtc == NULL)
    {
      wlerr("ERROR: nrf54l_grtc_init(%d) failed\n", NRF54L_BLE_GRTC);
      return -EBUSY;
    }

  /* The GRTC is not reset by a system reset */

  NRF54L_GRTC_DISABLEINT(dev->grtc, NRF54L_BLE_GRTC_CC_TMR);
  NRF54L_GRTC_DISABLECC(dev->grtc, NRF54L_BLE_GRTC_CC_TMR);
  NRF54L_GRTC_DISABLECC(dev->grtc, NRF54L_BLE_GRTC_CC_START);
  NRF54L_GRTC_ACKINT(dev->grtc, NRF54L_BLE_GRTC_CC_TMR);
  NRF54L_GRTC_ACKINT(dev->grtc, NRF54L_BLE_GRTC_CC_START);
  NRF54L_GRTC_START(dev->grtc);

  nrf54l_ble_dppi_init();

  /* Constant latency mode, as used by the Zephyr controller on nRF54L */

  putreg32(POWER_TASKS_CONSTLAT, NRF54L_POWER_TASKS_CONSTLAT);

  /* Interrupts stay at the default priority so that critical sections in
   * the Link Layer mask them.
   */

  irq_attach(NRF54L_IRQ_RADIO_0, nrf54l_ble_phy_isr, dev);
  NRF54L_GRTC_SETISR(dev->grtc, nrf54l_ble_grtc_isr, dev);

  ret = ble_ll_register(&dev->lower, 0);
  if (ret < 0)
    {
      wlerr("ERROR: ble_ll_register failed: %d\n", ret);
    }

  return ret;
}
