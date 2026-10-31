/****************************************************************************
 * arch/arm/src/nrf52/nrf52_ble.c
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
 * Derived from Apache Mynewt NimBLE (porting/nimble/src/hal_timer.c).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/bluetooth/ble_ll.h>

#include "arm_internal.h"
#include "nrf52_ble.h"
#include "nrf52_ble_priv.h"
#include "hardware/nrf52_rng.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_BLE_LL_TMR_FREQ != 32768
#  error "nRF52 BLE lower half requires CONFIG_BLE_LL_TMR_FREQ=32768"
#endif

/* Time from scheduler timer event to TX start: radio ramp-up and
 * processing delay.
 */

#define NRF52_BLE_SCHED_DELAY_US  (40 + 153)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nrf52_ble_bind(FAR struct ble_ll_lower_s *lower,
                           FAR const struct ble_ll_lowercb_s *cb);
static void nrf52_ble_tmr_start(FAR struct ble_ll_lower_s *lower,
                                uint32_t target);
static void nrf52_ble_tmr_stop(FAR struct ble_ll_lower_s *lower);
static int nrf52_ble_rtc0_isr(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ble_ll_lower_ops_s g_nrf52_ble_ops =
{
  .bind                    = nrf52_ble_bind,
  .init                    = nrf52_ble_phy_init,
  .setchan                 = nrf52_ble_phy_setchan,
  .chan_get                = nrf52_ble_phy_chan_get,
  .access_addr_get         = nrf52_ble_phy_access_addr_get,
  .tifs_txtx_set           = nrf52_ble_phy_tifs_txtx_set,
  .tx_set_start_time       = nrf52_ble_phy_tx_set_start_time,
  .rx_set_start_time       = nrf52_ble_phy_rx_set_start_time,
  .set_txend_cb            = nrf52_ble_phy_set_txend_cb,
  .tx                      = nrf52_ble_phy_tx,
  .restart_rx              = nrf52_ble_phy_restart_rx,
  .wfr_enable              = nrf52_ble_phy_wfr_enable,
  .disable                 = nrf52_ble_phy_disable,
  .rfclk_enable            = nrf52_ble_phy_rfclk_enable,
  .rfclk_disable           = nrf52_ble_phy_rfclk_disable,
  .state_get               = nrf52_ble_phy_state_get,
  .xcvr_state_get          = nrf52_ble_phy_xcvr_state_get,
  .rx_started              = nrf52_ble_phy_rx_started,
  .max_data_pdu_pyld       = nrf52_ble_phy_max_data_pdu_pyld,
  .tx_power_set            = nrf52_ble_phy_tx_power_set,
  .tx_power_round          = nrf52_ble_phy_tx_power_round,
  .tx_power_get            = nrf52_ble_phy_tx_power_get,
#ifdef CONFIG_BLE_LL_PHY
  .mode_set                = nrf52_ble_phy_mode_set,
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
  .encrypt_enable          = nrf52_ble_phy_encrypt_enable,
  .encrypt_header_mask_set = nrf52_ble_phy_encrypt_header_mask_set,
  .encrypt_iv_set          = nrf52_ble_phy_encrypt_iv_set,
  .encrypt_counter_set     = nrf52_ble_phy_encrypt_counter_set,
  .encrypt_disable         = nrf52_ble_phy_encrypt_disable,
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  .resolv_list_enable      = nrf52_ble_phy_resolv_list_enable,
  .resolv_list_disable     = nrf52_ble_phy_resolv_list_disable,
  .resolv_list_clear       = nrf52_ble_hw_resolv_list_clear,
  .resolv_list_add         = nrf52_ble_hw_resolv_list_add,
  .resolv_list_rmv         = nrf52_ble_hw_resolv_list_rmv,
  .resolv_list_size        = nrf52_ble_hw_resolv_list_size,
  .resolv_list_match       = nrf52_ble_hw_resolv_list_match,
#endif
  .whitelist_size          = nrf52_ble_hw_whitelist_size,
  .whitelist_clear         = nrf52_ble_hw_whitelist_clear,
  .whitelist_add           = nrf52_ble_hw_whitelist_add,
  .whitelist_rmv           = nrf52_ble_hw_whitelist_rmv,
  .whitelist_enable        = nrf52_ble_hw_whitelist_enable,
  .whitelist_disable       = nrf52_ble_hw_whitelist_disable,
  .whitelist_match         = nrf52_ble_hw_whitelist_match,
  .encrypt_block           = nrf52_ble_hw_encrypt_block,
  .rng_start               = nrf52_ble_hw_rng_start,
  .rng_stop                = nrf52_ble_hw_rng_stop,
  .get_public_addr         = nrf52_ble_hw_get_public_addr,
  .get_static_addr         = nrf52_ble_hw_get_static_addr,
  .tmr_get                 = nrf52_ble_tmr_get,
  .tmr_start               = nrf52_ble_tmr_start,
  .tmr_stop                = nrf52_ble_tmr_stop,
};

/* One RADIO peripheral per chip */

static struct nrf52_ble_dev_s g_nrf52_ble_dev =
{
  .lower =
  {
    .ops            = &g_nrf52_ble_ops,
    .sched_delay_us = NRF52_BLE_SCHED_DELAY_US,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_ble_bind
 ****************************************************************************/

static void nrf52_ble_bind(FAR struct ble_ll_lower_s *lower,
                           FAR const struct ble_ll_lowercb_s *cb)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;

  dev->cb = cb;
}

/****************************************************************************
 * Name: nrf52_ble_tmr_start
 *
 * Description:
 *   Program RTC0 CC[2] for the given 32-bit expiry time.  The COMPARE
 *   event is only guaranteed if CC is at least 2 ticks ahead of the
 *   counter, so near expiries use the TICK event or a pending interrupt.
 *
 ****************************************************************************/

static void nrf52_ble_tmr_start(FAR struct ble_ll_lower_s *lower,
                                uint32_t target)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;
  irqstate_t flags;
  uintptr_t ccreg;
  int32_t delta_t;
  uint32_t temp;
  uint32_t cntr;

  ccreg = NRF52_BLE_RTC0(NRF52_RTC_CC_OFFSET(NRF52_BLE_RTC_CC_TMR));

  flags = enter_critical_section();

  putreg32(RTC_INT_COMPARE(NRF52_BLE_RTC_CC_TMR),
           NRF52_BLE_RTC0(NRF52_RTC_INTENCLR_OFFSET));

  temp = dev->rtc_cntr;
  cntr = getreg32(NRF52_BLE_RTC0(NRF52_RTC_COUNTER_OFFSET));
  if (getreg32(NRF52_BLE_RTC0(NRF52_RTC_EVENTS_OVRFLW_OFFSET)))
    {
      temp += (1ul << 24);
      cntr  = getreg32(NRF52_BLE_RTC0(NRF52_RTC_COUNTER_OFFSET));
    }

  temp   |= cntr;
  delta_t = (int32_t)(target - temp);

  if (delta_t < 1)
    {
      /* Already expired, fire immediately */

      putreg32(RTC_INT_TICK, NRF52_BLE_RTC0(NRF52_RTC_INTENCLR_OFFSET));
      up_trigger_irq(NRF52_IRQ_RTC0, 0);
    }
  else if (delta_t < 3)
    {
      /* Interrupt on each tick until expired */

      putreg32(RTC_INT_TICK, NRF52_BLE_RTC0(NRF52_RTC_INTENSET_OFFSET));
      if (getreg32(NRF52_BLE_RTC0(NRF52_RTC_COUNTER_OFFSET)) != cntr)
        {
          up_trigger_irq(NRF52_IRQ_RTC0, 0);
        }
    }
  else
    {
      putreg32(RTC_INT_TICK, NRF52_BLE_RTC0(NRF52_RTC_INTENCLR_OFFSET));

      if (delta_t < (1l << 24))
        {
          putreg32(target & 0x00ffffff, ccreg);
        }
      else
        {
          /* Too far ahead, set compare far ahead */

          putreg32((cntr + (1ul << 23)) & 0x00ffffff, ccreg);
        }

      putreg32(RTC_INT_COMPARE(NRF52_BLE_RTC_CC_TMR),
               NRF52_BLE_RTC0(NRF52_RTC_INTENSET_OFFSET));
    }

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf52_ble_tmr_stop
 ****************************************************************************/

static void nrf52_ble_tmr_stop(FAR struct ble_ll_lower_s *lower)
{
  putreg32(RTC_INT_COMPARE(NRF52_BLE_RTC_CC_TMR) | RTC_INT_TICK,
           NRF52_BLE_RTC0(NRF52_RTC_INTENCLR_OFFSET));
}

/****************************************************************************
 * Name: nrf52_ble_rtc0_isr
 ****************************************************************************/

static int nrf52_ble_rtc0_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)arg;
  uintptr_t evt_compare;

  evt_compare = NRF52_BLE_RTC0(NRF52_RTC_EVENTS_COMPARE_OFFSET(
                               NRF52_BLE_RTC_CC_TMR));

  if (getreg32(evt_compare))
    {
      putreg32(0, evt_compare);
    }

  if (getreg32(NRF52_BLE_RTC0(NRF52_RTC_EVENTS_TICK_OFFSET)))
    {
      putreg32(0, NRF52_BLE_RTC0(NRF52_RTC_EVENTS_TICK_OFFSET));
    }

  if (getreg32(NRF52_BLE_RTC0(NRF52_RTC_EVENTS_OVRFLW_OFFSET)))
    {
      putreg32(0, NRF52_BLE_RTC0(NRF52_RTC_EVENTS_OVRFLW_OFFSET));
      dev->rtc_cntr += (1ul << 24);
    }

  /* There is no way to force a COMPARE event so a late timer is handled
   * with a pending interrupt.  Always check the timer queue.
   */

  if (dev->cb != NULL)
    {
      dev->cb->tmr_exp(dev->cb);
    }

  /* Make sure the event is cleared before returning */

  getreg32(evt_compare);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_ble_tmr_get
 *
 * Description:
 *   Return the 32-bit RTC0 counter.
 *
 ****************************************************************************/

uint32_t nrf52_ble_tmr_get(FAR struct ble_ll_lower_s *lower)
{
  FAR struct nrf52_ble_dev_s *dev = (FAR struct nrf52_ble_dev_s *)lower;
  irqstate_t flags;
  uint32_t tcntr;
  uint32_t low32;

  flags = enter_critical_section();

  tcntr = dev->rtc_cntr;
  low32 = getreg32(NRF52_BLE_RTC0(NRF52_RTC_COUNTER_OFFSET));
  if (getreg32(NRF52_BLE_RTC0(NRF52_RTC_EVENTS_OVRFLW_OFFSET)))
    {
      tcntr += (1ul << 24);
      dev->rtc_cntr = tcntr;
      low32 = getreg32(NRF52_BLE_RTC0(NRF52_RTC_COUNTER_OFFSET));
      putreg32(0, NRF52_BLE_RTC0(NRF52_RTC_EVENTS_OVRFLW_OFFSET));
      up_trigger_irq(NRF52_IRQ_RTC0, 0);
    }

  tcntr |= low32;

  leave_critical_section(flags);

  return tcntr;
}

/****************************************************************************
 * Name: nrf52_ble_initialize
 *
 * Description:
 *   Initialize the nRF52 radio lower half and register the BLE Link Layer.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nrf52_ble_initialize(void)
{
  int ret;

  /* RTC0: 24-bit counter at 32768 Hz extended to 32 bits in software */

  putreg32(1, NRF52_BLE_RTC0(NRF52_RTC_TASKS_STOP_OFFSET));
  putreg32(1, NRF52_BLE_RTC0(NRF52_RTC_TASKS_CLEAR_OFFSET));
  putreg32(0, NRF52_BLE_RTC0(NRF52_RTC_PRESCALER_OFFSET));
  putreg32(0, NRF52_BLE_RTC0(NRF52_RTC_EVENTS_OVRFLW_OFFSET));
  putreg32(RTC_INT_OVRFLW, NRF52_BLE_RTC0(NRF52_RTC_INTENSET_OFFSET));
  putreg32(1, NRF52_BLE_RTC0(NRF52_RTC_TASKS_START_OFFSET));

  /* RNG with bias correction */

  putreg32(RNG_CONFIG_DERCEN, NRF52_RNG_CONFIG);

  /* Interrupts stay at the default priority so that critical sections in
   * the Link Layer mask them.
   */

  irq_attach(NRF52_IRQ_RADIO, nrf52_ble_phy_isr, &g_nrf52_ble_dev);
  irq_attach(NRF52_IRQ_RTC0, nrf52_ble_rtc0_isr, &g_nrf52_ble_dev);
  irq_attach(NRF52_IRQ_RNG, nrf52_ble_hw_rng_isr, &g_nrf52_ble_dev);

  up_enable_irq(NRF52_IRQ_RTC0);
  up_enable_irq(NRF52_IRQ_RNG);

  ret = ble_ll_register(&g_nrf52_ble_dev.lower, 0);
  if (ret < 0)
    {
      wlerr("ERROR: ble_ll_register failed: %d\n", ret);
    }

  return ret;
}
