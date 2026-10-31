/****************************************************************************
 * arch/sim/src/sim/sim_ble_ll.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <syslog.h>

#include <nuttx/clock.h>
#include <nuttx/wdog.h>
#include <nuttx/wireless/bluetooth/ble_ll.h>

#include "sim_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Null radio instance.  Transmissions are logged and discarded; nothing is
 * ever received.
 */

struct sim_ble_ll_s
{
  struct ble_ll_lower_s lower;
  FAR const struct ble_ll_lowercb_s *cb;
  struct wdog_s wdog;
  uint32_t cmp;           /* Last timer target programmed by tmr_start() */
  uint32_t txcnt;
  uint8_t addr[6];
  uint8_t chan;
  uint32_t access_addr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void sim_ble_ll_bind(FAR struct ble_ll_lower_s *lower,
                            FAR const struct ble_ll_lowercb_s *cb);
static int sim_ble_ll_init(FAR struct ble_ll_lower_s *lower);
static int sim_ble_ll_setchan(FAR struct ble_ll_lower_s *lower,
                              uint8_t chan, uint32_t access_addr,
                              uint32_t crcinit);
static uint8_t sim_ble_ll_chan_get(FAR struct ble_ll_lower_s *lower);
static uint32_t sim_ble_ll_access_addr_get(FAR struct ble_ll_lower_s *lower);
static void sim_ble_ll_tifs_set(FAR struct ble_ll_lower_s *lower,
                                uint16_t tifs);
static void sim_ble_ll_tifs_txtx_set(FAR struct ble_ll_lower_s *lower,
                                     uint16_t usecs, uint8_t anchor);
static int sim_ble_ll_set_start_time(FAR struct ble_ll_lower_s *lower,
                                     uint32_t cputime, uint8_t rem_usecs);
static void sim_ble_ll_set_txend_cb(FAR struct ble_ll_lower_s *lower,
                                    ble_phy_tx_end_func txend_cb,
                                    FAR void *arg);
static int sim_ble_ll_tx(FAR struct ble_ll_lower_s *lower,
                         ble_phy_tx_pducb_t pducb, FAR void *pducb_arg,
                         uint8_t end_trans);
static void sim_ble_ll_noop(FAR struct ble_ll_lower_s *lower);
static void sim_ble_ll_wfr_enable(FAR struct ble_ll_lower_s *lower,
                                  int txrx, uint8_t tx_phy_mode,
                                  uint32_t wfr_usecs);
static int sim_ble_ll_zero(FAR struct ble_ll_lower_s *lower);
static uint8_t sim_ble_ll_zero8(FAR struct ble_ll_lower_s *lower);
static uint8_t
sim_ble_ll_max_data_pdu_pyld(FAR struct ble_ll_lower_s *lower);
static int sim_ble_ll_power(FAR struct ble_ll_lower_s *lower, int dbm);
static void sim_ble_ll_mode_set(FAR struct ble_ll_lower_s *lower,
                                uint8_t tx_phy_mode, uint8_t rx_phy_mode);
static void sim_ble_ll_encrypt_enable(FAR struct ble_ll_lower_s *lower,
                                      FAR const uint8_t *key);
static void
sim_ble_ll_encrypt_header_mask_set(FAR struct ble_ll_lower_s *lower,
                                   uint8_t mask);
static void sim_ble_ll_encrypt_iv_set(FAR struct ble_ll_lower_s *lower,
                                      FAR const uint8_t *iv);
static void sim_ble_ll_encrypt_counter_set(FAR struct ble_ll_lower_s *lower,
                                           uint64_t counter,
                                           uint8_t dir_bit);
static int sim_ble_ll_whitelist_add(FAR struct ble_ll_lower_s *lower,
                                    FAR const uint8_t *addr,
                                    uint8_t addr_type);
static void sim_ble_ll_whitelist_rmv(FAR struct ble_ll_lower_s *lower,
                                     FAR const uint8_t *addr,
                                     uint8_t addr_type);
static int sim_ble_ll_resolv_list_add(FAR struct ble_ll_lower_s *lower,
                                      FAR const uint8_t *irk);
static void sim_ble_ll_resolv_list_rmv(FAR struct ble_ll_lower_s *lower,
                                       int index);
static int sim_ble_ll_get_public_addr(FAR struct ble_ll_lower_s *lower,
                                      FAR uint8_t *addr);
static uint32_t sim_ble_ll_tmr_get(FAR struct ble_ll_lower_s *lower);
static void sim_ble_ll_tmr_expiry(wdparm_t arg);
static void sim_ble_ll_tmr_start(FAR struct ble_ll_lower_s *lower,
                                 uint32_t target);
static void sim_ble_ll_tmr_stop(FAR struct ble_ll_lower_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct ble_ll_lower_ops_s g_sim_ble_ll_ops =
{
  .bind                    = sim_ble_ll_bind,
  .init                    = sim_ble_ll_init,
  .setchan                 = sim_ble_ll_setchan,
  .chan_get                = sim_ble_ll_chan_get,
  .access_addr_get         = sim_ble_ll_access_addr_get,
  .tifs_set                = sim_ble_ll_tifs_set,
  .tifs_txtx_set           = sim_ble_ll_tifs_txtx_set,
  .tx_set_start_time       = sim_ble_ll_set_start_time,
  .rx_set_start_time       = sim_ble_ll_set_start_time,
  .set_txend_cb            = sim_ble_ll_set_txend_cb,
  .tx                      = sim_ble_ll_tx,
  .restart_rx              = sim_ble_ll_noop,
  .wfr_enable              = sim_ble_ll_wfr_enable,
  .disable                 = sim_ble_ll_noop,
  .rfclk_enable            = sim_ble_ll_noop,
  .rfclk_disable           = sim_ble_ll_noop,
  .state_get               = sim_ble_ll_zero,
  .xcvr_state_get          = sim_ble_ll_zero8,
  .rx_started              = sim_ble_ll_zero,
  .max_data_pdu_pyld       = sim_ble_ll_max_data_pdu_pyld,
  .tx_power_set            = sim_ble_ll_power,
  .tx_power_round          = sim_ble_ll_power,
  .tx_power_get            = sim_ble_ll_zero,
  .mode_set                = sim_ble_ll_mode_set,
  .encrypt_enable          = sim_ble_ll_encrypt_enable,
  .encrypt_header_mask_set = sim_ble_ll_encrypt_header_mask_set,
  .encrypt_iv_set          = sim_ble_ll_encrypt_iv_set,
  .encrypt_counter_set     = sim_ble_ll_encrypt_counter_set,
  .encrypt_disable         = sim_ble_ll_noop,
  .resolv_list_enable      = sim_ble_ll_noop,
  .resolv_list_disable     = sim_ble_ll_noop,
  .whitelist_size          = sim_ble_ll_zero8,
  .whitelist_clear         = sim_ble_ll_noop,
  .whitelist_add           = sim_ble_ll_whitelist_add,
  .whitelist_rmv           = sim_ble_ll_whitelist_rmv,
  .whitelist_enable        = sim_ble_ll_noop,
  .whitelist_disable       = sim_ble_ll_noop,
  .whitelist_match         = sim_ble_ll_zero,
  .resolv_list_clear       = sim_ble_ll_noop,
  .resolv_list_add         = sim_ble_ll_resolv_list_add,
  .resolv_list_rmv         = sim_ble_ll_resolv_list_rmv,
  .resolv_list_size        = sim_ble_ll_zero8,
  .resolv_list_match       = sim_ble_ll_zero,
  .get_public_addr         = sim_ble_ll_get_public_addr,
  .tmr_get                 = sim_ble_ll_tmr_get,
  .tmr_start               = sim_ble_ll_tmr_start,
  .tmr_stop                = sim_ble_ll_tmr_stop,

  /* encrypt_block, rng_start, rng_stop, get_static_addr: NULL selects the
   * software fallback in the upper half.
   */
};

static struct sim_ble_ll_s g_sim_ble_ll[CONFIG_SIM_BLE_LL_NINSTANCES];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_ble_ll_bind
 ****************************************************************************/

static void sim_ble_ll_bind(FAR struct ble_ll_lower_s *lower,
                            FAR const struct ble_ll_lowercb_s *cb)
{
  FAR struct sim_ble_ll_s *dev = (FAR struct sim_ble_ll_s *)lower;

  dev->cb = cb;
}

/****************************************************************************
 * Name: sim_ble_ll_init
 ****************************************************************************/

static int sim_ble_ll_init(FAR struct ble_ll_lower_s *lower)
{
  return 0;
}

/****************************************************************************
 * Name: sim_ble_ll_setchan
 ****************************************************************************/

static int sim_ble_ll_setchan(FAR struct ble_ll_lower_s *lower,
                              uint8_t chan, uint32_t access_addr,
                              uint32_t crcinit)
{
  FAR struct sim_ble_ll_s *dev = (FAR struct sim_ble_ll_s *)lower;

  dev->chan = chan;
  dev->access_addr = access_addr;
  return 0;
}

/****************************************************************************
 * Name: sim_ble_ll_chan_get
 ****************************************************************************/

static uint8_t sim_ble_ll_chan_get(FAR struct ble_ll_lower_s *lower)
{
  FAR struct sim_ble_ll_s *dev = (FAR struct sim_ble_ll_s *)lower;

  return dev->chan;
}

/****************************************************************************
 * Name: sim_ble_ll_access_addr_get
 ****************************************************************************/

static uint32_t sim_ble_ll_access_addr_get(FAR struct ble_ll_lower_s *lower)
{
  FAR struct sim_ble_ll_s *dev = (FAR struct sim_ble_ll_s *)lower;

  return dev->access_addr;
}

/****************************************************************************
 * Name: sim_ble_ll_tifs_set
 ****************************************************************************/

static void sim_ble_ll_tifs_set(FAR struct ble_ll_lower_s *lower,
                                uint16_t tifs)
{
}

/****************************************************************************
 * Name: sim_ble_ll_tifs_txtx_set
 ****************************************************************************/

static void sim_ble_ll_tifs_txtx_set(FAR struct ble_ll_lower_s *lower,
                                     uint16_t usecs, uint8_t anchor)
{
}

/****************************************************************************
 * Name: sim_ble_ll_set_start_time
 *
 * Description:
 *   Shared stub for tx_set_start_time and rx_set_start_time.
 *
 ****************************************************************************/

static int sim_ble_ll_set_start_time(FAR struct ble_ll_lower_s *lower,
                                     uint32_t cputime, uint8_t rem_usecs)
{
  return 0;
}

/****************************************************************************
 * Name: sim_ble_ll_set_txend_cb
 ****************************************************************************/

static void sim_ble_ll_set_txend_cb(FAR struct ble_ll_lower_s *lower,
                                    ble_phy_tx_end_func txend_cb,
                                    FAR void *arg)
{
}

/****************************************************************************
 * Name: sim_ble_ll_tx
 *
 * Description:
 *   Discard the PDU.  Logs once per instance so a test can confirm that
 *   this radio, and not another instance, transmitted.
 *
 ****************************************************************************/

static int sim_ble_ll_tx(FAR struct ble_ll_lower_s *lower,
                         ble_phy_tx_pducb_t pducb, FAR void *pducb_arg,
                         uint8_t end_trans)
{
  FAR struct sim_ble_ll_s *dev = (FAR struct sim_ble_ll_s *)lower;

  if (dev->txcnt == 0)
    {
      syslog(LOG_INFO, "sim_ble_ll%d: TX\n", dev->addr[0]);
    }

  dev->txcnt++;
  return 0;
}

/****************************************************************************
 * Name: sim_ble_ll_noop
 *
 * Description:
 *   Shared stub for the ops that have no state to change.
 *
 ****************************************************************************/

static void sim_ble_ll_noop(FAR struct ble_ll_lower_s *lower)
{
}

/****************************************************************************
 * Name: sim_ble_ll_wfr_enable
 ****************************************************************************/

static void sim_ble_ll_wfr_enable(FAR struct ble_ll_lower_s *lower,
                                  int txrx, uint8_t tx_phy_mode,
                                  uint32_t wfr_usecs)
{
}

/****************************************************************************
 * Name: sim_ble_ll_zero
 *
 * Description:
 *   Shared stub for the ops that report an idle/empty int status.
 *
 ****************************************************************************/

static int sim_ble_ll_zero(FAR struct ble_ll_lower_s *lower)
{
  return 0;
}

/****************************************************************************
 * Name: sim_ble_ll_zero8
 *
 * Description:
 *   Shared stub for the ops that report an idle/empty uint8_t status.
 *
 ****************************************************************************/

static uint8_t sim_ble_ll_zero8(FAR struct ble_ll_lower_s *lower)
{
  return 0;
}

/****************************************************************************
 * Name: sim_ble_ll_max_data_pdu_pyld
 ****************************************************************************/

static uint8_t
sim_ble_ll_max_data_pdu_pyld(FAR struct ble_ll_lower_s *lower)
{
  return BLE_LL_DATA_PDU_MAX_PYLD;
}

/****************************************************************************
 * Name: sim_ble_ll_power
 *
 * Description:
 *   Shared stub for tx_power_set and tx_power_round.
 *
 ****************************************************************************/

static int sim_ble_ll_power(FAR struct ble_ll_lower_s *lower, int dbm)
{
  return 0;
}

/****************************************************************************
 * Name: sim_ble_ll_mode_set
 ****************************************************************************/

static void sim_ble_ll_mode_set(FAR struct ble_ll_lower_s *lower,
                                uint8_t tx_phy_mode, uint8_t rx_phy_mode)
{
}

/****************************************************************************
 * Name: sim_ble_ll_encrypt_enable
 ****************************************************************************/

static void sim_ble_ll_encrypt_enable(FAR struct ble_ll_lower_s *lower,
                                      FAR const uint8_t *key)
{
}

/****************************************************************************
 * Name: sim_ble_ll_encrypt_header_mask_set
 ****************************************************************************/

static void
sim_ble_ll_encrypt_header_mask_set(FAR struct ble_ll_lower_s *lower,
                                   uint8_t mask)
{
}

/****************************************************************************
 * Name: sim_ble_ll_encrypt_iv_set
 ****************************************************************************/

static void sim_ble_ll_encrypt_iv_set(FAR struct ble_ll_lower_s *lower,
                                      FAR const uint8_t *iv)
{
}

/****************************************************************************
 * Name: sim_ble_ll_encrypt_counter_set
 ****************************************************************************/

static void sim_ble_ll_encrypt_counter_set(FAR struct ble_ll_lower_s *lower,
                                           uint64_t counter,
                                           uint8_t dir_bit)
{
}

/****************************************************************************
 * Name: sim_ble_ll_whitelist_add
 ****************************************************************************/

static int sim_ble_ll_whitelist_add(FAR struct ble_ll_lower_s *lower,
                                    FAR const uint8_t *addr,
                                    uint8_t addr_type)
{
  return 0;
}

/****************************************************************************
 * Name: sim_ble_ll_whitelist_rmv
 ****************************************************************************/

static void sim_ble_ll_whitelist_rmv(FAR struct ble_ll_lower_s *lower,
                                     FAR const uint8_t *addr,
                                     uint8_t addr_type)
{
}

/****************************************************************************
 * Name: sim_ble_ll_resolv_list_add
 ****************************************************************************/

static int sim_ble_ll_resolv_list_add(FAR struct ble_ll_lower_s *lower,
                                      FAR const uint8_t *irk)
{
  return 0;
}

/****************************************************************************
 * Name: sim_ble_ll_resolv_list_rmv
 ****************************************************************************/

static void sim_ble_ll_resolv_list_rmv(FAR struct ble_ll_lower_s *lower,
                                       int index)
{
}

/****************************************************************************
 * Name: sim_ble_ll_get_public_addr
 ****************************************************************************/

static int sim_ble_ll_get_public_addr(FAR struct ble_ll_lower_s *lower,
                                      FAR uint8_t *addr)
{
  FAR struct sim_ble_ll_s *dev = (FAR struct sim_ble_ll_s *)lower;

  memcpy(addr, dev->addr, sizeof(dev->addr));
  return 0;
}

/****************************************************************************
 * Name: sim_ble_ll_tmr_get
 ****************************************************************************/

static uint32_t sim_ble_ll_tmr_get(FAR struct ble_ll_lower_s *lower)
{
  return (uint32_t)((uint64_t)TICK2USEC(clock_systime_ticks()) *
                     32768 / 1000000);
}

/****************************************************************************
 * Name: sim_ble_ll_tmr_expiry
 ****************************************************************************/

static void sim_ble_ll_tmr_expiry(wdparm_t arg)
{
  FAR struct sim_ble_ll_s *dev = (FAR struct sim_ble_ll_s *)arg;

  dev->cb->tmr_exp(dev->cb);
}

/****************************************************************************
 * Name: sim_ble_ll_tmr_start
 *
 * Description:
 *   Map the lower-half 32768 Hz target onto a wdog_s relative delay in
 *   system clock ticks.
 *
 ****************************************************************************/

static void sim_ble_ll_tmr_start(FAR struct ble_ll_lower_s *lower,
                                 uint32_t target)
{
  FAR struct sim_ble_ll_s *dev = (FAR struct sim_ble_ll_s *)lower;
  int32_t delta;
  clock_t ticks;

  dev->cmp = target;
  delta = (int32_t)(target - sim_ble_ll_tmr_get(lower));
  ticks = delta > 0 ? USEC2TICK((uint64_t)delta * 1000000 / 32768) : 0;

  wd_start(&dev->wdog, ticks, sim_ble_ll_tmr_expiry, (wdparm_t)dev);
}

/****************************************************************************
 * Name: sim_ble_ll_tmr_stop
 ****************************************************************************/

static void sim_ble_ll_tmr_stop(FAR struct ble_ll_lower_s *lower)
{
  FAR struct sim_ble_ll_s *dev = (FAR struct sim_ble_ll_s *)lower;

  wd_cancel(&dev->wdog);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_ble_ll_initialize
 *
 * Description:
 *   Register one null radio instance as Bluetooth HCI device 'devid'.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sim_ble_ll_initialize(int devid)
{
  FAR struct sim_ble_ll_s *dev = &g_sim_ble_ll[devid];

  memset(dev, 0, sizeof(*dev));
  dev->lower.ops = &g_sim_ble_ll_ops;
  dev->addr[0] = devid;
  dev->addr[4] = 0xef;
  dev->addr[5] = 0xbe;

  return ble_ll_register(&dev->lower, devid);
}
