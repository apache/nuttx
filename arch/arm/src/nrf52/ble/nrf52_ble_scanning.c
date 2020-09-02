/****************************************************************************
 * arch/arm/src/chip/ble/nrf52_ble_scanning.c
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

#include "nrf52_ble.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_ble_setup_scan
 *
 * Description:
 *   This function is called when the host layer sets the scan parameters.
 *
 ****************************************************************************/

int nrf52_ble_setup_scan(struct bt_hci_cp_le_set_scan_params_s *params)
{
  memcpy(&g_ble_dev.scan_parameters, params, sizeof(*params));

  return OK;
}

/****************************************************************************
 * Name: nrf52_ble_scan
 *
 * Description:
 *   This function is called when the host layer wants to start/stop the scan
 *
 ****************************************************************************/

int nrf52_ble_scan(bool enable)
{
  int ret = OK;

  if (enable)
    {
      uint32_t window = g_ble_dev.scan_parameters.window * 625;
      uint32_t interval = g_ble_dev.scan_parameters.interval * 625;

      /* Ensure we're in STANDBY state */

      if (g_ble_dev.state != BT_LL_STATE_STANDBY)
        {
          return -EBUSY;
        }

      /* If window < interval, there is some "off time" between scans.
       * For every new scan window, a different channel will be used to
       * listen. When window == interval, scanning is continuous. However,
       * to switch channels we also stop and start listening. Thus, the
       * following logic works on both cases. However, to ensure correct
       * order of operations (first disable RADIO, then restart scan) in
       * this case we add a tiny offset to the expiration times.
       */

      /* First we configure the RTC to trigger after the given scan window
       * is over. The RTC COMPARE0 event is mapped to the RADIO DISABLE task
       * using the PPI channel 0.
       */

      /* TODO: this should be:
       *   nrf52_ble_set_rtc_timeout(1, 0, window);
       * TEST!
       */

      nrf52_ble_set_rtc_timeout(1, 0, 32768);

      NRF52_RTC_ENABLEEVT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE0);
      nrf52_ppi_set_event_ep(0, NRF52_RTC_EVENTS_COMPARE_OFFSET(0)
                                            + BLE_RTC_BASE);
      nrf52_ppi_set_task_ep(0, NRF52_RADIO_TASKS_DISABLE);
      nrf52_ppi_set_task2_ep(0, 0);
      nrf52_ppi_channel_enable(0, true);

      /* We then setup an RTC COMPARE1 event for the scanning interval
       * to start a new scan window by resetting the RTC counter
       * using PPI channel 1.
       */

      /* TODO: this should be:
       *   nrf52_ble_set_rtc_timeout(1, 1, interval * 100 +
       *                             (window == interval ? 1 : 0));
       * TEST!
       */

      nrf52_ble_set_rtc_timeout(1, 1, 32768 + (window == interval ? 1 : 0));

      NRF52_RTC_ENABLEEVT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE1);
      nrf52_ppi_set_event_ep(1, NRF52_RTC_EVENTS_COMPARE_OFFSET(1)
                                            + BLE_RTC_BASE);
      nrf52_ppi_set_task_ep(1, NRF52_RTC_TASKS_CLEAR_OFFSET +
                                           BLE_RTC_BASE);
      nrf52_ppi_set_task2_ep(1, 0);
      nrf52_ppi_channel_enable(1, true);

      /* Finally, we configure an interrupt for COMPARE1 event so that
       * the actual scan is restarted when the interval is over
       */

      NRF52_RTC_ENABLEINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE1);

      /* configure ISR to run on CRCOK event so that only valid packets
       * are considered.
       */

      putreg32(0, NRF52_RADIO_EVENTS_CRCOK);
      putreg32(RADIO_INT_CRCOK, NRF52_RADIO_INTENSET);

      /* Enable READY->START shortcut only, we will restart RX manually
       * once a packet is processed.
       */

      putreg32(RADIO_SHORTS_READY_START, NRF52_RADIO_SHORTS);

      /* Enable logical address 0 (ADV access address) for RX and TX */

      putreg32(0x0, NRF52_RADIO_TXADDRESS);
      putreg32(0x1, NRF52_RADIO_RXADDRESSES);

      /* Configure packet for advertising PDUs */

      g_radio_dev->ops->pkt_cfg(g_radio_dev, &g_ble_adv_pkt_config);

      /* Set the new state */

      g_ble_dev.state = BT_LL_STATE_SCANNING;

      /* start the RTC, which times the scanning procedure */

      NRF52_RTC_CLEAR(g_ble_dev.rtc);
      NRF52_RTC_START(g_ble_dev.rtc);

      /* Kick off channel switching */

      g_ble_dev.channel = BLUETOOTH_LE_FIRST_ADV_CH;

      /* start scanning */

      ret = nrf52_ble_trigger_scan(true);

      if (ret < 0)
        {
          /* Something failed, go back to STANDBY */

          g_ble_dev.state = BT_LL_STATE_STANDBY;
        }
    }
  else
    {
      /* Ensure we're indeed scanning */

      if (g_ble_dev.state != BT_LL_STATE_SCANNING)
        {
          return -EINVAL;
        }

      /* First stop the timer */

      NRF52_RTC_STOP(g_ble_dev.rtc);

      /* Now stop reception, in case it was active */

      putreg32(0, NRF52_RADIO_SHORTS);
      putreg32(0, NRF52_RADIO_EVENTS_DISABLED);
      putreg32(1, NRF52_RADIO_TASKS_DISABLE);
      while (!getreg32(NRF52_RADIO_EVENTS_DISABLED));

      /* Go back to standby */

      g_ble_dev.state = BT_LL_STATE_STANDBY;

      /* Disable used PPI channels */

      nrf52_ppi_channel_enable(0, false);
      nrf52_ppi_channel_enable(1, false);

      ret = OK;
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_ble_trigger_scan
 *
 * Description:
 *   This function start a new scan window by selecting the next
 *   advertisement channel and starts reception. An interrupt will be
 *   generated when a valid packet is received and then it will be processed.
 *   This function will be manually invoked the first time and then by a
 *    periodical timer.
 *
 ****************************************************************************/

int nrf52_ble_trigger_scan(bool start)
{
  /* rotate advertising channel */

  if (start || g_ble_dev.channel == BLUETOOTH_LE_LAST_ADV_CH)
    {
      nrf52_ble_setchannel(BLUETOOTH_LE_FIRST_ADV_CH);
    }
  else
    {
      nrf52_ble_setchannel(g_ble_dev.channel + 1);
    }

  wlinfo("trigger scan on ch %i\n", g_ble_dev.channel);

  /* enable RX */

  g_radio_dev->ops->rx_enable(g_radio_dev);

  /* Clear interrupt flag */

  putreg32(0, NRF52_RADIO_EVENTS_CRCOK);

  return OK;
}
