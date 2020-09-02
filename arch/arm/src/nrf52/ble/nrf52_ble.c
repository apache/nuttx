/****************************************************************************
 * /home/v01d/coding/nuttx_nrf_ble/nuttx/arch/arm/src/nrf52/nrf52_ble.c
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

/* The buffer where the RADIO peripheral will write data and read from */

uint8_t g_ble_buffer[BLUETOOTH_LE_PDU_MAXSIZE + 1];

/* Premade empty PDU header */

const struct bt_data_pdu_hdr_s g_empty_pdu =
{
  .llid   = BT_LE_DATA_PDU1,
  .nesn   = 0,
  .sn     = 0,
  .md     = 0,
  .rfu    = 0,
  .length = 0,
  .rfu2   = 0,
};

/* Instance to store the state of the BLE device */

struct nrf52_ble_dev_s g_ble_dev =
{
  .channel = 0,
  .state   = BT_LL_STATE_STANDBY,
};

/* The lower half of the link-layer */

static const struct bt_ll_lowerhalf_s g_lower =
{
  .state                = nrf52_ble_state,
  .get_remote_features  = nrf52_ble_get_remote_features,
  .set_randaddr         = nrf52_ble_set_random_addr,
  .setup_scan           = nrf52_ble_setup_scan,
  .scan                 = nrf52_ble_scan,
  .set_advdata          = nrf52_ble_set_advdata,
  .set_rspdata          = nrf52_ble_set_rspdata,
  .setup_advertise      = nrf52_ble_setup_advertise,
  .advertise            = nrf52_ble_advertise,
  .send_data            = nrf52_ble_send_data,
  .reset                = nrf52_ble_reset,
  .txpower              = nrf52_ble_txpower,
  .setchannel           = nrf52_ble_setchannel,
  .getaddress           = nrf52_ble_getaddress,
  .rand                 = nrf52_ble_rand,
  .set_ltk              = nrf52_ble_set_ltk,
};

/* Global pointer to RADIO peripheral interface */

struct nrf52_radio_dev_s *g_radio_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_ble_ll_initialize
 *
 * Description:
 *   Initialize the lower-half of the Link Layer. The resulting pointer
 *   should be used to initialize the upper-half.
 *
 ****************************************************************************/

const struct bt_ll_lowerhalf_s *nrf52_ble_ll_initialize(void)
{
  struct bt_hci_cp_le_set_scan_rsp_data_s rsp_data;
  struct bt_hci_cp_le_set_adv_data_s adv_data;

  /* Instantiate low-level generic radio interface */

  g_radio_dev = nrf52_radio_initialize(0, NULL);

  /* Configure radio for BLE mode */

  nrf52_ble_configure();

  /* Initialize some data */

  rsp_data.len = adv_data.len = 0;
  nrf52_ble_set_rspdata(&rsp_data);
  nrf52_ble_set_advdata(&adv_data);

  /* Initialize security component */

  nrf52_ble_sec_initialize();

  /* Initialize data buffer handling semaphore */

  nxsem_init(&g_ble_dev.data_buffer_sem, 0, 1);

  /* Initialize data buffer with empty PDUs */

  memcpy(g_ble_dev.data_buffer[0], (uint8_t *)&g_empty_pdu,
      sizeof(g_empty_pdu));

  memcpy(g_ble_dev.data_buffer[1], (uint8_t *)&g_empty_pdu,
      sizeof(g_empty_pdu));

  g_ble_dev.active_data_buffer = 0;

  /* Setup RTC instances */

  g_ble_dev.rtc = nrf52_rtc_init(BLE_RTC_INSTANCE);

  NRF52_RTC_STOP(g_ble_dev.rtc);
  NRF52_RTC_SETPRE(g_ble_dev.rtc, BLE_RTC_PRESCALER);
  NRF52_RTC_CLEAR(g_ble_dev.rtc);

  NRF52_RTC_SETISR(g_ble_dev.rtc, &nrf52_ble_rtc_isr, NULL);

  g_ble_dev.rtc2 = nrf52_rtc_init(BLE_RTC2_INSTANCE);

  NRF52_RTC_STOP(g_ble_dev.rtc2);
  NRF52_RTC_SETPRE(g_ble_dev.rtc2, BLE_RTC_PRESCALER);
  NRF52_RTC_CLEAR(g_ble_dev.rtc2);

  NRF52_RTC_SETISR(g_ble_dev.rtc2, &nrf52_ble_rtc2_isr, NULL);

  /* Setup TIMER
   *
   * Use 1MHz frequency, which give 1uSec resolution, good enough to be
   * within IFS timing expectations. This consumes considerable less power
   * than using frequencies higher than 1MHz.
   * We enable CLEAR and STOP shorts so this is a oneshot timer.
   */

  g_ble_dev.tim = nrf52_tim_init(BLE_TIM_INSTANCE);
  NRF52_TIM_CONFIGURE(g_ble_dev.tim, NRF52_TIM_MODE_TIMER,
                      NRF52_TIM_WIDTH_8B);
  NRF52_TIM_SETPRE(g_ble_dev.tim, NRF52_TIM_PRE_1000000);
  NRF52_TIM_SHORTS(g_ble_dev.tim, NRF52_TIM_SHORT_COMPARE_CLEAR, 0, true);
  NRF52_TIM_SHORTS(g_ble_dev.tim, NRF52_TIM_SHORT_COMPARE_STOP, 0, true);
  NRF52_TIM_CLEAR(g_ble_dev.tim);

  /* The following enables low-level tracing of RADIO state
   * via GPIOs: we use PPI to change GPIO states directly via hardware
   */

#if BLE_PINDEBUG
  nrf52_ppi_set_event_ep(15, NRF52_RADIO_EVENTS_ADDRESS);
  nrf52_ppi_set_task_ep(15, NRF52_GPIOTE_TASKS_SET(1));
  nrf52_ppi_channel_enable(15, true);

  nrf52_ppi_set_event_ep(16, NRF52_RADIO_EVENTS_END);
  nrf52_ppi_set_task_ep(16, NRF52_GPIOTE_TASKS_CLR(1));
  nrf52_ppi_set_task2_ep(16, NRF52_GPIOTE_TASKS_CLR(2));
  nrf52_ppi_channel_enable(16, true);

  nrf52_ppi_set_event_ep(17, NRF52_RADIO_EVENTS_READY);
  nrf52_ppi_set_task_ep(17, NRF52_GPIOTE_TASKS_SET(2));
  nrf52_ppi_channel_enable(17, true);

  nrf52_ppi_set_event_ep(18, NRF52_RADIO_EVENTS_DISABLED);
  nrf52_ppi_set_task_ep(18, NRF52_GPIOTE_TASKS_CLR(2));
  nrf52_ppi_channel_enable(18, true);

  nrf52_ppi_set_event_ep(19, BLE_RTC2_BASE +
                         NRF52_RTC_EVENTS_COMPARE_OFFSET(0));
  nrf52_ppi_set_task_ep(19, NRF52_GPIOTE_TASKS_OUT(3));
  nrf52_ppi_channel_enable(19, true);

  nrf52_ppi_set_event_ep(14, BLE_RTC2_BASE +
                         NRF52_RTC_EVENTS_COMPARE_OFFSET(1));
  nrf52_ppi_set_task_ep(14, NRF52_GPIOTE_TASKS_OUT(3));
  nrf52_ppi_channel_enable(14, true);

  nrf52_gpiotaskset(GPIO_PORT0 | GPIO_PIN(BLE_PINDEBUG_ADDRESS_END), 1,
                    false, NRF52_GPIOTE_SET);
  nrf52_gpiotaskset(GPIO_PORT0 | GPIO_PIN(BLE_PINDEBUG_READY_DISABLED), 2,
                    false, NRF52_GPIOTE_SET);
  nrf52_gpiotaskset(GPIO_PORT0 | GPIO_PIN(BLE_PINDEBUG_RX_WINDOW), 3,
                    false, NRF52_GPIOTE_TOGGLE);
  nrf52_gpiotaskset(GPIO_PORT0 | GPIO_PIN(BLE_PINDEBUG_AUX), 4, false,
                    NRF52_GPIOTE_TOGGLE);
  nrf52_gpiotaskset(GPIO_PORT0 | GPIO_PIN(BLE_PINDEBUG_AUX2), 5, false,
                    NRF52_GPIOTE_TOGGLE);
  nrf52_gpiotaskset(GPIO_PORT0 | GPIO_PIN(BLE_PINDEBUG_AUX3), 6, false,
                    NRF52_GPIOTE_TOGGLE);
#endif

  return &g_lower;
}
