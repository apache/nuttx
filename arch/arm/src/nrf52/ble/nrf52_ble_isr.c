/****************************************************************************
 * arch/arm/src/chip/ble/nrf52_ble_isr.c
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
#include "hardware/nrf52_egu.h"

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
 * Name: nrf52_ble_isr
 *
 * Description:
 *   RADIO ISR, takes care of handling packets.
 *
 ****************************************************************************/

int nrf52_ble_isr(int irq, FAR void *context, FAR void *arg)
{
  putreg32(1, NRF52_GPIOTE_TASKS_OUT(5));

  switch (g_ble_dev.state)
    {
      case BT_LL_STATE_SCANNING:
        {
          /* ack interrupt */

          putreg32(0, NRF52_RADIO_EVENTS_CRCOK);

          /* if we reached here, we know CRC passed, handle this PDU */

          nrf52_handle_adv_packet();
        }
        break;
      case BT_LL_STATE_ADVERTISING:
        {
          if (getreg32(NRF52_RADIO_EVENTS_END))
            {
              /* ack interrupt */

              putreg32(0x0, NRF52_RADIO_EVENTS_END);

              if (getreg32(NRF52_RADIO_STATE) == RADIO_STATE_TXIDLE)
                {
                  /* finished transmitting, go into RX mode */

                  g_radio_dev->ops->rx_enable(g_radio_dev);
                }
            }

          if (getreg32(NRF52_RADIO_EVENTS_CRCOK))
            {
              /* ack interrupt */

              putreg32(0, NRF52_RADIO_EVENTS_CRCOK);

              if (g_ble_dev.adv_parameters.type != BT_LE_ADV_NONCONN_IND)
                {
                  /* handle packet */

                  nrf52_handle_adv_packet();
                }
            }
        }
        break;
      case BT_LL_STATE_CONNECTION:
        {
          if ((getreg32(NRF52_RADIO_INTENSET) & RADIO_INT_END) &&
              getreg32(NRF52_RADIO_EVENTS_END))
            {
              DEBUGASSERT(!g_ble_dev.connections[0].more_data);

              /* Packet just sent */

              uint32_t regval;
              UNUSED(regval);

              putreg32(0, NRF52_RADIO_EVENTS_END);

              regval = getreg32(NRF52_RADIO_STATE);
              DEBUGASSERT(regval == RADIO_STATE_TXIDLE);

              /* Disable C0 INT (don't stop radio and change channel yet) */

              NRF52_RTC_DISABLEINT(g_ble_dev.rtc2, NRF52_RTC_EVT_COMPARE0);
              NRF52_RTC_ACKINT(g_ble_dev.rtc2, NRF52_RTC_EVT_COMPARE0);

              /* no more data expected (end of connection event) */

              nrf52_ble_close_connection_event();
            }

          /* packet received */

          if ((getreg32(NRF52_RADIO_INTENSET) & NRF52_RADIO_EVENTS_CRCOK) &&
                    (getreg32(NRF52_RADIO_EVENTS_CRCOK) ||
                    getreg32(NRF52_RADIO_EVENTS_CRCERROR)))
            {
              bool crc_pass;
              uint32_t window_widening;

              /* we should be ramping up or already done */

              uint32_t regval = getreg32(NRF52_RADIO_STATE);
              uint32_t regval2;

              DEBUGASSERT(regval == RADIO_STATE_TXRU ||
                          regval == RADIO_STATE_TXIDLE);

              crc_pass = getreg32(NRF52_RADIO_EVENTS_CRCOK);

              /* ack ints */

              putreg32(0x0, NRF52_RADIO_EVENTS_CRCOK);
              putreg32(0x0, NRF52_RADIO_EVENTS_CRCERROR);

              /* Set C0 to widening (actually only needed the first time) */

              window_widening =
                  ((BLE_SCA + (uint32_t)g_ble_dev.connections[0].clk_acc) *
                    g_ble_dev.connections[0].conn_interval) / 1000000;

              window_widening += BLE_RADIO_TIME_ADDRESS_TO_CRC +
                                 BLE_RADIO_TIME_RXEN_TO_RXIDLE;

              /* we use twice the value since we're measuring from
               * interval - widening already
               */

              nrf52_ble_set_rtc_timeout(2, 0, 2 * window_widening);
              putreg32(1, NRF52_GPIOTE_TASKS_OUT(4));

              /* Prepare for handling TX END */

              putreg32(0, NRF52_RADIO_EVENTS_END);
              putreg32(RADIO_INT_END, NRF52_RADIO_INTENSET);
              putreg32(RADIO_INT_CRCERROR | RADIO_INT_CRCOK,
                       NRF52_RADIO_INTENCLR);

              /* Prepare for encrypted send, if applicable */

              if (g_ble_dev.connections[0].encryption)
                {
                  nrf52_ble_sec_mode(BLE_SEC_ENCRYPT);
                }

              /* handle current PDU */

              nrf52_handle_data_packet(crc_pass);

#if 1
              /* TODO: the EGU event seems to be set late (it is zero
               * and if I stop the debugger it ends up being one)
               */

              /* check if the TIMER expired and we didn't actually send */

              regval = getreg32(NRF52_EGU0_EVENTS_TRIGGERED0);
              regval2 = getreg32(NRF52_RADIO_STATE);

              if (NRF52_TIM_CHECKINT(g_ble_dev.tim, NRF52_TIM_INT_COMPARE0)
                  && regval != 1 && regval2 == RADIO_STATE_RXIDLE)
                {
                  /* We will have to close this connection event */

                  nrf52_ble_close_connection_event();

                  _info("Closing due to missed TX\n");
                }
#endif
            }
          break;
        }
    }

  putreg32(1, NRF52_GPIOTE_TASKS_OUT(5));

  return OK;
}

/****************************************************************************
 * Name: nrf52_ble_rtc_isr
 *
 * Description:
 *   Main RTC ISR
 *
 ****************************************************************************/

int nrf52_ble_rtc_isr(int irq, FAR void *context, FAR void *arg)
{
  switch (g_ble_dev.state)
    {
      case BT_LL_STATE_SCANNING:
        {
          /* If we're here while scanning it means a new scan window is to be
           * started
           */

          NRF52_RTC_ACKINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE1);

          nrf52_ble_trigger_scan(false);
        }
        break;
      case BT_LL_STATE_ADVERTISING:
        {
          if (NRF52_RTC_CHECKINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE0) ||
              NRF52_RTC_CHECKINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE1))
            {
              /* Timeout for advertising on current channel, go to next
               * one
               */

              NRF52_RTC_ACKINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE0);
              NRF52_RTC_ACKINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE1);

              nrf52_ble_trigger_advertisement(false);
            }
          else if (NRF52_RTC_CHECKINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE2))
            {
              /* End of advertising event, start again from the first
               * channel
               */

              NRF52_RTC_ACKINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE2);

              nrf52_ble_trigger_advertisement(true);
            }
        }
        break;
      case BT_LL_STATE_CONNECTION:
        {
          if (NRF52_RTC_CHECKINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE0))
            {
              NRF52_RTC_ACKINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE0);

              /* Supervision timer expired, abort connection */

              nrf52_ble_abort_connection(0);

              bt_ll_on_connection_closed(0, BT_HCI_ERR_CONNECTION_TIMEOUT);

              _info("Supervision timer expired\n");
            }
        }
        break;
    }

  return OK;
}

/****************************************************************************
 * Name: nrf52_ble_rtc2_isr
 *
 * Description:
 *   Second RTC ISR, used to time connection events
 *
 ****************************************************************************/

int nrf52_ble_rtc2_isr(int irq, FAR void *context, FAR void *arg)
{
  switch (g_ble_dev.state)
    {
      case BT_LL_STATE_CONNECTION:
        {
          if (NRF52_RTC_CHECKINT(g_ble_dev.rtc2, NRF52_RTC_EVT_COMPARE1) &&
              (getreg32(BLE_RTC2_BASE + NRF52_RTC_INTENSET_OFFSET) &
               RTC_INT_COMPARE(1)))
            {
              struct nrf52_ble_conn_s *conn = &g_ble_dev.connections[0];
              uint32_t window_widening;

              /* Change C1 to expire on next connection interval */

              window_widening = ((BLE_SCA + conn->clk_acc) *
                                  conn->conn_interval) / 1000000;

              /* Enlarge window to account for RADIO times */

              window_widening += BLE_RADIO_TIME_ADDRESS_TO_CRC +
                                 BLE_RADIO_TIME_RXEN_TO_RXIDLE;

              nrf52_ble_set_rtc_timeout(2, 1, conn->conn_interval -
                                        window_widening);

              NRF52_RTC_ACKINT(g_ble_dev.rtc2, NRF52_RTC_EVT_COMPARE1);

              /* Update C0 (window close) to account for potentially new
               * window start
               */

              nrf52_ble_set_rtc_timeout(2, 0, conn->winsize +
                                        2 * window_widening);

              /* Enable C0 int to expire on transmision window end */

              NRF52_RTC_ACKINT(g_ble_dev.rtc2, NRF52_RTC_EVT_COMPARE0);
              NRF52_RTC_ENABLEINT(g_ble_dev.rtc2, NRF52_RTC_EVT_COMPARE0);

              /* if we're starting an interval without having closed
               * the previous one due to a wait for more data, we need to
               * switch channel here as well
               */

              if (g_ble_dev.connections[0].more_data)
                {
                  DEBUGASSERT(false);

                  nrf52_ble_next_data_channel(0);

                  g_ble_dev.connections[0].more_data = false;
                }
            }

          if (NRF52_RTC_CHECKINT(g_ble_dev.rtc2, NRF52_RTC_EVT_COMPARE0) &&
              (getreg32(BLE_RTC2_BASE + NRF52_RTC_INTENSET_OFFSET) &
               RTC_INT_COMPARE(0)))
            {
              /* transmission window ended before receiving PDU */

              NRF52_RTC_ACKINT(g_ble_dev.rtc2, NRF52_RTC_EVT_COMPARE0);

              nrf52_ble_close_connection_event();
            }
          break;
        }
    }

  return OK;
}
