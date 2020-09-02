/****************************************************************************
 * arch/arm/src/nrf52/ble/nrf52_ble_advertising.c
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

#define NRF52_LL_ADV_SINGLE_CHANNEL     1  /* Use only first adv. channel */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Separate buffer to store the advertisement PDUs so that we do not have
 * to recreate it everytime and have it ready when needed.
 */

uint8_t g_ble_adv_buffer[BLUETOOTH_LE_PDU_MAXSIZE + 1];

/* As above, but for a scan response message */

uint8_t g_ble_rsp_buffer[BLUETOOTH_LE_PDU_MAXSIZE + 1];

/* Advertisement packet configuration
 *
 * See note in nrf52_ble_connection.c for more details. This differs only
 * slightly with a LENGTH field of one bit larger.
 */

const struct nrf52_radio_pktcfg_s g_ble_adv_pkt_config =
{
  .max_len  = BLUETOOTH_LE_PDU_MAXSIZE - 2, /* Maximum length of payload */
  .stat_len = 0,
  .bal_len  = 3,                            /* 4 bytes (3 base + 1) address length */
  .s0_len   = 1,                            /* S0 field (bytes) */
  .lf_len   = 6,                            /* LENGTH field (bits) */
  .s1_len   = 2,                            /* S1 field (bits) */
  .pl_len   = RADIO_PCNF0_PLEN_8BIT,        /* 8 bit preamble */
  .endian   = RADIO_PCNF1_ENDIAN_LITTLE,    /* LSB first on air */
  .whiteen  = true,                         /* Enable packet whitening */
};

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
 * Name: nrf52_ble_set_advdata
 *
 * Description:
 *   This function receives the advertisement data and saves it
 *
 ****************************************************************************/

int nrf52_ble_set_advdata(struct bt_hci_cp_le_set_adv_data_s *data)
{
  memcpy(&g_ble_dev.adv_data, data, sizeof(*data));

  return OK;
}

/****************************************************************************
 * Name: nrf52_ble_set_rspdata
 *
 * Description:
 *   This function receives the scan response data and saves it
 *
 ****************************************************************************/

int nrf52_ble_set_rspdata(struct bt_hci_cp_le_set_scan_rsp_data_s *
                                 data)
{
  struct bt_adv_pdu_hdr_s *hdr = (struct bt_adv_pdu_hdr_s *)g_ble_rsp_buffer;
  struct bt_scan_resp_pdu_payload_s *payload =
      (struct bt_scan_resp_pdu_payload_s *)(g_ble_rsp_buffer + 3);

  /* copy data into rsp buffer */

  memcpy(payload->scan_resp_data, data->data, data->len);
  hdr->length = BLUETOOTH_ADDRSIZE + data->len;

  /* finish setting up PDU */

  hdr->type = BT_LE_PDUTYPE_SCAN_RSP;

  return OK;
}

/****************************************************************************
 * Name: nrf52_ble_setup_advertise
 *
 * Description:
 *   This function receives the advertisement parameters and saves them
 *
 ****************************************************************************/

int nrf52_ble_setup_advertise(struct bt_hci_cp_le_set_adv_parameters_s
                              *params)
{
  memcpy(&g_ble_dev.adv_parameters, params, sizeof(*params));

  if ((params->channel_map & 0b111) != 0b111)
    {
      /* TODO: handle this */

      wlwarn("Channel map not supported, ignoring\n");
    }

  return OK;
}

/****************************************************************************
 * Name: nrf52_ble_advertise
 *
 * Description:
 *   This function start advertisment
 *
 ****************************************************************************/

int nrf52_ble_advertise(bool enable)
{
  int ret = OK;

  if (enable)
    {
      struct bt_adv_pdu_hdr_s *pdu_hdr;
      void *payload_ptr;

      /* Ensure we're in STANDBY */

      if (g_ble_dev.state != BT_LL_STATE_STANDBY)
        {
          return -EBUSY;
        }

      /* Get pointers to ADV PDU buffer, considering nRF52 RAM layout */

      pdu_hdr        = (struct bt_adv_pdu_hdr_s *)g_ble_adv_buffer;
      payload_ptr    = g_ble_adv_buffer + 3; /* skip S1 byte */

      /* Configure PDU type */

      pdu_hdr->type =
          bt_ll_advtype_to_pdutype(g_ble_dev.adv_parameters.type);

      /* Set own address (the payload always starts with the ADV address) */

      if (g_ble_dev.adv_parameters.own_addr_type == BT_ADDR_LE_PUBLIC)
        {
          if (nrf52_ble_getaddress((struct bt_addr_s *)payload_ptr) < 0)
            {
              /* In case a public address was not set */

              wlwarn("Public address not set, advertising with dummy "
                     "public address\n");

              memset(payload_ptr, 0xaa, sizeof(struct bt_addr_s));
            }
        }
      else
        {
          memcpy(payload_ptr, &g_ble_dev.random_address,
                 sizeof(struct bt_addr_s));
        }

      /* Also copy own address into scan response buffer */

        {
          struct bt_adv_pdu_hdr_s *rsp_hsr =
              (struct bt_adv_pdu_hdr_s *)g_ble_rsp_buffer;
          uint8_t *rsp_payload_ptr = g_ble_rsp_buffer + 3;

          rsp_hsr->txadd = g_ble_dev.adv_parameters.own_addr_type;
          memcpy(rsp_payload_ptr, payload_ptr, sizeof(struct bt_addr_s));
        }

      /* Fill payload */

      if (g_ble_dev.adv_parameters.type == BT_LE_ADV_DIRECT_IND)
        {
          /* Copy initiator's address into payload */

          struct bt_adv_direct_ind_pdu_payload_s *payload = payload_ptr;

          pdu_hdr->rxadd = g_ble_dev.adv_parameters.direct_addr.type;

          memcpy(payload->init_addr,
                 g_ble_dev.adv_parameters.direct_addr.val,
                 sizeof(BLUETOOTH_ADDRSIZE));

          pdu_hdr->length = sizeof(struct bt_adv_direct_ind_pdu_payload_s);
        }
      else
        {
          /* Copy advertisement data into payload */

          bt_adv_indirect_pdu_payload_t *payload = payload_ptr;

          memcpy(payload->adv_data, &g_ble_dev.adv_data.data,
                 g_ble_dev.adv_data.len);

          pdu_hdr->length = BLUETOOTH_ADDRSIZE + g_ble_dev.adv_data.len;
        }

      if (g_ble_dev.adv_parameters.type != BT_LE_ADV_DIRECT_IND)
        {
          /* TODO: apply filter */

          wlwarn("Advertising filter policy not supported\n");
        }

      /* Set packet pointer */

      g_radio_dev->ops->set_packetptr(g_radio_dev, g_ble_dev.buffer);

      /* Enable logical address 0 for RX and TX, which contains the
       * advertising access address.
       */

      putreg32(0x0, NRF52_RADIO_TXADDRESS);
      putreg32(0x1, NRF52_RADIO_RXADDRESSES);

      /* Use appropriate value for initial CRC */

      putreg32(BLE_CRC_INIT_VALUE, NRF52_RADIO_CRCINIT);

      /* Configure packet format for advertising PDUs */

      g_radio_dev->ops->pkt_cfg(g_radio_dev, &g_ble_adv_pkt_config);

      /* Setup RADIO shortcuts */

      if (g_ble_dev.adv_parameters.type == BT_LE_ADV_NONCONN_IND)
        {
          /* Non-connectable advertisement is TX only, we disable RADIO
           * once PDU is sent (until next channel change).
           */

          putreg32(RADIO_SHORTS_READY_START | RADIO_SHORTS_END_DISABLE,
                   NRF52_RADIO_SHORTS);
        }
      else
        {
          /* Connectable advertisements require listening right after
           * transmitting, so we will leave RADIO enabled. We do not use RXEN
           * shortcut since we need dead time in between to avoid receiving
           * while processing data.
           */

          putreg32(RADIO_SHORTS_READY_START, NRF52_RADIO_SHORTS);
        }

      /* Setup ISR (or not) */

      if (g_ble_dev.adv_parameters.type == BT_LE_ADV_NONCONN_IND)
        {
          /* we do not expect responses */

          putreg32(0xffffffff, NRF52_RADIO_INTENCLR);
        }
      else
        {
          /* We expect responses, which we will process after CRC passes */

          putreg32(0, NRF52_RADIO_EVENTS_CRCOK);
          putreg32(RADIO_INT_CRCOK, NRF52_RADIO_INTENSET);

          /* We also need to manually activate RX right after transmitting,
           * regardless of CRC result, so we do this the RX END.
           */

          putreg32(0, NRF52_RADIO_EVENTS_END);
          putreg32(RADIO_INT_END, NRF52_RADIO_INTENSET);
        }

      /* Setup timer to switch channels during an advertisment event. This
       * also acts as a timeout since it may cancel handling of responses to
       * advertisements on the current channel. This is done so to enforce
       * timings required by the standard for maximum time between
       * successive PDUs within event.
       */

        {
          uint32_t timeout; /* uS */

          if (g_ble_dev.adv_parameters.type == BT_LE_ADV_DIRECT_IND)
            {
              /* Directed advertisements we change channels every 1ms */

              timeout = 1000;
            }
          else
            {
              /* For undirected advertisements the standard mandates that
               * we cannot remain on a channel for more than 10ms
               */

              timeout = 10000;
            }

          /* We set COMPARE0/1 to trigger an interrupt to change into the
           * second and third channel.
           */

          nrf52_ble_set_rtc_timeout(1, 0, timeout);
          nrf52_ble_set_rtc_timeout(1, 1, 2 * timeout);
          NRF52_RTC_ACKINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE0);
          NRF52_RTC_ACKINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE1);
          NRF52_RTC_ENABLEINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE0);
          NRF52_RTC_ENABLEINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE1);
        }

      /* Setup timer to start a new advertisement after a given interval */

      /* We will use COMPARE2 on RTC to match the advertising event
       * duration. We setup PPI channel 0 to make COMPARE2 event trigger
       * RTC counter clear, which will restart the timer for the next event.
       */

      NRF52_RTC_ENABLEEVT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE2);
      nrf52_ppi_set_event_ep(0, NRF52_RTC_EVENTS_COMPARE_OFFSET(2)
                                            + BLE_RTC_BASE);
      nrf52_ppi_set_task_ep(0, NRF52_RTC_TASKS_CLEAR_OFFSET +
                                           BLE_RTC_BASE);
      nrf52_ppi_set_task2_ep(0, 0);
      nrf52_ppi_channel_enable(0, true);

      /* For undirected advertisements, the actual event duration is
       * adv_interval + adv_delay, where this delay is a random number
       * [0,10] ms. Since this changes for every adv. event, we will
       * set the RTC compare register now exacly but modify it later with
       * the random delay.
       *
       * For directed advertisements, we are required to repeat the event
       * at most every 3.75ms, so we can hardcode the value here.
       */

      if (g_ble_dev.adv_parameters.type != BT_LE_ADV_DIRECT_IND)
        {
          uint32_t adv_event = g_ble_dev.adv_parameters.min_interval * 625;

          nrf52_ble_set_rtc_timeout(1, 2, adv_event);
        }
      else
        {
          nrf52_ble_set_rtc_timeout(1, 2, 3750);
        }

      /* We then enable an interrupt for this event so that we can call
       * the routine which starts a new advertising event.
       */

      NRF52_RTC_ACKINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE2);
      NRF52_RTC_ENABLEINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE2);

      /* Setup second RTC to trigger on every message so that by the
       * time we receive a CONNECT_REQ, we're already set to measure
       * the connection interval beginning.
       *
       * We could do this with a TIMER instance but this ensure that
       * we can sleep while waiting for events and the timer will keep
       * counting.
       */

      nrf52_ppi_set_event_ep(1, NRF52_RADIO_EVENTS_ADDRESS);
      nrf52_ppi_set_task_ep(1, NRF52_RTC_TASKS_CLEAR_OFFSET +
                                           BLE_RTC2_BASE);
      nrf52_ppi_set_task2_ep(1, 0);
      nrf52_ppi_channel_enable(1, true);

      NRF52_RTC_CLEAR(g_ble_dev.rtc2);
      NRF52_RTC_START(g_ble_dev.rtc2);

      /* Raise TX power */

      g_radio_dev->ops->txpower_set(g_radio_dev, 4);

      /* Set the new state */

      g_ble_dev.state = BT_LL_STATE_ADVERTISING;

      /* Start counting */

      NRF52_RTC_CLEAR(g_ble_dev.rtc);
      NRF52_RTC_START(g_ble_dev.rtc);

      /* start new advertisement event */

      ret = nrf52_ble_trigger_advertisement(true);

      if (ret < 0)
        {
          /* Something failed, go back to STANDBY */

          g_ble_dev.state = BT_LL_STATE_STANDBY;
        }
    }
  else
    {
      /* Ensure we're advertising */

      if (g_ble_dev.state != BT_LL_STATE_ADVERTISING)
        {
          return -EBUSY;
        }

      /* Stop counting */

      NRF52_RTC_STOP(g_ble_dev.rtc);

      /* Disable RTC ints */

      NRF52_RTC_DISABLEINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE0);
      NRF52_RTC_DISABLEINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE1);
      NRF52_RTC_DISABLEINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE2);

      /* Disable used PPI channels */

      nrf52_ppi_channel_enable(0, false);
      nrf52_ppi_channel_enable(1, false);

      /* Disable any shortcuts needed for disabling the RADIO */

      putreg32(RADIO_SHORTS_READY_START, NRF52_RADIO_SHORTS);

      /* Disable RADIO */

      putreg32(0, NRF52_RADIO_SHORTS);
      putreg32(0, NRF52_RADIO_EVENTS_DISABLED);
      putreg32(1, NRF52_RADIO_TASKS_DISABLE);

      while (!getreg32(NRF52_RADIO_EVENTS_DISABLED))
        {
          /* Wait until RADIO is disabled */
        }

      /* Set the new state */

      g_ble_dev.state = BT_LL_STATE_STANDBY;
    }

  return ret;
}

/****************************************************************************
 * Name: nrf52_ble_trigger_advertisement
 *
 * Description:
 *   Trigger start of an advertisement
 *
 ****************************************************************************/

int nrf52_ble_trigger_advertisement(bool start)
{
  uint32_t adv_event;
  uint32_t adv_delay;

  /* rotate advertising channel */

  /* TODO: consider channel map */

  if (start || g_ble_dev.channel == BLUETOOTH_LE_LAST_ADV_CH)
    {
      nrf52_ble_setchannel(BLUETOOTH_LE_FIRST_ADV_CH);
    }
#if !defined(NRF52_LL_ADV_SINGLE_CHANNEL)
  else
    {
      nrf52_ble_setchannel(g_ble_dev.channel + 1);
    }
#endif

  if (start && g_ble_dev.adv_parameters.type != BT_LE_ADV_DIRECT_IND)
    {
      uint8_t rand_byte;

      /* Setup advertising event period time. For now we just select the
       * minimum interval allowed, this could be later be slowly increased
       * over time up to the maximum to reduce battery consumption.
       */

      adv_event = g_ble_dev.adv_parameters.min_interval * 625;

      /* Add random delay of up to 10ms */

      nrf52_ble_rand(&rand_byte, 1);

      /* 255 * 39 ~ 10000, we create a random value in [0,10000] uS */

      adv_delay = rand_byte * 39;

      adv_event += adv_delay;

      nrf52_ble_set_rtc_timeout(1, 2, adv_event);
    }

  /* Copy advertisement PDU into RADIO buffer */

  memcpy(g_ble_dev.buffer, g_ble_adv_buffer, sizeof(g_ble_dev.buffer));

  if (g_ble_dev.adv_parameters.type != BT_LE_ADV_NONCONN_IND)
    {
      /* Clear interrupt flags */

      putreg32(0, NRF52_RADIO_EVENTS_END);
      putreg32(0, NRF52_RADIO_EVENTS_CRCOK);
    }

  /* Start transmitting. RADIO will be either RX or DISABLED */

  if (getreg32(NRF52_RADIO_STATE) == RADIO_STATE_RX)
    {
      /* stop RX before going to TX */

      putreg32(0x1, NRF52_RADIO_TASKS_STOP);
    }

  /* Start RX */

  g_radio_dev->ops->tx_enable(g_radio_dev);

  return OK;
}

/****************************************************************************
 * Name: nrf52_ble_send_scanrsp
 *
 * Description:
 *   Send a SCAN_RESP packet
 *
 ****************************************************************************/

void nrf52_ble_send_scanrsp(void)
{
  /* copy scan response data into buffer */

  memcpy(g_ble_dev.buffer, g_ble_rsp_buffer, sizeof(g_ble_rsp_buffer));

  /* enable TX to transmit the packet */

  g_radio_dev->ops->tx_enable(g_radio_dev);
}

/****************************************************************************
 * Name: nrf52_handle_adv_packet
 *
 * Description:
 *   Called from RADIO ISR when a valid advertisement PDU arrives, this
 *   will send the packet to the upper-half of the Link Layer or handle
 *   the rest of the communication, according to current state and packet
 *   type.
 *
 ****************************************************************************/

void nrf52_handle_adv_packet(void)
{
  struct bt_adv_pdu_hdr_s *pdu_hdr =
      (struct bt_adv_pdu_hdr_s *)g_ble_dev.buffer;
  uint8_t *payload_ptr = g_ble_dev.buffer + 3;

  uint32_t regval = getreg32(NRF52_RADIO_STATE);
  DEBUGASSERT(regval == RADIO_STATE_RXIDLE);

  if (g_ble_dev.state == BT_LL_STATE_SCANNING)
    {
      /* TODO: send scan_req/conn_req when appropriate (active scanning) */

      /* Pass on to upper layer */

      bt_ll_handle_adv_pdu(pdu_hdr, payload_ptr);

      /* The radio is enabled but no started, so now continue listening */

      putreg32(0x1, NRF52_RADIO_TASKS_START);

      return;
    }
  else if (g_ble_dev.state == BT_LL_STATE_ADVERTISING)
    {
      if ((pdu_hdr->type == BT_LE_PDUTYPE_SCAN_REQ) &&
          (g_ble_dev.adv_parameters.type == BT_LE_ADV_SCAN_IND ||
           g_ble_dev.adv_parameters.type == BT_LE_ADV_IND))
        {
          /* Handle SCAN_REQ PDU */

          struct bt_scan_req_pdu_payload_s *recvd =
              (struct bt_scan_req_pdu_payload_s *)payload_ptr;

          bt_adv_indirect_pdu_payload_t *sent =
              (bt_adv_indirect_pdu_payload_t *)(g_ble_adv_buffer + 3);

          /* scan request received, check if it is for us */

          if (pdu_hdr->rxadd == g_ble_dev.adv_parameters.own_addr_type &&
              BLUETOOTH_ADDRCMP(sent->adv_addr, recvd->adv_addr))
            {
              /* it is, send response */

              nrf52_ble_send_scanrsp();

              return;
            }
        }
      else if (pdu_hdr->type == BT_LE_PDUTYPE_CONNECT_REQ)
        {
          /* Handle CONNECT_REQ PDU */

          struct bt_connect_req_pdu_payload_s *recvd =
              (struct bt_connect_req_pdu_payload_s *)payload_ptr;

          if (g_ble_dev.adv_parameters.type == BT_LE_ADV_IND)
            {
              /* We were using undirected advertising, so any connect request
               * addressed to us initiates a connection
               */

              struct bt_adv_ind_pdu_payload_s *sent =
                  (struct bt_adv_ind_pdu_payload_s *)(g_ble_adv_buffer + 3);

              if (pdu_hdr->rxadd == g_ble_dev.adv_parameters.own_addr_type &&
                  BLUETOOTH_ADDRCMP(sent->adv_addr, recvd->adv_addr))
                {
                  /* Start connection as slave */

                  if (nrf52_ble_start_connection(pdu_hdr, recvd,
                                                 BT_LL_ROLE_SLAVE) == 0)
                    {
                      return;
                    }
                }
            }
          else if (g_ble_dev.adv_parameters.type == BT_LE_ADV_DIRECT_IND)
            {
              /* We were using directed advertising, so we care about a
               * connection request from our target sent to us
               */

              struct bt_adv_direct_ind_pdu_payload_s *sent =
                  (struct bt_adv_direct_ind_pdu_payload_s *)
                      (g_ble_adv_buffer + 3);

              if (pdu_hdr->rxadd == g_ble_dev.adv_parameters.own_addr_type &&
                  BLUETOOTH_ADDRCMP(sent->adv_addr, recvd->adv_addr) &&
                  pdu_hdr->txadd == g_ble_dev.adv_parameters.direct_addr.type
                  && BLUETOOTH_ADDRCMP(sent->init_addr, recvd->init_addr))
                {
                  /* Start connection as slave */

                  if (nrf52_ble_start_connection(pdu_hdr, recvd,
                                                 BT_LL_ROLE_SLAVE) == 0)
                    {
                      return;
                    }
                }
            }
        }
    }

  /* Something failed or invalid packet received, continue listening
   * until next channel change (RADIO is currently in RXIDLE).
   */

  putreg32(0x1, NRF52_RADIO_TASKS_START);
}
