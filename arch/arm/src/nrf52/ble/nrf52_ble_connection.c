/****************************************************************************
 * arch/arm/src/chip/ble/nrf52_ble_connection.c
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

/* TODO:
 * - inhibit WFI/WFE while within connection event? otherwise TIMER would
 *   not work
 * - support md bit
 *
 * General:
 * - Kconfig: add options, check enabled resources (TIMER, RTC, PPIs)
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "nrf52_ble.h"
#include "hardware/nrf52_egu.h"
#include "hardware/nrf52_ccm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* #define NRF52_LL_DISABLE_SUPERVISION_TIMER 1 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void nrf52_ble_count_used_channels(uint8_t conn_id);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* BLE data packet configuration:
 * Address is four bytes (thus: 1 base address + 3 more bytes).
 * S0 is always one byte (the part of the header before the length field)
 * For an advertisement PDU, LENGTH is 6 bits followed by 2 unused bits.
 * For a data PDU, LENGTH is 5 bits with 3 unused bits.
 * The maximum packet length is set to the maximum payload excluding
 * BLE header (thus 39 - 2 = 37 bytes).
 *
 * NOTE: LENGTH and S1 will be stored in two separate bytes in memory by
 * the RADIO peripheral. Thus, the buffer will require an extra byte than
 * usual and upon receiving/transmitting this will have to be fixed to build
 * a proper PDU header.
 */

/* Data packet configuration */

const struct nrf52_radio_pktcfg_s g_ble_data_pkt_config =
{
  .max_len  = BLUETOOTH_LE_PDU_MAXSIZE - 2, /* Maximum length of payload */
  .stat_len = 0,
  .bal_len  = 3,                            /* 4 bytes (3 base + 1) address length */
  .s0_len   = 1,                            /* S0 field (bytes) */
  .lf_len   = 5,                            /* LENGTH field (bits) */
  .s1_len   = 3,                            /* S1 field (bits) */
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

int nrf52_ble_start_connection(struct bt_adv_pdu_hdr_s *hdr,
                               struct bt_connect_req_pdu_payload_s *payload,
                               uint8_t role)
{
  struct nrf52_ble_conn_s *conn;
  int conn_slot;

  /* Master not supported yet */

  ASSERT(role != BT_LL_ROLE_MASTER);

  /* Check if there's an available slot for a new connection
   *
   * NOTE: a slave would be only connected to a single master and until we
   * support simultaneous multiple roles, the following should always
   * return the first slot as free.
   */

  for (conn_slot = 0; conn_slot < BLE_MAX_CONNECTIONS; conn_slot++)
    {
      if (!g_ble_dev.connections[conn_slot].used)
        {
          /* found an unused one, clear it and mark it used */

          memset(&g_ble_dev.connections[conn_slot], 0,
                 sizeof(struct nrf52_ble_conn_s));

          g_ble_dev.connections[conn_slot].used = true;
          break;
        }
    }

  if (conn_slot == BLE_MAX_CONNECTIONS)
    {
      return -ENOSPC;
    }

  conn = &g_ble_dev.connections[conn_slot];

  /* Disable all RADIO interrupts */

  putreg32(0xffffffff, NRF52_RADIO_INTENCLR);

  /* RADIO is RXIDLE, DISABLE until connection interval starts */

  putreg32(0, NRF52_RADIO_SHORTS);
  putreg32(0x1, NRF52_RADIO_TASKS_DISABLE);

  /* Stop main RTC instance for now. RTC2 already started measuring start of
   * connection event and we'll continue using it for the same purpose.
   */

  NRF52_RTC_STOP(g_ble_dev.rtc);
  NRF52_RTC_CLEAR(g_ble_dev.rtc);

  /* Disable all RTC interrupts */

  putreg32(0xffffffff, (BLE_RTC_BASE + NRF52_RTC_INTENCLR_OFFSET));
  putreg32(0xffffffff, (BLE_RTC2_BASE + NRF52_RTC_INTENCLR_OFFSET));

  /* Leave everything ACKd */

  NRF52_RTC_ACKINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE0);
  NRF52_RTC_ACKINT(g_ble_dev.rtc2, NRF52_RTC_EVT_COMPARE0);
  NRF52_RTC_ACKINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE1);
  NRF52_RTC_ACKINT(g_ble_dev.rtc2, NRF52_RTC_EVT_COMPARE1);
  NRF52_RTC_ACKINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE2);
  NRF52_RTC_ACKINT(g_ble_dev.rtc2, NRF52_RTC_EVT_COMPARE2);
  NRF52_RTC_ACKINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE3);
  NRF52_RTC_ACKINT(g_ble_dev.rtc2, NRF52_RTC_EVT_COMPARE3);

  /* Disable PPIs used during advertising */

  nrf52_ppi_channel_enable(0, false);
  nrf52_ppi_channel_enable(1, false);

  /* Enable reception for this access address (and ensure advertising access
   * address (number 0) is disabled). We use logical addresses 4-7
   * (those without conflicts between each other, as other ones share parts)
   * from the RADIO.
   */

  modifyreg32(NRF52_RADIO_RXADDRESSES, 1, 1 << (conn_slot + 4));

  g_radio_dev->ops->addr_set(g_radio_dev, conn_slot + 4,
                             payload->aa >> 24, payload->aa << 8);

  /* Set CRC data for this connection */

    {
      uint32_t crcinit = payload->crcint[0] | payload->crcint[1] << 8 |
                         payload->crcint[2] << 16;

      putreg32(crcinit, NRF52_RADIO_CRCINIT);
    }

  /* Send packets with the same access address */

  putreg32(conn_slot + 4, NRF52_RADIO_TXADDRESS);

  /* Setup connection */

  conn->master_sca = payload->sca;
  memcpy(conn->channel_map, payload->chm, 5);
  conn->hop_increment = payload->hop;
  conn->slave_latency = payload->latency * 1250;
  conn->conn_interval = payload->interval * 1250;
  conn->clk_acc = bt_ll_sca_to_ppm(payload->sca);
  conn->winoffset = payload->winoffset * 1250;
  conn->winsize = payload->winsize * 1250;
  conn->supervision_timeout = payload->timeout * 10000;

  /* generate security values */

  nrf52_ble_rand((uint8_t *)&conn->iv, sizeof(conn->iv));
  nrf52_ble_rand((uint8_t *)&conn->skd, sizeof(conn->skd));

  /* TODO: slave latency is not implemented, we will simply listen to every
   * event, even when master will not speak to us.
   */

  /* Set TXSN to 1 so that when master sends first packet with NESN 0 we
   * do not mistake it for a resend (since otherwise NESN == TXSN)
   */

  conn->txsn = 1;

  /* count number of used channels in channel map */

  nrf52_ble_count_used_channels(conn_slot);

  /* TODO: validate data, the other side could be malicious */

  /* Setup timers to handle expected:
   *   1. RX window start,
   *   2. channel change time.
   *
   * During connection setup, RX time is given by the definition of "transmit
   * window" from window offset + window size. For subsequent connection
   * events after this initial one, the RX window will only be defined by
   * the "widening" (given by considering own clock accuracy and the other's
   * side accuracy) around the expected anchor point (the time when we are
   * supposed to ideally expect a data packet from master).
   *
   * The channel change time is given by connection interval: every new
   * interval happens on a different channel (according to channel switching
   * logic). This interval is measured from the anchor point, which will
   * initially be given by the time the CONNECT_REQ PDU arrived (and, later
   * on, by each new data PDU from master).
   *
   * We will use the (second) RTC timer, which will reset at each anchor
   * point (and already did at the CONNECT_REQ, so it is currently counting).
   *
   * First we will set COMPARE1 (C1) to match start of window:
   *
   *   C1 = 1.25ms + offset - widening
   *
   * On compare, we will use the PPI to initiate RADIO RX and reset the RTC:
   *
   *   PPI CH1: C1 -> RADIO RXEN, RTC RESET
   *
   * The RTC reset marks our expected anchor point. We do this because
   * we may not receive that first PDU from master and we will have to use
   * this time point for subsequent connection events, until we receive
   * something.
   *
   * We will handle "closing" the window by disabling the RADIO from within
   * C0 ISR, setting C0 as:
   *
   *   C0 = window size + widening
   *
   * Note that this compare setting assumes the counter started at this
   * expected anchor. For this reason, we keep the ISR disabled
   * until C1 expires.
   *
   * In the ISRs of the RTC we will:
   *   - C1 (window open):
   *     - Set C1 = conn interval - widening (only need to do once)
   *     - Enable C0 ISR
   *     - Increment interval counter, if > 6, tear down connection
   *   - C0 (initially disabled, as it would otherwise match before C1):
   *     - Disable RADIO
   *     - Switch to next channel
   *
   * At this point timers are set to wait for the initial anchor, which
   * could not arrive after various intervals. To react to the initial PDU
   * and align to this anchor we also set:
   *
   *   - PPI CH3/4: RADIO CRCOK/ERROR -> RTC RESET (align to actual anchor) +
   *   - RADIO END ISR: set C0 = widening (only needed once)
   *
   * After this PDU arrives, setup is complete. C0 will close the RX window
   * if no packet received and C1 will reopen it (start a new event).
   * If a packet is received, once communication on this event finishes,
   * the next channel is selected.
   *
   * Finally, the supervision timer will be handled via the other RTC, since
   * it needs to keep counting for a much longer time. We will use C0
   * for this and handle it in the ISR.
   */

  /* At this point we will leave interrupts enabled preparing ourselves for
   * the next RX, to handle received data PDUs once we know the result of CRC
   * (and we need to consider even those that fail).
   */

  putreg32(0, NRF52_RADIO_EVENTS_END);
  putreg32(0, NRF52_RADIO_EVENTS_CRCOK);
  putreg32(0, NRF52_RADIO_EVENTS_CRCERROR);

  putreg32(RADIO_INT_CRCOK | RADIO_INT_CRCERROR, NRF52_RADIO_INTENSET);

    {
      /* Compute expected window start, size and widening */

      uint32_t window_start = 1250 + (uint32_t)payload->winoffset * 1250;
      uint32_t window_size = (uint32_t)payload->winsize * 1250;

      uint32_t window_widening =
          ((BLE_SCA + bt_ll_sca_to_ppm(payload->sca)) *
           window_start) / 1000000;

      /* Setup C1 (window start) */

      nrf52_ble_set_rtc_timeout(2, 1, window_start - window_widening);
      NRF52_RTC_ENABLEEVT(g_ble_dev.rtc2, NRF52_RTC_EVT_COMPARE1);

      /* PPI CH1: C1 -> RADIO RXEN, RTC RESET */

      nrf52_ppi_set_event_ep(1, BLE_RTC2_BASE +
                             NRF52_RTC_EVENTS_COMPARE_OFFSET(1));
      nrf52_ppi_set_task_ep(1, NRF52_RADIO_TASKS_RXEN);
      nrf52_ppi_set_task2_ep(1, BLE_RTC2_BASE +
                             NRF52_RTC_TASKS_CLEAR_OFFSET);
      nrf52_ppi_channel_enable(1, true);

      /* Setup C0 (window size) */

      window_widening = ((BLE_SCA + bt_ll_sca_to_ppm(payload->sca)) *
                          window_size) / 1000000;

      nrf52_ble_set_rtc_timeout(2, 0, window_size + window_widening);
      NRF52_RTC_ENABLEEVT(g_ble_dev.rtc2, NRF52_RTC_EVT_COMPARE0);
    }

  /* Enable C1 ISR */

  NRF52_RTC_ACKINT(g_ble_dev.rtc2, NRF52_RTC_EVT_COMPARE1);
  NRF52_RTC_ENABLEINT(g_ble_dev.rtc2, NRF52_RTC_EVT_COMPARE1);

  /* PPI CH3/4: RADIO CRCOK/CRCERROR -> RTC RESET + PPI CH0 DISABLE */

  nrf52_ppi_set_event_ep(3, NRF52_RADIO_EVENTS_CRCOK);
  nrf52_ppi_set_task_ep(3, BLE_RTC2_BASE + NRF52_RTC_TASKS_CLEAR_OFFSET);
  nrf52_ppi_set_task2_ep(3, 0);
  nrf52_ppi_channel_enable(3, true);

  nrf52_ppi_set_event_ep(4, NRF52_RADIO_EVENTS_CRCERROR);
  nrf52_ppi_set_task_ep(4, BLE_RTC2_BASE + NRF52_RTC_TASKS_CLEAR_OFFSET);
  nrf52_ppi_set_task2_ep(4, 0);
  nrf52_ppi_channel_enable(4, true);

#if !defined(NRF52_LL_DISABLE_SUPERVISION_TIMER)
  /* C0 for supervision timer in main RTC */

  nrf52_ble_set_rtc_timeout(1, 0, (uint32_t)payload->timeout * 10000);

  /* Reset the timer whenever a packet is received */

  nrf52_ppi_set_event_ep(5, NRF52_RADIO_EVENTS_CRCOK);
  nrf52_ppi_set_task_ep(5, BLE_RTC_BASE + NRF52_RTC_TASKS_CLEAR_OFFSET);
  nrf52_ppi_set_task2_ep(5, 0);
  nrf52_ppi_channel_enable(5, true);

  nrf52_ppi_set_event_ep(6, NRF52_RADIO_EVENTS_CRCERROR);
  nrf52_ppi_set_task_ep(6, BLE_RTC_BASE + NRF52_RTC_TASKS_CLEAR_OFFSET);
  nrf52_ppi_set_task2_ep(6, 0);
  nrf52_ppi_channel_enable(6, true);

  /* Enable interrupt, where we will abort connection on match */

  NRF52_RTC_ACKINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE0);
  NRF52_RTC_ENABLEINT(g_ble_dev.rtc, NRF52_RTC_EVT_COMPARE0);

  /* Start counting */

  NRF52_RTC_START(g_ble_dev.rtc);
#endif

  /* We also setup our own shortcuts using PPI, since RADIO shortcuts
   * can also be disabled/enabled by register writes and PPI can use events.
   * We needs this since we would like READY->START but only for RXEN and
   * not TXEN, and the RADIO shortcuts do not distinguish this.
   * We thus make this shortcut auto-disable itself to happen only
   * when we want.
   */

  nrf52_ppi_set_event_ep(7, NRF52_RADIO_EVENTS_READY);
  nrf52_ppi_set_task_ep(7, NRF52_RADIO_TASKS_START);
  nrf52_ppi_set_task2_ep(7, NRF52_PPI_TASK_CHGDIS(1));
  nrf52_ppi_grp_clear(1);
  nrf52_ppi_grp_channel_enable(1, 7, true);

  /* We leave this enabled so that once the connection event starts via
   * timer, we start RX after ramp-up. After that this shortcut will be
   * disabled.
   */

  nrf52_ppi_grp_enable(1, true);

  /* We set a second shortcut to go from RX to TX so that right after
   * receiving we have TX already ramped up to send the response (always
   * required by the slave, even if CRC fails). The RADIO supports this
   * shortcut but it requires to be END->DISABLE->TXEN which we do not want,
   * we will do END->TXEN directly. We also want this only for RX->TX so this
   * again auto disables itself.
   */

  nrf52_ppi_set_event_ep(8, NRF52_RADIO_EVENTS_END);
  nrf52_ppi_set_task_ep(8, NRF52_RADIO_TASKS_TXEN);
  nrf52_ppi_set_task2_ep(8, NRF52_PPI_TASK_CHGDIS(2));
  nrf52_ppi_grp_clear(2);
  nrf52_ppi_grp_channel_enable(2, 8, true);

  /* We leave this enabled because the first END will be from the received
   * packet.
   */

  nrf52_ppi_grp_enable(2, true);

  /* Finally, we need to ensure that we will send the response 150uS
   * (T_IFS) after packet received from master. To do so, we setup a timer
   * to trigger the START of TX. Note that this means that we have a fixed
   * ammount of time to prepare the response right after reception.
   */

  /* We start the TIMER right after a message is received */

  nrf52_ppi_set_event_ep(9, NRF52_RADIO_EVENTS_END);
  nrf52_ppi_set_task_ep(9, BLE_TIM_BASE + NRF52_TIM_TASKS_START_OFFSET);
  nrf52_ppi_set_task2_ep(9, 0);
  nrf52_ppi_channel_enable(9, true);

  /* The timer is configured to expire after T_IFS (150uS), via COMPARE0 */

  NRF52_TIM_SETCC(g_ble_dev.tim, 0, 140); /* there appears to be some 10us overhead somewhere */
  NRF52_TIM_CLEAR(g_ble_dev.tim);

  /* Now set the action that will trigger the TX start on COMPARE0 */

  nrf52_ppi_set_event_ep(10, BLE_TIM_BASE +
                         NRF52_TIM_EVENTS_COMPARE_OFFSET(0));
  nrf52_ppi_set_task_ep(10, NRF52_EGU0_TASKS_TRIGGER0);
  nrf52_ppi_set_task2_ep(10, 0);

  /* We want this to work only between RX->TX and not after TX so after
   * it fires we disable the PPI channel
   */

  nrf52_ppi_grp_clear(3);
  nrf52_ppi_grp_channel_enable(3, 10, true);

  /* We will enable this right after preparing a response so we now we're
   * sending a valid one. Thus, leave the group disabled
   */

  nrf52_ppi_grp_enable(3, false);

  /* We use this intermediary event from the EGU so that we can know
   * if it fired by checking the EGU event later
   */

  nrf52_ppi_set_event_ep(11, NRF52_EGU0_EVENTS_TRIGGERED0);
  nrf52_ppi_set_task_ep(11, NRF52_PPI_TASK_CHGDIS(3));
  nrf52_ppi_set_task2_ep(11, NRF52_RADIO_TASKS_START);
  nrf52_ppi_channel_enable(11, true);

  /* We also want to align encryption task to TX */

  nrf52_ppi_set_event_ep(12, NRF52_EGU0_EVENTS_TRIGGERED0);
  nrf52_ppi_set_task_ep(12, NRF52_CCM_TASKS_KSGEN);
  nrf52_ppi_set_task2_ep(12, 0);
  nrf52_ppi_channel_enable(12, true);

  /* ACK compare event on TIM and EGU event, so we can detect if we managed
   * to actually transmit by the time the timer expired
   */

  NRF52_TIM_ACKINT(g_ble_dev.tim, NRF52_TIM_INT_COMPARE0);
  putreg32(0, NRF52_EGU0_EVENTS_TRIGGERED0);

  /* Change state */

  g_ble_dev.state = BT_LL_STATE_CONNECTION;

  /* Set role */

  g_ble_dev.role = BT_LL_ROLE_SLAVE;

  /* Set packet pointer to our buffer, which we'll use just for RX */

  g_radio_dev->ops->set_packetptr(g_radio_dev, g_ble_dev.buffer);

  /* Choose initial data channel */

  nrf52_ble_next_data_channel(conn_slot);

  /* Inform upper layer of the new connection */

  bt_ll_on_connection(hdr, payload, BT_HCI_SUCCESS, conn_slot,
                      BT_HCI_ROLE_SLAVE);
  return OK;
}

/****************************************************************************
 * Name: nrf52_handle_data_packet
 *
 * Description:
 *   Handle a packet received in data channel (connection event)
 *
 ****************************************************************************/

void nrf52_handle_data_packet(bool crc_pass)
{
  uint8_t *active_buffer;
  struct bt_data_pdu_hdr_s *hdr_out;
  uint8_t *payload_out_ptr;
  const struct bt_data_pdu_hdr_s *hdr;
  const uint8_t *payload_ptr;
  bool resend;
  bool new_data;
  bool terminate = false;
  bool send_ack = false;
  struct nrf52_ble_conn_s *conn;

  conn = &g_ble_dev.connections[0];

  /* Get pointers to input data */

  hdr = (const struct bt_data_pdu_hdr_s *)g_ble_dev.buffer;
  payload_ptr = (const uint8_t *)(g_ble_dev.buffer + 3);

  /* Determine if we should resend last data and if the data we received is
   * new or if it is a resend.
   */

  resend = (hdr->nesn == conn->txsn);
  new_data = (hdr->sn == conn->nesn);

  /* We can only interpret these flags if CRC passed */

  if (crc_pass)
    {
      if (!resend)
        {
          /* last packet we sent was received, increase transmit SN */

          conn->txsn++;

          /* If last time we were sending a non-empty PDU, and encryption
           * is enabled, we should now increase the encrypted message TX
           * count since it was acked.
           */

          if (conn->encryption && conn->will_send_nonempty)
            {
              nrf52_ble_sec_inctx();
              conn->will_send_nonempty = false;
              putreg32(1, NRF52_GPIOTE_TASKS_OUT(6));
            }
        }

      /* wether we can close the connection event after this */
#if 0
      g_ble_dev.connections[0].more_data = hdr->md;
#else
      /* TODO: for now we ignore this and let the master retransmit the
       * any packet we missed inside this interval
       */

      conn->more_data = 0;
#endif
    }

  if (new_data && crc_pass)
    {
      /* increase NESN to indicate we received the packet */

      conn->nesn++;

      /* If encryption is enabled, and we're receiving a non-empty PDU,
       * increase RX encrypted count
       */

      if (conn->encryption &&
          !(hdr->llid == BT_LE_DATA_PDU1 && hdr->length == 0))
        {
          nrf52_ble_sec_incrx();
        }
    }

  if (new_data && crc_pass && hdr->llid == BT_LE_CONTROL_PDU)
    {
      /* If this is a control PDU, we will create the response ourselves */

      bool supported = true;
      bool invalid = false;

      const struct bt_control_pdu_payload_s *ctr_payload =
          (struct bt_control_pdu_payload_s *)payload_ptr;

      hdr_out = (struct bt_data_pdu_hdr_s *)g_ble_dev.ctrl_buffer;
      payload_out_ptr = g_ble_dev.ctrl_buffer + 3;

      struct bt_control_pdu_payload_s *ctr_payload_out =
          (struct bt_control_pdu_payload_s *)payload_out_ptr;

      /* process packet */

      switch (ctr_payload->opcode)
        {
          case BT_LL_CONNECTION_UPDATE_REQ:
            {
              const struct bt_control_pdu_conn_update_req_s *req =
                (const struct bt_control_pdu_conn_update_req_s *)
                    ctr_payload->ctr_data;

                if ((req->instant - conn->event_counter) % (1 << 16) >=
                    ((1 << 15) - 1))
                  {
                    /* invalid, abort connection */

                    nrf52_ble_abort_connection(0);

                    bt_ll_on_connection_closed(0, BT_HCI_ERR_INSTANT_PASSED);

                    _info("invalid channel map request, aborting\n");

                    return;
                  }
                else
                  {
                    /* Store the map and note that we will need to update
                     * it
                     */

                    conn->update_conn_params = true;
                    memcpy(&conn->next_conn_params, req, sizeof(*req));

                    /* respond with empty PDU */

                    memcpy(hdr_out, &g_empty_pdu, sizeof(g_empty_pdu));
                  }
            }
            break;
          case BT_LL_CHANNEL_MAP_REQ:
            {
              const struct bt_control_pdu_channel_map_req_s *req =
                (const struct bt_control_pdu_channel_map_req_s *)
                    ctr_payload->ctr_data;

              if ((req->instant - conn->event_counter) % (1 << 16) >=
                  ((1 << 15) - 1))
                {
                  /* invalid, abort connection */

                  nrf52_ble_abort_connection(0);

                  bt_ll_on_connection_closed(0, BT_HCI_ERR_INSTANT_PASSED);

                  _info("invalid channel map request, aborting\n");

                  return;
                }
              else
                {
                  /* store the map and note that we will need to update it */

                  conn->update_map = true;
                  memcpy(&conn->next_channel_map, req, sizeof(*req));

                  /* respond with empty PDU */

                  memcpy(hdr_out, &g_empty_pdu, sizeof(g_empty_pdu));
                }
            }
            break;
          case BT_LL_TERMINATE_IND:
            {
              terminate = true;

              /* respond with empty PDU */

              memcpy(hdr_out, &g_empty_pdu, sizeof(g_empty_pdu));
            }
            break;
          case BT_LL_FEATURE_REQ:
            {
              /* report no features for now */

              memcpy(hdr_out, hdr, sizeof(*hdr));
              ctr_payload_out->opcode = BT_LL_FEATURE_RSP;
              memset(ctr_payload_out->ctr_data, 0, 8);
              ctr_payload_out->ctr_data[0] = 1; /* encryption supported */

              memcpy(conn->remote_features, ctr_payload->ctr_data,
                     sizeof(conn->remote_features));

              conn->got_rem_features = true;
            }
            break;
          case BT_LL_VERSION_IND:
            {
              struct bt_control_pdu_version_ind_s *vers_out =
                  (struct bt_control_pdu_version_ind_s *)
                      ctr_payload_out->ctr_data;

              memcpy(hdr_out, hdr, sizeof(*hdr));

              ctr_payload_out->opcode = BT_LL_VERSION_IND;

              vers_out->versnum = BT_LL_VERSNUM_4_0;
              vers_out->subvers = 0;
              vers_out->compid = BT_LL_COMPID_NORDIC;
            }
            break;
          case BT_LL_ENC_REQ:
            {
              struct bt_control_pdu_enc_resp_s *resp =
                (struct bt_control_pdu_enc_resp_s *)
                    ctr_payload_out->ctr_data;

              /* send an ENC_RSP with our IV and SKD */

              memcpy(hdr_out, hdr, sizeof(*hdr));

              hdr_out->length = sizeof(*resp) + sizeof(uint8_t);

              ctr_payload_out->opcode = BT_LL_ENC_RSP;

              resp->ivs = conn->iv;
              resp->skds = conn->skd;
            }
            break;
          case BT_LL_START_ENC_RSP:
            {
              /* Send a START_ENC_RSP back */

              ctr_payload_out->opcode = BT_LL_START_ENC_RSP;

              memcpy(hdr_out, hdr, sizeof(*hdr));
            }
            break;
          default:
            invalid = true;
            break;
        }

      /* We received an invalid PDU or we simply don't support it */

      if (invalid || !supported)
        {
          memcpy(hdr_out, hdr, sizeof(*hdr));
          hdr_out->length = 2;

          if (invalid)
            {
              ctr_payload_out->opcode = BT_LL_UNKNOWN_RSP;
              ctr_payload_out->ctr_data[0] = ctr_payload->opcode;
            }
          else
            {
              ctr_payload_out->opcode = BT_LL_REJECT_IND;

              ctr_payload_out->ctr_data[0] =
                  BT_HCI_ERR_UNSUPP_REMOTE_FEATURE;
            }
        }

      if (conn->encryption)
        {
          /* when using encryption, we setup the CCM to encrypt to a separate
           * buffer which the RADIO will use during send
           */

          g_radio_dev->ops->set_packetptr(g_radio_dev,
                                          g_ble_dev.encrypted_buffer);

          nrf52_ble_sec_set_packetptr(g_ble_dev.ctrl_buffer,
                                      g_ble_dev.encrypted_buffer);
        }
      else
        {
          g_radio_dev->ops->set_packetptr(g_radio_dev,
                                          g_ble_dev.ctrl_buffer);
        }
    }
  else
    {
      active_buffer = g_ble_dev.data_buffer[g_ble_dev.active_data_buffer];

      /* We can send a data PDU */

      if (crc_pass && !resend)
        {
          hdr_out = (struct bt_data_pdu_hdr_s *)active_buffer;

          /* We do not have to resend last PDU */

          /* First we check if last sent packet was a non-empty PDU, if so
           * the controller just acked a message sent by host
           */

          if ((hdr_out->llid == BT_LE_DATA_PDU1 && hdr_out->length != 0) ||
               hdr_out->llid == BT_LE_DATA_PDU2)
            {
#if 0
              /* decrease the unacked packet count */

              DEBUGASSERT(conn->unacked_packets > 0);
              conn->unacked_packets--;
#endif

              /* inform the host */

              send_ack = true;
            }

          /* First leave an empty PDU in active buffer */

          memcpy(active_buffer,
                 (uint8_t *)&g_empty_pdu, sizeof(g_empty_pdu));

          /* Switch buffers, make the inactive active */

          g_ble_dev.active_data_buffer =
              (g_ble_dev.active_data_buffer + 1) % 2;

          active_buffer =
              g_ble_dev.data_buffer[g_ble_dev.active_data_buffer];
        }

      /* Set RADIO to use the data buffer for sending */

      if (conn->encryption)
        {
          g_radio_dev->ops->set_packetptr(g_radio_dev,
                                          g_ble_dev.encrypted_buffer);

          nrf52_ble_sec_set_packetptr(active_buffer,
                                      g_ble_dev.encrypted_buffer);
        }
      else
        {
          g_radio_dev->ops->set_packetptr(g_radio_dev, active_buffer);
        }

      /* Get pointers to output HDR and payload */

      hdr_out = (struct bt_data_pdu_hdr_s *)active_buffer;
      payload_out_ptr = active_buffer + 3;
    }

  /* Remember if the outgoing message will be a non-empty PDU */

  if ((hdr_out->llid == BT_LE_DATA_PDU1 && hdr_out->length != 0) ||
      hdr_out->llid == BT_LE_DATA_PDU2 ||
      hdr_out->llid == BT_LE_CONTROL_PDU)
    {
      conn->will_send_nonempty = true;
    }

  /* Send confirmation of packet received */

  hdr_out->nesn = conn->nesn;

  if (!resend)
    {
      /* Set sequence number of new packet */

      hdr_out->sn = conn->txsn;
    }

  /* All setup done, the TIMER will take care of initiating the send
   * at the right time, we just need to enable the action of
   * TIMER COMPARE -> TXEN. If we were late to do this, we will not have
   * sent the packet in time.
   */

  nrf52_ppi_grp_enable(3, true);

  if (terminate)
    {
      /* we will wait for the ACK to be sent right here and then
       * abort the connection
       */

      while (!getreg32(NRF52_EGU0_EVENTS_TRIGGERED0))
        {
          /* wait for TX to be issued */
        }

      while (!getreg32(NRF52_RADIO_EVENTS_END))
        {
          /* for for transmission to complete */
        }

      /* now we can abort */

      nrf52_ble_abort_connection(0);

      _info("finished connection\n");

      /* inform the host */

      bt_ll_on_connection_closed(0, BT_HCI_ERR_REMOTE_USER_TERM_CONN);

      return;
    }

  /* Now that we are all setup, we can let the host know about this */

  if (send_ack)
    {
      /* Last packet completed, report it */

      bt_ll_ack_packets(0, 1);
    }

  /* If we just send a START encryption request, mark encryption as ON */

  if (!conn->encryption && hdr_out->llid == BT_LE_CONTROL_PDU &&
      *payload_out_ptr == BT_LL_START_ENC_REQ)
    {
      conn->encryption = true;
    }

  /* Pass data on to host */

  if (crc_pass && new_data)
    {
      if ((hdr->llid == BT_LE_DATA_PDU1 && hdr->length != 0) ||
           hdr->llid == BT_LE_DATA_PDU2)
        {
          bt_ll_on_data(hdr, payload_ptr, 0);
        }
      else if (hdr->llid == BT_LE_CONTROL_PDU)
        {
          struct bt_control_pdu_payload_s *ctr_payload =
              (struct bt_control_pdu_payload_s *)payload_ptr;

          if (ctr_payload->opcode == BT_LL_FEATURE_REQ &&
              conn->req_rem_features)
            {
              bt_ll_send_rem_features(0, conn->remote_features);

              conn->req_rem_features = false;
            }
          else if (ctr_payload->opcode == BT_LL_ENC_REQ)
            {
              struct bt_control_pdu_enc_req_s *req =
                (struct bt_control_pdu_enc_req_s *)ctr_payload->ctr_data;

              /* Remember master's SKD */

              conn->skdm = req->skdm;

              /* Generate IV from IVs and IVm */

              nrf52_ble_sec_set_iv(conn->iv, req->ivm);

              /* Request the LTK from the host */

              bt_ll_request_ltk(0, req->ediv, req->rand);
            }
          else if (ctr_payload->opcode == BT_LL_START_ENC_RSP)
            {
              /* notify the host on encryption initiated */

              bt_ll_encryption_changed(0, true);
            }
        }
    }
}

/****************************************************************************
 * Name: nrf52_ble_send_data
 *
 * Description:
 *   Send the data supplied by the host
 *
 ****************************************************************************/

void nrf52_ble_send_data(const uint8_t *data, size_t len, bool continuation,
                         bool broadcast)
{
  irqstate_t flags;
  uint8_t *inactive_buffer;
  struct bt_data_pdu_hdr_s *hdr;
  uint8_t *payload;

  flags = enter_critical_section();

  /* Get exclusive access to inactive active buffer */

  inactive_buffer =
      g_ble_dev.data_buffer[(g_ble_dev.active_data_buffer + 1) % 2];

  /* Get pointers to header and payload */

  hdr = (struct bt_data_pdu_hdr_s *)inactive_buffer;
  payload = inactive_buffer + 3;

  /* Build header */

  hdr->llid = (continuation ? BT_LE_DATA_PDU1 : BT_LE_DATA_PDU2);
  hdr->length = len;

  /* Copy payload */

  memcpy(payload, data, len);

#if 0
  /* Increased unacked packet count */

  g_ble_dev.connections[0].unacked_packets++;
#endif

  leave_critical_section(flags);
}

/****************************************************************************
 * Name: nrf52_ble_send_ctrl_pdu
 *
 * Description:
 *   Prepare a data PDU to be sent
 *
 ****************************************************************************/

void nrf52_ble_send_ctrl_pdu(struct bt_control_pdu_payload_s *payload,
                             size_t len)
{
  irqstate_t flags;
  uint8_t *inactive_buffer;
  struct bt_data_pdu_hdr_s *hdr;

  flags = enter_critical_section();

  /* Get exclusive access to inactive active buffer */

  inactive_buffer =
      g_ble_dev.data_buffer[(g_ble_dev.active_data_buffer + 1) % 2];

  /* Get pointer to header */

  hdr = (struct bt_data_pdu_hdr_s *)inactive_buffer;

  /* Build header */

  hdr->llid = BT_LE_CONTROL_PDU;
  hdr->length = len;

  /* Copy payload */

  memcpy(inactive_buffer + 3, payload, len);

  leave_critical_section(flags);

  _info("host send: %i\n", hdr->length);
}

/****************************************************************************
 * Name: nrf52_ble_abort_connection
 *
 * Description:
 *   Abort the connection in progress, go back to standby state
 *
 ****************************************************************************/

void nrf52_ble_abort_connection(int conn)
{
  /* Disable all RTC and RADIO interrupts */

  putreg32(0xffffffff, NRF52_RADIO_INTENCLR);
  putreg32(0xffffffff, BLE_RTC_BASE + NRF52_RTC_INTENCLR_OFFSET);
  putreg32(0xffffffff, BLE_RTC2_BASE + NRF52_RTC_INTENCLR_OFFSET);

  /* Disable all RTC events */

  putreg32(0xffffffff, BLE_RTC_BASE + NRF52_RTC_EVTENCLR_OFFSET);
  putreg32(0xffffffff, BLE_RTC2_BASE + NRF52_RTC_EVTENCLR_OFFSET);

  /* Stop TIMER and RTC */

  NRF52_RTC_STOP(g_ble_dev.rtc);
  NRF52_RTC_STOP(g_ble_dev.rtc2);
  NRF52_TIM_STOP(g_ble_dev.tim);

  /* Disable all PPIs */

  nrf52_ppi_channel_enable(1, false);
  nrf52_ppi_channel_enable(3, false);
  nrf52_ppi_channel_enable(4, false);
  nrf52_ppi_channel_enable(5, false);
  nrf52_ppi_channel_enable(6, false);
  nrf52_ppi_channel_enable(9, false);
  nrf52_ppi_channel_enable(11, false);
  nrf52_ppi_channel_enable(12, false);

  /* Disable PPI groups */

  nrf52_ppi_grp_enable(0, false);
  nrf52_ppi_grp_enable(1, false);
  nrf52_ppi_grp_enable(2, false);

  /* Clear all PPI groups */

  nrf52_ppi_grp_clear(1);
  nrf52_ppi_grp_clear(2);
  nrf52_ppi_grp_clear(3);

  /* disable the radio */

  putreg32(1, NRF52_RADIO_TASKS_DISABLE);

  /* disable encryption */

  nrf52_ble_sec_mode(BLE_SEC_DISABLE);

  nrf52_ble_sec_reset_pktctr();

  /* Disable reception for this address */

  modifyreg32(NRF52_RADIO_RXADDRESSES, 1 << (conn + 4), 0);

  /* Reset connection state */

  memset(&g_ble_dev.connections[conn], 0, sizeof(struct nrf52_ble_conn_s));

  /* Go to stanby state */

  g_ble_dev.state = BT_LL_STATE_STANDBY;
}

/****************************************************************************
 * Name: nrf52_ble_next_data_channel
 *
 * Description:
 *   Apply the data channel selection algorithm to select the next data
 *   channel for the given connection.
 *
 ****************************************************************************/

void nrf52_ble_next_data_channel(int conn_slot)
{
  uint8_t unmapped_channel;
  struct nrf52_ble_conn_s *conn = &g_ble_dev.connections[conn_slot];

  /* if there's a channel map update pending and it should be applied at
   * this connection event, copy it
   */

  if (conn->update_map &&
      conn->event_counter == conn->next_channel_map.instant)
    {
      memcpy(conn->channel_map, conn->next_channel_map.channel_map,
             sizeof(conn->channel_map));

      nrf52_ble_count_used_channels(0);

      conn->update_map = false;
    }

  /* Find an unmapped channel */

  unmapped_channel = (conn->last_unmapped_ch + conn->hop_increment) % 37;

  /* Is this channel available for this connection according to the map? */

  if (!(conn->channel_map[unmapped_channel / 8] &
        (1 << (unmapped_channel % 8))))
    {
      /* No, map it to one of the used channels */

      int i;
      uint8_t remapped_channel = 0;
      uint8_t index = unmapped_channel % conn->num_used_channels;

      /* Look for the index-th used channel */

      for (i = 0; i < 37; i++)
        {
          if (conn->channel_map[i / 8] & (1 << (i % 8)))
            {
              if (index == 0)
                {
                  remapped_channel = i;
                  break;
                }
              else
                {
                  index--;
                }
            }
        }

      /* choose the remapped channel */

      nrf52_ble_setchannel(remapped_channel);
    }
  else
    {
      /* choose the unmapped channel */

      nrf52_ble_setchannel(unmapped_channel);
    }

  /* remember last unmapped channel */

  g_ble_dev.connections[conn_slot].last_unmapped_ch = unmapped_channel;
}

/****************************************************************************
 * Name: nrf52_ble_count_used_channels
 *
 * Description:
 *   Count number of used channels in channel map
 *
 ****************************************************************************/

static void nrf52_ble_count_used_channels(uint8_t conn_id)
{
  int i;
  struct nrf52_ble_conn_s *conn = &g_ble_dev.connections[conn_id];

  conn->num_used_channels = 0;

  for (i = 0; i < 5; i++)
    {
      uint8_t b = conn->channel_map[i];

      while (b)
        {
          if (b & 1)
            {
              conn->num_used_channels++;
            }

          b >>= 1;
        }
    }
}

/****************************************************************************
 * Name: nrf52_ble_close_connection_event
 *
 * Description:
 *   Closes current connection event, leaving everything prepared for the
 *   next one.
 *
 ****************************************************************************/

void nrf52_ble_close_connection_event(void)
{
  struct nrf52_ble_conn_s *conn = &g_ble_dev.connections[0];

  /* Disable RADIO, no more RX until next event */

  putreg32(1, NRF52_RADIO_TASKS_DISABLE);

  /* ensure packet pointer set to RX buffer for next event */

  if (conn->encryption)
    {
      /* when using encryption, we make the RADIO write into a separate
       * buffer, which the CCM will use to decrypt into the normal I/O
       * buffer
       */

      g_radio_dev->ops->set_packetptr(g_radio_dev,
                                      g_ble_dev.encrypted_buffer);

      nrf52_ble_sec_set_packetptr(g_ble_dev.encrypted_buffer,
                                  g_ble_dev.buffer);

      /* Setup for decryption */

      nrf52_ble_sec_mode(BLE_SEC_DECRYPT);
    }
  else
    {
      g_radio_dev->ops->set_packetptr(g_radio_dev, g_ble_dev.buffer);
    }

  /* we will be starting a new connection event */

  conn->event_counter++;

  putreg32(1, NRF52_GPIOTE_TASKS_OUT(4));

  /* go to next channel */

  nrf52_ble_next_data_channel(0);

  /* Prepare for next RX and RX->TX transition
   * (rearm READY->START and END->TXEN)
   */

  nrf52_ppi_grp_enable(1, true);
  nrf52_ppi_grp_enable(2, true);

  /* Prepare to handle next RX */

  putreg32(0, NRF52_RADIO_EVENTS_CRCERROR);
  putreg32(0, NRF52_RADIO_EVENTS_CRCOK);
  putreg32(RADIO_INT_CRCERROR | RADIO_INT_CRCOK, NRF52_RADIO_INTENSET);
  putreg32(RADIO_INT_END, NRF52_RADIO_INTENCLR);

  /* ACK compare event on TIM and EGU event, so we can detect if we managed
   * to actually transmit by the time the timer expired
   */

  NRF52_TIM_ACKINT(g_ble_dev.tim, NRF52_TIM_INT_COMPARE0);
  putreg32(0, NRF52_EGU0_EVENTS_TRIGGERED0);

  /* check if we should apply new connection parameters */

  if (conn->update_conn_params &&
      conn->event_counter == conn->next_conn_params.instant)
    {
      uint32_t window_start;
      uint32_t window_widening;

      uint32_t new_interval = conn->next_conn_params.interval * 1250;
      uint32_t new_offset = conn->next_conn_params.winoffset * 1250;
      uint32_t new_size = conn->next_conn_params.winsize * 1250;
      uint32_t new_latency = conn->next_conn_params.latency;
      uint32_t new_timeout = conn->next_conn_params.timeout;

      /* compute new values */

      window_start = new_offset + (uint32_t)conn->conn_interval;

      window_widening = ((BLE_SCA + bt_ll_sca_to_ppm(conn->master_sca)) *
                          window_start) / 1000000;

      /* Setup C1 (window start) */

      nrf52_ble_set_rtc_timeout(2, 1, window_start - window_widening);

      /* Setup C0 (window end). this will be actually modified in C1 ISR */

      window_widening = ((BLE_SCA + bt_ll_sca_to_ppm(conn->master_sca)) *
                          new_size) / 1000000;

      nrf52_ble_set_rtc_timeout(2, 0, new_size + window_widening);

      /* setup supervision timer */

      nrf52_ble_set_rtc_timeout(1, 0,
                                conn->next_conn_params.timeout * 10000);

      NRF52_RTC_CLEAR(g_ble_dev.rtc);

      /* remember the settings */

      conn->conn_interval = new_interval;
      conn->winoffset = new_offset;
      conn->winsize = new_size;
      conn->slave_latency = conn->next_conn_params.latency;

      /* notify the host */

      if (new_interval != conn->conn_interval ||
          new_latency != conn->slave_latency ||
          new_timeout != conn->supervision_timeout)
        {
          bt_ll_update_conn_params(0, conn->next_conn_params.interval,
                                   conn->next_conn_params.latency,
                                   conn->next_conn_params.timeout);
        }

      conn->update_conn_params = false;
    }
}

/****************************************************************************
 * Name: nrf52_ble_get_remote_features
 *
 * Description:
 *   Called by the upper layer to request the features of the remote
 * controller
 *
 ****************************************************************************/

int nrf52_ble_get_remote_features(uint8_t conn)
{
#if 0
  if (g_ble_dev.connections[conn].got_rem_features)
    {
      bt_ll_send_rem_features(conn,
                              g_ble_dev.connections[conn].remote_features);
      _info("sent features\n");

      return OK;
    }
  else
    {
      _info("waiting for features\n");
      return -EBUSY;
    }
#else
  uint8_t features[8];
  memset(features, 0, sizeof(features));
  bt_ll_send_rem_features(conn, features);
#endif

  return 0;
}
