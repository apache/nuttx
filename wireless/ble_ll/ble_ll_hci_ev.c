/****************************************************************************
 * wireless/ble_ll/ble_ll_hci_ev.c
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
 * Derived from Apache Mynewt NimBLE (nimble/controller/src/ble_ll_hci_ev.c).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ble.h"
#include "hci_common.h"
#include "ble_ll.h"
#include "ble_ll_hci.h"
#include "ble_ll_ctrl.h"
#include "ble_ll_conn_priv.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_ll_hci_ev_datalen_chg
 *
 * Description:
 *   Send a data length change event for a connection to the host.
 *
 * Input Parameters:
 *   connsm - Pointer to connection state machine
 *
 ****************************************************************************/

void ble_ll_hci_ev_datalen_chg(struct ble_ll_conn_sm *connsm)
{
  struct ble_hci_ev_le_subev_data_len_chg *ev;
  struct ble_hci_ev *hci_ev;

  if (ble_ll_hci_is_le_event_enabled(BLE_HCI_LE_SUBEV_DATA_LEN_CHG))
    {
      hci_ev = ble_ll_hci_evt_alloc(0);
      if (hci_ev)
        {
          hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
          hci_ev->length = sizeof(*ev);
          ev = (void *)hci_ev->data;

          ev->subev_code = BLE_HCI_LE_SUBEV_DATA_LEN_CHG;
          ev->conn_handle = htole16(connsm->conn_handle);

          ev->max_tx_octets = htole16(connsm->eff_max_tx_octets);
          ev->max_tx_time = htole16(connsm->eff_max_tx_time);
          ev->max_rx_octets = htole16(connsm->eff_max_rx_octets);
          ev->max_rx_time = htole16(connsm->eff_max_rx_time);

          ble_ll_hci_event_send(hci_ev);
        }
    }
}

/****************************************************************************
 * Name: ble_ll_hci_ev_rem_conn_parm_req
 *
 * Description:
 *   Send a connection parameter request event for a connection to the host.
 *
 * Input Parameters:
 *   connsm - Pointer to connection state machine
 *
 ****************************************************************************/

void ble_ll_hci_ev_rem_conn_parm_req(struct ble_ll_conn_sm *connsm,
                                     struct ble_ll_conn_params *cp)
{
  struct ble_hci_ev_le_subev_rem_conn_param_req *ev;
  struct ble_hci_ev *hci_ev;

  if (ble_ll_hci_is_le_event_enabled(BLE_HCI_LE_SUBEV_REM_CONN_PARM_REQ))
    {
      hci_ev = ble_ll_hci_evt_alloc(0);
      if (hci_ev)
        {
          hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
          hci_ev->length = sizeof(*ev);
          ev = (void *)hci_ev->data;

          ev->subev_code = BLE_HCI_LE_SUBEV_REM_CONN_PARM_REQ;
          ev->conn_handle = htole16(connsm->conn_handle);
          ev->min_interval = htole16(cp->interval_min);
          ev->max_interval = htole16(cp->interval_max);
          ev->latency = htole16(cp->latency);
          ev->timeout = htole16(cp->timeout);

          ble_ll_hci_event_send(hci_ev);
        }
    }
}

/****************************************************************************
 * Name: ble_ll_hci_ev_conn_update
 *
 * Description:
 *   Send a connection update event.
 *
 * Input Parameters:
 *   connsm - Pointer to connection state machine
 *   status - The error code.
 *
 ****************************************************************************/

void ble_ll_hci_ev_conn_update(struct ble_ll_conn_sm *connsm, uint8_t status)
{
  struct ble_hci_ev_le_subev_conn_upd_complete *ev;
  struct ble_hci_ev *hci_ev;

  if (ble_ll_hci_is_le_event_enabled(BLE_HCI_LE_SUBEV_CONN_UPD_COMPLETE))
    {
      hci_ev = ble_ll_hci_evt_alloc(0);
      if (hci_ev)
        {
          hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
          hci_ev->length = sizeof(*ev);
          ev = (void *)hci_ev->data;

          ev->subev_code = BLE_HCI_LE_SUBEV_CONN_UPD_COMPLETE;
          ev->status = status;
          ev->conn_handle = htole16(connsm->conn_handle);
          ev->conn_itvl = htole16(connsm->conn_itvl);
          ev->conn_latency = htole16(connsm->periph_latency);
          ev->supervision_timeout = htole16(connsm->supervision_tmo);

          ble_ll_hci_event_send(hci_ev);
        }
    }
}

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
/****************************************************************************
 * Name: ble_ll_hci_ev_encrypt_chg
 *
 ****************************************************************************/

void ble_ll_hci_ev_encrypt_chg(struct ble_ll_conn_sm *connsm, uint8_t status)
{
  struct ble_hci_ev_enc_key_refresh *ev_key_refresh;
  struct ble_hci_ev_enrypt_chg *ev_enc_chf;
  struct ble_hci_ev *hci_ev;

  if (connsm->flags.encrypt_event_sent == 0)
    {
      if (ble_ll_hci_is_event_enabled(BLE_HCI_EVCODE_ENCRYPT_CHG))
        {
          hci_ev = ble_ll_hci_evt_alloc(0);
          if (hci_ev)
            {
              hci_ev->opcode = BLE_HCI_EVCODE_ENCRYPT_CHG;
              hci_ev->length = sizeof(*ev_enc_chf);
              ev_enc_chf = (void *)hci_ev->data;

              ev_enc_chf->status = status;
              ev_enc_chf->connection_handle = htole16(connsm->conn_handle);
              ev_enc_chf->enabled =
                  (status == BLE_ERR_SUCCESS) ? 0x01 : 0x00;

              ble_ll_hci_event_send(hci_ev);
            }
        }

      connsm->flags.encrypt_event_sent = 1;
      return;
    }

  if (ble_ll_hci_is_event_enabled(BLE_HCI_EVCODE_ENC_KEY_REFRESH))
    {
      hci_ev = ble_ll_hci_evt_alloc(0);
      if (hci_ev)
        {
          hci_ev->opcode = BLE_HCI_EVCODE_ENC_KEY_REFRESH;
          hci_ev->length = sizeof(*ev_key_refresh);
          ev_key_refresh = (void *)hci_ev->data;

          ev_key_refresh->status = status;
          ev_key_refresh->conn_handle = htole16(connsm->conn_handle);

          ble_ll_hci_event_send(hci_ev);
        }
    }
}

/****************************************************************************
 * Name: ble_ll_hci_ev_ltk_req
 *
 * Description:
 *   Send a long term key request event for a connection to the host.
 *
 * Input Parameters:
 *   connsm - Pointer to connection state machine
 *
 ****************************************************************************/

int ble_ll_hci_ev_ltk_req(struct ble_ll_conn_sm *connsm)
{
  struct ble_hci_ev_le_subev_lt_key_req *ev;
  struct ble_hci_ev *hci_ev;
  int rc;

  if (ble_ll_hci_is_le_event_enabled(BLE_HCI_LE_SUBEV_LT_KEY_REQ))
    {
      hci_ev = ble_ll_hci_evt_alloc(0);
      if (hci_ev)
        {
          hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
          hci_ev->length = sizeof(*ev);
          ev = (void *)hci_ev->data;

          ev->subev_code = BLE_HCI_LE_SUBEV_LT_KEY_REQ;
          ev->conn_handle = htole16(connsm->conn_handle);
          ev->rand = htole64(connsm->enc_data.host_rand_num);
          ev->div = htole16(connsm->enc_data.enc_div);

          ble_ll_hci_event_send(hci_ev);
        }

      rc = 0;
    }
  else
    {
      rc = -1;
    }

  return rc;
}

#endif /* CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION */

/****************************************************************************
 * Name: ble_ll_hci_ev_rd_rem_used_feat
 *
 ****************************************************************************/

void ble_ll_hci_ev_rd_rem_used_feat(struct ble_ll_conn_sm *connsm,
                                    uint8_t status)
{
  struct ble_hci_ev_le_subev_rd_rem_used_feat *ev;
  struct ble_hci_ev *hci_ev;

  if (ble_ll_hci_is_le_event_enabled(BLE_HCI_LE_SUBEV_RD_REM_USED_FEAT))
    {
      hci_ev = ble_ll_hci_evt_alloc(0);
      if (hci_ev)
        {
          hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
          hci_ev->length = sizeof(*ev);
          ev = (void *)hci_ev->data;

          ev->subev_code = BLE_HCI_LE_SUBEV_RD_REM_USED_FEAT;
          ev->status = status;
          ev->conn_handle = htole16(connsm->conn_handle);
          ev->features[0] = connsm->conn_features;
          memcpy(ev->features + 1, connsm->remote_features, 7);

          ble_ll_hci_event_send(hci_ev);
        }
    }
}

/****************************************************************************
 * Name: ble_ll_hci_ev_rd_rem_ver
 *
 ****************************************************************************/

void ble_ll_hci_ev_rd_rem_ver(struct ble_ll_conn_sm *connsm, uint8_t status)
{
  struct ble_hci_ev_rd_rem_ver_info_cmp *ev;
  struct ble_hci_ev *hci_ev;

  if (ble_ll_hci_is_event_enabled(BLE_HCI_EVCODE_RD_REM_VER_INFO_CMP))
    {
      hci_ev = ble_ll_hci_evt_alloc(0);
      if (hci_ev)
        {
          hci_ev->opcode = BLE_HCI_EVCODE_RD_REM_VER_INFO_CMP;
          hci_ev->length = sizeof(*ev);
          ev = (void *)hci_ev->data;

          ev->status = status;
          ev->conn_handle = htole16(connsm->conn_handle);
          ev->version = connsm->vers_nr;
          ev->manufacturer = htole16(connsm->comp_id);
          ev->subversion = htole16(connsm->sub_vers_nr);

          ble_ll_hci_event_send(hci_ev);
        }
    }
}

/****************************************************************************
 * Name: ble_ll_hci_ev_hw_err
 *
 * Description:
 *   Send a HW error to the host.
 *
 * Input Parameters:
 *   hw_err
 *
 * Returned Value:
 *   int 0: event masked or event sent, -1 otherwise
 *
 ****************************************************************************/

int ble_ll_hci_ev_hw_err(uint8_t hw_err)
{
  struct ble_hci_ev_hw_error *ev;
  struct ble_hci_ev *hci_ev;
  int rc;

  rc = 0;
  if (ble_ll_hci_is_event_enabled(BLE_HCI_EVCODE_HW_ERROR))
    {
      hci_ev = ble_ll_hci_evt_alloc(0);
      if (hci_ev)
        {
          hci_ev->opcode = BLE_HCI_EVCODE_HW_ERROR;
          hci_ev->length = sizeof(*ev);
          ev = (void *)hci_ev->data;

          ev->hw_code = hw_err;

          ble_ll_hci_event_send(hci_ev);
        }
      else
        {
          rc = -1;
        }
    }

  return rc;
}

/****************************************************************************
 * Name: ble_ll_hci_ev_databuf_overflow
 *
 ****************************************************************************/

void ble_ll_hci_ev_databuf_overflow(void)
{
  struct ble_hci_ev_data_buf_overflow *ev;
  struct ble_hci_ev *hci_ev;

  if (ble_ll_hci_is_event_enabled(BLE_HCI_EVCODE_DATA_BUF_OVERFLOW))
    {
      hci_ev = ble_ll_hci_evt_alloc(0);
      if (hci_ev)
        {
          hci_ev->opcode = BLE_HCI_EVCODE_DATA_BUF_OVERFLOW;
          hci_ev->length = sizeof(*ev);
          ev = (void *)hci_ev->data;

          ev->link_type = BLE_HCI_EVENT_ACL_BUF_OVERFLOW;

          ble_ll_hci_event_send(hci_ev);
        }
    }
}

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_CSA2
/****************************************************************************
 * Name: ble_ll_hci_ev_le_csa
 *
 * Description:
 *   Send a LE Channel Selection Algorithm event.
 *
 * Input Parameters:
 *   connsm - Pointer to connection state machine
 *
 ****************************************************************************/

void ble_ll_hci_ev_le_csa(struct ble_ll_conn_sm *connsm)
{
  struct ble_hci_ev_le_subev_chan_sel_alg *ev;
  struct ble_hci_ev *hci_ev;

  if (ble_ll_hci_is_le_event_enabled(BLE_HCI_LE_SUBEV_CHAN_SEL_ALG))
    {
      hci_ev = ble_ll_hci_evt_alloc(0);
      if (hci_ev)
        {
          hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
          hci_ev->length = sizeof(*ev);
          ev = (void *)hci_ev->data;

          ev->subev_code = BLE_HCI_LE_SUBEV_CHAN_SEL_ALG;
          ev->conn_handle = htole16(connsm->conn_handle);
          ev->csa = connsm->flags.csa2 ? 0x01 : 0x00;

          ble_ll_hci_event_send(hci_ev);
        }
    }
}

#endif /* CONFIG_BLE_LL_CFG_FEAT_LE_CSA2 */

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV
/****************************************************************************
 * Name: ble_ll_hci_ev_send_scan_req_recv
 *
 * Description:
 *   Sends the LE Scan Request Received event
 *
 ****************************************************************************/

void ble_ll_hci_ev_send_scan_req_recv(uint8_t adv_handle,
                                      const uint8_t *peer,
                                      uint8_t peer_addr_type)
{
  struct ble_hci_ev_le_subev_scan_req_rcvd *ev;
  struct ble_hci_ev *hci_ev;

  if (ble_ll_hci_is_le_event_enabled(BLE_HCI_LE_SUBEV_SCAN_REQ_RCVD))
    {
      hci_ev = ble_ll_hci_evt_alloc(0);
      if (hci_ev)
        {
          hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
          hci_ev->length = sizeof(*ev);
          ev = (void *)hci_ev->data;

          ev->subev_code = BLE_HCI_LE_SUBEV_SCAN_REQ_RCVD;
          ev->adv_handle = adv_handle;
          ev->peer_addr_type = peer_addr_type;
          memcpy(ev->peer_addr, peer, BLE_DEV_ADDR_LEN);

          ble_ll_hci_event_send(hci_ev);
        }
    }
}

#endif /* CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV */

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV
/****************************************************************************
 * Name: ble_ll_hci_ev_send_scan_timeout
 *
 * Description:
 *   Sends the  LE Scan Timeout Event
 *
 ****************************************************************************/

void ble_ll_hci_ev_send_scan_timeout(void)
{
  struct ble_hci_ev_le_subev_scan_timeout *ev;
  struct ble_hci_ev *hci_ev;

  if (ble_ll_hci_is_le_event_enabled(BLE_HCI_LE_SUBEV_SCAN_TIMEOUT))
    {
      hci_ev = ble_ll_hci_evt_alloc(0);
      if (hci_ev)
        {
          hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
          hci_ev->length = sizeof(*ev);
          ev = (void *)hci_ev->data;

          ev->subev_code = BLE_HCI_LE_SUBEV_SCAN_TIMEOUT;

          ble_ll_hci_event_send(hci_ev);
        }
    }
}

#endif /* CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV */

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV
/****************************************************************************
 * Name: ble_ll_hci_ev_send_adv_set_terminated
 *
 * Description:
 *   Sends the LE Advertising Set Terminated event
 *
 ****************************************************************************/

void ble_ll_hci_ev_send_adv_set_terminated(uint8_t status,
                                           uint8_t adv_handle,
                                           uint16_t conn_handle,
                                           uint8_t events)
{
  struct ble_hci_ev_le_subev_adv_set_terminated *ev;
  struct ble_hci_ev *hci_ev;

  if (ble_ll_hci_is_le_event_enabled(BLE_HCI_LE_SUBEV_ADV_SET_TERMINATED))
    {
      hci_ev = ble_ll_hci_evt_alloc(0);
      if (hci_ev)
        {
          hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
          hci_ev->length = sizeof(*ev);
          ev = (void *)hci_ev->data;

          ev->subev_code = BLE_HCI_LE_SUBEV_ADV_SET_TERMINATED;
          ev->status = status;
          ev->adv_handle = adv_handle;
          ev->conn_handle = htole16(conn_handle);
          ev->num_events = events;

          ble_ll_hci_event_send(hci_ev);
        }
    }
}

#endif /* CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV */

#ifdef CONFIG_BLE_LL_PHY
/****************************************************************************
 * Name: ble_ll_hci_ev_phy_update
 *
 * Description:
 *   Send a PHY update complete event
 *
 * Input Parameters:
 *   connsm - Pointer to connection state machine
 *   status - error status of event
 *
 ****************************************************************************/

int ble_ll_hci_ev_phy_update(struct ble_ll_conn_sm *connsm, uint8_t status)
{
  struct ble_hci_ev_le_subev_phy_update_complete *ev;
  struct ble_hci_ev *hci_ev;
  int rc;

  rc = 0;
  if (ble_ll_hci_is_le_event_enabled(BLE_HCI_LE_SUBEV_PHY_UPDATE_COMPLETE))
    {
      hci_ev = ble_ll_hci_evt_alloc(0);
      if (hci_ev)
        {
          hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
          hci_ev->length = sizeof(*ev);
          ev = (void *)hci_ev->data;

          ev->subev_code = BLE_HCI_LE_SUBEV_PHY_UPDATE_COMPLETE;
          ev->status = status;
          ev->conn_handle = htole16(connsm->conn_handle);
          ev->tx_phy = connsm->phy_data.cur_tx_phy;
          ev->rx_phy = connsm->phy_data.cur_rx_phy;

          ble_ll_hci_event_send(hci_ev);
        }
      else
        {
          rc = BLE_ERR_MEM_CAPACITY;
        }
    }

  return rc;
}

#endif /* CONFIG_BLE_LL_PHY */

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_SCA_UPDATE
/****************************************************************************
 * Name: ble_ll_hci_ev_sca_update
 *
 ****************************************************************************/

void ble_ll_hci_ev_sca_update(struct ble_ll_conn_sm *connsm, uint8_t status,
                              uint8_t peer_sca)
{
  struct ble_hci_ev_le_subev_peer_sca_complete *ev;
  struct ble_hci_ev *hci_ev;

  if (!ble_ll_hci_is_le_event_enabled(BLE_HCI_LE_SUBEV_REQ_PEER_SCA_COMP))
    {
      return;
    }

  hci_ev = ble_ll_hci_evt_alloc(0);
  if (!hci_ev)
    {
      return;
    }

  hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
  hci_ev->length = sizeof(*ev);
  ev = (void *)hci_ev->data;

  ev->subev_code = BLE_HCI_LE_SUBEV_REQ_PEER_SCA_COMP;
  ev->status = status;
  ev->conn_handle = htole16(connsm->conn_handle);
  ev->sca = peer_sca;

  ble_ll_hci_event_send(hci_ev);
}

#endif /* CONFIG_BLE_LL_CFG_FEAT_LL_SCA_UPDATE */

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_ENHANCED_CONN_UPDATE
/****************************************************************************
 * Name: ble_ll_hci_ev_subrate_change
 *
 ****************************************************************************/

void ble_ll_hci_ev_subrate_change(struct ble_ll_conn_sm *connsm,
                                  uint8_t status)
{
  struct ble_hci_ev_le_subev_subrate_change *ev;
  struct ble_hci_ev *hci_ev;

  if (!ble_ll_hci_is_le_event_enabled(BLE_HCI_LE_SUBEV_SUBRATE_CHANGE))
    {
      return;
    }

  hci_ev = ble_ll_hci_evt_alloc(0);
  if (!hci_ev)
    {
      return;
    }

  hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
  hci_ev->length = sizeof(*ev);
  ev = (void *)hci_ev->data;

  ev->subev_code = BLE_HCI_LE_SUBEV_SUBRATE_CHANGE;
  ev->status = status;
  ev->conn_handle = htole16(connsm->conn_handle);
  ev->subrate_factor = htole16(connsm->subrate_factor);
  ev->periph_latency = htole16(connsm->periph_latency);
  ev->cont_num = htole16(connsm->cont_num);
  ev->supervision_tmo = htole16(connsm->supervision_tmo);

  ble_ll_hci_event_send(hci_ev);
}

#endif /* CONFIG_BLE_LL_CFG_FEAT_LL_ENHANCED_CONN_UPDATE */

/****************************************************************************
 * Name: ble_ll_hci_ev_send_vs_assert
 *
 ****************************************************************************/

void ble_ll_hci_ev_send_vs_assert(const char *file, uint32_t line)
{
  struct ble_hci_ev_vs *ev;
  struct ble_hci_ev *hci_ev;
  unsigned int str_len;
  bool skip = true;
  uint8_t digit;
  int max_len;
  int i;

  /* 6 is for line number ":00000" , we assume files have no more than 64k of
   * lines
   */

  max_len = BLE_HCI_MAX_DATA_LEN - sizeof(*ev) - 6;

  hci_ev = ble_ll_hci_evt_alloc(0);
  if (hci_ev)
    {
      hci_ev->opcode = BLE_HCI_EVCODE_VS;
      hci_ev->length = sizeof(*ev);
      ev = (void *)hci_ev->data;

      /* Debug id for future use */

      ev->id = BLE_HCI_VS_SUBEV_ID_ASSERT;

      /* snprintf would be nicer but this is heavy on flash
       * len = snprintf((char *) ev->data, max_len, "%s:%u", file, line);
       * if (len < 0) {
       *     len = 0;
       * } else if (len > max_len) {
       *     len = max_len;
       * }
       *
       *  hci_ev->length += len;
       */

      str_len = strlen(file);
      if (str_len > (unsigned int)max_len)
        {
          str_len = max_len;
        }

      memcpy(ev->data, file, str_len);
      ev->data[str_len++] = ':';

      for (i = 100000; i >= 10; i /= 10)
        {
          digit = (line % i) / (i / 10);

          if (!digit && skip)
            {
              continue;
            }

          skip = false;
          ev->data[str_len++] = '0' + digit;
        }

      hci_ev->length += str_len;

      ble_ll_hci_event_send(hci_ev);
    }
}

/****************************************************************************
 * Name: ble_ll_hci_ev_send_vs_printf
 *
 ****************************************************************************/

void ble_ll_hci_ev_send_vs_printf(uint8_t id, const char *fmt, ...)
{
  struct ble_hci_ev_vs *ev;
  struct ble_hci_ev *hci_ev;
  va_list ap;
  int len;

  hci_ev = ble_ll_hci_evt_alloc(1);
  if (!hci_ev)
    {
      return;
    }

  hci_ev->opcode = BLE_HCI_EVCODE_VS;
  hci_ev->length = sizeof(*ev);

  ev = (void *)hci_ev->data;
  ev->id = id;

  va_start(ap, fmt);
  len = vsnprintf((void *)ev->data, BLE_HCI_MAX_DATA_LEN - sizeof(*ev), fmt,
                  ap);
  va_end(ap);

  ev->data[len] = 0;
  hci_ev->length += len + 1;

  ble_ll_hci_event_send(hci_ev);
}
