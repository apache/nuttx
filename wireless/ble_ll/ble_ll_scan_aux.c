/****************************************************************************
 * wireless/ble_ll/ble_ll_scan_aux.c
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
 * Derived from Apache Mynewt NimBLE
 * (nimble/controller/src/ble_ll_scan_aux.c).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#if defined(CONFIG_BLE_LL_ROLE_OBSERVER) && defined(CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV)
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "ble.h"
#include "hci_common.h"
#include "ble_ll_utils.h"
#include "ble_phy.h"
#include "ble_hw.h"
#include "ble_ll.h"
#include "ble_ll_pdu.h"
#include "ble_ll_sched.h"
#include "ble_ll_scan.h"
#include "ble_ll_scan_aux.h"
#include "ble_ll_hci.h"
#include "ble_ll_whitelist.h"
#include "ble_ll_resolv.h"
#include "ble_ll_sync.h"
#include "ble_ll_priv.h"

#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_BLE_LL_ROLE_OBSERVER) && defined(CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV)
#define BLE_LL_SCAN_AUX_F_AUX_ADV          0x0001
#define BLE_LL_SCAN_AUX_F_AUX_CHAIN        0x0002
#define BLE_LL_SCAN_AUX_F_MATCHED          0x0004
#define BLE_LL_SCAN_AUX_F_W4_SCAN_RSP      0x0008
#define BLE_LL_SCAN_AUX_F_SCANNED          0x0010
#define BLE_LL_SCAN_AUX_F_HAS_ADVA         0x0020
#define BLE_LL_SCAN_AUX_F_HAS_TARGETA      0x0040
#define BLE_LL_SCAN_AUX_F_HAS_ADI          0x0080
#define BLE_LL_SCAN_AUX_F_RESOLVED_ADVA    0x0100
#define BLE_LL_SCAN_AUX_F_RESOLVED_TARGETA 0x0200
#define BLE_LL_SCAN_AUX_F_CONNECTABLE      0x0400
#define BLE_LL_SCAN_AUX_F_W4_CONNECT_RSP   0x0800
#define BLE_LL_SCAN_AUX_H_SENT_ANY  0x01
#define BLE_LL_SCAN_AUX_H_DONE      0x02
#define BLE_LL_SCAN_AUX_H_TRUNCATED 0x04

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#if defined(CONFIG_BLE_LL_ROLE_OBSERVER) && defined(CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV)
struct ble_ll_scan_aux_data
{
  FAR struct ble_ll_scan_aux_data *next;
  uint16_t flags;
  uint8_t hci_state;

  uint8_t scan_type;

  uint8_t pri_phy;
  uint8_t sec_phy;
  uint8_t chan;
  uint16_t wfr_us;
  uint32_t aux_ptr;
  struct ble_ll_sched_item sch;
  struct ble_ll_event_s break_ev;
  struct ble_hci_ev *hci_ev;

  uint16_t adi;

  uint8_t adva[6];
  uint8_t targeta[6];
  uint8_t adva_type : 1;
  uint8_t targeta_type : 1;

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  int8_t rpa_index;
#endif
#ifdef CONFIG_BLE_LL_ADV_CODING_SELECTION
  uint8_t pri_phy_mode;
#endif
};

#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_BLE_LL_ROLE_OBSERVER) && defined(CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV)
static void ble_ll_hci_ev_send_ext_adv_truncated_report(
    struct ble_ll_scan_aux_data *aux);

#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_BLE_LL_ROLE_OBSERVER) && defined(CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV)
static struct ble_ll_scan_aux_data
  g_aux_data[CONFIG_BLE_LL_SCAN_AUX_SEGMENT_CNT];

static FAR struct ble_ll_scan_aux_data *g_aux_data_free;

static struct ble_ll_scan_aux_data *aux_data_current;

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_BLE_LL_ROLE_OBSERVER) && defined(CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV)
/****************************************************************************
 * Name: ble_ll_scan_aux_get_own_addr
 *
 ****************************************************************************/

static inline uint8_t *ble_ll_scan_aux_get_own_addr(void)
{
  uint8_t own_addr_type;

  own_addr_type = ble_ll_scan_get_own_addr_type() & 1;

  return ble_ll_get_our_devaddr(own_addr_type);
}

/****************************************************************************
 * Name: ble_ll_scan_aux_sched_cb
 *
 ****************************************************************************/

static int ble_ll_scan_aux_sched_cb(struct ble_ll_sched_item *sch)
{
  struct ble_ll_scan_aux_data *aux = sch->cb_arg;
#ifdef CONFIG_BLE_LL_PHY
  uint8_t phy_mode;
#endif
  uint8_t lls;
  int rc;

  DEBUGASSERT(aux);

  lls = ble_ll_state_get();
  DEBUGASSERT(lls == BLE_LL_STATE_STANDBY);

  rc = ble_phy_setchan(aux->chan, BLE_ACCESS_ADDR_ADV, BLE_LL_CRCINIT_ADV);
  DEBUGASSERT(rc == 0);

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
  ble_phy_encrypt_disable();
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  if (ble_ll_resolv_enabled())
    {
      ble_phy_resolv_list_enable();
    }
  else
    {
      ble_phy_resolv_list_disable();
    }
#endif

#ifdef CONFIG_BLE_LL_PHY
  phy_mode = ble_ll_phy_to_phy_mode(aux->sec_phy, BLE_HCI_LE_PHY_CODED_ANY);
  ble_phy_mode_set(phy_mode, phy_mode);
#endif

  /* if scan is not passive we need to set tx power as we may end up sending
   * package
   */

  /* TODO do this only on first AUX? */

  if (aux->scan_type != BLE_SCAN_TYPE_PASSIVE)
    {
      ble_ll_tx_power_set(g_ble_ll_tx_power);
    }

  rc = ble_phy_rx_set_start_time(
      sch->start_time + g_ble_ll_sched_offset_ticks, sch->remainder);
  if (rc != 0 && rc != BLE_PHY_ERR_RX_LATE)
    {
      ble_ll_scan_aux_break(aux);
      return BLE_LL_SCHED_STATE_DONE;
    }

  /* Keep listening even if we are late, we may still receive something */

  if (ble_ll_scan_get_filt_policy() & 1)
    {
      ble_ll_whitelist_enable();
    }
  else
    {
      ble_ll_whitelist_disable();
    }

  ble_phy_wfr_enable(BLE_PHY_WFR_ENABLE_RX, 0, aux->wfr_us);

  aux_data_current = aux;

  ble_ll_state_set(BLE_LL_STATE_SCAN_AUX);

  return BLE_LL_SCHED_STATE_RUNNING;
}

/****************************************************************************
 * Name: ble_ll_scan_aux_alloc
 *
 ****************************************************************************/

static struct ble_ll_scan_aux_data *ble_ll_scan_aux_alloc(void)
{
  struct ble_ll_scan_aux_data *aux;
  irqstate_t flags;

  flags = enter_critical_section();
  aux = g_aux_data_free;
  if (aux)
    {
      g_aux_data_free = aux->next;
    }

  leave_critical_section(flags);

  if (!aux)
    {
      return NULL;
    }

  memset(aux, 0, sizeof(*aux));

  aux->sch.sched_cb = ble_ll_scan_aux_sched_cb;
  aux->sch.sched_type = BLE_LL_SCHED_TYPE_SCAN_AUX;
  aux->sch.cb_arg = aux;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  aux->rpa_index = -1;
#endif

  return aux;
}

/****************************************************************************
 * Name: ble_ll_scan_aux_free
 *
 ****************************************************************************/

static void ble_ll_scan_aux_free(struct ble_ll_scan_aux_data *aux)
{
  irqstate_t flags;

  DEBUGASSERT(!aux->sch.enqueued);
  DEBUGASSERT(aux->hci_ev == NULL);
  DEBUGASSERT((aux->hci_state & BLE_LL_SCAN_AUX_H_DONE) ||
              !(aux->hci_state & BLE_LL_SCAN_AUX_H_SENT_ANY));

  flags = enter_critical_section();
  aux->next = g_aux_data_free;
  g_aux_data_free = aux;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: ble_ll_scan_aux_need_truncation
 *
 ****************************************************************************/

static inline bool ble_ll_scan_aux_need_truncation(
    struct ble_ll_scan_aux_data *aux)
{
  return (aux->hci_state & BLE_LL_SCAN_AUX_H_SENT_ANY) &&
         !(aux->hci_state & BLE_LL_SCAN_AUX_H_DONE);
}

/****************************************************************************
 * Name: ble_ll_hci_ev_alloc_ext_adv_report_for_aux
 *
 ****************************************************************************/

static struct ble_hci_ev *ble_ll_hci_ev_alloc_ext_adv_report_for_aux(
    struct ble_mbuf_hdr_rxinfo *rxinfo, struct ble_ll_scan_addr_data *addrd,
    struct ble_ll_scan_aux_data *aux)
{
  struct ble_hci_ev_le_subev_ext_adv_rpt *hci_subev;
  struct ext_adv_report *report;
  struct ble_hci_ev *hci_ev;

  hci_ev = ble_ll_hci_evt_alloc(1);
  if (!hci_ev)
    {
      return NULL;
    }

  hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
  hci_ev->length = sizeof(*hci_subev) + sizeof(*report);

  hci_subev = (void *)hci_ev->data;
  hci_subev->subev_code = BLE_HCI_LE_SUBEV_EXT_ADV_RPT;
  hci_subev->num_reports = 1;

  report = hci_subev->reports;

  memset(report, 0, sizeof(*report));

  report->evt_type = 0;
  if (addrd->adva)
    {
      report->addr_type = addrd->adv_addr_type;
      memcpy(report->addr, addrd->adv_addr, 6);
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
      if (addrd->adva_resolved)
        {
          report->addr_type += 2;
        }
#endif
    }
  else
    {
      report->addr_type = 0xff;
    }

  report->pri_phy = aux->pri_phy;
  report->sec_phy = aux->sec_phy;
  report->sid = aux->adi >> 12;
  report->tx_power = 0x7f;
  report->rssi = 0x7f;
  report->periodic_itvl = 0;

#ifdef CONFIG_BLE_LL_ADV_CODING_SELECTION
  if (aux->pri_phy_mode == BLE_PHY_MODE_CODED_500KBPS)
    {
      report->pri_phy = 0x04;
    }

  if (rxinfo->phy_mode == BLE_PHY_MODE_CODED_500KBPS)
    {
      report->sec_phy = 0x04;
    }
#endif

  if (addrd->targeta)
    {
      report->evt_type |= BLE_HCI_ADV_DIRECT_MASK;
      report->dir_addr_type = addrd->targeta_type;
      memcpy(report->dir_addr, addrd->targeta, 6);
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
      if (addrd->targeta_resolved)
        {
          report->dir_addr_type += 2;
        }
      else if (ble_ll_is_rpa(addrd->targeta, addrd->targeta_type))
        {
          report->dir_addr_type = 0xfe;
        }
#endif
    }

  report->data_len = 0;

  return hci_ev;
}

/****************************************************************************
 * Name: ble_ll_hci_ev_dup_ext_adv_report
 *
 ****************************************************************************/

static struct ble_hci_ev *ble_ll_hci_ev_dup_ext_adv_report(
    struct ble_hci_ev *hci_ev_src)
{
  struct ble_hci_ev_le_subev_ext_adv_rpt *hci_subev;
  struct ext_adv_report *report;
  struct ble_hci_ev *hci_ev;

  hci_ev = ble_ll_hci_evt_alloc(1);
  if (!hci_ev)
    {
      return NULL;
    }

  memcpy(hci_ev, hci_ev_src,
         sizeof(*hci_ev) + sizeof(*hci_subev) + sizeof(*report));
  hci_ev->length = sizeof(*hci_subev) + sizeof(*report);

  hci_subev = (void *)hci_ev->data;

  report = hci_subev->reports;
  report->data_len = 0;

  return hci_ev;
}

/****************************************************************************
 * Name: ble_ll_hci_ev_update_ext_adv_report_from_aux
 *
 ****************************************************************************/

static void ble_ll_hci_ev_update_ext_adv_report_from_aux(
    struct ble_hci_ev *hci_ev, struct iob_s *rxpdu,
    struct ble_mbuf_hdr *rxhdr)
{
  struct ble_hci_ev_le_subev_ext_adv_rpt *hci_subev;
  struct ext_adv_report *report;
  uint8_t adv_mode;
  uint8_t eh_len;
  uint8_t eh_flags;
  uint8_t *eh_data;
  uint8_t *rxbuf;

  hci_subev = (void *)hci_ev->data;
  report = hci_subev->reports;
  rxbuf = IOB_DATA(rxpdu);

  adv_mode = rxbuf[2] >> 6;
  eh_len = rxbuf[2] & 0x3f;
  eh_flags = rxbuf[3];
  eh_data = &rxbuf[4];

  report->evt_type |= adv_mode;
  if (rxhdr->rxinfo.flags & BLE_MBUF_HDR_F_SCAN_RSP_RXD)
    {
      report->evt_type |= BLE_HCI_ADV_SCAN_MASK | BLE_HCI_ADV_SCAN_RSP_MASK;
    }

  report->sec_phy = rxhdr->rxinfo.phy;

#ifdef CONFIG_BLE_LL_ADV_CODING_SELECTION
  if (rxhdr->rxinfo.phy_mode == BLE_PHY_MODE_CODED_500KBPS)
    {
      report->sec_phy = 0x04;
    }
#endif

  /* Strip PDU header and ext header, leave only AD */

  ble_ll_iob_adj(rxpdu, 3 + eh_len);

  /* We only care about SyncInfo and TxPower so don't bother parsing if they
   * are not present, just set to 'unknown' values.
   */

  if ((eh_len == 0) || !(eh_flags & ((1 << BLE_LL_EXT_ADV_SYNC_INFO_BIT) |
                                     (1 << BLE_LL_EXT_ADV_TX_POWER_BIT))))
    {
      report->periodic_itvl = 0;
      report->tx_power = 0x7f;
      return;
    }

  /* Now parse extended header... */

  if (eh_flags & (1 << BLE_LL_EXT_ADV_ADVA_BIT))
    {
      eh_data += BLE_LL_EXT_ADV_ADVA_SIZE;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_TARGETA_BIT))
    {
      eh_data += BLE_LL_EXT_ADV_TARGETA_SIZE;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_CTE_INFO_BIT))
    {
      eh_data += BLE_LL_EXT_ADV_CTE_INFO_SIZE;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_DATA_INFO_BIT))
    {
      eh_data += BLE_LL_EXT_ADV_DATA_INFO_SIZE;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_AUX_PTR_BIT))
    {
      eh_data += BLE_LL_EXT_ADV_AUX_PTR_SIZE;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_SYNC_INFO_BIT))
    {
      report->periodic_itvl = get_le16(eh_data + 2);
      eh_data += BLE_LL_EXT_ADV_SYNC_INFO_SIZE;
    }
  else
    {
      report->periodic_itvl = 0;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_TX_POWER_BIT))
    {
      report->tx_power = *eh_data;
    }
  else
    {
      report->tx_power = 0x7f;
    }
}

/****************************************************************************
 * Name: ble_ll_hci_ev_update_ext_adv_report_from_ext
 *
 ****************************************************************************/

static void ble_ll_hci_ev_update_ext_adv_report_from_ext(
    struct ble_hci_ev *hci_ev, struct iob_s *rxpdu,
    struct ble_mbuf_hdr *rxhdr)
{
  struct ble_mbuf_hdr_rxinfo *rxinfo = &rxhdr->rxinfo;
  struct ble_hci_ev_le_subev_ext_adv_rpt *hci_subev;
  struct ext_adv_report *report;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  struct ble_ll_resolv_entry *rl;
#endif
  uint8_t pdu_hdr;
  uint8_t adv_mode;
  uint8_t eh_flags;
  uint8_t *eh_data;
  uint8_t *rxbuf;

  rxbuf = IOB_DATA(rxpdu);

  pdu_hdr = rxbuf[0];
  adv_mode = rxbuf[2] >> 6;
  eh_flags = rxbuf[3];
  eh_data = &rxbuf[4];

  hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
  hci_ev->length = sizeof(*hci_subev) + sizeof(*report);

  hci_subev = (void *)hci_ev->data;
  hci_subev->subev_code = BLE_HCI_LE_SUBEV_EXT_ADV_RPT;
  hci_subev->num_reports = 1;

  report = hci_subev->reports;

  memset(report, 0, sizeof(*report));

  report->evt_type = adv_mode;

  report->pri_phy = rxinfo->phy;
  report->sec_phy = 0;
  report->sid = 0xff;
  report->rssi = rxhdr->rxinfo.rssi - ble_ll_rx_gain();
  report->periodic_itvl = 0;
  report->data_len = 0;

#ifdef CONFIG_BLE_LL_ADV_CODING_SELECTION
  if (rxinfo->phy_mode == BLE_PHY_MODE_CODED_500KBPS)
    {
      report->pri_phy = 0x04;
    }
#endif

  /* Now parse extended header... */

  if (eh_flags & (1 << BLE_LL_EXT_ADV_ADVA_BIT))
    {
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
      if (rxinfo->flags & BLE_MBUF_HDR_F_RESOLVED)
        {
          rl = &g_ble_ll_resolv_list[rxinfo->rpa_index];
          report->addr_type = rl->rl_addr_type + 2;
          memcpy(report->addr, rl->rl_identity_addr, 6);
        }
      else
        {
          report->addr_type = !!(pdu_hdr & BLE_ADV_PDU_HDR_TXADD_MASK);
          memcpy(report->addr, eh_data, 6);
        }
#else
      report->addr_type = !!(pdu_hdr & BLE_ADV_PDU_HDR_TXADD_MASK);
      memcpy(report->addr, eh_data, 6);
#endif
      eh_data += BLE_LL_EXT_ADV_ADVA_SIZE;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_TARGETA_BIT))
    {
      report->evt_type |= BLE_HCI_ADV_DIRECT_MASK;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
      if (rxinfo->flags & BLE_MBUF_HDR_F_TARGETA_RESOLVED)
        {
          report->dir_addr_type = (ble_ll_scan_get_own_addr_type() & 1) + 2;
          memcpy(report->dir_addr, ble_ll_scan_aux_get_own_addr(), 6);
        }
      else
        {
          if (ble_ll_is_rpa(eh_data, pdu_hdr & BLE_ADV_PDU_HDR_RXADD_MASK))
            {
              report->dir_addr_type = 0xfe;
            }
          else
            {
              report->dir_addr_type =
                  !!(pdu_hdr & BLE_ADV_PDU_HDR_RXADD_MASK);
            }

          memcpy(report->dir_addr, eh_data, 6);
        }
#else
      report->dir_addr_type = !!(pdu_hdr & BLE_ADV_PDU_HDR_RXADD_MASK);
      memcpy(report->dir_addr, eh_data, 6);
#endif
      eh_data += BLE_LL_EXT_ADV_TARGETA_SIZE;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_CTE_INFO_BIT))
    {
      eh_data += BLE_LL_EXT_ADV_CTE_INFO_SIZE;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_DATA_INFO_BIT))
    {
      eh_data += BLE_LL_EXT_ADV_DATA_INFO_SIZE;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_AUX_PTR_BIT))
    {
      eh_data += BLE_LL_EXT_ADV_AUX_PTR_SIZE;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_SYNC_INFO_BIT))
    {
      eh_data += BLE_LL_EXT_ADV_SYNC_INFO_SIZE;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_TX_POWER_BIT))
    {
      report->tx_power = *eh_data;
    }
  else
    {
      report->tx_power = 0x7f;
    }

  /* AdvData in ADV_EXT_IND is RFU so we should ignore it, i.e. we can just
   * remove any data left in mbuf
   */

  ble_ll_iob_adj(rxpdu, rxpdu->io_pktlen);
}

/****************************************************************************
 * Name: ble_ll_hci_ev_send_ext_adv_truncated_report
 *
 ****************************************************************************/

static void ble_ll_hci_ev_send_ext_adv_truncated_report(
    struct ble_ll_scan_aux_data *aux)
{
  struct ble_hci_ev_le_subev_ext_adv_rpt *hci_subev;
  struct ext_adv_report *report;
  struct ble_hci_ev *hci_ev;

  if (!ble_ll_hci_is_le_event_enabled(BLE_HCI_LE_SUBEV_EXT_ADV_RPT))
    {
      return;
    }

  hci_ev = aux->hci_ev;
  aux->hci_ev = NULL;

  DEBUGASSERT(hci_ev);

  hci_subev = (void *)hci_ev->data;
  report = hci_subev->reports;
  report->evt_type |= BLE_HCI_ADV_DATA_STATUS_TRUNCATED;

  ble_ll_hci_event_send(hci_ev);

  aux->hci_state |= BLE_LL_SCAN_AUX_H_DONE | BLE_LL_SCAN_AUX_H_TRUNCATED;
}

/****************************************************************************
 * Name: ble_ll_hci_ev_send_ext_adv_report
 *
 ****************************************************************************/

static int ble_ll_hci_ev_send_ext_adv_report(struct iob_s *rxpdu,
                                             struct ble_mbuf_hdr *rxhdr,
                                             struct ble_hci_ev **hci_ev)
{
  struct ble_mbuf_hdr_rxinfo *rxinfo = &rxhdr->rxinfo;
  struct ble_hci_ev_le_subev_ext_adv_rpt *hci_subev;
  struct ble_hci_ev *hci_ev_next;
  struct ext_adv_report *report;
  bool truncated = false;
  int max_data_len;
  int data_len;
  int offset;

  data_len = rxpdu->io_pktlen;
  max_data_len = BLE_LL_MAX_EVT_LEN - sizeof(**hci_ev) - sizeof(*hci_subev) -
                 sizeof(*report);
  offset = 0;
  hci_ev_next = NULL;

  do
    {
      hci_subev = (void *)(*hci_ev)->data;
      report = hci_subev->reports;

      report->rssi = rxinfo->rssi - ble_ll_rx_gain();

      report->data_len = MIN(max_data_len, data_len - offset);
      iob_copyout(report->data, rxpdu, report->data_len, offset);
      (*hci_ev)->length += report->data_len;

      offset += report->data_len;

      /* We need another event if either there are still some data left in
       * this PDU or scan for next aux is scheduled.
       */

      if ((offset < data_len) ||
          (rxinfo->flags & BLE_MBUF_HDR_F_AUX_PTR_WAIT))
        {
          hci_ev_next = ble_ll_hci_ev_dup_ext_adv_report(*hci_ev);
          if (hci_ev_next)
            {
              report->evt_type |= BLE_HCI_ADV_DATA_STATUS_INCOMPLETE;
            }
          else
            {
              report->evt_type |= BLE_HCI_ADV_DATA_STATUS_TRUNCATED;
            }
        }
      else if (rxinfo->flags & BLE_MBUF_HDR_F_AUX_PTR_FAILED)
        {
          report->evt_type |= BLE_HCI_ADV_DATA_STATUS_TRUNCATED;
        }

      switch (report->evt_type & BLE_HCI_ADV_DATA_STATUS_MASK)
        {
          case BLE_HCI_ADV_DATA_STATUS_TRUNCATED:
            truncated = true;

            /* no break */

          case BLE_HCI_ADV_DATA_STATUS_COMPLETE:
            DEBUGASSERT(!hci_ev_next);
            break;
          case BLE_HCI_ADV_DATA_STATUS_INCOMPLETE:
            DEBUGASSERT(hci_ev_next);
            break;
          default:
            DEBUGASSERT(0);
        }

      ble_ll_hci_event_send(*hci_ev);

      *hci_ev = hci_ev_next;
      hci_ev_next = NULL;
    }
  while ((offset < data_len) && *hci_ev);

  return truncated ? -1 : 0;
}

/****************************************************************************
 * Name: ble_ll_hci_ev_send_ext_adv_report_for_aux
 *
 ****************************************************************************/

static int ble_ll_hci_ev_send_ext_adv_report_for_aux(
    struct iob_s *rxpdu, struct ble_mbuf_hdr *rxhdr,
    struct ble_ll_scan_aux_data *aux, struct ble_ll_scan_addr_data *addrd)
{
  struct ble_hci_ev *hci_ev;
  int rc;

  if (!ble_ll_hci_is_le_event_enabled(BLE_HCI_LE_SUBEV_EXT_ADV_RPT))
    {
      aux->hci_state = BLE_LL_SCAN_AUX_H_DONE;
      return -1;
    }

  /* We need to always keep one event allocated in aux to be able to truncate
   * data properly in case of an error. If there is no event in aux it means
   * this is first event and we can silently ignore in case of an error.
   */

  if (aux->hci_ev)
    {
      hci_ev = aux->hci_ev;
      aux->hci_ev = NULL;
    }
  else
    {
      hci_ev = ble_ll_hci_ev_alloc_ext_adv_report_for_aux(&rxhdr->rxinfo,
                                                          addrd, aux);
      if (!hci_ev)
        {
          aux->hci_state = BLE_LL_SCAN_AUX_H_DONE;
          return -1;
        }
    }

  ble_ll_hci_ev_update_ext_adv_report_from_aux(hci_ev, rxpdu, rxhdr);

  rc = ble_ll_hci_ev_send_ext_adv_report(rxpdu, rxhdr, &hci_ev);
  if (rc < 0)
    {
      DEBUGASSERT(!hci_ev);
      aux->hci_state = BLE_LL_SCAN_AUX_H_DONE | BLE_LL_SCAN_AUX_H_TRUNCATED;
    }
  else if (hci_ev)
    {
      aux->hci_state = BLE_LL_SCAN_AUX_H_SENT_ANY;
    }
  else
    {
      aux->hci_state = BLE_LL_SCAN_AUX_H_DONE;
    }

  aux->hci_ev = hci_ev;

  return rc;
}

/****************************************************************************
 * Name: ble_ll_hci_ev_send_ext_adv_report_for_ext
 *
 ****************************************************************************/

static void ble_ll_hci_ev_send_ext_adv_report_for_ext(
    struct iob_s *rxpdu, struct ble_mbuf_hdr *rxhdr)
{
  struct ble_hci_ev *hci_ev;

  if (!ble_ll_hci_is_le_event_enabled(BLE_HCI_LE_SUBEV_EXT_ADV_RPT))
    {
      return;
    }

  hci_ev = ble_ll_hci_evt_alloc(1);
  if (!hci_ev)
    {
      return;
    }

  ble_ll_hci_ev_update_ext_adv_report_from_ext(hci_ev, rxpdu, rxhdr);

  ble_ll_hci_ev_send_ext_adv_report(rxpdu, rxhdr, &hci_ev);

  DEBUGASSERT(!hci_ev);
}

/****************************************************************************
 * Name: ble_ll_scan_aux_break_ev
 *
 ****************************************************************************/

static void ble_ll_scan_aux_break_ev(FAR void *arg)
{
  struct ble_ll_scan_aux_data *aux = arg;

  DEBUGASSERT(aux);

  if (ble_ll_scan_aux_need_truncation(aux))
    {
      ble_ll_hci_ev_send_ext_adv_truncated_report(aux);
    }

  /* Update backoff if we were waiting for scan response */

  if (aux->flags & BLE_LL_SCAN_AUX_F_W4_SCAN_RSP)
    {
      ble_ll_scan_backoff_update(0);
    }

  ble_ll_scan_aux_free(aux);
  ble_ll_scan_chk_resume();
}

/****************************************************************************
 * Name: ble_ll_scan_aux_phy_to_phy
 *
 ****************************************************************************/

static int ble_ll_scan_aux_phy_to_phy(uint8_t aux_phy, uint8_t *phy)
{
  switch (aux_phy)
    {
      case 0:
        *phy = BLE_PHY_1M;
        break;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_2M_PHY
      case 1:
        *phy = BLE_PHY_2M;
        break;
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_CODED_PHY
      case 2:
        *phy = BLE_PHY_CODED;
        break;
#endif
      default:
        return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: ble_ll_scan_aux_parse_to_aux_data
 *
 ****************************************************************************/

static int ble_ll_scan_aux_parse_to_aux_data(
    struct ble_ll_scan_aux_data *aux, uint8_t *rxbuf,
    struct ble_mbuf_hdr *rxhdr)
{
  uint8_t pdu_hdr;
  uint8_t pdu_len;
  uint8_t adv_mode;
  uint8_t eh_len;
  uint8_t eh_flags;
  uint8_t *eh_data;

  pdu_hdr = rxbuf[0];
  pdu_len = rxbuf[1];

  /* PDU without at least Extended Header Length is invalid */

  if (pdu_len == 0)
    {
      return -1;
    }

  /* Mark as AUX received on 1st PDU, then as CHAIN received on subsequent */

  if (aux->flags & BLE_LL_SCAN_AUX_F_AUX_ADV)
    {
      aux->flags |= BLE_LL_SCAN_AUX_F_AUX_CHAIN;
    }
  else
    {
      aux->flags |= BLE_LL_SCAN_AUX_F_AUX_ADV;
    }

  adv_mode = rxbuf[2] >> 6;
  eh_len = rxbuf[2] & 0x3f;

  /* Only AUX_CHAIN_IND is valid without Extended Header */

  if (eh_len == 0)
    {
      if (!(aux->flags & BLE_LL_SCAN_AUX_F_AUX_CHAIN) || adv_mode)
        {
          return -1;
        }

      return 0;
    }

  eh_flags = rxbuf[3];
  eh_data = &rxbuf[4];

  /* Now parse extended header... */

  /* AdvA is only valid in AUX_ADV_IND if not already present in
   * ADV_EXT_IND
   */

  if (eh_flags & (1 << BLE_LL_EXT_ADV_ADVA_BIT))
    {
      if (!(aux->flags & BLE_LL_SCAN_AUX_F_HAS_ADVA) &&
          !(aux->flags & BLE_LL_SCAN_AUX_F_AUX_CHAIN))
        {
          memcpy(aux->adva, eh_data, 6);
          aux->adva_type = !!(pdu_hdr & BLE_ADV_PDU_HDR_TXADD_MASK);
          aux->flags |= BLE_LL_SCAN_AUX_F_HAS_ADVA;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
          aux->rpa_index = ble_hw_resolv_list_match();
#endif
        }

      eh_data += BLE_LL_EXT_ADV_ADVA_SIZE;
    }

  /* TargetA is only valid in AUX_ADV_IND if not already present in
   * ADV_EXT_IND
   */

  if (eh_flags & (1 << BLE_LL_EXT_ADV_TARGETA_BIT))
    {
      if (!(aux->flags & BLE_LL_SCAN_AUX_F_HAS_TARGETA) &&
          !(aux->flags & BLE_LL_SCAN_AUX_F_AUX_CHAIN))
        {
          memcpy(aux->targeta, eh_data, 6);
          aux->targeta_type = !!(pdu_hdr & BLE_ADV_PDU_HDR_RXADD_MASK);
          aux->flags |= BLE_LL_SCAN_AUX_F_HAS_TARGETA;
        }

      eh_data += BLE_LL_EXT_ADV_TARGETA_SIZE;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_CTE_INFO_BIT))
    {
      eh_data += BLE_LL_EXT_ADV_CTE_INFO_SIZE;
    }

  /* ADI handling is a bit convoluted.... ADI is mandatory in ADV_EXT_IND
   * with AuxPtr and is also mandatory in PDU if included in superior PDU.
   * This implies that each AUX_CHAIN shall have ADI. However... AUX_SCAN_RSP
   * does not need to have ADI, so if there's no ADI in AUX_SCAN_RSP we allow
   * it and clear corresponding flag to skip ADI checks on subsequent PDUs.
   */

  if (eh_flags & (1 << BLE_LL_EXT_ADV_DATA_INFO_BIT))
    {
      if (aux->flags & BLE_LL_SCAN_AUX_F_HAS_ADI)
        {
          if (get_le16(eh_data) != aux->adi)
            {
              return -1;
            }
        }

      eh_data += BLE_LL_EXT_ADV_DATA_INFO_SIZE;
    }
  else if (aux->flags & BLE_LL_SCAN_AUX_F_HAS_ADI)
    {
      if (rxhdr->rxinfo.flags & BLE_MBUF_HDR_F_SCAN_RSP_RXD)
        {
          aux->flags &= ~BLE_LL_SCAN_AUX_F_HAS_ADI;
        }
      else
        {
          return -1;
        }
    }

  /* AuxPtr is RFU if AdvMode!=0 so ignore in such case */

  if ((adv_mode == 0) && (eh_flags & (1 << BLE_LL_EXT_ADV_AUX_PTR_BIT)))
    {
      aux->aux_ptr = get_le24(eh_data);
      rxhdr->rxinfo.flags |= BLE_MBUF_HDR_F_AUX_PTR_WAIT;
    }

  return 0;
}

/****************************************************************************
 * Name: ble_ll_scan_aux_ext_parse
 *
 ****************************************************************************/

static int ble_ll_scan_aux_ext_parse(uint8_t *rxbuf,
                                     struct ble_ll_scan_addr_data *addrd,
                                     uint16_t *adi, uint32_t *aux_ptr)
{
  uint8_t pdu_hdr;
  uint8_t pdu_len;
  uint8_t adv_mode;
  uint8_t eh_len;
  uint8_t eh_flags;
  uint8_t *eh_data;

  pdu_hdr = rxbuf[0];
  pdu_len = rxbuf[1];

  if (pdu_len == 0)
    {
      return -1;
    }

  adv_mode = rxbuf[2] >> 6;
  eh_len = rxbuf[2] & 0x3f;

  if ((adv_mode == 3) || (eh_len == 0))
    {
      return -1;
    }

  eh_flags = rxbuf[3];
  eh_data = &rxbuf[4];

  /* ADV_EXT_IND with AuxPtr but without ADI is invalid */

  if ((eh_flags & (1 << BLE_LL_EXT_ADV_AUX_PTR_BIT)) &&
      !(eh_flags & (1 << BLE_LL_EXT_ADV_DATA_INFO_BIT)))
    {
      return -1;
    }

  /* ADV_EXT_IND without either AdvA or AuxPtr is not valid */

  if (!(eh_flags & ((1 << BLE_LL_EXT_ADV_ADVA_BIT) |
                    (1 << BLE_LL_EXT_ADV_AUX_PTR_BIT))))
    {
      return -1;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_ADVA_BIT))
    {
      addrd->adva = eh_data;
      addrd->adva_type = !!(pdu_hdr & BLE_ADV_PDU_HDR_TXADD_RAND);
      eh_data += BLE_LL_EXT_ADV_ADVA_SIZE;
    }
  else
    {
      addrd->adva = NULL;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_TARGETA_BIT))
    {
      addrd->targeta = eh_data;
      addrd->targeta_type = !!(pdu_hdr & BLE_ADV_PDU_HDR_RXADD_RAND);
      eh_data += BLE_LL_EXT_ADV_TARGETA_SIZE;
    }
  else
    {
      addrd->targeta = NULL;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_CTE_INFO_BIT))
    {
      eh_data += BLE_LL_EXT_ADV_CTE_INFO_SIZE;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_DATA_INFO_BIT))
    {
      *adi = get_le16(eh_data);
      eh_data += BLE_LL_EXT_ADV_DATA_INFO_SIZE;
    }
  else
    {
      *adi = 0;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_AUX_PTR_BIT))
    {
      *aux_ptr = get_le24(eh_data);
    }
  else
    {
      *aux_ptr = 0;
    }

  return 0;
}

/****************************************************************************
 * Name: ble_ll_scan_aux_update_rxinfo_from_addrd
 *
 ****************************************************************************/

static void ble_ll_scan_aux_update_rxinfo_from_addrd(
    struct ble_ll_scan_addr_data *addrd, struct ble_mbuf_hdr_rxinfo *rxinfo)
{
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  rxinfo->rpa_index = addrd->rpa_index;
  if (addrd->adva_resolved)
    {
      rxinfo->flags |= BLE_MBUF_HDR_F_RESOLVED;
    }

  if (addrd->targeta_resolved)
    {
      rxinfo->flags |= BLE_MBUF_HDR_F_TARGETA_RESOLVED;
    }
#endif
}

/****************************************************************************
 * Name: ble_ll_scan_aux_update_aux_data_from_addrd
 *
 ****************************************************************************/

static void ble_ll_scan_aux_update_aux_data_from_addrd(
    struct ble_ll_scan_addr_data *addrd, struct ble_ll_scan_aux_data *aux)
{
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  aux->rpa_index = addrd->rpa_index;
  if (addrd->adva_resolved)
    {
      aux->flags |= BLE_LL_SCAN_AUX_F_RESOLVED_ADVA;
    }

  if (addrd->targeta_resolved)
    {
      aux->flags |= BLE_LL_SCAN_AUX_F_RESOLVED_TARGETA;
    }
#endif
}

/****************************************************************************
 * Name: ble_ll_scan_aux_scan_req_tx_pdu_cb
 *
 ****************************************************************************/

static uint8_t ble_ll_scan_aux_scan_req_tx_pdu_cb(uint8_t *dptr, void *arg,
                                                  uint8_t *hdr_byte)
{
  struct ble_ll_scan_aux_data *aux = arg;
  int rpa_index;

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  rpa_index = aux->rpa_index;
#else
  rpa_index = -1;
#endif

  ble_ll_scan_make_req_pdu(ble_ll_scan_sm_get(), dptr, hdr_byte,
                           aux->adva_type, aux->adva, rpa_index);

  return BLE_DEV_ADDR_LEN * 2;
}

/****************************************************************************
 * Name: ble_ll_scan_aux_send_scan_req
 *
 ****************************************************************************/

static bool ble_ll_scan_aux_send_scan_req(
    struct ble_ll_scan_aux_data *aux, struct ble_ll_scan_addr_data *addrd)
{
  int rc;

  /* Check if we already scanned this device successfully */

  if (ble_ll_scan_have_rxd_scan_rsp(addrd->adv_addr, addrd->adv_addr_type,
                                    true, aux->adi))
    {
      return false;
    }

  /* We want to send a request, see if backoff allows us */

  if (ble_ll_scan_backoff_kick() != 0)
    {
      return false;
    }

  /* TODO perhaps we should check if scan req+rsp won't overlap with
   * scheduler item (old code did it), but for now let's just scan and we
   * will be interrupted if scheduler kicks in.
   */

  rc = ble_phy_tx(ble_ll_scan_aux_scan_req_tx_pdu_cb, aux,
                  BLE_PHY_TRANSITION_TX_RX);
  if (rc)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: ble_ll_scan_aux_init_addrd_from_aux_data
 *
 ****************************************************************************/

static void ble_ll_scan_aux_init_addrd_from_aux_data(
    struct ble_ll_scan_aux_data *aux, struct ble_ll_scan_addr_data *addrd)
{
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  struct ble_ll_resolv_entry *rl;
#endif

  if (aux->flags & BLE_LL_SCAN_AUX_F_HAS_ADVA)
    {
      addrd->adva = aux->adva;
      addrd->adva_type = aux->adva_type;
    }
  else
    {
      addrd->adva = NULL;
      addrd->adva_type = 0;
    }

  if (aux->flags & BLE_LL_SCAN_AUX_F_HAS_TARGETA)
    {
      addrd->targeta = aux->targeta;
      addrd->targeta_type = aux->targeta_type;
    }
  else
    {
      addrd->targeta = NULL;
      addrd->targeta_type = 0;
    }

  addrd->adv_addr = addrd->adva;
  addrd->adv_addr_type = addrd->adva_type;

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  addrd->rpa_index = aux->rpa_index;

  if (aux->flags & BLE_LL_SCAN_AUX_F_RESOLVED_ADVA)
    {
      DEBUGASSERT(aux->rpa_index >= 0);
      rl = &g_ble_ll_resolv_list[aux->rpa_index];
      addrd->adv_addr = rl->rl_identity_addr;
      addrd->adv_addr_type = rl->rl_addr_type;
      addrd->adva_resolved = 1;
    }
  else
    {
      addrd->adva_resolved = 0;
    }

  if (aux->flags & BLE_LL_SCAN_AUX_F_RESOLVED_TARGETA)
    {
      addrd->targeta = ble_ll_scan_aux_get_own_addr();
      addrd->targeta_type = ble_ll_scan_get_own_addr_type() & 1;
      addrd->targeta_resolved = 1;
    }
  else
    {
      addrd->targeta_resolved = 0;
    }
#endif
}

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV
/****************************************************************************
 * Name: ble_ll_scan_aux_sync_check
 *
 ****************************************************************************/

static void ble_ll_scan_aux_sync_check(struct iob_s *rxpdu,
                                       struct ble_ll_scan_addr_data *addrd)
{
  struct ble_mbuf_hdr *rxhdr = BLE_MBUF_HDR_PTR(rxpdu);
  uint8_t *rxbuf = IOB_DATA(rxpdu);
  uint8_t adv_mode;
  uint8_t eh_len;
  uint8_t eh_flags;
  uint8_t *eh_data;
  uint8_t sid;

  adv_mode = rxbuf[2] >> 6;

  if (adv_mode != BLE_LL_EXT_ADV_MODE_NON_CONN)
    {
      return;
    }

  eh_len = rxbuf[2] & 0x3f;

  if (eh_len == 0)
    {
      return;
    }

  eh_flags = rxbuf[3];
  eh_data = &rxbuf[4];

  /* Need ADI and SyncInfo */

  if (!(eh_flags & (1 << BLE_LL_EXT_ADV_DATA_INFO_BIT)) ||
      !(eh_flags & (1 << BLE_LL_EXT_ADV_SYNC_INFO_BIT)))
    {
      return;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_ADVA_BIT))
    {
      eh_data += BLE_LL_EXT_ADV_ADVA_SIZE;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_TARGETA_BIT))
    {
      eh_data += BLE_LL_EXT_ADV_TARGETA_SIZE;
    }

  if (eh_flags & (1 << BLE_LL_EXT_ADV_CTE_INFO_BIT))
    {
      eh_data += 1;
    }

  sid = get_le16(eh_data) >> 12;
  eh_data += BLE_LL_EXT_ADV_DATA_INFO_SIZE;

  if (eh_flags & (1 << BLE_LL_EXT_ADV_AUX_PTR_BIT))
    {
      eh_data += BLE_LL_EXT_ADV_AUX_PTR_SIZE;
    }

  ble_ll_sync_info_event(addrd, sid, rxhdr, eh_data);
}
#endif /* CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV */

#ifdef CONFIG_BLE_LL_ROLE_CENTRAL
/****************************************************************************
 * Name: ble_ll_scan_aux_check_connect_rsp
 *
 ****************************************************************************/

static int ble_ll_scan_aux_check_connect_rsp(
    uint8_t *rxbuf, struct ble_ll_scan_pdu_data *pdu_data,
    struct ble_ll_scan_addr_data *addrd)
{
  uint8_t pdu_hdr;
  uint8_t pdu_len;
  uint8_t adv_mode;
  uint8_t eh_len;
  uint8_t eh_flags;
  uint8_t *eh_data;
  uint8_t adva_type;
  uint8_t *adva;
  uint8_t targeta_type;
  uint8_t *targeta;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  struct ble_ll_resolv_entry *rl = NULL;
#endif
  uint8_t match_adva = 1;

  pdu_hdr = rxbuf[0];
  pdu_len = rxbuf[1];

  if (pdu_len == 0)
    {
      return -1;
    }

  adv_mode = rxbuf[2] >> 6;
  eh_len = rxbuf[2] & 0x3f;

  if ((adv_mode != 0) || (eh_len == 0))
    {
      return -1;
    }

  eh_flags = rxbuf[3];
  eh_data = &rxbuf[4];

  /* AUX_CONNECT_RSP without AdvA or TargetA is not valid */

  if (!(eh_flags & ((1 << BLE_LL_EXT_ADV_ADVA_BIT) |
                    (1 << BLE_LL_EXT_ADV_TARGETA_BIT))))
    {
      return -1;
    }

  adva = eh_data;
  adva_type = !!(pdu_hdr & BLE_ADV_PDU_HDR_TXADD_RAND);
  eh_data += BLE_LL_EXT_ADV_ADVA_SIZE;

  targeta = eh_data;
  targeta_type = !!(pdu_hdr & BLE_ADV_PDU_HDR_RXADD_RAND);

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY

  /* If AdvA is an RPA and we have peer IRK, we need to check if it resolves
   * using that RPA because peer can change RPA between advertising PDU and
   * AUX_CONNECT_RSP. In other case, we expect AdvA to be the same as in
   * advertising PDU.
   */

  if ((addrd->rpa_index >= 0) && ble_ll_is_rpa(adva, adva_type))
    {
      rl = &g_ble_ll_resolv_list[addrd->rpa_index];

      if (rl->rl_has_peer)
        {
          if (!ble_ll_resolv_rpa(adva, rl->rl_peer_irk))
            {
              return -1;
            }

          addrd->adva_resolved = 1;
          addrd->adva = adva;
          addrd->adva_type = adva_type;

          match_adva = 0;
        }
    }
#endif /* BLE_LL_CFG_FEAT_LL_PRIVACY */

  if (match_adva &&
      ((adva_type != !!(pdu_data->hdr_byte & BLE_ADV_PDU_HDR_RXADD_MASK)) ||
       (memcmp(adva, pdu_data->adva, 6) != 0)))
    {
      return -1;
    }

  if ((targeta_type !=
       !!(pdu_data->hdr_byte & BLE_ADV_PDU_HDR_TXADD_MASK)) ||
      (memcmp(targeta, pdu_data->inita, 6) != 0))
    {
      return -1;
    }

  return 0;
}

/****************************************************************************
 * Name: ble_ll_scan_aux_rx_pkt_in_for_initiator
 *
 ****************************************************************************/

static void ble_ll_scan_aux_rx_pkt_in_for_initiator(
    struct iob_s *rxpdu, struct ble_mbuf_hdr *rxhdr)
{
  struct ble_ll_scan_addr_data addrd;
  struct ble_mbuf_hdr_rxinfo *rxinfo;
  struct ble_ll_scan_aux_data *aux;

  rxinfo = &rxhdr->rxinfo;
  aux = rxinfo->user_data;

  if (rxinfo->flags & BLE_MBUF_HDR_F_IGNORED)
    {
      if (aux->flags & BLE_LL_SCAN_AUX_F_W4_CONNECT_RSP)
        {
          ble_ll_conn_send_connect_req_cancel();
        }

      ble_ll_scan_aux_free(aux);
      ble_ll_scan_chk_resume();
      return;
    }

  if (!(rxinfo->flags & BLE_MBUF_HDR_F_CONNECT_RSP_RXD))
    {
      DEBUGASSERT(rxinfo->flags & BLE_MBUF_HDR_F_CONNECT_REQ_TXD);

      /* Waiting for AUX_CONNECT_RSP, do nothing */

      return;
    }

  ble_ll_scan_aux_init_addrd_from_aux_data(aux, &addrd);

  if (ble_ll_scan_aux_check_connect_rsp(
          IOB_DATA(rxpdu), ble_ll_scan_get_pdu_data(), &addrd) < 0)
    {
      ble_ll_conn_send_connect_req_cancel();
      ble_ll_scan_aux_free(aux);
      ble_ll_scan_chk_resume();
      return;
    }

  ble_ll_scan_sm_stop(0);
  ble_ll_conn_created_on_aux(rxpdu, &addrd, aux->targeta);
  ble_ll_scan_aux_free(aux);
}

#endif /* CONFIG_BLE_LL_ROLE_CENTRAL */

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(CONFIG_BLE_LL_ROLE_OBSERVER) && defined(CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV)
/****************************************************************************
 * Name: ble_ll_scan_aux_break
 *
 ****************************************************************************/

void ble_ll_scan_aux_break(struct ble_ll_scan_aux_data *aux)
{
#ifdef CONFIG_BLE_LL_ROLE_CENTRAL
  if (aux->flags & BLE_LL_SCAN_AUX_F_W4_CONNECT_RSP)
    {
      ble_ll_conn_send_connect_req_cancel();
    }
#endif

  ble_ll_event_init(&aux->break_ev, ble_ll_scan_aux_break_ev, aux);
  ble_ll_event_add(&aux->break_ev);
}

/****************************************************************************
 * Name: ble_ll_scan_aux_sched
 *
 ****************************************************************************/

int ble_ll_scan_aux_sched(struct ble_ll_scan_aux_data *aux,
                          uint32_t pdu_ticks, uint8_t pdu_rem_us,
                          uint32_t aux_ptr)
{
  struct ble_ll_sched_item *sch;
  uint32_t aux_offset;
  uint8_t offset_unit;
  uint8_t aux_phy;
  uint8_t chan;
  uint8_t ca;
  uint8_t phy;
  uint32_t offset_us;
  uint16_t pdu_rx_us;
  uint16_t aux_ww_us;
  uint16_t aux_tx_win_us;

  /* Parse AuxPtr */

  chan = aux_ptr & 0x3f;
  ca = aux_ptr & 0x40;
  offset_unit = aux_ptr & 0x80;
  aux_offset = (aux_ptr >> 8) & 0x1fff;
  aux_phy = (aux_ptr >> 21) & 0x07;

  if (chan >= BLE_PHY_NUM_DATA_CHANS)
    {
      return -1;
    }

  if (ble_ll_scan_aux_phy_to_phy(aux_phy, &phy) < 0)
    {
      return -1;
    }

  /* Actual offset */

  offset_us = aux_offset * (offset_unit ? 300 : 30);

  /* Time to scan aux PDU */

  pdu_rx_us = ble_ll_pdu_us(CONFIG_BLE_LL_SCHED_SCAN_AUX_PDU_LEN,
                            ble_ll_phy_to_phy_mode(phy, 0));

  /* Transmit window */

  aux_tx_win_us = offset_unit ? 300 : 30;

  /* Window widening to include drift due to sleep clock accuracy and
   * jitter
   */

  aux_ww_us = offset_us * (ca ? 50 : 500) / 1000000 + BLE_LL_JITTER_USECS;

  aux->sec_phy = phy;
  aux->chan = chan;
  aux->wfr_us = aux_ww_us + aux_tx_win_us;

  sch = &aux->sch;

  sch->start_time = pdu_ticks;
  sch->remainder = pdu_rem_us;
  ble_ll_tmr_add(&sch->start_time, &sch->remainder, offset_us - aux_ww_us);

  sch->end_time =
      sch->start_time + ble_ll_tmr_u2t_up(sch->remainder + 2 * aux_ww_us +
                                          aux_tx_win_us + pdu_rx_us);

  sch->start_time -= g_ble_ll_sched_offset_ticks;

  return ble_ll_sched_scan_aux(&aux->sch);
}

/****************************************************************************
 * Name: ble_ll_scan_aux_rx_isr_start
 *
 ****************************************************************************/

int ble_ll_scan_aux_rx_isr_start(uint8_t pdu_type,
                                 struct ble_mbuf_hdr *rxhdr)
{
  struct ble_ll_scan_aux_data *aux;

  DEBUGASSERT(aux_data_current);
  aux = aux_data_current;

  if (aux->flags & BLE_LL_SCAN_AUX_F_W4_CONNECT_RSP)
    {
      if (pdu_type != BLE_ADV_PDU_TYPE_AUX_CONNECT_RSP)
        {
          aux_data_current = NULL;
          ble_ll_scan_aux_break(aux);
          ble_ll_state_set(BLE_LL_STATE_STANDBY);
          return -1;
        }

      return 0;
    }

  if (pdu_type != BLE_ADV_PDU_TYPE_ADV_EXT_IND)
    {
      aux_data_current = NULL;
      ble_ll_scan_aux_break(aux);
      ble_ll_state_set(BLE_LL_STATE_STANDBY);
      return -1;
    }

  /* Prepare TX transition when doing active scanning and receiving 1st PDU
   * since we may want to send AUX_SCAN_REQ.
   */

  if ((aux->scan_type != BLE_SCAN_TYPE_PASSIVE) &&
      !(aux->flags & BLE_LL_SCAN_AUX_F_AUX_ADV))
    {
      return 1;
    }

  return 0;
}

/****************************************************************************
 * Name: ble_ll_scan_aux_rx_isr_end_on_ext
 *
 ****************************************************************************/

int ble_ll_scan_aux_rx_isr_end_on_ext(struct ble_ll_scan_sm *scansm,
                                      struct iob_s *rxpdu)
{
  struct ble_mbuf_hdr *rxhdr = BLE_MBUF_HDR_PTR(rxpdu);
  struct ble_mbuf_hdr_rxinfo *rxinfo = &rxhdr->rxinfo;
  struct ble_ll_scan_addr_data addrd;
  struct ble_ll_scan_aux_data *aux;
  uint8_t *rxbuf;
  uint32_t aux_ptr;
  uint16_t adi;
  bool do_match;
  int rc;

  rxbuf = IOB_DATA(rxpdu);

  rc = ble_ll_scan_aux_ext_parse(rxbuf, &addrd, &adi, &aux_ptr);
  if (rc < 0)
    {
      return -1;
    }

  /* We can filter based on ext PDU alone if both AdvA and TargetA are
   * present or there's no AuxPtr. Otherwise, we need to wait for aux PDU to
   * filter.
   */

  do_match = !aux_ptr || (addrd.adva && addrd.targeta);
  if (do_match)
    {
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
      addrd.rpa_index = ble_hw_resolv_list_match();
#endif
      rc =
          ble_ll_scan_rx_filter(ble_ll_scan_get_own_addr_type(),
                                ble_ll_scan_get_filt_policy(), &addrd, NULL);
      if (rc < 0)
        {
          return -1;
        }

      /* Don't care about initiator here, there are no AdvA and TargetA in
       * connectable ADV_EXT_IND so no filtering to do.
       */

      if (!aux_ptr)
        {
          /* We do not allocate aux_data for ADV_EXT_IND without AuxPtr so
           * need to pass match data in rxinfo.
           */

          ble_ll_scan_aux_update_rxinfo_from_addrd(&addrd, rxinfo);
        }
    }

  if (aux_ptr)
    {
      aux = ble_ll_scan_aux_alloc();
      if (!aux)
        {
          return -1;
        }

      aux->scan_type = scansm->scanp->scan_type;

      aux->pri_phy = rxinfo->phy;
      aux->aux_ptr = aux_ptr;

#ifdef CONFIG_BLE_LL_ADV_CODING_SELECTION
      aux->pri_phy_mode = rxinfo->phy_mode;
#endif

      if (addrd.adva)
        {
          memcpy(aux->adva, addrd.adva, 6);
          aux->adva_type = addrd.adva_type;
          aux->flags |= BLE_LL_SCAN_AUX_F_HAS_ADVA;
        }

      if (addrd.targeta)
        {
          memcpy(aux->targeta, addrd.targeta, 6);
          aux->targeta_type = addrd.targeta_type;
          aux->flags |= BLE_LL_SCAN_AUX_F_HAS_TARGETA;
        }

      aux->adi = adi;
      aux->flags |= BLE_LL_SCAN_AUX_F_HAS_ADI;

#ifdef CONFIG_BLE_LL_ROLE_CENTRAL
      if (aux->scan_type == BLE_SCAN_TYPE_INITIATE)
        {
          aux->flags |= BLE_LL_SCAN_AUX_F_CONNECTABLE;
        }
#endif

      if (do_match)
        {
          aux->flags |= BLE_LL_SCAN_AUX_F_MATCHED;
          ble_ll_scan_aux_update_aux_data_from_addrd(&addrd, aux);
        }
      else if (addrd.adva)
        {
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY

          /* If ext PDU has AdvA, we need to store rpa_index to be able to
           * reuse it for filtering when done on aux PDU.
           */

          aux->rpa_index = ble_hw_resolv_list_match();
#endif
        }

      rxinfo->user_data = aux;
    }

  return 0;
}

/****************************************************************************
 * Name: ble_ll_scan_aux_pkt_in_on_ext
 *
 ****************************************************************************/

void ble_ll_scan_aux_pkt_in_on_ext(struct iob_s *rxpdu,
                                   struct ble_mbuf_hdr *rxhdr)
{
  struct ble_mbuf_hdr_rxinfo *rxinfo = &rxhdr->rxinfo;
  struct ble_ll_scan_aux_data *aux = rxinfo->user_data;
  int rc;

  if (rxinfo->flags & BLE_MBUF_HDR_F_IGNORED)
    {
      DEBUGASSERT(!aux);
      return;
    }

  if (!aux)
    {
      ble_ll_hci_ev_send_ext_adv_report_for_ext(rxpdu, rxhdr);
      return;
    }

  DEBUGASSERT(aux->aux_ptr);

  rc = ble_ll_scan_aux_sched(aux, rxhdr->beg_cputime, rxhdr->rem_usecs,
                             aux->aux_ptr);
  if (rc < 0)
    {
      ble_ll_scan_aux_free(aux);
    }
}

/****************************************************************************
 * Name: ble_ll_scan_aux_rx_isr_end
 *
 ****************************************************************************/

int ble_ll_scan_aux_rx_isr_end(struct iob_s *rxpdu, uint8_t crcok)
{
  struct ble_ll_scan_addr_data addrd;
  struct ble_mbuf_hdr_rxinfo *rxinfo;
  struct ble_ll_scan_aux_data *aux;
  struct ble_mbuf_hdr *rxhdr;
  uint8_t scan_filt_policy;
  uint8_t scan_ok;
  uint8_t adv_mode;
  uint8_t *rxbuf;
  int rc;

  DEBUGASSERT(aux_data_current);
  aux = aux_data_current;
  aux_data_current = NULL;

  if (rxpdu == NULL)
    {
      ble_ll_scan_aux_break(aux);
      goto done;
    }

  rxhdr = BLE_MBUF_HDR_PTR(rxpdu);
  rxinfo = &rxhdr->rxinfo;
  rxinfo->user_data = aux;

  /* It's possible that we received aux while scan was just being disabled in
   * LL task. In such case simply ignore aux.
   */

  if (!crcok || !ble_ll_scan_enabled())
    {
      rxinfo->flags |= BLE_MBUF_HDR_F_IGNORED;
      goto done;
    }

  rxbuf = IOB_DATA(rxpdu);

  if (aux->flags & BLE_LL_SCAN_AUX_F_W4_CONNECT_RSP)
    {
      aux->flags &= ~BLE_LL_SCAN_AUX_F_W4_CONNECT_RSP;
      rxinfo->flags |= BLE_MBUF_HDR_F_CONNECT_RSP_RXD;
      goto done;
    }

  if (aux->flags & BLE_LL_SCAN_AUX_F_W4_SCAN_RSP)
    {
      aux->flags &= ~BLE_LL_SCAN_AUX_F_W4_SCAN_RSP;
      aux->flags |= BLE_LL_SCAN_AUX_F_SCANNED;
      rxinfo->flags |= BLE_MBUF_HDR_F_SCAN_RSP_RXD;
    }

  rc = ble_ll_scan_aux_parse_to_aux_data(aux, rxbuf, rxhdr);
  if (rc < 0)
    {
      rxinfo->flags |= BLE_MBUF_HDR_F_IGNORED;
      goto done;
    }

  if (aux->flags & BLE_LL_SCAN_AUX_F_MATCHED)
    {
      goto done;
    }

  /* We do filtering on either ADV_EXT_IND or AUX_ADV_IND so we should not be
   * here when processing AUX_CHAIN_IND.
   */

  DEBUGASSERT(!(aux->flags & BLE_LL_SCAN_AUX_F_AUX_CHAIN));

  if (aux->flags & BLE_LL_SCAN_AUX_F_HAS_ADVA)
    {
      addrd.adva = aux->adva;
      addrd.adva_type = aux->adva_type;
    }
  else
    {
      addrd.adva = NULL;
    }

  if (aux->flags & BLE_LL_SCAN_AUX_F_HAS_TARGETA)
    {
      addrd.targeta = aux->targeta;
      addrd.targeta_type = aux->targeta_type;
    }
  else
    {
      addrd.targeta = NULL;
    }

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  addrd.rpa_index = aux->rpa_index;
#endif

  scan_filt_policy = ble_ll_scan_get_filt_policy();

  rc = ble_ll_scan_rx_filter(ble_ll_scan_get_own_addr_type(),
                             scan_filt_policy, &addrd, &scan_ok);
  if (rc < 0)
    {
      rxinfo->flags |= BLE_MBUF_HDR_F_IGNORED;

      /* XXX hack warning Even if PDU was not allowed by current scan filter
       * policy, we should still allow it to sync if SyncInfo is present.
       * Since we do not use F_DEVMATCH in aux code for its intended purpose,
       * let's use it here to indicate no match due to scan filter policy.
       */

      if (rc == -2)
        {
          rxinfo->flags |= BLE_MBUF_HDR_F_DEVMATCH;
        }

      goto done;
    }

#ifdef CONFIG_BLE_LL_ROLE_CENTRAL
  if ((aux->scan_type == BLE_SCAN_TYPE_INITIATE) &&
      !(scan_filt_policy & 0x01))
    {
      rc = ble_ll_scan_rx_check_init(&addrd);
      if (rc < 0)
        {
          rxinfo->flags |= BLE_MBUF_HDR_F_IGNORED;
          goto done;
        }
    }
#endif

  aux->flags |= BLE_LL_SCAN_AUX_F_MATCHED;

  ble_ll_scan_aux_update_aux_data_from_addrd(&addrd, aux);

  adv_mode = rxbuf[2] >> 6;

  switch (aux->scan_type)
    {
      case BLE_SCAN_TYPE_ACTIVE:
        if ((adv_mode == BLE_LL_EXT_ADV_MODE_SCAN) && scan_ok &&
            ble_ll_scan_aux_send_scan_req(aux, &addrd))
          {
            /* AUX_SCAN_REQ sent, keep PHY enabled to continue */

            aux->flags |= BLE_LL_SCAN_AUX_F_W4_SCAN_RSP;
            rxinfo->flags |= BLE_MBUF_HDR_F_SCAN_REQ_TXD;
            aux_data_current = aux;
            return 0;
          }

        break;
#ifdef CONFIG_BLE_LL_ROLE_CENTRAL
      case BLE_SCAN_TYPE_INITIATE:
        if (ble_ll_conn_send_connect_req(rxpdu, &addrd, 1) == 0)
          {
            /* AUX_CONNECT_REQ sent, keep PHY enabled to continue */

            aux->flags |= BLE_LL_SCAN_AUX_F_W4_CONNECT_RSP;
            rxinfo->flags |= BLE_MBUF_HDR_F_CONNECT_REQ_TXD;
            aux_data_current = aux;
            return 0;
          }
        else
          {
            rxinfo->flags |= BLE_MBUF_HDR_F_IGNORED;
          }

        break;
#endif
      default:
        break;
    }

done:

  /* We are done with this PDU so go to standby and let LL resume if
   * needed
   */

  ble_ll_state_set(BLE_LL_STATE_STANDBY);
  return -1;
}

/****************************************************************************
 * Name: ble_ll_scan_aux_rx_pkt_in
 *
 ****************************************************************************/

void ble_ll_scan_aux_rx_pkt_in(struct iob_s *rxpdu,
                               struct ble_mbuf_hdr *rxhdr)
{
  struct ble_ll_scan_addr_data addrd;
  struct ble_mbuf_hdr_rxinfo *rxinfo;
  struct ble_ll_scan_aux_data *aux;
  bool scan_duplicate = false;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV
  bool sync_check = false;
#endif
  int rc;

  rxinfo = &rxhdr->rxinfo;
  aux = rxinfo->user_data;

  DEBUGASSERT(aux);

#ifdef CONFIG_BLE_LL_ROLE_CENTRAL
  if (aux->scan_type == BLE_SCAN_TYPE_INITIATE)
    {
      ble_ll_scan_aux_rx_pkt_in_for_initiator(rxpdu, rxhdr);
      return;
    }
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV
  sync_check =
      ble_ll_sync_enabled() && !(aux->flags & BLE_LL_SCAN_AUX_F_AUX_CHAIN);

  /* PDU was not allowed due to scan filter policy, but we can still try to
   * sync since separate filter policy is used for this purpose.
   */

  if ((rxinfo->flags & BLE_MBUF_HDR_F_DEVMATCH) && sync_check)
    {
      ble_ll_scan_aux_init_addrd_from_aux_data(aux, &addrd);
      ble_ll_scan_aux_sync_check(rxpdu, &addrd);
    }
#endif

  if (rxinfo->flags & BLE_MBUF_HDR_F_IGNORED)
    {
      if (ble_ll_scan_aux_need_truncation(aux))
        {
          ble_ll_hci_ev_send_ext_adv_truncated_report(aux);
        }
      else
        {
          aux->hci_state |= BLE_LL_SCAN_AUX_H_DONE;
        }

      /* Update backoff if we were waiting for scan response */

      if (aux->flags & BLE_LL_SCAN_AUX_F_W4_SCAN_RSP)
        {
          ble_ll_scan_backoff_update(0);
        }
    }
  else if (rxinfo->flags & BLE_MBUF_HDR_F_SCAN_RSP_RXD)
    {
      /* We assume scan success when AUX_SCAN_RSP is received, no need to
       * wait for complete chain (Core 5.3, Vol 6, Part B, 4.4.3.1).
       */

      ble_ll_scan_backoff_update(1);
    }

  if (aux->hci_state & BLE_LL_SCAN_AUX_H_DONE)
    {
      ble_ll_scan_aux_free(aux);
      ble_ll_scan_chk_resume();
      return;
    }

  /* Try to schedule scan for subsequent aux asap, if needed */

  if (rxinfo->flags & BLE_MBUF_HDR_F_AUX_PTR_WAIT)
    {
      rc = ble_ll_scan_aux_sched(aux, rxhdr->beg_cputime, rxhdr->rem_usecs,
                                 aux->aux_ptr);
      if (rc < 0)
        {
          rxinfo->flags &= ~BLE_MBUF_HDR_F_AUX_PTR_WAIT;
          rxinfo->flags |= BLE_MBUF_HDR_F_AUX_PTR_FAILED;
        }
    }

  ble_ll_scan_aux_init_addrd_from_aux_data(aux, &addrd);

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV
  if (sync_check)
    {
      ble_ll_scan_aux_sync_check(rxpdu, &addrd);
    }
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  if (addrd.adva_resolved)
    {
      DEBUGASSERT(addrd.rpa_index >= 0);
      ble_ll_resolv_set_peer_rpa(addrd.rpa_index, addrd.adva);
    }
#endif

  scan_duplicate = ble_ll_scan_get_filt_dups() &&
                   ble_ll_scan_dup_check_ext(addrd.adv_addr_type,
                                             addrd.adv_addr, true, aux->adi);
  if (!scan_duplicate)
    {
      rc = ble_ll_hci_ev_send_ext_adv_report_for_aux(rxpdu, rxhdr, aux,
                                                     &addrd);

      /* Update duplicates list if report was sent successfully and we are
       * done with this chain. On error, status is already set to 'done' so
       * we will cancel aux scan (if any) and stop further processing.
       * However, if we send AUX_SCAN_REQ for this PDU then need to remove
       * 'done' as we should continue with scanning after AUX_SCAN_RSP.
       */

      if (rc == 0)
        {
          if (rxinfo->flags & BLE_MBUF_HDR_F_SCAN_REQ_TXD)
            {
              aux->hci_state &= ~BLE_LL_SCAN_AUX_H_DONE;
            }
          else if (aux->hci_state & BLE_LL_SCAN_AUX_H_DONE)
            {
              DEBUGASSERT(!(rxinfo->flags & BLE_MBUF_HDR_F_AUX_PTR_WAIT));
              if (ble_ll_scan_get_filt_dups())
                {
                  ble_ll_scan_dup_update_ext(addrd.adv_addr_type,
                                             addrd.adv_addr, true, aux->adi);
                }

              if (aux->flags & BLE_LL_SCAN_AUX_F_SCANNED)
                {
                  ble_ll_scan_add_scan_rsp_adv(
                      addrd.adv_addr, addrd.adv_addr_type, 1, aux->adi);
                }
            }
        }
    }
  else
    {
      /* This is a duplicate, we don't want to scan it anymore */

      aux->hci_state |= BLE_LL_SCAN_AUX_H_DONE;
    }

  /* If we are done processing this chain we can remove aux_data now if:
   * - we did not send AUX_SCAN_REQ for this PDU
   * - there was no aux scan scheduled from this PDU
   * - there was aux scan scheduled from this PDU but we removed it
   * In other cases, we'll remove aux_data on next pkt_in.
   */

  if ((aux->hci_state & BLE_LL_SCAN_AUX_H_DONE) &&
      !(rxinfo->flags & BLE_MBUF_HDR_F_SCAN_REQ_TXD) &&
      (!(rxinfo->flags & BLE_MBUF_HDR_F_AUX_PTR_WAIT) ||
       (ble_ll_sched_rmv_elem(&aux->sch) == 0)))
    {
      ble_ll_scan_aux_free(aux);
    }

  ble_ll_scan_chk_resume();
}

/****************************************************************************
 * Name: ble_ll_scan_aux_wfr_timer_exp
 *
 ****************************************************************************/

void ble_ll_scan_aux_wfr_timer_exp(void)
{
  ble_phy_disable();
  ble_ll_scan_aux_halt();
}

/****************************************************************************
 * Name: ble_ll_scan_aux_halt
 *
 ****************************************************************************/

void ble_ll_scan_aux_halt(void)
{
  struct ble_ll_scan_aux_data *aux = aux_data_current;

  DEBUGASSERT(aux);

  aux_data_current = NULL;

  ble_ll_state_set(BLE_LL_STATE_STANDBY);

  ble_ll_scan_aux_break(aux);
}

/****************************************************************************
 * Name: ble_ll_scan_aux_sched_remove
 *
 ****************************************************************************/

void ble_ll_scan_aux_sched_remove(struct ble_ll_sched_item *sch)
{
  ble_ll_scan_aux_break(sch->cb_arg);
}

/****************************************************************************
 * Name: ble_ll_scan_aux_init
 *
 ****************************************************************************/

void ble_ll_scan_aux_init(void)
{
  int i;

  g_aux_data_free = NULL;
  for (i = 0; i < CONFIG_BLE_LL_SCAN_AUX_SEGMENT_CNT; i++)
    {
      ble_ll_event_remove(&g_aux_data[i].break_ev);
      g_aux_data[i].next = g_aux_data_free;
      g_aux_data_free = &g_aux_data[i];
    }
}

#endif
