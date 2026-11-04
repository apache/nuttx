/****************************************************************************
 * wireless/ble_ll/ble_ll_sync.c
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
 * Derived from Apache Mynewt NimBLE (nimble/controller/src/ble_ll_sync.c).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "ble_ll.h"
#include "ble_ll_pdu.h"
#include "ble_ll_hci.h"
#include "ble_ll_sync.h"
#include "ble_ll_utils.h"
#include "ble_ll_sched.h"
#include "ble_ll_whitelist.h"
#include "ble_ll_scan.h"
#include "ble_ll_resolv.h"
#include "ble_ll_rfmgmt.h"
#include "ble_ll_tmr.h"
#include "ble.h"
#include "hci_common.h"
#include "ble_ll_conn_priv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV) && defined(CONFIG_BLE_LL_ROLE_OBSERVER)

/* defines number of events that can be lost during sync establishment before
 * failed to be established error is reported
 */

#define BLE_LL_SYNC_ESTABLISH_CNT 6
#define BLE_LL_SYNC_CNT CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV_SYNC_CNT
#define BLE_LL_SYNC_LIST_CNT \
  CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV_SYNC_LIST_CNT

#define BLE_LL_SYNC_SM_FLAG_RESERVED      0x0001
#define BLE_LL_SYNC_SM_FLAG_ESTABLISHING  0x0002
#define BLE_LL_SYNC_SM_FLAG_ESTABLISHED   0x0004
#define BLE_LL_SYNC_SM_FLAG_SET_ANCHOR    0x0008
#define BLE_LL_SYNC_SM_FLAG_OFFSET_300    0x0010
#define BLE_LL_SYNC_SM_FLAG_SYNC_INFO     0x0020
#define BLE_LL_SYNC_SM_FLAG_DISABLED      0x0040
#define BLE_LL_SYNC_SM_FLAG_ADDR_RESOLVED 0x0080
#define BLE_LL_SYNC_SM_FLAG_HCI_TRUNCATED 0x0100
#define BLE_LL_SYNC_SM_FLAG_NEW_CHANMAP   0x0200
#define BLE_LL_SYNC_SM_FLAG_CHAIN         0x0400
#define BLE_LL_SYNC_SM_FLAG_DUPLICATES    0x0800
#define BLE_LL_SYNC_ITVL_USECS 1250
#define BLE_LL_SYNC_BIGINFO_LEN     33
#define BLE_LL_SYNC_BIGINFO_LEN_ENC 57

#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

#if defined(CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV) && defined(CONFIG_BLE_LL_ROLE_OBSERVER)
struct ble_ll_sync_sm
{
  uint16_t flags;

  uint8_t adv_sid;
  uint8_t adv_addr[BLE_DEV_ADDR_LEN];
  uint8_t adv_addr_type;

  uint8_t sca;
  uint8_t chan_map[BLE_LL_CHAN_MAP_LEN];
  uint8_t chan_map_used;

  uint8_t chan_map_new[BLE_LL_CHAN_MAP_LEN];
  uint16_t chan_map_new_instant;

  uint8_t chan_index;
  uint8_t chan_chain;

  uint8_t phy_mode;

  uint8_t sync_pending_cnt;

  uint32_t timeout;
  uint16_t skip;

  uint16_t itvl;
  uint8_t itvl_usecs;
  uint32_t itvl_ticks;

  uint32_t crcinit; /* only 3 bytes are used */
  uint32_t access_addr;
  uint16_t event_cntr;
  uint16_t channel_id;

  uint32_t window_widening;
  uint32_t last_anchor_point;
  uint32_t anchor_point;
  uint8_t anchor_point_usecs;

  struct ble_ll_sched_item sch;

  struct ble_ll_event_s sync_ev_end;

  uint8_t *next_report;

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV_SYNC_TRANSFER
  struct ble_ll_conn_sm *transfer_conn;
  uint8_t *transfer_received_ev;
  uint16_t transfer_id;
  uint16_t event_cntr_last_received;
  uint8_t adv_addr_rpa[6];
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV_ADI_SUPPORT
  uint16_t prev_adi;
#endif
};

struct ble_ll_sync_adv_s
{
  uint8_t adv_sid;
  uint8_t adv_addr[BLE_DEV_ADDR_LEN];
  uint8_t adv_addr_type;
};

struct ble_ll_sync_create_params_s
{
  uint32_t timeout;
  uint16_t max_skip;
  uint16_t options;
};

#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if defined(CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV) && defined(CONFIG_BLE_LL_ROLE_OBSERVER)
static int ble_ll_sync_event_start_cb(struct ble_ll_sched_item *sch);

#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined(CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV) && defined(CONFIG_BLE_LL_ROLE_OBSERVER)
static struct ble_ll_sync_sm g_ble_ll_sync_sm[BLE_LL_SYNC_CNT];

static struct ble_ll_sync_adv_s g_ble_ll_sync_adv_list[BLE_LL_SYNC_LIST_CNT];

static struct ble_ll_sync_create_params_s g_ble_ll_sync_create_params;

/* if this is set HCI LE Sync Create is pending */

static uint8_t *g_ble_ll_sync_create_comp_ev;

static struct ble_ll_sync_sm *g_ble_ll_sync_sm_current;

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV) && defined(CONFIG_BLE_LL_ROLE_OBSERVER)
/****************************************************************************
 * Name: ble_ll_sync_on_list
 *
 ****************************************************************************/

static int ble_ll_sync_on_list(const uint8_t *addr, uint8_t addr_type,
                               uint8_t sid)
{
  unsigned int i;

  for (i = 0; i < nitems(g_ble_ll_sync_adv_list); i++)
    {
      if ((g_ble_ll_sync_adv_list[i].adv_sid == sid) &&
          (g_ble_ll_sync_adv_list[i].adv_addr_type == addr_type) &&
          !memcmp(g_ble_ll_sync_adv_list[i].adv_addr, addr,
                  BLE_DEV_ADDR_LEN))
        {
          return i;
        }
    }

  return -1;
}

/****************************************************************************
 * Name: ble_ll_sync_list_get_free
 *
 ****************************************************************************/

static int ble_ll_sync_list_get_free(void)
{
  unsigned int i;

  for (i = 0; i < nitems(g_ble_ll_sync_adv_list); i++)
    {
      if (g_ble_ll_sync_adv_list[i].adv_sid == 0xff)
        {
          return i;
        }
    }

  return -1;
}

/****************************************************************************
 * Name: ble_ll_sync_list_empty
 *
 ****************************************************************************/

static bool ble_ll_sync_list_empty(void)
{
  unsigned int i;

  for (i = 0; i < nitems(g_ble_ll_sync_adv_list); i++)
    {
      if (g_ble_ll_sync_adv_list[i].adv_sid != 0xff)
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Name: ble_ll_sync_get_handle
 *
 ****************************************************************************/

static uint8_t ble_ll_sync_get_handle(struct ble_ll_sync_sm *sm)
{
  /* handle number is offset in global array */

  return sm - g_ble_ll_sync_sm;
}

/****************************************************************************
 * Name: ble_ll_sync_sm_clear
 *
 ****************************************************************************/

static void ble_ll_sync_sm_clear(struct ble_ll_sync_sm *sm)
{
  if (sm->flags &
      (BLE_LL_SYNC_SM_FLAG_ESTABLISHING | BLE_LL_SYNC_SM_FLAG_ESTABLISHED))
    {
      ble_ll_sched_rmv_elem(&sm->sch);
      ble_ll_event_remove(&sm->sync_ev_end);
    }

  if (sm->next_report)
    {
      ble_ll_hci_buf_free(sm->next_report);
    }

  if (g_ble_ll_sync_sm_current == sm)
    {
      ble_phy_disable();
      ble_ll_state_set(BLE_LL_STATE_STANDBY);
      g_ble_ll_sync_sm_current = NULL;
      ble_ll_scan_chk_resume();
    }

  ble_ll_rfmgmt_release();
  DEBUGASSERT(ble_ll_event_is_queued(&sm->sync_ev_end) == 0);
  DEBUGASSERT(sm->sch.enqueued == 0);

  memset(sm, 0, sizeof(*sm));

  sm->sch.sched_cb = ble_ll_sync_event_start_cb;
  sm->sch.cb_arg = sm;
  sm->sch.sched_type = BLE_LL_SCHED_TYPE_SYNC;
}

/****************************************************************************
 * Name: ble_ll_sync_phy_mode_to_hci
 *
 ****************************************************************************/

static uint8_t ble_ll_sync_phy_mode_to_hci(int8_t phy_mode)
{
#ifdef CONFIG_BLE_LL_PHY
  switch (phy_mode)
    {
      case BLE_PHY_MODE_1M:
        return BLE_HCI_LE_PHY_1M;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_2M_PHY
      case BLE_PHY_MODE_2M:
        return BLE_HCI_LE_PHY_2M;
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_CODED_PHY
      case BLE_PHY_MODE_CODED_125KBPS:
      case BLE_PHY_MODE_CODED_500KBPS:
        return BLE_HCI_LE_PHY_CODED;
#endif
      default:
        DEBUGASSERT(false);
        return BLE_PHY_MODE_1M;
    }
#else
  return BLE_PHY_MODE_1M;
#endif
}

/****************************************************************************
 * Name: ble_ll_sync_find
 *
 ****************************************************************************/

static struct ble_ll_sync_sm *ble_ll_sync_find(const uint8_t *addr,
                                               uint8_t addr_type,
                                               uint8_t sid)
{
  struct ble_ll_sync_sm *sm;
  int i;

  for (i = 0; i < BLE_LL_SYNC_CNT; i++)
    {
      sm = &g_ble_ll_sync_sm[i];

      if (!sm->flags)
        {
          continue;
        }

      if ((sm->adv_sid == sid) && (sm->adv_addr_type == addr_type) &&
          !memcmp(&sm->adv_addr, addr, BLE_DEV_ADDR_LEN))
        {
          return sm;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: get_max_skip
 *
 ****************************************************************************/

static uint16_t get_max_skip(uint32_t interval_us, uint32_t timeout_us)
{
  uint16_t max_skip;

  DEBUGASSERT(interval_us);
  DEBUGASSERT(timeout_us);

  if (timeout_us <= interval_us)
    {
      return 0;
    }

  /* Calculate max allowed skip to receive something before timeout. We
   * adjust current skip value to be no more than max_skip-6 so we have at
   * least few attempts to receive an event (so we don't timeout immediately
   * after just one missed event).
   */

  max_skip = (timeout_us / interval_us) - 1;

  if (max_skip < 6)
    {
      return 0;
    }

  return max_skip - 6;
}

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV_SYNC_TRANSFER
/****************************************************************************
 * Name: ble_ll_sync_transfer_received
 *
 ****************************************************************************/

static void ble_ll_sync_transfer_received(struct ble_ll_sync_sm *sm,
                                          uint8_t status)
{
  struct ble_hci_ev_le_subev_periodic_adv_sync_transfer *ev;
  struct ble_hci_ev *hci_ev;

  DEBUGASSERT(sm->transfer_received_ev);

  if (ble_ll_hci_is_le_event_enabled(
          BLE_HCI_LE_SUBEV_PERIODIC_ADV_SYNC_TRANSFER))
    {
      hci_ev = (void *)sm->transfer_received_ev;

      hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
      hci_ev->length = sizeof(*ev);

      ev = (void *)hci_ev->data;
      ev->subev_code = BLE_HCI_LE_SUBEV_PERIODIC_ADV_SYNC_TRANSFER;

      ev->status = status;
      ev->conn_handle = htole16(sm->transfer_conn->conn_handle);
      ev->service_data = htole16(sm->transfer_id);

      /* this is ignored by host on error */

      ev->sync_handle = htole16(ble_ll_sync_get_handle(sm));
      ev->sid = sm->adv_sid;
      ev->peer_addr_type = sm->adv_addr_type;
      if (sm->flags & BLE_LL_SYNC_SM_FLAG_ADDR_RESOLVED)
        {
          ev->peer_addr_type += 2;
        }

      memcpy(ev->peer_addr, sm->adv_addr, BLE_DEV_ADDR_LEN);
      ev->phy = ble_ll_sync_phy_mode_to_hci(sm->phy_mode);
      ev->interval = htole16(sm->itvl);
      ev->aca = sm->sca;

      ble_ll_hci_event_send(hci_ev);
    }
  else
    {
      ble_ll_hci_buf_free(sm->transfer_received_ev);
    }

  sm->transfer_received_ev = NULL;
  sm->transfer_conn = NULL;
}
#endif

/****************************************************************************
 * Name: ble_ll_sync_est_event_success
 *
 ****************************************************************************/

static void ble_ll_sync_est_event_success(struct ble_ll_sync_sm *sm)
{
  struct ble_hci_ev_le_subev_periodic_adv_sync_estab *ev;
  struct ble_hci_ev *hci_ev;

  DEBUGASSERT(g_ble_ll_sync_create_comp_ev);

  if (ble_ll_hci_is_le_event_enabled(
          BLE_HCI_LE_SUBEV_PERIODIC_ADV_SYNC_ESTAB))
    {
      hci_ev = (void *)g_ble_ll_sync_create_comp_ev;

      hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
      hci_ev->length = sizeof(*ev);
      ev = (void *)hci_ev->data;

      ev->subev_code = BLE_HCI_LE_SUBEV_PERIODIC_ADV_SYNC_ESTAB;
      ev->status = BLE_ERR_SUCCESS;
      ev->sync_handle = htole16(ble_ll_sync_get_handle(sm));
      ev->sid = sm->adv_sid;
      ev->peer_addr_type = sm->adv_addr_type;
      if (sm->flags & BLE_LL_SYNC_SM_FLAG_ADDR_RESOLVED)
        {
          ev->peer_addr_type += 2;
        }

      memcpy(ev->peer_addr, sm->adv_addr, BLE_DEV_ADDR_LEN);
      ev->phy = ble_ll_sync_phy_mode_to_hci(sm->phy_mode);
      ev->interval = htole16(sm->itvl);
      ev->aca = sm->sca;

      ble_ll_hci_event_send(hci_ev);
    }
  else
    {
      ble_ll_hci_buf_free(g_ble_ll_sync_create_comp_ev);
    }

  g_ble_ll_sync_create_comp_ev = NULL;
}

/****************************************************************************
 * Name: ble_ll_sync_est_event_failed
 *
 ****************************************************************************/

static void ble_ll_sync_est_event_failed(uint8_t status)
{
  struct ble_hci_ev_le_subev_periodic_adv_sync_estab *ev;
  struct ble_hci_ev *hci_ev;

  DEBUGASSERT(g_ble_ll_sync_create_comp_ev);

  if (ble_ll_hci_is_le_event_enabled(
          BLE_HCI_LE_SUBEV_PERIODIC_ADV_SYNC_ESTAB))
    {
      hci_ev = (void *)g_ble_ll_sync_create_comp_ev;

      hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
      hci_ev->length = sizeof(*ev);
      ev = (void *)hci_ev->data;

      memset(ev, 0, sizeof(*ev));

      ev->subev_code = BLE_HCI_LE_SUBEV_PERIODIC_ADV_SYNC_ESTAB;
      ev->status = status;

      ble_ll_hci_event_send(hci_ev);
    }
  else
    {
      ble_ll_hci_buf_free(g_ble_ll_sync_create_comp_ev);
    }

  g_ble_ll_sync_create_comp_ev = NULL;
}

/****************************************************************************
 * Name: ble_ll_sync_lost_event
 *
 ****************************************************************************/

static void ble_ll_sync_lost_event(struct ble_ll_sync_sm *sm)
{
  struct ble_hci_ev_le_subev_periodic_adv_sync_lost *ev;
  struct ble_hci_ev *hci_ev;

  if (ble_ll_hci_is_le_event_enabled(
          BLE_HCI_LE_SUBEV_PERIODIC_ADV_SYNC_LOST))
    {
      hci_ev = ble_ll_hci_evt_alloc(0);
      if (hci_ev)
        {
          hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
          hci_ev->length = sizeof(*ev);
          ev = (void *)hci_ev->data;

          ev->subev_code = BLE_HCI_LE_SUBEV_PERIODIC_ADV_SYNC_LOST;
          ev->sync_handle = htole16(ble_ll_sync_get_handle(sm));

          ble_ll_hci_event_send(hci_ev);
        }
    }
}

/****************************************************************************
 * Name: ble_ll_sync_current_sm_over
 *
 ****************************************************************************/

static void ble_ll_sync_current_sm_over(void)
{
  /* Disable the PHY */

  ble_phy_disable();

  /* Link-layer is in standby state now */

  ble_ll_state_set(BLE_LL_STATE_STANDBY);

  /* Set current LL sync to NULL */

  g_ble_ll_sync_sm_current = NULL;
}

/****************************************************************************
 * Name: ble_ll_sync_event_start_cb
 *
 ****************************************************************************/

static int ble_ll_sync_event_start_cb(struct ble_ll_sched_item *sch)
{
  struct ble_ll_sync_sm *sm;
  uint32_t wfr_usecs;
  uint32_t start;
  uint8_t chan;
  int rc;

  /* Set current connection state machine */

  sm = sch->cb_arg;
  DEBUGASSERT(sm);

  g_ble_ll_sync_sm_current = sm;

  /* Disable whitelisting */

  ble_ll_whitelist_disable();

  /* Set LL state */

  ble_ll_state_set(BLE_LL_STATE_SYNC);

  /* Set channel */

  chan = sm->flags & BLE_LL_SYNC_SM_FLAG_CHAIN ? sm->chan_chain :
                                                 sm->chan_index;
  ble_phy_setchan(chan, sm->access_addr, sm->crcinit);

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  ble_phy_resolv_list_disable();
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
  ble_phy_encrypt_disable();
#endif

#ifdef CONFIG_BLE_LL_PHY
  ble_phy_mode_set(sm->phy_mode, sm->phy_mode);
#endif

  start = sch->start_time + g_ble_ll_sched_offset_ticks;
  rc = ble_phy_rx_set_start_time(start, sch->remainder);
  if (rc && rc != BLE_PHY_ERR_RX_LATE)
    {
      BLE_LL_STATS_INC(ble_ll_stats, sync_event_failed);
      ble_ll_event_add(&sm->sync_ev_end);
      ble_ll_sync_current_sm_over();
      rc = BLE_LL_SCHED_STATE_DONE;
    }
  else
    {
      if (sm->flags & BLE_LL_SYNC_SM_FLAG_CHAIN)
        {
          wfr_usecs =
              (sm->flags & BLE_LL_SYNC_SM_FLAG_OFFSET_300) ? 300 : 30;
        }
      else
        {
          /* Set flag that tells to set last anchor point if a packet has
           * been received.
           */

          sm->flags |= BLE_LL_SYNC_SM_FLAG_SET_ANCHOR;

          /* Set WFR timer. If establishing we always adjust with offset
           * unit. If this is first packet of sync (one that was pointed by
           * from SyncInfo we don't adjust WFR with window widening.
           */

          if (sm->flags & BLE_LL_SYNC_SM_FLAG_ESTABLISHING)
            {
              wfr_usecs =
                  (sm->flags & BLE_LL_SYNC_SM_FLAG_OFFSET_300) ? 300 : 30;
              if (!(sm->flags & BLE_LL_SYNC_SM_FLAG_SYNC_INFO))
                {
                  wfr_usecs += 2 * sm->window_widening;
                }
            }
          else
            {
              wfr_usecs = 2 * sm->window_widening;
            }
        }

      ble_phy_wfr_enable(BLE_PHY_WFR_ENABLE_RX, 0, wfr_usecs);
      rc = BLE_LL_SCHED_STATE_RUNNING;
    }

  sm->flags &= ~BLE_LL_SYNC_SM_FLAG_SYNC_INFO;

  return rc;
}

/****************************************************************************
 * Name: ble_ll_sync_parse_ext_hdr
 *
 ****************************************************************************/

static int ble_ll_sync_parse_ext_hdr(struct iob_s *om, uint8_t **aux,
                                     int8_t *tx_power, uint8_t **acad,
                                     uint8_t *acad_len, uint16_t **adi)
{
  uint8_t *rxbuf = IOB_DATA(om);
  uint8_t ext_hdr_flags;
  uint8_t ext_hdr_len;
  uint8_t *ext_hdr;
  uint8_t pdu_len;
  int i;

  pdu_len = rxbuf[1];
  if (pdu_len == 0)
    {
      return -1;
    }

  ext_hdr_len = rxbuf[2] & 0x3f;
  if (ext_hdr_len > (pdu_len - 1))
    {
      return -1;
    }

  if (ext_hdr_len)
    {
      ext_hdr_flags = rxbuf[3];
      ext_hdr = &rxbuf[4];

      i = 0;

      /* there should be no AdvA in Sync or chain, skip it */

      if (ext_hdr_flags & (1 << BLE_LL_EXT_ADV_ADVA_BIT))
        {
          i += BLE_LL_EXT_ADV_ADVA_SIZE;
        }

      /* there should be no TargetA in Sync or chain, skip it */

      if (ext_hdr_flags & (1 << BLE_LL_EXT_ADV_TARGETA_BIT))
        {
          i += BLE_LL_EXT_ADV_TARGETA_SIZE;
        }

      /* Ignore CTE for now */

      if (ext_hdr_flags & (1 << BLE_LL_EXT_ADV_CTE_INFO_BIT))
        {
          i += BLE_LL_EXT_ADV_CTE_INFO_SIZE;
        }

      if (ext_hdr_flags & (1 << BLE_LL_EXT_ADV_DATA_INFO_BIT))
        {
          if (BLE_LL_PERIODIC_ADV_ADI_SUPPORT)
            {
              *adi = (uint16_t *)(ext_hdr + i);
            }

          i += BLE_LL_EXT_ADV_DATA_INFO_SIZE;
        }

      /* get AuXPTR if present */

      if (ext_hdr_flags & (1 << BLE_LL_EXT_ADV_AUX_PTR_BIT))
        {
          *aux = ext_hdr + i;
          i += BLE_LL_EXT_ADV_AUX_PTR_SIZE;
        }

      /* there should be no SyncInfo in Sync or chain, skip it */

      if (ext_hdr_flags & (1 << BLE_LL_EXT_ADV_SYNC_INFO_BIT))
        {
          i += BLE_LL_EXT_ADV_SYNC_INFO_SIZE;
        }

      if (ext_hdr_flags & (1 << BLE_LL_EXT_ADV_TX_POWER_BIT))
        {
          *tx_power = *(ext_hdr + i);
          i += BLE_LL_EXT_ADV_TX_POWER_SIZE;
        }

      /* sanity check */

      if (i > ext_hdr_len)
        {
          return -1;
        }

      /* ACAD */

      if (ext_hdr_len > (i + 1))
        {
          *acad = ext_hdr + i;
          *acad_len = ext_hdr_len - i - 1;
        }
    }

  return pdu_len - ext_hdr_len - 1;
}

/****************************************************************************
 * Name: ble_ll_sync_adjust_ext_hdr
 *
 ****************************************************************************/

static void ble_ll_sync_adjust_ext_hdr(struct iob_s *om)
{
  uint8_t *rxbuf = IOB_DATA(om);
  uint8_t ext_hdr_len;

  /* this was already verified in ble_ll_sync_parse_ext_hdr() */

  ext_hdr_len = rxbuf[2] & 0x3f;

  ble_ll_iob_adj(om, 3 + ext_hdr_len);
}

/****************************************************************************
 * Name: ble_ll_sync_send_truncated_per_adv_rpt
 *
 ****************************************************************************/

static void ble_ll_sync_send_truncated_per_adv_rpt(struct ble_ll_sync_sm *sm,
                                                   uint8_t *evbuf)
{
  struct ble_hci_ev_le_subev_periodic_adv_rpt *ev;
  struct ble_hci_ev *hci_ev;

  if (!ble_ll_hci_is_le_event_enabled(BLE_HCI_LE_SUBEV_PERIODIC_ADV_RPT) ||
      (sm->flags & BLE_LL_SYNC_SM_FLAG_DISABLED))
    {
      ble_ll_hci_buf_free(evbuf);
      return;
    }

  hci_ev = (void *)evbuf;

  hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
  hci_ev->length = sizeof(*ev);
  ev = (void *)hci_ev->data;

  ev->subev_code = BLE_HCI_LE_SUBEV_PERIODIC_ADV_RPT;
  ev->sync_handle = htole16(ble_ll_sync_get_handle(sm));
  ev->tx_power = 127; /* not available */
  ev->rssi = 127;     /* not available */
  ev->cte_type = 0xff;
  ev->data_status = BLE_HCI_PERIODIC_DATA_STATUS_TRUNCATED;
  ev->data_len = 0;

  ble_ll_hci_event_send(hci_ev);
}

/****************************************************************************
 * Name: ble_ll_sync_send_per_adv_rpt
 *
 ****************************************************************************/

static void ble_ll_sync_send_per_adv_rpt(struct ble_ll_sync_sm *sm,
                                         struct iob_s *rxpdu, int8_t rssi,
                                         int8_t tx_power, int datalen,
                                         uint8_t *aux, bool aux_scheduled)
{
  struct ble_hci_ev_le_subev_periodic_adv_rpt *ev;
  struct ble_hci_ev *hci_ev;
  struct ble_hci_ev *hci_ev_next = NULL;
  uint8_t max_data_len;
  int offset;

  /* use next report buffer if present, this means we are chaining */

  if (sm->next_report)
    {
      hci_ev = (void *)sm->next_report;
      sm->next_report = NULL;
    }
  else
    {
      hci_ev = ble_ll_hci_evt_alloc(1);
      if (!hci_ev)
        {
          goto done;
        }
    }

  max_data_len = BLE_LL_MAX_EVT_LEN - sizeof(*hci_ev) - sizeof(*ev);
  offset = 0;

  do
    {
      if (hci_ev_next)
        {
          hci_ev = hci_ev_next;
          hci_ev_next = NULL;
        }

      hci_ev->opcode = BLE_HCI_EVCODE_LE_META;
      hci_ev->length = sizeof(*ev);

      ev = (void *)hci_ev->data;

      ev->subev_code = BLE_HCI_LE_SUBEV_PERIODIC_ADV_RPT;
      ev->sync_handle = htole16(ble_ll_sync_get_handle(sm));
      ev->tx_power = tx_power;
      ev->rssi = rssi;
      ev->cte_type = 0xff;

      ev->data_len = MIN(max_data_len, datalen - offset);

      /* adjust event length */

      hci_ev->length += ev->data_len;

      iob_copyout(ev->data, rxpdu, ev->data_len, offset);
      offset += ev->data_len;

      /* Need another event for next fragment of this PDU */

      if (offset < datalen)
        {
          hci_ev_next = ble_ll_hci_evt_alloc(1);
          if (hci_ev_next)
            {
              ev->data_status = BLE_HCI_PERIODIC_DATA_STATUS_INCOMPLETE;
            }
          else
            {
              ev->data_status = BLE_HCI_PERIODIC_DATA_STATUS_TRUNCATED;
            }
        }
      else
        {
          /* last report of this PDU */

          if (aux)
            {
              if (aux_scheduled)
                {
                  /* if we scheduled aux, we need buffer for next report */

                  hci_ev_next = ble_ll_hci_evt_alloc(1);
                  if (hci_ev_next)
                    {
                      ev->data_status =
                          BLE_HCI_PERIODIC_DATA_STATUS_INCOMPLETE;
                    }
                  else
                    {
                      ev->data_status =
                          BLE_HCI_PERIODIC_DATA_STATUS_TRUNCATED;
                    }
                }
              else
                {
                  ev->data_status = BLE_HCI_PERIODIC_DATA_STATUS_TRUNCATED;
                }
            }
          else
            {
              ev->data_status = BLE_HCI_PERIODIC_DATA_STATUS_COMPLETE;
            }
        }

      ble_ll_hci_event_send(hci_ev);
    }
  while ((offset < datalen) && hci_ev_next);

done:

  /* this means that we already truncated data (or didn't sent first at all)
   * in HCI report but has scheduled for next PDU in chain. In that case mark
   * it so that we end event properly when next PDU is received.
   */

  if (aux_scheduled && !hci_ev_next)
    {
      sm->flags |= BLE_LL_SYNC_SM_FLAG_HCI_TRUNCATED;
    }

  /* store for chain */

  sm->next_report = (void *)hci_ev_next;
}

/****************************************************************************
 * Name: ble_ll_sync_phy_mode_to_aux_phy
 *
 ****************************************************************************/

static uint8_t ble_ll_sync_phy_mode_to_aux_phy(uint8_t phy_mode)
{
  switch (phy_mode)
    {
      case BLE_PHY_MODE_1M:
        return 0x00;
      case BLE_PHY_MODE_2M:
        return 0x01;
      case BLE_PHY_MODE_CODED_125KBPS:
      case BLE_PHY_MODE_CODED_500KBPS:
        return 0x02;
      default:
        DEBUGASSERT(false);
        return 0x00;
    }
}

/****************************************************************************
 * Name: ble_ll_sync_parse_aux_ptr
 *
 ****************************************************************************/

static void ble_ll_sync_parse_aux_ptr(const uint8_t *buf, uint8_t *chan,
                                      uint32_t *offset,
                                      uint8_t *offset_units, uint8_t *phy)
{
  uint32_t aux_ptr_field = get_le24(buf);

  *chan = aux_ptr_field & 0x3f;

  /* TODO use CA aux_ptr_field >> 6 */

  if ((aux_ptr_field >> 7) & 0x01)
    {
      *offset = 300 * ((aux_ptr_field >> 8) & 0x1fff);
      *offset_units = 1;
    }
  else
    {
      *offset = 30 * ((aux_ptr_field >> 8) & 0x1fff);
      *offset_units = 0;
    }

  *phy = (aux_ptr_field >> 21) & 0x07;
}

/****************************************************************************
 * Name: ble_ll_sync_sched_set
 *
 ****************************************************************************/

static void ble_ll_sync_sched_set(struct ble_ll_sched_item *sch,
                                  uint32_t start_ticks, uint8_t start_rem_us,
                                  uint32_t offset, uint8_t phy_mode)
{
  uint32_t duration_us;
  uint32_t duration;

  if (offset > 0)
    {
      ble_ll_tmr_add(&start_ticks, &start_rem_us, offset);
    }

  duration_us =
      ble_ll_pdu_us(CONFIG_BLE_LL_SCHED_SCAN_SYNC_PDU_LEN, phy_mode);
  duration = ble_ll_tmr_u2t(duration_us);

  sch->start_time = start_ticks - g_ble_ll_sched_offset_ticks;
  sch->remainder = start_rem_us;
  sch->end_time = start_ticks + duration;
}

/****************************************************************************
 * Name: ble_ll_sync_schedule_chain
 *
 ****************************************************************************/

static int ble_ll_sync_schedule_chain(struct ble_ll_sync_sm *sm,
                                      struct ble_mbuf_hdr *hdr,
                                      const uint8_t *aux)
{
  uint8_t offset_units;
  uint32_t offset;
  uint8_t chan;
  uint8_t phy;

  ble_ll_sync_parse_aux_ptr(aux, &chan, &offset, &offset_units, &phy);

  if (chan >= BLE_PHY_NUM_DATA_CHANS)
    {
      return -1;
    }

  if (offset < BLE_LL_MAFS)
    {
      return -1;
    }

  /* chain should use same PHY as central PDU */

  if (phy != ble_ll_sync_phy_mode_to_aux_phy(sm->phy_mode))
    {
      return -1;
    }

  if (offset_units)
    {
      sm->flags |= BLE_LL_SYNC_SM_FLAG_OFFSET_300;
    }
  else
    {
      sm->flags &= ~BLE_LL_SYNC_SM_FLAG_OFFSET_300;
    }

  sm->chan_chain = chan;

  sm->flags |= BLE_LL_SYNC_SM_FLAG_CHAIN;

  ble_ll_sync_sched_set(&sm->sch, hdr->beg_cputime, hdr->rem_usecs, offset,
                        sm->phy_mode);

  return ble_ll_sched_sync(&sm->sch);
}

/****************************************************************************
 * Name: ble_ll_sync_established
 *
 ****************************************************************************/

static void ble_ll_sync_established(struct ble_ll_sync_sm *sm)
{
  DEBUGASSERT(sm->sync_pending_cnt);

  /* mark as established */

  sm->flags |= BLE_LL_SYNC_SM_FLAG_ESTABLISHED;
  sm->flags &= ~BLE_LL_SYNC_SM_FLAG_ESTABLISHING;

  sm->sync_pending_cnt = 0;

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV_SYNC_TRANSFER
  if (sm->transfer_conn)
    {
      ble_ll_sync_transfer_received(sm, BLE_ERR_SUCCESS);
      return;
    }
#endif

  ble_ll_sync_est_event_success(sm);
}

/****************************************************************************
 * Name: ble_ll_sync_check_failed
 *
 ****************************************************************************/

static void ble_ll_sync_check_failed(struct ble_ll_sync_sm *sm)
{
  DEBUGASSERT(sm->sync_pending_cnt);

  /* if we can retry on next event */

  if (--sm->sync_pending_cnt)
    {
      return;
    }

  sm->flags &= ~BLE_LL_SYNC_SM_FLAG_ESTABLISHING;

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV_SYNC_TRANSFER
  if (sm->transfer_conn)
    {
      ble_ll_sync_transfer_received(sm, BLE_ERR_CONN_ESTABLISHMENT);
      return;
    }
#endif

  ble_ll_sync_est_event_failed(BLE_ERR_CONN_ESTABLISHMENT);
}

/****************************************************************************
 * Name: ble_ll_sync_check_acad
 *
 ****************************************************************************/

static bool ble_ll_sync_check_acad(struct ble_ll_sync_sm *sm,
                                   const uint8_t *acad, uint8_t acad_len,
                                   const uint8_t **biginfo,
                                   uint8_t *biginfo_len)
{
  const struct ble_ll_acad_channel_map_update_ind *chmu;
  unsigned int ad_len;
  uint8_t ad_type;

  /* assume no empty fields */

  while (acad_len > 2)
    {
      ad_len = acad[0];
      ad_type = acad[1];

      /* early termination should not happen in ACAD */

      if (ad_len == 0)
        {
          return false;
        }

      /* check if not passing pass acad data */

      if (ad_len + 1 > acad_len)
        {
          return false;
        }

      switch (ad_type)
        {
          case BLE_LL_ACAD_CHANNEL_MAP_UPDATE_IND:
            chmu = (const void *)&acad[2];

            if (ad_len - 1 != sizeof(*chmu))
              {
                return false;
              }

            /* Channels Mask (37 bits) TODO should we check
             * this?
             */

            sm->chan_map_new[0] = chmu->map[0];
            sm->chan_map_new[1] = chmu->map[1];
            sm->chan_map_new[2] = chmu->map[2];
            sm->chan_map_new[3] = chmu->map[3];
            sm->chan_map_new[4] = chmu->map[4] & 0x1f;

            /* drop if channel map is invalid */

            if (ble_ll_utils_chan_map_used_get(sm->chan_map_new) == 0)
              {
                return false;
              }

            sm->chan_map_new_instant = le16toh(chmu->instant);
            sm->flags |= BLE_LL_SYNC_SM_FLAG_NEW_CHANMAP;
            break;

          default:
            break;
        }

      acad += ad_len + 1;
      acad_len -= ad_len + 1;
    }

  /* should have no trailing zeros */

  if (acad_len)
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: ble_ll_sync_next_event
 *
 ****************************************************************************/

static int ble_ll_sync_next_event(struct ble_ll_sync_sm *sm,
                                  uint32_t cur_ww_adjust)
{
  uint32_t cur_ww;
  uint32_t max_ww;
  uint32_t itvl;
  uint16_t skip = sm->skip;

  /* don't skip if are establishing sync or we missed last event */

  if (skip && ((sm->flags & BLE_LL_SYNC_SM_FLAG_ESTABLISHING) ||
               LL_TMR_LT(sm->last_anchor_point, sm->anchor_point)))
    {
      skip = 0;
    }

  /* Set next event start time, we can use pre-calculated values for one
   * interval if not skipping
   */

  if (skip == 0)
    {
      sm->anchor_point += sm->itvl_ticks;
      ble_ll_tmr_add_u(&sm->anchor_point, &sm->anchor_point_usecs,
                       sm->itvl_usecs);
    }
  else
    {
      itvl = sm->itvl * BLE_LL_SYNC_ITVL_USECS * (1 + skip);
      ble_ll_tmr_add(&sm->anchor_point, &sm->anchor_point_usecs, itvl);
    }

  /* Set event counter to the next event */

  sm->event_cntr += 1 + skip;

  /* update channel map if needed */

  if (sm->flags & BLE_LL_SYNC_SM_FLAG_NEW_CHANMAP)
    {
      if (((int16_t)(sm->event_cntr - sm->chan_map_new_instant)) >= 0)
        {
          /* map was verified on reception */

          sm->chan_map[0] = sm->chan_map_new[0];
          sm->chan_map[1] = sm->chan_map_new[1];
          sm->chan_map[2] = sm->chan_map_new[2];
          sm->chan_map[3] = sm->chan_map_new[3];
          sm->chan_map[4] = sm->chan_map_new[4];
          sm->chan_map_used = ble_ll_utils_chan_map_used_get(sm->chan_map);
          sm->flags &= ~BLE_LL_SYNC_SM_FLAG_NEW_CHANMAP;
        }
    }

  /* Calculate channel index of next event */

  sm->chan_index = ble_ll_utils_dci_csa2(sm->event_cntr, sm->channel_id,
                                         sm->chan_map_used, sm->chan_map);

  cur_ww = ble_ll_utils_calc_window_widening(sm->anchor_point,
                                             sm->last_anchor_point, sm->sca);

  cur_ww += cur_ww_adjust;

  max_ww = (sm->itvl * (BLE_LL_SYNC_ITVL_USECS / 2)) - BLE_LL_IFS;
  if (cur_ww >= max_ww)
    {
      return -1;
    }

  cur_ww += BLE_LL_JITTER_USECS;

  /* if updated anchor is pass last anchor + timeout it means we will not be
   * able to get it in time and hit sync timeout
   *
   * note that this may result in sync timeout being sent before real timeout
   * but we won't be able to fit in time anyway..
   *
   * We don't do that when establishing since we try up to
   * BLE_LL_SYNC_ESTABLISH_CNT events before failing regardless of timeout
   */

  if (!(sm->flags & BLE_LL_SYNC_SM_FLAG_ESTABLISHING))
    {
      if (LL_TMR_GT(sm->anchor_point - ble_ll_tmr_u2t(cur_ww),
                    sm->last_anchor_point + sm->timeout))
        {
          return -1;
        }
    }

  sm->window_widening = cur_ww;

  return 0;
}

/****************************************************************************
 * Name: ble_ll_sync_event_end
 *
 ****************************************************************************/

static void ble_ll_sync_event_end(FAR void *arg)
{
  struct ble_ll_sync_sm *sm;

  /* Better be a connection state machine! */

  sm = arg;
  DEBUGASSERT(sm);

  ble_ll_rfmgmt_release();

  if (sm->flags & BLE_LL_SYNC_SM_FLAG_ESTABLISHING)
    {
      ble_ll_sync_check_failed(sm);
    }

  /* Check if we need to resume scanning */

  ble_ll_scan_chk_resume();

  /* Remove any end events that might be enqueued */

  ble_ll_event_remove(&sm->sync_ev_end);

  /* don't schedule next event if sync is not established nor establishing at
   * this point SM is no longer valid
   */

  if (!(sm->flags & (BLE_LL_SYNC_SM_FLAG_ESTABLISHED |
                     BLE_LL_SYNC_SM_FLAG_ESTABLISHING)))
    {
      ble_ll_sync_sm_clear(sm);
      return;
    }

  /* if we had prepared buffer for next even it means we were chaining and
   * must send truncated report to host
   */

  if (sm->next_report)
    {
      DEBUGASSERT(!(sm->flags & BLE_LL_SYNC_SM_FLAG_HCI_TRUNCATED));
      ble_ll_sync_send_truncated_per_adv_rpt(sm, sm->next_report);
      sm->next_report = NULL;
    }

  /* Event ended so we are no longer chaining */

  sm->flags &= ~BLE_LL_SYNC_SM_FLAG_HCI_TRUNCATED;
  sm->flags &= ~BLE_LL_SYNC_SM_FLAG_CHAIN;

  do
    {
      if (ble_ll_sync_next_event(sm, 0) < 0)
        {
          if (sm->flags & BLE_LL_SYNC_SM_FLAG_ESTABLISHING)
            {
              /* don't allow any retry if this failed */

              sm->sync_pending_cnt = 1;
              ble_ll_sync_check_failed(sm);
            }
          else
            {
              ble_ll_sync_lost_event(sm);
            }

          /* at this point SM is no longer valid */

          ble_ll_sync_sm_clear(sm);
          return;
        }

      ble_ll_sync_sched_set(&sm->sch, sm->anchor_point,
                            sm->anchor_point_usecs, 0, sm->phy_mode);
    }
  while (ble_ll_sched_sync_reschedule(&sm->sch, sm->window_widening));
}

/****************************************************************************
 * Name: ble_ll_sync_reserve
 *
 ****************************************************************************/

static struct ble_ll_sync_sm *ble_ll_sync_reserve(void)
{
  struct ble_ll_sync_sm *sm;
  int i;

  for (i = 0; i < BLE_LL_SYNC_CNT; i++)
    {
      sm = &g_ble_ll_sync_sm[i];

      if (!sm->flags)
        {
          sm->flags |= BLE_LL_SYNC_SM_FLAG_RESERVED;
          return sm;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: ble_ll_sync_cancel_complete_event
 *
 ****************************************************************************/

static void ble_ll_sync_cancel_complete_event(void *user_data)
{
  ble_ll_sync_est_event_failed(BLE_ERR_OPERATION_CANCELLED);
}

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV_SYNC_TRANSFER
/****************************************************************************
 * Name: ble_ll_sync_transfer_get
 *
 ****************************************************************************/

static struct ble_ll_sync_sm *ble_ll_sync_transfer_get(const uint8_t *addr,
                                                       uint8_t addr_type,
                                                       uint8_t sid)
{
  struct ble_ll_sync_sm *sm;
  int i;

  for (i = 0; i < BLE_LL_SYNC_CNT; i++)
    {
      sm = &g_ble_ll_sync_sm[i];

      if (!sm->flags)
        {
          /* allocate event for transfer received event */

          sm->transfer_received_ev = ble_ll_hci_evt_alloc(0);
          if (!sm->transfer_received_ev)
            {
              break;
            }

          sm->adv_sid = sid;
          sm->adv_addr_type = addr_type;
          memcpy(&sm->adv_addr, addr, BLE_DEV_ADDR_LEN);

          sm->flags |= BLE_LL_SYNC_SM_FLAG_ESTABLISHING;
          return sm;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: ble_ll_sync_put_syncinfo
 *
 ****************************************************************************/

static void ble_ll_sync_put_syncinfo(struct ble_ll_sync_sm *syncsm,
                                     struct ble_ll_conn_sm *connsm,
                                     uint8_t *conn_event_cnt, uint8_t *dptr)
{
  uint8_t anchor_usecs;
  uint16_t conn_cnt;
  uint32_t offset;
  uint32_t anchor;
  uint8_t units;

  anchor = connsm->anchor_point;
  anchor_usecs = connsm->anchor_point_usecs;
  conn_cnt = connsm->event_cntr;

  /* get anchor for conn event that is before
   * periodic_adv_event_start_time
   */

  while (LL_TMR_GT(anchor, syncsm->anchor_point))
    {
      ble_ll_conn_anchor_event_cntr_get(connsm, --conn_cnt, &anchor,
                                        &anchor_usecs);
    }

  offset = ble_ll_tmr_t2u(syncsm->anchor_point - anchor);
  offset -= anchor_usecs;
  offset += syncsm->anchor_point_usecs;

  /* connEventCount */

  put_le16(conn_event_cnt, conn_cnt);

  /* Sync Packet Offset (13 bits), Offset Units (1 bit), Offset Adjust (1
   * bit), RFU (1 bit)
   */

  if (offset > 245700)
    {
      units = 0x20;

      if (offset >= 0x2000)
        {
          offset -= 0x2000;
          units |= 0x40;
        }

      offset = offset / 300;
    }
  else
    {
      units = 0x00;
      offset = offset / 30;
    }

  dptr[0] = (offset & 0x000000ff);
  dptr[1] = ((offset >> 8) & 0x0000001f) | units;

  /* Interval (2 bytes) */

  put_le16(&dptr[2], syncsm->itvl);

  /* Channels Mask (37 bits) */

  dptr[4] = syncsm->chan_map[0];
  dptr[5] = syncsm->chan_map[1];
  dptr[6] = syncsm->chan_map[2];
  dptr[7] = syncsm->chan_map[3];
  dptr[8] = syncsm->chan_map[4] & 0x1f;

  /* SCA (3 bits) */

  dptr[8] |= syncsm->sca << 5;

  /* AA (4 bytes) */

  put_le32(&dptr[9], syncsm->access_addr);

  /* CRCInit (3 bytes) */

  dptr[13] = (uint8_t)syncsm->crcinit;
  dptr[14] = (uint8_t)(syncsm->crcinit >> 8);
  dptr[15] = (uint8_t)(syncsm->crcinit >> 16);

  /* Event Counter (2 bytes) */

  put_le16(&dptr[16], syncsm->event_cntr);
}

/****************************************************************************
 * Name: ble_ll_sync_send_sync_ind
 *
 ****************************************************************************/

static int ble_ll_sync_send_sync_ind(struct ble_ll_sync_sm *syncsm,
                                     struct ble_ll_conn_sm *connsm,
                                     uint16_t service_data)
{
  struct iob_s *om;
  uint8_t *sync_ind;

  om = ble_ll_iob_alloc();
  if (!om)
    {
      return BLE_ERR_MEM_CAPACITY;
    }

  IOB_DATA(om)[0] = BLE_LL_CTRL_PERIODIC_SYNC_IND;

  sync_ind = IOB_DATA(om) + 1;

  /* ID (service_data), already in LE order */

  memcpy(sync_ind, &service_data, sizeof(service_data));

  /* fill in syncinfo */

  ble_ll_sync_put_syncinfo(syncsm, connsm, sync_ind + 20, sync_ind + 2);

  /* lastPaEventCounter */

  put_le16(sync_ind + 22, syncsm->event_cntr_last_received);

  /* SID, AType, SCA */

  sync_ind[24] = syncsm->adv_sid;

  if (syncsm->flags & BLE_LL_SYNC_SM_FLAG_ADDR_RESOLVED)
    {
      sync_ind[24] |= 1 << 4;
    }
  else
    {
      sync_ind[24] |= (syncsm->adv_addr_type == BLE_ADDR_RANDOM) << 4;
    }

  sync_ind[24] |= BLE_LL_SCA_ENUM << 5;

  /* PHY */

  sync_ind[25] =
      (0x01 << (ble_ll_sync_phy_mode_to_hci(syncsm->phy_mode) - 1));

  /* AdvA */

  if (syncsm->flags & BLE_LL_SYNC_SM_FLAG_ADDR_RESOLVED)
    {
      memcpy(sync_ind + 26, syncsm->adv_addr_rpa, BLE_DEV_ADDR_LEN);
    }
  else
    {
      memcpy(sync_ind + 26, syncsm->adv_addr, BLE_DEV_ADDR_LEN);
    }

  /* syncConnEventCount */

  put_le16(sync_ind + 32, connsm->event_cntr);

  ble_ll_conn_enqueue_pkt(connsm, om, BLE_LL_LLID_CTRL,
                          BLE_LL_CTRL_PERIODIC_SYNC_IND_LEN + 1);

  return BLE_ERR_SUCCESS;
}

#endif

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV) && defined(CONFIG_BLE_LL_ROLE_OBSERVER)
/****************************************************************************
 * Name: ble_ll_sync_rx_isr_start
 *
 * Description:
 *   Called when a receive PDU has started.
 *
 *   Context: interrupt
 *
 * Returned Value:
 *   int
 *   < 0: A frame we dont want to receive.
 *   = 0: Continue to receive frame. Dont go from rx to tx
 *
 ****************************************************************************/

int ble_ll_sync_rx_isr_start(uint8_t pdu_type, struct ble_mbuf_hdr *rxhdr)
{
  DEBUGASSERT(g_ble_ll_sync_sm_current);

  /* this also handles chains as those have same PDU type */

  if (pdu_type != BLE_ADV_PDU_TYPE_AUX_SYNC_IND)
    {
      ble_ll_event_add(&g_ble_ll_sync_sm_current->sync_ev_end);
      ble_ll_sync_current_sm_over();
      BLE_LL_STATS_INC(ble_ll_stats, sched_invalid_pdu);
      return -1;
    }

  BLE_LL_STATS_INC(ble_ll_stats, sync_received);
  return 0;
}

/****************************************************************************
 * Name: ble_ll_sync_rx_isr_end
 *
 * Description:
 *   Called when a receive PDU has ended.
 *
 *   Context: Interrupt
 *
 * Input Parameters:
 *   rxpdu
 *
 * Returned Value:
 *   int
 *   < 0: Disable the phy after reception.
 *   == 0: Success. Do not disable the PHY.
 *   > 0: Do not disable PHY as that has already been done.
 *
 ****************************************************************************/

int ble_ll_sync_rx_isr_end(uint8_t *rxbuf, struct ble_mbuf_hdr *rxhdr)
{
  struct ble_mbuf_hdr *ble_hdr;
  struct iob_s *rxpdu;

  DEBUGASSERT(g_ble_ll_sync_sm_current);

  /* type was verified in isr_start */

  rxpdu = ble_ll_rxpdu_alloc(rxbuf[1] + BLE_LL_PDU_HDR_LEN);
  if (rxpdu)
    {
      ble_phy_rxpdu_copy(rxbuf, rxpdu);

      ble_hdr = BLE_MBUF_HDR_PTR(rxpdu);
      ble_hdr->rxinfo.user_data = g_ble_ll_sync_sm_current;

      ble_ll_rx_pdu_in(rxpdu);
    }
  else
    {
      BLE_LL_STATS_INC(ble_ll_stats, sync_rx_buf_err);
      ble_ll_event_add(&g_ble_ll_sync_sm_current->sync_ev_end);
    }

  /* PHY is disabled here */

  ble_ll_sync_current_sm_over();

  return 1;
}

/****************************************************************************
 * Name: ble_ll_sync_wfr_timer_exp
 *
 * Description:
 *   Called when the wait for response timer expires while in the sync state.
 *
 *   Context: Interrupt.
 *
 ****************************************************************************/

void ble_ll_sync_wfr_timer_exp(void)
{
  struct ble_ll_sync_sm *sm = g_ble_ll_sync_sm_current;

  DEBUGASSERT(g_ble_ll_sync_sm_current);
  BLE_LL_STATS_INC(ble_ll_stats, sync_missed_err);

  ble_ll_sync_current_sm_over();
  ble_ll_event_add(&sm->sync_ev_end);
}

/****************************************************************************
 * Name: ble_ll_sync_halt
 *
 * Description:
 *   Called when sync event needs to be halted. This normally should not be
 *   called and is only called when a scheduled item executes but scanning
 *   for sync/chain is still ongoing.
 *   Context: Interrupt
 *
 ****************************************************************************/

void ble_ll_sync_halt(void)
{
  struct ble_ll_sync_sm *sm = g_ble_ll_sync_sm_current;

  ble_ll_sync_current_sm_over();

  if (sm)
    {
      ble_ll_event_add(&sm->sync_ev_end);
    }
}

/****************************************************************************
 * Name: ble_ll_sync_get_event_end_time
 *
 ****************************************************************************/

uint32_t ble_ll_sync_get_event_end_time(void)
{
  uint32_t end_time;

  if (g_ble_ll_sync_sm_current)
    {
      end_time = g_ble_ll_sync_sm_current->sch.end_time;
    }
  else
    {
      end_time = ble_ll_tmr_get();
    }

  return end_time;
}

/****************************************************************************
 * Name: ble_ll_sync_rx_pkt_in
 *
 ****************************************************************************/

void ble_ll_sync_rx_pkt_in(struct iob_s *rxpdu, struct ble_mbuf_hdr *hdr)
{
  struct ble_ll_sync_sm *sm = hdr->rxinfo.user_data;
  bool aux_scheduled = false;
  int8_t tx_power = 127; /* defaults to not available */
  uint8_t *aux = NULL;
  uint8_t *acad = NULL;
  uint8_t acad_len = 0;
  uint16_t *adi = NULL;
  const uint8_t *biginfo = NULL;
  uint8_t biginfo_len = 0;
  int datalen;
  bool reports_enabled;
  bool is_duplicate = false;

  DEBUGASSERT(sm);

  /* this could happen if sync was cancelled or terminated while pkt_in was
   * already in LL queue, just drop in that case
   */

  if (!sm->flags)
    {
      ble_ll_scan_chk_resume();
      ble_ll_rfmgmt_release();
      return;
    }

  /* Set anchor point (and last) if 1st rxd frame in sync event. According to
   * spec this should be done even if CRC is not valid so we can store it
   * here
   */

  if (sm->flags & BLE_LL_SYNC_SM_FLAG_SET_ANCHOR)
    {
      sm->flags &= ~BLE_LL_SYNC_SM_FLAG_SET_ANCHOR;

      sm->anchor_point = hdr->beg_cputime;
      sm->anchor_point_usecs = hdr->rem_usecs;
      sm->last_anchor_point = sm->anchor_point;
    }

  /* CRC error, end event */

  if (!BLE_MBUF_HDR_CRC_OK(hdr))
    {
      BLE_LL_STATS_INC(ble_ll_stats, sync_crc_err);
      goto end_event;
    }

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV_SYNC_TRANSFER

  /* save last pa counter */

  sm->event_cntr_last_received = sm->event_cntr;
#endif

  /* this means we are chaining but due to low buffers already sent data
   * truncated report to host (or didn't sent any at all). If this happens
   * next_buf should be already set to NULL and we just end event.
   */

  if (sm->flags & BLE_LL_SYNC_SM_FLAG_HCI_TRUNCATED)
    {
      DEBUGASSERT(!sm->next_report);
      goto end_event;
    }

  /* get ext header data */

  datalen = ble_ll_sync_parse_ext_hdr(rxpdu, &aux, &tx_power, &acad,
                                      &acad_len, &adi);
  if (datalen < 0)
    {
      /* we got bad packet, end event */

      goto end_event;
    }

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV_ADI_SUPPORT
  if (adi != NULL)
    {
      is_duplicate = (sm->flags & BLE_LL_SYNC_SM_FLAG_DUPLICATES) &&
                     (get_le16(adi) == sm->prev_adi);
      sm->prev_adi = get_le16(adi);
    }
#endif

  reports_enabled =
      ble_ll_hci_is_le_event_enabled(BLE_HCI_LE_SUBEV_PERIODIC_ADV_RPT) &&
      !(sm->flags & BLE_LL_SYNC_SM_FLAG_DISABLED) && !is_duplicate;

  /* no need to schedule for chain if reporting is disabled */

  if (reports_enabled)
    {
      /* if aux is present, we need to schedule ASAP */

      if (aux && (ble_ll_sync_schedule_chain(sm, hdr, aux) == 0))
        {
          aux_scheduled = true;
        }
    }

  /* check ACAD, needs to be done before rxpdu is adjusted for ADV data */

  if (acad &&
      !ble_ll_sync_check_acad(sm, acad, acad_len, &biginfo, &biginfo_len))
    {
      /* we got bad packet (bad ACAD data), end event */

      goto end_event;
    }

  /* we need to establish link even if reporting was disabled */

  if (sm->flags & BLE_LL_SYNC_SM_FLAG_ESTABLISHING)
    {
      ble_ll_sync_established(sm);
    }

  /* only if reporting is enabled */

  if (reports_enabled)
    {
      /* Adjust rxpdu to contain advertising data only */

      ble_ll_sync_adjust_ext_hdr(rxpdu);

      /* send reports from this PDU */

      ble_ll_sync_send_per_adv_rpt(sm, rxpdu, hdr->rxinfo.rssi, tx_power,
                                   datalen, aux, aux_scheduled);
    }

  /* if chain was scheduled we don't end event yet */

  /* TODO should we check resume only if offset is high? */

  if (aux_scheduled)
    {
      ble_ll_scan_chk_resume();
      ble_ll_rfmgmt_release();
      return;
    }

end_event:
  ble_ll_event_add(&sm->sync_ev_end);
  ble_ll_rfmgmt_release();
}

/****************************************************************************
 * Name: ble_ll_sync_info_event
 *
 ****************************************************************************/

void ble_ll_sync_info_event(struct ble_ll_scan_addr_data *addrd, uint8_t sid,
                            struct ble_mbuf_hdr *rxhdr,
                            const uint8_t *syncinfo)
{
  struct ble_ll_sync_sm *sm = NULL;
  uint16_t max_skip;
  uint32_t offset;
  uint32_t usecs;
  uint16_t itvl;
  int i;

  /* ignore if not synchronizing */

  if (!g_ble_ll_sync_create_comp_ev)
    {
      return;
    }

  /* get reserved SM */

  for (i = 0; i < BLE_LL_SYNC_CNT; i++)
    {
      if (g_ble_ll_sync_sm[i].flags & BLE_LL_SYNC_SM_FLAG_RESERVED)
        {
          sm = &g_ble_ll_sync_sm[i];
          break;
        }
    }

  /* this means we already got sync info event and pending sync */

  if (!sm)
    {
      return;
    }

  /* check peer */

  if (g_ble_ll_sync_create_params.options &
      BLE_HCI_LE_PERIODIC_ADV_CREATE_SYNC_OPT_FILTER)
    {
      if (ble_ll_sync_on_list(addrd->adv_addr, addrd->adv_addr_type, sid) <
          0)
        {
          return;
        }

      /* set addr and sid in sm */

      sm->adv_sid = sid;
      sm->adv_addr_type = addrd->adv_addr_type;
      memcpy(sm->adv_addr, addrd->adv_addr, BLE_DEV_ADDR_LEN);
    }
  else
    {
      if ((sm->adv_sid != sid) ||
          (sm->adv_addr_type != addrd->adv_addr_type) ||
          memcmp(sm->adv_addr, addrd->adv_addr, BLE_DEV_ADDR_LEN))
        {
          return;
        }
    }

  /* Sync Packet Offset (13 bits), Offset Units (1 bit), RFU (2 bits) */

  offset = syncinfo[0];
  offset |= (uint16_t)(syncinfo[1] & 0x1f) << 8;

  /* ignore if offset is not valid */

  if (!offset)
    {
      return;
    }

  /* Interval (2 bytes), ignore if invalid */

  itvl = get_le16(&syncinfo[2]);
  if (itvl < 6)
    {
      return;
    }

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  if (addrd->adva_resolved)
    {
      sm->flags |= BLE_LL_SYNC_SM_FLAG_ADDR_RESOLVED;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV_SYNC_TRANSFER
      memcpy(sm->adv_addr_rpa, addrd->adva, BLE_DEV_ADDR_LEN);
#endif
    }
#endif

  /* set params from HCI LE Create Periodic Sync */

  sm->timeout = g_ble_ll_sync_create_params.timeout;
  sm->skip = g_ble_ll_sync_create_params.max_skip;
  sm->sync_pending_cnt = BLE_LL_SYNC_ESTABLISH_CNT;

  if (syncinfo[1] & 0x20)
    {
      offset *= 300;
      sm->flags |= BLE_LL_SYNC_SM_FLAG_OFFSET_300;
    }
  else
    {
      offset *= 30;
      sm->flags &= ~BLE_LL_SYNC_SM_FLAG_OFFSET_300;
    }

  /* sync end event */

  ble_ll_event_init(&sm->sync_ev_end, ble_ll_sync_event_end, sm);

  sm->itvl = itvl;

  /* precalculate interval ticks and usecs */

  usecs = sm->itvl * BLE_LL_SYNC_ITVL_USECS;
  sm->itvl_ticks = ble_ll_tmr_u2t_r(usecs, &sm->itvl_usecs);

  /* Channels Mask (37 bits) */

  sm->chan_map[0] = syncinfo[4];
  sm->chan_map[1] = syncinfo[5];
  sm->chan_map[2] = syncinfo[6];
  sm->chan_map[3] = syncinfo[7];
  sm->chan_map[4] = syncinfo[8] & 0x1f;
  sm->chan_map_used = ble_ll_utils_chan_map_used_get(sm->chan_map);

  /* SCA (3 bits) */

  sm->sca = syncinfo[8] >> 5;

  /* AA (4 bytes) */

  sm->access_addr = get_le32(&syncinfo[9]);
  sm->channel_id = ((sm->access_addr & 0xffff0000) >> 16) ^
                   (sm->access_addr & 0x0000ffff);

  /* CRCInit (3 bytes) */

  sm->crcinit = syncinfo[15];
  sm->crcinit = (sm->crcinit << 8) | syncinfo[14];
  sm->crcinit = (sm->crcinit << 8) | syncinfo[13];

  /* Event Counter (2 bytes) */

  sm->event_cntr = get_le16(&syncinfo[16]);

  /* adjust skip if pass timeout */

  max_skip = get_max_skip(sm->itvl * BLE_LL_SYNC_ITVL_USECS, sm->timeout);
  if (sm->skip > max_skip)
    {
      sm->skip = max_skip;
    }

  /* from now on we only need timeout in ticks */

  sm->timeout = ble_ll_tmr_u2t(sm->timeout);

  sm->phy_mode = rxhdr->rxinfo.phy_mode;
  sm->window_widening = BLE_LL_JITTER_USECS;

  /* Calculate channel index of first event */

  sm->chan_index = ble_ll_utils_dci_csa2(sm->event_cntr, sm->channel_id,
                                         sm->chan_map_used, sm->chan_map);

  ble_ll_sync_sched_set(&sm->sch, rxhdr->beg_cputime, rxhdr->rem_usecs,
                        offset, sm->phy_mode);

  if (ble_ll_sched_sync(&sm->sch))
    {
      return;
    }

  sm->anchor_point = sm->sch.start_time + g_ble_ll_sched_offset_ticks;
  sm->anchor_point_usecs = sm->sch.remainder;
  sm->last_anchor_point = sm->anchor_point;

#if CONFIG_BLE_LL_BT_VERSION >= 51
  if (g_ble_ll_sync_create_params.options &
      BLE_HCI_LE_PERIODIC_ADV_CREATE_SYNC_OPT_DISABLED)
    {
      sm->flags |= BLE_LL_SYNC_SM_FLAG_DISABLED;
    }
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV_ADI_SUPPORT
  if (g_ble_ll_sync_create_params.options &
      BLE_HCI_LE_PERIODIC_ADV_CREATE_SYNC_OPT_DUPLICATES)
    {
      sm->flags |= BLE_LL_SYNC_SM_FLAG_DUPLICATES;
    }
#endif

  sm->flags &= ~BLE_LL_SYNC_SM_FLAG_RESERVED;
  sm->flags |= BLE_LL_SYNC_SM_FLAG_ESTABLISHING;
  sm->flags |= BLE_LL_SYNC_SM_FLAG_SYNC_INFO;
}

/****************************************************************************
 * Name: ble_ll_sync_create
 *
 ****************************************************************************/

int ble_ll_sync_create(const uint8_t *cmdbuf, uint8_t len)
{
  const struct ble_hci_le_periodic_adv_create_sync_cp *cmd =
      (const void *)cmdbuf;
  struct ble_ll_sync_sm *sm;
  uint16_t timeout;
  irqstate_t sr;

  if (g_ble_ll_sync_create_comp_ev)
    {
      return BLE_ERR_CMD_DISALLOWED;
    }

  if (len != sizeof(*cmd))
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  if (CONFIG_BLE_LL_BT_VERSION >= 53)
    {
      if (cmd->options & 0xf8)
        {
          return BLE_ERR_INV_HCI_CMD_PARMS;
        }

      if (!(cmd->options &
            BLE_HCI_LE_PERIODIC_ADV_CREATE_SYNC_OPT_DISABLED) &&
          (cmd->options &
           BLE_HCI_LE_PERIODIC_ADV_CREATE_SYNC_OPT_DUPLICATES) &&
          !BLE_LL_PERIODIC_ADV_ADI_SUPPORT)
        {
          /* We do not support ADI in periodic advertising thus cannot enable
           * duplicate filtering.
           */

          return BLE_ERR_UNSUPPORTED;
        }
    }
  else if (CONFIG_BLE_LL_BT_VERSION >= 51)
    {
      if (cmd->options & 0xfc)
        {
          return BLE_ERR_INV_HCI_CMD_PARMS;
        }
    }
  else if (cmd->options & 0xfe)
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  if (cmd->skip > 0x01f3)
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  timeout = le16toh(cmd->sync_timeout);
  if (timeout < 0x000a || timeout > 0x4000)
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

#if CONFIG_BLE_LL_BT_VERSION >= 51

  /* we don't support any CTE yet */

  if (cmd->sync_cte_type)
    {
      if (cmd->sync_cte_type > 4)
        {
          return BLE_ERR_INV_HCI_CMD_PARMS;
        }

      return BLE_ERR_UNSUPPORTED;
    }
#endif

  if (cmd->options & BLE_HCI_LE_PERIODIC_ADV_CREATE_SYNC_OPT_FILTER)
    {
      if (ble_ll_sync_list_empty())
        {
          return BLE_ERR_CMD_DISALLOWED;
        }
    }
  else
    {
      if (cmd->sid > 0x0f)
        {
          return BLE_ERR_INV_HCI_CMD_PARMS;
        }

      if (cmd->peer_addr_type > BLE_HCI_ADV_PEER_ADDR_MAX)
        {
          return BLE_ERR_INV_HCI_CMD_PARMS;
        }

      sr = enter_critical_section();
      sm = ble_ll_sync_find(cmd->peer_addr, cmd->peer_addr_type, cmd->sid);
      leave_critical_section(sr);

      if (sm)
        {
          return BLE_ERR_ACL_CONN_EXISTS;
        }
    }

  /* reserve buffer for sync complete event */

  g_ble_ll_sync_create_comp_ev = ble_ll_hci_evt_alloc(0);
  if (!g_ble_ll_sync_create_comp_ev)
    {
      return BLE_ERR_MEM_CAPACITY;
    }

  sr = enter_critical_section();

  /* reserve 1 SM for created sync */

  sm = ble_ll_sync_reserve();
  if (!sm)
    {
      ble_ll_hci_buf_free(g_ble_ll_sync_create_comp_ev);
      g_ble_ll_sync_create_comp_ev = NULL;
      leave_critical_section(sr);
      return BLE_ERR_MEM_CAPACITY;
    }

  /* if we don't use list, store expected address in reserved SM */

  if (!(cmd->options & BLE_HCI_LE_PERIODIC_ADV_CREATE_SYNC_OPT_FILTER))
    {
      sm->adv_sid = cmd->sid;
      sm->adv_addr_type = cmd->peer_addr_type;
      memcpy(&sm->adv_addr, cmd->peer_addr, BLE_DEV_ADDR_LEN);
    }

  g_ble_ll_sync_create_params.timeout =
      timeout * 10000; /* 10ms units, store in us */
  ;
  g_ble_ll_sync_create_params.max_skip = cmd->skip;
  g_ble_ll_sync_create_params.options = cmd->options;

  leave_critical_section(sr);
  return BLE_ERR_SUCCESS;
}

/****************************************************************************
 * Name: ble_ll_sync_cancel
 *
 ****************************************************************************/

int ble_ll_sync_cancel(void)
{
  struct ble_ll_sync_sm *sm;
  irqstate_t sr;
  int i;

  if (!g_ble_ll_sync_create_comp_ev)
    {
      return BLE_ERR_CMD_DISALLOWED;
    }

  sr = enter_critical_section();

  for (i = 0; i < BLE_LL_SYNC_CNT; i++)
    {
      sm = &g_ble_ll_sync_sm[i];

      /* cancelled before fist sync info packet */

      if (sm->flags & BLE_LL_SYNC_SM_FLAG_RESERVED)
        {
          memset(sm, 0, sizeof(*sm));
          break;
        }

      /* cancelled while pending sync */

      if (sm->flags & BLE_LL_SYNC_SM_FLAG_ESTABLISHING)
        {
          ble_ll_sync_sm_clear(sm);
          break;
        }
    }

  leave_critical_section(sr);

  /* g_ble_ll_sync_create_comp_ev will be cleared by this callback */

  ble_ll_hci_post_cmd_cb_set(ble_ll_sync_cancel_complete_event, NULL);

  return BLE_ERR_SUCCESS;
}

/****************************************************************************
 * Name: ble_ll_sync_terminate
 *
 ****************************************************************************/

int ble_ll_sync_terminate(const uint8_t *cmdbuf, uint8_t len)
{
  const struct ble_hci_le_periodic_adv_term_sync_cp *cmd =
      (const void *)cmdbuf;
  struct ble_ll_sync_sm *sm;
  uint16_t handle;
  irqstate_t sr;

  if (g_ble_ll_sync_create_comp_ev)
    {
      return BLE_ERR_CMD_DISALLOWED;
    }

  if (len != sizeof(*cmd))
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  handle = le16toh(cmd->sync_handle);
  if (handle > 0xeff)
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  if (handle >= BLE_LL_SYNC_CNT)
    {
      return BLE_ERR_UNK_ADV_INDENT;
    }

  sm = &g_ble_ll_sync_sm[handle];

  sr = enter_critical_section();

  if (!(sm->flags & BLE_LL_SYNC_SM_FLAG_ESTABLISHED))
    {
      leave_critical_section(sr);
      return BLE_ERR_UNK_ADV_INDENT;
    }

  ble_ll_sync_sm_clear(sm);

  leave_critical_section(sr);

  return BLE_ERR_SUCCESS;
}

/****************************************************************************
 * Name: ble_ll_sync_list_add
 *
 ****************************************************************************/

int ble_ll_sync_list_add(const uint8_t *cmdbuf, uint8_t len)
{
  const struct ble_hci_le_add_dev_to_periodic_adv_list_cp *cmd =
      (const void *)cmdbuf;
  int i;

  if (g_ble_ll_sync_create_comp_ev)
    {
      return BLE_ERR_CMD_DISALLOWED;
    }

  if (len != sizeof(*cmd))
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  if (cmd->peer_addr_type > BLE_HCI_ADV_PEER_ADDR_MAX)
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  if (cmd->sid > 0x0f)
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  i = ble_ll_sync_on_list(cmd->peer_addr, cmd->peer_addr_type, cmd->sid);
  if (i >= 0)
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  i = ble_ll_sync_list_get_free();
  if (i < 0)
    {
      return BLE_ERR_MEM_CAPACITY;
    }

  g_ble_ll_sync_adv_list[i].adv_sid = cmd->sid;
  g_ble_ll_sync_adv_list[i].adv_addr_type = cmd->peer_addr_type;
  memcpy(&g_ble_ll_sync_adv_list[i].adv_addr, cmd->peer_addr,
         BLE_DEV_ADDR_LEN);

  return BLE_ERR_SUCCESS;
}

/****************************************************************************
 * Name: ble_ll_sync_list_remove
 *
 ****************************************************************************/

int ble_ll_sync_list_remove(const uint8_t *cmdbuf, uint8_t len)
{
  const struct ble_hci_le_rem_dev_from_periodic_adv_list_cp *cmd =
      (const void *)cmdbuf;
  int i;

  if (len != sizeof(*cmd))
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  if (g_ble_ll_sync_create_comp_ev)
    {
      return BLE_ERR_CMD_DISALLOWED;
    }

  if (cmd->peer_addr_type > BLE_HCI_ADV_PEER_ADDR_MAX)
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  if (cmd->sid > 0x0f)
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  i = ble_ll_sync_on_list(cmd->peer_addr, cmd->peer_addr_type, cmd->sid);
  if (i < 0)
    {
      return BLE_ERR_UNK_ADV_INDENT;
    }

  memset(&g_ble_ll_sync_adv_list[i], 0, sizeof(g_ble_ll_sync_adv_list[i]));
  g_ble_ll_sync_adv_list[i].adv_sid = 0xff;

  return BLE_ERR_SUCCESS;
}

/****************************************************************************
 * Name: ble_ll_sync_list_clear
 *
 ****************************************************************************/

int ble_ll_sync_list_clear(void)
{
  unsigned int i;

  if (g_ble_ll_sync_create_comp_ev)
    {
      return BLE_ERR_CMD_DISALLOWED;
    }

  for (i = 0; i < nitems(g_ble_ll_sync_adv_list); i++)
    {
      memset(&g_ble_ll_sync_adv_list[i], 0,
             sizeof(g_ble_ll_sync_adv_list[i]));
      g_ble_ll_sync_adv_list[i].adv_sid = 0xff;
    }

  return BLE_ERR_SUCCESS;
}

/****************************************************************************
 * Name: ble_ll_sync_list_size
 *
 ****************************************************************************/

int ble_ll_sync_list_size(uint8_t *rspbuf, uint8_t *rsplen)
{
  struct ble_hci_le_rd_periodic_adv_list_size_rp *rsp = (void *)rspbuf;

  rsp->list_size = nitems(g_ble_ll_sync_adv_list);

  *rsplen = sizeof(*rsp);
  return BLE_ERR_SUCCESS;
}

#if CONFIG_BLE_LL_BT_VERSION >= 51
/****************************************************************************
 * Name: ble_ll_sync_receive_enable
 *
 ****************************************************************************/

int ble_ll_sync_receive_enable(const uint8_t *cmdbuf, uint8_t len)
{
  const struct ble_hci_le_periodic_adv_receive_enable_cp *cmd =
      (const void *)cmdbuf;
  struct ble_ll_sync_sm *sm;
  uint16_t handle;
  irqstate_t sr;

  if (len != sizeof(*cmd))
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  if (CONFIG_BLE_LL_BT_VERSION >= 53)
    {
      if (cmd->enable > 0x03)
        {
          return BLE_ERR_INV_HCI_CMD_PARMS;
        }
      else if ((cmd->enable == 0x03) &&
               !BLE_LL_PERIODIC_ADV_ADI_SUPPORT)
        {
          /* We do not support ADI in periodic advertising thus cannot enable
           * duplicate filtering.
           */

          return BLE_ERR_UNSUPPORTED;
        }
    }
  else if (cmd->enable > 0x01)
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  handle = le16toh(cmd->sync_handle);
  if (handle > 0xeff)
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  if (handle >= BLE_LL_SYNC_CNT)
    {
      return BLE_ERR_UNK_ADV_INDENT;
    }

  sm = &g_ble_ll_sync_sm[handle];

  sr = enter_critical_section();

  if (!(sm->flags & BLE_LL_SYNC_SM_FLAG_ESTABLISHED))
    {
      leave_critical_section(sr);
      return BLE_ERR_UNK_ADV_INDENT;
    }

  if (cmd->enable & 0x1)
    {
      sm->flags &= ~BLE_LL_SYNC_SM_FLAG_DISABLED;

      if (cmd->enable & 0x2)
        {
          sm->flags |= BLE_LL_SYNC_SM_FLAG_DUPLICATES;
        }
      else
        {
          sm->flags &= ~BLE_LL_SYNC_SM_FLAG_DUPLICATES;
        }
    }
  else
    {
      sm->flags |= BLE_LL_SYNC_SM_FLAG_DISABLED;
    }

  leave_critical_section(sr);
  return BLE_ERR_SUCCESS;
}
#endif /* CONFIG_BLE_LL_BT_VERSION >= 51 */

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV_SYNC_TRANSFER
/****************************************************************************
 * Name: ble_ll_sync_periodic_ind
 *
 ****************************************************************************/

void ble_ll_sync_periodic_ind(struct ble_ll_conn_sm *connsm,
                              const uint8_t *sync_ind, uint8_t mode,
                              uint16_t max_skip, uint32_t sync_timeout)
{
  const uint8_t *syncinfo = sync_ind + 2;
  uint16_t sync_conn_event_count;
  uint16_t last_pa_event_count;
  struct ble_ll_sync_sm *sm;
  uint16_t conn_event_count;
  uint8_t sync_anchor_usecs;
  const uint8_t *rpa = NULL;
  int last_pa_diff;
  uint32_t sync_anchor;
  const uint8_t *addr;
  uint16_t event_cntr;
  uint32_t itvl_usecs;
  uint32_t ww_adjust;
  uint8_t addr_type;
  uint8_t phy_mode;
  uint32_t offset;
  uint32_t future;
  uint16_t itvl;
  int rpa_index;
  uint8_t sid;
  uint8_t sca;
  irqstate_t sr;

  if (!mode)
    {
      return;
    }

  phy_mode = ble_ll_ctrl_phy_from_phy_mask(sync_ind[25]);
  itvl = get_le16(syncinfo + 2);

  /* ignore if sync params are not valid */

  if ((phy_mode == 0) || (itvl < 6))
    {
      return;
    }

  last_pa_event_count = get_le16(sync_ind + 22);
  event_cntr = get_le16(syncinfo + 16);
  itvl_usecs = itvl * BLE_LL_SYNC_ITVL_USECS;

  last_pa_diff = abs((int16_t)(event_cntr - last_pa_event_count));

  /* check if not 5 seconds apart, if so ignore sync transfer */

  if ((last_pa_diff * itvl_usecs) > 5000000)
    {
      return;
    }

  sid = (sync_ind[24] & 0x0f);
  addr_type = (sync_ind[24] & 0x10) ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
  addr = sync_ind + 26;

  rpa_index = -1;

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY

  /* check if need to resolve */

  if (ble_ll_is_rpa(addr, addr_type))
    {
      rpa_index = ble_ll_resolv_peer_rpa_any(addr);
      if (rpa_index >= 0)
        {
          rpa = addr;
          addr = g_ble_ll_resolv_list[rpa_index].rl_identity_addr;
          addr_type = g_ble_ll_resolv_list[rpa_index].rl_addr_type;
        }
    }
#endif

  sr = enter_critical_section();

  /* check if already synchronized with this peer */

  sm = ble_ll_sync_find(addr, addr_type, sid);
  if (sm)
    {
      leave_critical_section(sr);
      return;
    }

  /* ignore if no memory for new sync */

  sm = ble_ll_sync_transfer_get(addr, addr_type, sid);
  if (!sm)
    {
      leave_critical_section(sr);
      return;
    }

  leave_critical_section(sr);

  if (rpa_index >= 0)
    {
      sm->flags |= BLE_LL_SYNC_SM_FLAG_ADDR_RESOLVED;
      memcpy(sm->adv_addr_rpa, rpa, BLE_DEV_ADDR_LEN);
    }

  /* set params from transfer */

  sm->timeout = ble_ll_tmr_u2t(sync_timeout);
  sm->skip = max_skip;
  sm->sync_pending_cnt = BLE_LL_SYNC_ESTABLISH_CNT;
  sm->transfer_id = get_le16(sync_ind); /* first two bytes */
  sm->transfer_conn = connsm;

  /* Sync Packet Offset (13 bits), Offset Units (1 bit), Offset Adjust (1
   * bit), RFU (1 bit)
   */

  offset = syncinfo[0];
  offset |= (uint16_t)(syncinfo[1] & 0x1f) << 8;

  if (syncinfo[1] & 0x20)
    {
      if (syncinfo[1] & 0x40)
        {
          offset += 0x2000;
        }

      offset *= 300;
      sm->flags |= BLE_LL_SYNC_SM_FLAG_OFFSET_300;
    }
  else
    {
      offset *= 30;
      sm->flags &= ~BLE_LL_SYNC_SM_FLAG_OFFSET_300;
    }

  /* sync end event */

  ble_ll_event_init(&sm->sync_ev_end, ble_ll_sync_event_end, sm);

  sm->itvl = itvl;

  /* precalculate interval ticks and usecs */

  sm->itvl_ticks = ble_ll_tmr_u2t_r(itvl_usecs, &sm->itvl_usecs);

  /* Channels Mask (37 bits) */

  sm->chan_map[0] = syncinfo[4];
  sm->chan_map[1] = syncinfo[5];
  sm->chan_map[2] = syncinfo[6];
  sm->chan_map[3] = syncinfo[7];
  sm->chan_map[4] = syncinfo[8] & 0x1f;
  sm->chan_map_used = ble_ll_utils_chan_map_used_get(sm->chan_map);

  /* SCA (3 bits) */

  sm->sca = syncinfo[8] >> 5;

  /* AA (4 bytes) */

  sm->access_addr = get_le32(syncinfo + 9);
  sm->channel_id = ((sm->access_addr & 0xffff0000) >> 16) ^
                   (sm->access_addr & 0x0000ffff);

  /* CRCInit (3 bytes) */

  sm->crcinit = syncinfo[13];
  sm->crcinit |= syncinfo[14] << 8;
  sm->crcinit |= syncinfo[15] << 16;

  /* Event Counter (2 bytes) */

  sm->event_cntr = event_cntr;

  /* adjust skip if pass timeout */

  max_skip = get_max_skip(sm->itvl * BLE_LL_SYNC_ITVL_USECS, sync_timeout);
  if (sm->skip > max_skip)
    {
      sm->skip = max_skip;
    }

  sm->phy_mode = phy_mode;

  /* Calculate channel index of first event */

  sm->chan_index = ble_ll_utils_dci_csa2(sm->event_cntr, sm->channel_id,
                                         sm->chan_map_used, sm->chan_map);

  /* get anchor for specified conn event */

  conn_event_count = get_le16(sync_ind + 20);
  ble_ll_conn_anchor_event_cntr_get(
      connsm, conn_event_count, &sm->anchor_point, &sm->anchor_point_usecs);

  /* Set last anchor point */

  sm->last_anchor_point = sm->anchor_point - (last_pa_diff * sm->itvl_ticks);

  /* calculate extra window widening */

  sync_conn_event_count = get_le16(sync_ind + 32);
  sca = sync_ind[24] >> 5;
  ble_ll_conn_anchor_event_cntr_get(connsm, sync_conn_event_count,
                                    &sync_anchor, &sync_anchor_usecs);
  ww_adjust = ble_ll_utils_calc_window_widening(connsm->anchor_point,
                                                sync_anchor, sca);

  /* spin until we get anchor in future */

  future = ble_ll_tmr_get() + g_ble_ll_sched_offset_ticks;
  while (LL_TMR_LT(sm->anchor_point, future))
    {
      if (ble_ll_sync_next_event(sm, ww_adjust) < 0)
        {
          /* release SM if this failed */

          ble_ll_sync_transfer_received(sm, BLE_ERR_CONN_ESTABLISHMENT);
          memset(sm, 0, sizeof(*sm));
          return;
        }
    }

  ble_ll_sync_sched_set(&sm->sch, sm->anchor_point, sm->anchor_point_usecs,
                        offset, sm->phy_mode);

  if (ble_ll_sched_sync(&sm->sch))
    {
      /* release SM if this failed */

      ble_ll_sync_transfer_received(sm, BLE_ERR_CONN_ESTABLISHMENT);
      memset(sm, 0, sizeof(*sm));
      return;
    }

  /* Set new anchor point */

  sm->anchor_point = sm->sch.start_time + g_ble_ll_sched_offset_ticks;
  sm->anchor_point_usecs = sm->sch.remainder;

  switch (mode)
    {
      case 0x1:
        sm->flags |= BLE_LL_SYNC_SM_FLAG_DISABLED;
        break;
      case 0x3:
        sm->flags |= BLE_LL_SYNC_SM_FLAG_DUPLICATES;
        break;
      default:
        break;
    }
}

/****************************************************************************
 * Name: ble_ll_sync_transfer
 *
 ****************************************************************************/

int ble_ll_sync_transfer(const uint8_t *cmdbuf, uint8_t len, uint8_t *rspbuf,
                         uint8_t *rsplen)
{
  const struct ble_hci_le_periodic_adv_sync_transfer_cp *cmd =
      (const void *)cmdbuf;
  struct ble_hci_le_periodic_adv_sync_transfer_rp *rsp = (void *)rspbuf;
  struct ble_ll_conn_sm *connsm;
  struct ble_ll_sync_sm *sm;
  uint16_t handle;
  irqstate_t sr;
  int rc;

  if (len != sizeof(*cmd))
    {
      rc = BLE_ERR_INV_HCI_CMD_PARMS;
      goto done;
    }

  handle = le16toh(cmd->sync_handle);
  if (handle > 0xeff)
    {
      rc = BLE_ERR_INV_HCI_CMD_PARMS;
      goto done;
    }

  if (handle >= BLE_LL_SYNC_CNT)
    {
      rc = BLE_ERR_UNK_ADV_INDENT;
      goto done;
    }

  sm = &g_ble_ll_sync_sm[handle];

  sr = enter_critical_section();

  if (!(sm->flags & BLE_LL_SYNC_SM_FLAG_ESTABLISHED))
    {
      rc = BLE_ERR_UNK_ADV_INDENT;
      goto exit_crit;
    }

  handle = le16toh(cmd->conn_handle);
  if (handle > 0xeff)
    {
      rc = BLE_ERR_INV_HCI_CMD_PARMS;
      goto exit_crit;
    }

  connsm = ble_ll_conn_find_by_handle(handle);
  if (!connsm)
    {
      rc = BLE_ERR_UNK_CONN_ID;
      goto exit_crit;
    }

  /* Allow initiate LL procedure only if remote supports it. */

  if (!ble_ll_conn_rem_feature_check(connsm, BLE_LL_FEAT_SYNC_TRANS_RECV))
    {
      rc = BLE_ERR_UNSUPP_REM_FEATURE;
      goto exit_crit;
    }

  rc = ble_ll_sync_send_sync_ind(sm, connsm, cmd->service_data);

exit_crit:
  leave_critical_section(sr);
done:
  rsp->conn_handle = cmd->conn_handle;
  *rsplen = sizeof(*rsp);
  return rc;
}
#endif

/****************************************************************************
 * Name: ble_ll_sync_rmvd_from_sched
 *
 * Description:
 *   Called when a sync scan event has been removed from the scheduler
 *   without being run.
 *
 ****************************************************************************/

void ble_ll_sync_rmvd_from_sched(struct ble_ll_sync_sm *sm)
{
  ble_ll_event_add(&sm->sync_ev_end);
}

/****************************************************************************
 * Name: ble_ll_sync_enabled
 *
 ****************************************************************************/

bool ble_ll_sync_enabled(void)
{
  return g_ble_ll_sync_create_comp_ev != NULL;
}

/****************************************************************************
 * Name: ble_ll_sync_reset
 *
 * Description:
 *   Called to reset the sync module. When this function is called the
 *   scheduler has been stopped and the phy has been disabled. The LL should
 *   be in the standby state.
 *
 ****************************************************************************/

void ble_ll_sync_reset(void)
{
  unsigned int i;

  for (i = 0; i < BLE_LL_SYNC_CNT; i++)
    {
      ble_ll_sync_sm_clear(&g_ble_ll_sync_sm[i]);
    }

  for (i = 0; i < nitems(g_ble_ll_sync_adv_list); i++)
    {
      memset(&g_ble_ll_sync_adv_list[i], 0,
             sizeof(g_ble_ll_sync_adv_list[i]));
      g_ble_ll_sync_adv_list[i].adv_sid = 0xff;
    }

  g_ble_ll_sync_create_params.timeout = 0;
  g_ble_ll_sync_create_params.max_skip = 0;
  g_ble_ll_sync_create_params.options = 0;

  g_ble_ll_sync_sm_current = NULL;

  if (g_ble_ll_sync_create_comp_ev)
    {
      ble_ll_hci_buf_free(g_ble_ll_sync_create_comp_ev);
      g_ble_ll_sync_create_comp_ev = NULL;
    }
}

/****************************************************************************
 * Name: ble_ll_sync_init
 *
 ****************************************************************************/

void ble_ll_sync_init(void)
{
  unsigned int i;

  for (i = 0; i < nitems(g_ble_ll_sync_adv_list); i++)
    {
      g_ble_ll_sync_adv_list[i].adv_sid = 0xff;
    }
}

#endif
