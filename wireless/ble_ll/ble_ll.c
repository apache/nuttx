/****************************************************************************
 * wireless/ble_ll/ble_ll.c
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
 * Derived from Apache Mynewt NimBLE (nimble/controller/src/ble_ll.c).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "ble.h"
#include "hci_common.h"
#include "ble_ll_utils.h"
#include "ble_hw.h"
#include "ble_phy.h"
#include "ble_ll.h"
#include "ble_ll_pdu.h"
#include "ble_ll_adv.h"
#include "ble_ll_sched.h"
#include "ble_ll_scan.h"
#include "ble_ll_scan_aux.h"
#include "ble_ll_hci.h"
#include "ble_ll_whitelist.h"
#include "ble_ll_resolv.h"
#include "ble_ll_rfmgmt.h"
#include "ble_ll_sync.h"
#include "ble_ll_conn_priv.h"
#include "ble_ll_hci_priv.h"
#include "ble_ll_priv.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_BLE_LL_ROLE_BROADCASTER

/* Supported states */

#define BLE_LL_S_NCA ((uint64_t)1 << 0)
#define BLE_LL_S_SA  ((uint64_t)1 << 1)
#else
#define BLE_LL_S_NCA ((uint64_t)0 << 0)
#define BLE_LL_S_SA  ((uint64_t)0 << 1)
#endif

#ifdef CONFIG_BLE_LL_ROLE_PERIPHERAL
#define BLE_LL_S_CA   ((uint64_t)1 << 2)
#define BLE_LL_S_HDCA ((uint64_t)1 << 3)
#else
#define BLE_LL_S_CA   ((uint64_t)0 << 2)
#define BLE_LL_S_HDCA ((uint64_t)0 << 3)
#endif

#ifdef CONFIG_BLE_LL_ROLE_OBSERVER
#define BLE_LL_S_PS ((uint64_t)1 << 4)
#define BLE_LL_S_AS ((uint64_t)1 << 5)
#else
#define BLE_LL_S_PS ((uint64_t)0 << 4)
#define BLE_LL_S_AS ((uint64_t)0 << 5)
#endif

#ifdef CONFIG_BLE_LL_ROLE_CENTRAL
#define BLE_LL_S_INIT ((uint64_t)1 << 6)
#else
#define BLE_LL_S_INIT ((uint64_t)0 << 6)
#endif

#ifdef CONFIG_BLE_LL_ROLE_PERIPHERAL
#define BLE_LL_S_PERIPH ((uint64_t)1 << 7)
#else
#define BLE_LL_S_PERIPH ((uint64_t)0 << 7)
#endif

#if defined(CONFIG_BLE_LL_ROLE_BROADCASTER) && \
    defined(CONFIG_BLE_LL_ROLE_OBSERVER)
#define BLE_LL_S_NCA_PS ((uint64_t)1 << 8)
#define BLE_LL_S_SA_PS  ((uint64_t)1 << 9)
#else
#define BLE_LL_S_NCA_PS ((uint64_t)0 << 8)
#define BLE_LL_S_SA_PS  ((uint64_t)0 << 9)
#endif

#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) && \
    defined(CONFIG_BLE_LL_ROLE_OBSERVER)
#define BLE_LL_S_CA_PS   ((uint64_t)1 << 10)
#define BLE_LL_S_HDCA_PS ((uint64_t)1 << 11)
#else
#define BLE_LL_S_CA_PS   ((uint64_t)0 << 10)
#define BLE_LL_S_HDCA_PS ((uint64_t)0 << 11)
#endif

#if defined(CONFIG_BLE_LL_ROLE_BROADCASTER) && \
    defined(CONFIG_BLE_LL_ROLE_OBSERVER)
#define BLE_LL_S_NCA_AS ((uint64_t)1 << 12)
#define BLE_LL_S_SA_AS  ((uint64_t)1 << 13)
#else
#define BLE_LL_S_NCA_AS ((uint64_t)0 << 12)
#define BLE_LL_S_SA_AS  ((uint64_t)0 << 13)
#endif

#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) && \
    defined(CONFIG_BLE_LL_ROLE_OBSERVER)
#define BLE_LL_S_CA_AS   ((uint64_t)1 << 14)
#define BLE_LL_S_HDCA_AS ((uint64_t)1 << 15)
#else
#define BLE_LL_S_CA_AS   ((uint64_t)0 << 14)
#define BLE_LL_S_HDCA_AS ((uint64_t)0 << 15)
#endif

#if defined(CONFIG_BLE_LL_ROLE_BROADCASTER) && \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)
#define BLE_LL_S_NCA_INIT    ((uint64_t)1 << 16)
#define BLE_LL_S_SA_INIT     ((uint64_t)1 << 17)
#define BLE_LL_S_NCA_CENTRAL ((uint64_t)1 << 18)
#define BLE_LL_S_SA_CENTRAL  ((uint64_t)1 << 19)
#else
#define BLE_LL_S_NCA_INIT    ((uint64_t)0 << 16)
#define BLE_LL_S_SA_INIT     ((uint64_t)0 << 17)
#define BLE_LL_S_NCA_CENTRAL ((uint64_t)0 << 18)
#define BLE_LL_S_SA_CENTRAL  ((uint64_t)0 << 19)
#endif

#if defined(CONFIG_BLE_LL_ROLE_BROADCASTER) && \
    defined(CONFIG_BLE_LL_ROLE_PERIPHERAL)
#define BLE_LL_S_NCA_PERIPH ((uint64_t)1 << 20)
#define BLE_LL_S_SA_PERIPH  ((uint64_t)1 << 21)
#else
#define BLE_LL_S_NCA_PERIPH ((uint64_t)0 << 20)
#define BLE_LL_S_SA_PERIPH  ((uint64_t)0 << 21)
#endif

#if defined(CONFIG_BLE_LL_ROLE_OBSERVER) && \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)

/* We do not support passive scanning while initiating yet */

#define BLE_LL_S_PS_INIT ((uint64_t)0 << 22)

/* We do not support active scanning while initiating yet */

#define BLE_LL_S_AS_INIT    ((uint64_t)0 << 23)
#define BLE_LL_S_PS_CENTRAL ((uint64_t)1 << 24)
#define BLE_LL_S_AS_CENTRAL ((uint64_t)1 << 25)
#else
#define BLE_LL_S_PS_INIT    ((uint64_t)0 << 22)
#define BLE_LL_S_AS_INIT    ((uint64_t)0 << 23)
#define BLE_LL_S_PS_CENTRAL ((uint64_t)0 << 24)
#define BLE_LL_S_AS_CENTRAL ((uint64_t)0 << 25)
#endif

#if defined(CONFIG_BLE_LL_ROLE_OBSERVER) && \
    defined(CONFIG_BLE_LL_ROLE_PERIPHERAL)
#define BLE_LL_S_PS_PERIPH ((uint64_t)1 << 26)
#define BLE_LL_S_AS_PERIPH ((uint64_t)1 << 27)
#else
#define BLE_LL_S_PS_PERIPH ((uint64_t)0 << 26)
#define BLE_LL_S_AS_PERIPH ((uint64_t)0 << 27)
#endif

#ifdef CONFIG_BLE_LL_ROLE_CENTRAL
#define BLE_LL_S_INIT_CENTRAL ((uint64_t)1 << 28)
#else
#define BLE_LL_S_INIT_CENTRAL ((uint64_t)0 << 28)
#endif

#ifdef CONFIG_BLE_LL_ROLE_PERIPHERAL
#define BLE_LL_S_LDCA ((uint64_t)1 << 29)
#else
#define BLE_LL_S_LDCA ((uint64_t)0 << 29)
#endif

#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) && \
    defined(CONFIG_BLE_LL_ROLE_OBSERVER)
#define BLE_LL_S_LDCA_PS ((uint64_t)1 << 30)
#define BLE_LL_S_LDCA_AS ((uint64_t)1 << 31)
#else
#define BLE_LL_S_LDCA_PS ((uint64_t)0 << 30)
#define BLE_LL_S_LDCA_AS ((uint64_t)0 << 31)
#endif

#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) && \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)
#define BLE_LL_S_CA_INIT      ((uint64_t)1 << 32)
#define BLE_LL_S_HDCA_INIT    ((uint64_t)1 << 33)
#define BLE_LL_S_LDCA_INIT    ((uint64_t)1 << 34)
#define BLE_LL_S_CA_CENTRAL   ((uint64_t)1 << 35)
#define BLE_LL_S_HDCA_CENTRAL ((uint64_t)1 << 36)
#define BLE_LL_S_LDCA_CENTRAL ((uint64_t)1 << 37)
#else
#define BLE_LL_S_CA_INIT      ((uint64_t)0 << 32)
#define BLE_LL_S_HDCA_INIT    ((uint64_t)0 << 33)
#define BLE_LL_S_LDCA_INIT    ((uint64_t)0 << 34)
#define BLE_LL_S_CA_CENTRAL   ((uint64_t)0 << 35)
#define BLE_LL_S_HDCA_CENTRAL ((uint64_t)0 << 36)
#define BLE_LL_S_LDCA_CENTRAL ((uint64_t)0 << 37)
#endif

#ifdef CONFIG_BLE_LL_ROLE_PERIPHERAL
#define BLE_LL_S_CA_PERIPH   ((uint64_t)1 << 38)
#define BLE_LL_S_HDCA_PERIPH ((uint64_t)1 << 39)
#define BLE_LL_S_LDCA_PERIPH ((uint64_t)1 << 40)
#else
#define BLE_LL_S_CA_PERIPH   ((uint64_t)0 << 38)
#define BLE_LL_S_HDCA_PERIPH ((uint64_t)0 << 39)
#define BLE_LL_S_LDCA_PERIPH ((uint64_t)0 << 40)
#endif

#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) && \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)
#define BLE_LL_S_INIT_PERIPH ((uint64_t)1 << 41)
#else
#define BLE_LL_S_INIT_PERIPH ((uint64_t)0 << 41)
#endif

#define BLE_LL_SUPPORTED_STATES                                             \
  (BLE_LL_S_NCA | BLE_LL_S_SA | BLE_LL_S_CA | BLE_LL_S_HDCA | BLE_LL_S_PS | \
   BLE_LL_S_AS | BLE_LL_S_INIT | BLE_LL_S_PERIPH | BLE_LL_S_NCA_PS |        \
   BLE_LL_S_SA_PS | BLE_LL_S_CA_PS | BLE_LL_S_HDCA_PS | BLE_LL_S_NCA_AS |   \
   BLE_LL_S_SA_AS | BLE_LL_S_CA_AS | BLE_LL_S_HDCA_AS | BLE_LL_S_NCA_INIT | \
   BLE_LL_S_SA_INIT | BLE_LL_S_NCA_CENTRAL | BLE_LL_S_SA_CENTRAL |          \
   BLE_LL_S_NCA_PERIPH | BLE_LL_S_SA_PERIPH | BLE_LL_S_PS_INIT |            \
   BLE_LL_S_AS_INIT | BLE_LL_S_PS_CENTRAL | BLE_LL_S_AS_CENTRAL |           \
   BLE_LL_S_PS_PERIPH | BLE_LL_S_AS_PERIPH | BLE_LL_S_INIT_CENTRAL |        \
   BLE_LL_S_LDCA | BLE_LL_S_LDCA_PS | BLE_LL_S_LDCA_AS | BLE_LL_S_CA_INIT | \
   BLE_LL_S_HDCA_INIT | BLE_LL_S_LDCA_INIT | BLE_LL_S_CA_CENTRAL |          \
   BLE_LL_S_HDCA_CENTRAL | BLE_LL_S_LDCA_CENTRAL | BLE_LL_S_CA_PERIPH |     \
   BLE_LL_S_HDCA_PERIPH | BLE_LL_S_LDCA_PERIPH | BLE_LL_S_INIT_PERIPH)

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void ble_ll_event_rx_pkt(FAR void *arg);
#if defined(CONFIG_BLE_LL_ROLE_CENTRAL) || \
    defined(CONFIG_BLE_LL_ROLE_PERIPHERAL)
static void ble_ll_event_tx_pkt(FAR void *arg);

static void ble_ll_event_dbuf_overflow(FAR void *arg);

#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int8_t g_ble_ll_tx_power_phy_current;
#if BLE_LL_HOST_CONTROLLED_FEATURES
static const uint64_t g_ble_ll_host_controlled_features =
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_ENHANCED_CONN_UPDATE
    BLE_LL_FEAT_CONN_SUBRATING_HOST |
#endif
#ifdef CONFIG_BLE_LL_ADV_CODING_SELECTION
    BLE_LL_FEAT_ADV_CODING_SEL_HOST |
#endif
    0;

#endif /* BLE_LL_HOST_CONTROLLED_FEATURES */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* XXX:
 *
 * 1) use the sanity task!
 * 2) Need to figure out what to do with packets that we hand up that did
 * not pass the filter policy for the given state. Currently I count all
 * packets I think. Need to figure out what to do with this.
 * 3) For the features defined, we need to conditionally compile code.
 * 4) Should look into always disabled the wfr interrupt if we receive the
 * start of a frame. Need to look at the various states to see if this is the
 * right thing to do.
 */

/* This is TX power on PHY (or FEM PA if enabled) */

int8_t g_ble_ll_tx_power;

int8_t g_ble_ll_tx_power_compensation;

int8_t g_ble_ll_rx_power_compensation;

/* The global BLE LL data object */

struct ble_ll_obj g_ble_ll_data;

/* Global link layer statistics */

struct ble_ll_stats ble_ll_stats;

/* Our global device address (public) */

uint8_t g_dev_addr[BLE_DEV_ADDR_LEN];

/* Our random address */

uint8_t g_random_addr[BLE_DEV_ADDR_LEN];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_ll_count_rx_adv_pdus
 *
 * Description:
 *   Counts the number of advertising PDU's received, by type. For
 * advertising PDU's that contain a destination address, we still count these
 * packets even if they are not for us.
 *
 * Input Parameters:
 *   pdu_type
 *
 ****************************************************************************/

static void ble_ll_count_rx_adv_pdus(uint8_t pdu_type)
{
  /* Count received packet types */

  switch (pdu_type)
    {
      case BLE_ADV_PDU_TYPE_ADV_EXT_IND:
        BLE_LL_STATS_INC(ble_ll_stats, rx_adv_ext_ind);
        break;
      case BLE_ADV_PDU_TYPE_ADV_IND:
        BLE_LL_STATS_INC(ble_ll_stats, rx_adv_ind);
        break;
      case BLE_ADV_PDU_TYPE_ADV_DIRECT_IND:
        BLE_LL_STATS_INC(ble_ll_stats, rx_adv_direct_ind);
        break;
      case BLE_ADV_PDU_TYPE_ADV_NONCONN_IND:
        BLE_LL_STATS_INC(ble_ll_stats, rx_adv_nonconn_ind);
        break;
      case BLE_ADV_PDU_TYPE_SCAN_REQ:
        BLE_LL_STATS_INC(ble_ll_stats, rx_scan_reqs);
        break;
      case BLE_ADV_PDU_TYPE_SCAN_RSP:
        BLE_LL_STATS_INC(ble_ll_stats, rx_scan_rsps);
        break;
      case BLE_ADV_PDU_TYPE_CONNECT_IND:
        BLE_LL_STATS_INC(ble_ll_stats, rx_connect_reqs);
        break;
      case BLE_ADV_PDU_TYPE_AUX_CONNECT_RSP:
        BLE_LL_STATS_INC(ble_ll_stats, rx_aux_connect_rsp);
        break;
      case BLE_ADV_PDU_TYPE_ADV_SCAN_IND:
        BLE_LL_STATS_INC(ble_ll_stats, rx_scan_ind);
        break;
      default:
        break;
    }
}

/****************************************************************************
 * Name: ble_ll_is_valid_addr
 *
 ****************************************************************************/

static int ble_ll_is_valid_addr(const uint8_t *addr)
{
  int i;

  for (i = 0; i < BLE_DEV_ADDR_LEN; ++i)
    {
      if (addr[i])
        {
          return 1;
        }
    }

  return 0;
}

#if defined(CONFIG_BLE_LL_ROLE_CENTRAL) || \
    defined(CONFIG_BLE_LL_ROLE_PERIPHERAL)
/****************************************************************************
 * Name: ble_ll_tx_pkt_in
 *
 * Description:
 *   ll tx pkt in proc
 *
 *   Process ACL data packet input from host
 *
 *   Context: Link layer task
 *
 ****************************************************************************/

static void ble_ll_tx_pkt_in(void)
{
  uint16_t handle;
  uint16_t length;
  uint16_t pb;
  struct ble_ll_pkthdr_s *pkthdr;
  struct iob_s *om;
  irqstate_t sr;

  /* Drain all packets off the queue */

  while (STAILQ_FIRST(&g_ble_ll_data.ll_tx_pkt_q))
    {
      /* Get mbuf pointer from packet header pointer */

      pkthdr = STAILQ_FIRST(&g_ble_ll_data.ll_tx_pkt_q);
      om = pkthdr->iob;

      /* Remove from queue */

      sr = enter_critical_section();
      STAILQ_REMOVE_HEAD(&g_ble_ll_data.ll_tx_pkt_q, next);
      leave_critical_section(sr);

      /* Strip HCI ACL header to get handle and length */

      handle = get_le16(IOB_DATA(om));
      length = get_le16(IOB_DATA(om) + 2);
      ble_ll_iob_adj(om, sizeof(struct hci_data_hdr));

      /* Do some basic error checking */

      pb = handle & 0x3000;
      if ((pkthdr->iob->io_pktlen != length) || (pb > 0x1000))
        {
          /* This is a bad ACL packet. Count a stat and free it */

          BLE_LL_STATS_INC(ble_ll_stats, bad_acl_hdr);
          iob_free_chain(om);
          continue;
        }

      /* Hand to connection state machine */

      ble_ll_conn_tx_pkt_in(om, handle, length);
    }
}

#endif

/****************************************************************************
 * Name: ble_ll_count_rx_stats
 *
 * Description:
 *   Count Link Layer statistics for received PDUs
 *
 *   Context: Link layer task
 *
 * Input Parameters:
 *   hdr
 *   len
 *
 ****************************************************************************/

static void ble_ll_count_rx_stats(struct ble_mbuf_hdr *hdr, uint16_t len,
                                  uint8_t pdu_type)
{
  uint8_t crcok;
  bool connection_data;

  crcok = BLE_MBUF_HDR_CRC_OK(hdr);

#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) || \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)
  connection_data = (BLE_MBUF_HDR_RX_STATE(hdr) == BLE_LL_STATE_CONNECTION);
#else
  connection_data = false;
#endif

  if (crcok)
    {
      if (connection_data)
        {
          BLE_LL_STATS_INC(ble_ll_stats, rx_data_pdu_crc_ok);
          BLE_LL_STATS_INCN(ble_ll_stats, rx_data_bytes_crc_ok, len);
        }
      else
        {
          BLE_LL_STATS_INC(ble_ll_stats, rx_adv_pdu_crc_ok);
          BLE_LL_STATS_INCN(ble_ll_stats, rx_adv_bytes_crc_ok, len);
          ble_ll_count_rx_adv_pdus(pdu_type);
        }
    }
  else
    {
      if (connection_data)
        {
          BLE_LL_STATS_INC(ble_ll_stats, rx_data_pdu_crc_err);
          BLE_LL_STATS_INCN(ble_ll_stats, rx_data_bytes_crc_err, len);
        }
      else
        {
          BLE_LL_STATS_INC(ble_ll_stats, rx_adv_pdu_crc_err);
          BLE_LL_STATS_INCN(ble_ll_stats, rx_adv_bytes_crc_err, len);
        }
    }
}

/****************************************************************************
 * Name: ble_ll_rx_pkt_in
 *
 * Description:
 *   ll rx pkt in
 *
 *   Process received packet from PHY.
 *
 *   Context: Link layer task
 *
 ****************************************************************************/

static void ble_ll_rx_pkt_in(void)
{
  irqstate_t sr;
  uint8_t pdu_type;
  uint8_t *rxbuf;
  struct ble_ll_pkthdr_s *pkthdr;
  struct ble_mbuf_hdr *ble_hdr;
  struct iob_s *m;

  /* Drain all packets off the queue */

  while (STAILQ_FIRST(&g_ble_ll_data.ll_rx_pkt_q))
    {
      /* Get mbuf pointer from packet header pointer */

      pkthdr = STAILQ_FIRST(&g_ble_ll_data.ll_rx_pkt_q);
      m = pkthdr->iob;

      /* Remove from queue */

      sr = enter_critical_section();
      STAILQ_REMOVE_HEAD(&g_ble_ll_data.ll_rx_pkt_q, next);
      leave_critical_section(sr);

      /* Note: pdu type won't get used unless this is an advertising pdu */

      ble_hdr = BLE_MBUF_HDR_PTR(m);
      rxbuf = IOB_DATA(m);
      pdu_type = rxbuf[0] & BLE_ADV_PDU_HDR_TYPE_MASK;
      ble_ll_count_rx_stats(ble_hdr, pkthdr->iob->io_pktlen, pdu_type);

      /* Process the data or advertising pdu */

      /* Process the PDU */

      switch (BLE_MBUF_HDR_RX_STATE(ble_hdr))
        {
#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) || \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)
          case BLE_LL_STATE_CONNECTION:
            ble_ll_conn_rx_data_pdu(m, ble_hdr);

            /* m is going to be free by function above */

            m = NULL;
            break;
#endif
#ifdef CONFIG_BLE_LL_ROLE_BROADCASTER
          case BLE_LL_STATE_ADV:
            ble_ll_adv_rx_pkt_in(pdu_type, rxbuf, ble_hdr);
            break;
#endif
#ifdef CONFIG_BLE_LL_ROLE_OBSERVER
          case BLE_LL_STATE_SCANNING:
            ble_ll_scan_rx_pkt_in(pdu_type, m, ble_hdr);
            break;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV
          case BLE_LL_STATE_SYNC:
            ble_ll_sync_rx_pkt_in(m, ble_hdr);
            break;
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV
          case BLE_LL_STATE_SCAN_AUX:
            ble_ll_scan_aux_rx_pkt_in(m, ble_hdr);
            break;
#endif
#endif
          default:

            /* Any other state should never occur */

            BLE_LL_STATS_INC(ble_ll_stats, bad_ll_state);
            break;
        }

      if (m)
        {
          /* Free the packet buffer */

          iob_free_chain(m);
        }
    }
}

/****************************************************************************
 * Name: ble_ll_hw_err_timer_cb
 *
 * Description:
 *   Called when the HW error timer expires.
 *
 * Input Parameters:
 *   arg
 *
 ****************************************************************************/

static void ble_ll_hw_err_timer_cb(FAR void *arg)
{
  if (ble_ll_hci_ev_hw_err(BLE_HW_ERR_HCI_SYNC_LOSS))
    {
      /* Restart callout if failed to allocate event. Try to allocate an
       * event every 50 milliseconds (or each OS tick if a tick is longer
       * than 100 msecs).
       */

      ble_ll_callout_reset(&g_ble_ll_data.ll_hw_err_timer,
                            MSEC2TICK(50));
    }
}

/****************************************************************************
 * Name: ble_ll_event_rx_pkt
 *
 ****************************************************************************/

static void ble_ll_event_rx_pkt(FAR void *arg)
{
  ble_ll_rx_pkt_in();
}

#if defined(CONFIG_BLE_LL_ROLE_CENTRAL) || \
    defined(CONFIG_BLE_LL_ROLE_PERIPHERAL)
/****************************************************************************
 * Name: ble_ll_event_tx_pkt
 *
 ****************************************************************************/

static void ble_ll_event_tx_pkt(FAR void *arg)
{
  ble_ll_tx_pkt_in();
}

/****************************************************************************
 * Name: ble_ll_event_dbuf_overflow
 *
 ****************************************************************************/

static void ble_ll_event_dbuf_overflow(FAR void *arg)
{
  ble_ll_hci_ev_databuf_overflow();
}

/****************************************************************************
 * Name: ble_ll_event_comp_pkts
 *
 ****************************************************************************/

static void ble_ll_event_comp_pkts(FAR void *arg)
{
  ble_ll_conn_num_comp_pkts_event_send(NULL);
}

#endif

/****************************************************************************
 * Name: ble_ll_flush_pkt_queue
 *
 * Description:
 *   Flush a link layer packet queue.
 *
 * Input Parameters:
 *   pktq
 *
 ****************************************************************************/

static void ble_ll_flush_pkt_queue(struct ble_ll_pkt_q *pktq)
{
  struct ble_ll_pkthdr_s *pkthdr;
  struct iob_s *om;

  /* FLush all packets from Link layer queues */

  while (STAILQ_FIRST(pktq))
    {
      /* Get mbuf pointer from packet header pointer */

      pkthdr = STAILQ_FIRST(pktq);
      om = pkthdr->iob;

      /* Remove from queue and free the mbuf */

      STAILQ_REMOVE_HEAD(pktq, next);
      iob_free_chain(om);
    }
}

/****************************************************************************
 * Name: ble_ll_is_addr_empty
 *
 ****************************************************************************/

static inline bool ble_ll_is_addr_empty(const uint8_t *addr)
{
  return memcmp(addr, BLE_ADDR_ANY, BLE_DEV_ADDR_LEN) == 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_ll_is_rpa
 *
 * Description:
 *   Checks to see if the address is a resolvable private address.
 *
 *   NOTE: the addr_type parameter will be 0 if the address is public;
 *   any other value is random (all non-zero values).
 *
 * Input Parameters:
 *   addr
 *   addr_type - Public (zero) or Random (non-zero) address
 *
 * Returned Value:
 *   int
 *
 ****************************************************************************/

int ble_ll_is_rpa(const uint8_t *addr, uint8_t addr_type)
{
  int rc;

  if (addr_type && ((addr[5] & 0xc0) == 0x40))
    {
      rc = 1;
    }
  else
    {
      rc = 0;
    }

  return rc;
}

/****************************************************************************
 * Name: ble_ll_addr_is_id
 *
 ****************************************************************************/

int ble_ll_addr_is_id(uint8_t *addr, uint8_t addr_type)
{
  return !addr_type || ((addr[5] & 0xc0) == 0xc0);
}

/****************************************************************************
 * Name: ble_ll_addr_subtype
 *
 ****************************************************************************/

int ble_ll_addr_subtype(const uint8_t *addr, uint8_t addr_type)
{
  if (!addr_type)
    {
      return BLE_LL_ADDR_SUBTYPE_IDENTITY;
    }

  switch (addr[5] >> 6)
    {
      case 0:
        return BLE_LL_ADDR_SUBTYPE_NRPA; /* NRPA */
      case 1:
        return BLE_LL_ADDR_SUBTYPE_RPA; /* RPA */
      default:
        return BLE_LL_ADDR_SUBTYPE_IDENTITY; /* static random */
    }
}

/****************************************************************************
 * Name: ble_ll_is_valid_random_addr
 *
 * Description:
 *   Checks to see that the device is a valid random address
 *
 ****************************************************************************/

int ble_ll_is_valid_random_addr(const uint8_t *addr)
{
  int i;
  int rc;
  uint16_t sum;
  uint8_t addr_type;

  /* Make sure all bits are neither one nor zero */

  sum = 0;
  for (i = 0; i < (BLE_DEV_ADDR_LEN - 1); ++i)
    {
      sum += addr[i];
    }

  sum += addr[5] & 0x3f;

  if ((sum == 0) || (sum == ((5 * 255) + 0x3f)))
    {
      return 0;
    }

  /* Get the upper two bits of the address */

  rc = 1;
  addr_type = addr[5] & 0xc0;
  if (addr_type == 0xc0)
    {
      /* Static random address. No other checks needed */
    }
  else if (addr_type == 0x40)
    {
      /* Resolvable */

      sum = addr[3] + addr[4] + (addr[5] & 0x3f);
      if ((sum == 0) || (sum == (255 + 255 + 0x3f)))
        {
          rc = 0;
        }
    }
  else if (addr_type == 0)
    {
      /* non-resolvable. Can't be equal to public */

      if (!memcmp(g_dev_addr, addr, BLE_DEV_ADDR_LEN))
        {
          rc = 0;
        }
    }
  else
    {
      /* Invalid upper two bits */

      rc = 0;
    }

  return rc;
}

/****************************************************************************
 * Name: ble_ll_is_valid_own_addr_type
 *
 ****************************************************************************/

int ble_ll_is_valid_own_addr_type(uint8_t own_addr_type,
                                  const uint8_t *random_addr)
{
  int rc;

  switch (own_addr_type)
    {
      case BLE_HCI_ADV_OWN_ADDR_PUBLIC:
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
      case BLE_HCI_ADV_OWN_ADDR_PRIV_PUB:
#endif
        rc = ble_ll_is_valid_addr(g_dev_addr);
        break;
      case BLE_HCI_ADV_OWN_ADDR_RANDOM:
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
      case BLE_HCI_ADV_OWN_ADDR_PRIV_RAND:
#endif
        rc = ble_ll_is_valid_addr(random_addr);
        break;
      default:
        rc = 0;
        break;
    }

  return rc;
}

/****************************************************************************
 * Name: ble_ll_set_public_addr
 *
 ****************************************************************************/

int ble_ll_set_public_addr(const uint8_t *addr)
{
  memcpy(g_dev_addr, addr, BLE_DEV_ADDR_LEN);

  return BLE_ERR_SUCCESS;
}

/****************************************************************************
 * Name: ble_ll_set_random_addr
 *
 * Description:
 *   Called from the HCI command parser when the set random address command
 *   is received.
 *
 *   Context: Link Layer task (HCI command parser)
 *
 * Input Parameters:
 *   addr - Pointer to address
 *
 * Returned Value:
 *   int 0: success
 *
 ****************************************************************************/

int ble_ll_set_random_addr(const uint8_t *cmdbuf, uint8_t len,
                           bool hci_adv_ext)
{
  const struct ble_hci_le_set_rand_addr_cp *cmd = (const void *)cmdbuf;

  if (len < sizeof(*cmd))
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

#ifdef CONFIG_BLE_LL_ROLE_CENTRAL
  /* If the Host issues this command when scanning or legacy advertising is
   * enabled, the Controller shall return the error code Command
   * Disallowed.
   *
   * Test specification extends this also to initiating.
   */

  if (g_ble_ll_conn_create_sm.connsm)
    {
      return BLE_ERR_CMD_DISALLOWED;
    }
#endif

#ifdef CONFIG_BLE_LL_ROLE_OBSERVER
  if (ble_ll_scan_enabled())
    {
      return BLE_ERR_CMD_DISALLOWED;
    }
#endif

#ifdef CONFIG_BLE_LL_ROLE_BROADCASTER
  if (!hci_adv_ext && ble_ll_adv_enabled())
    {
      return BLE_ERR_CMD_DISALLOWED;
    }
#endif

  memcpy(g_random_addr, cmd->addr, BLE_DEV_ADDR_LEN);

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV
#ifdef CONFIG_BLE_LL_ROLE_BROADCASTER

  /* For instance 0 we need same address if legacy advertising might be
   * used. If extended advertising is in use than this command doesn't
   * affect instance 0.
   */

  if (!hci_adv_ext)
    {
      ble_ll_adv_set_random_addr(cmd->addr, 0);
    }
#endif
#endif

  return BLE_ERR_SUCCESS;
}

/****************************************************************************
 * Name: ble_ll_is_our_devaddr
 *
 * Description:
 *   Checks to see if an address is our device address (either public or
 *   random)
 *
 * Input Parameters:
 *   addr
 *   addr_type
 *
 * Returned Value:
 *   int 0: not our device address. 1: is our device address
 *
 ****************************************************************************/

int ble_ll_is_our_devaddr(uint8_t *addr, int addr_type)
{
  int rc;
  uint8_t *our_addr;

  if (addr_type)
    {
      our_addr = g_random_addr;
    }
  else
    {
      our_addr = g_dev_addr;
    }

  rc = 0;
  if (!memcmp(our_addr, addr, BLE_DEV_ADDR_LEN))
    {
      rc = 1;
    }

  return rc;
}

/****************************************************************************
 * Name: ble_ll_get_our_devaddr
 *
 * Description:
 *   Get identity address
 *
 * Input Parameters:
 *   addr_type - Random (1). Public(0)
 *
 * Returned Value:
 *   pointer to identity address of given type.
 *
 ****************************************************************************/

uint8_t *ble_ll_get_our_devaddr(uint8_t addr_type)
{
  if (addr_type)
    {
      return g_random_addr;
    }

  return g_dev_addr;
}

/****************************************************************************
 * Name: ble_ll_wfr_timer_exp
 *
 * Description:
 *   Wait for response timeout function
 *
 *   Context: interrupt (ble scheduler)
 *
 * Input Parameters:
 *   arg
 *
 ****************************************************************************/

void ble_ll_wfr_timer_exp(void *arg)
{
  int rx_start;
  uint8_t lls;

  rx_start = ble_phy_rx_started();
  lls = g_ble_ll_data.ll_state;

  /* If we have started a reception, there is nothing to do here */

  if (!rx_start)
    {
      switch (lls)
        {
#ifdef CONFIG_BLE_LL_ROLE_BROADCASTER
          case BLE_LL_STATE_ADV:
            ble_ll_adv_wfr_timer_exp();
            break;
#endif
#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) || \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)
          case BLE_LL_STATE_CONNECTION:
            ble_ll_conn_wfr_timer_exp();
            break;
#endif
#ifdef CONFIG_BLE_LL_ROLE_OBSERVER
          case BLE_LL_STATE_SCANNING:
            ble_ll_scan_wfr_timer_exp();
            break;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV
          case BLE_LL_STATE_SCAN_AUX:
            ble_ll_scan_aux_wfr_timer_exp();
            break;
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV
          case BLE_LL_STATE_SYNC:
            ble_ll_sync_wfr_timer_exp();
            break;
#endif
#endif
          default:
            break;
        }
    }
}

/****************************************************************************
 * Name: ble_ll_rx_pdu_in
 *
 * Description:
 *   Called to put a packet on the Link Layer receive packet queue.
 *
 * Input Parameters:
 *   rxpdu - Pointer to received PDU
 *
 ****************************************************************************/

void ble_ll_rx_pdu_in(struct iob_s *rxpdu)
{
  struct ble_ll_pkthdr_s *pkthdr;

  pkthdr = BLE_LL_PKTHDR(rxpdu);
  STAILQ_INSERT_TAIL(&g_ble_ll_data.ll_rx_pkt_q, pkthdr, next);
  ble_ll_event_add(&g_ble_ll_data.ll_rx_pkt_ev);
}

#if defined(CONFIG_BLE_LL_ROLE_CENTRAL) || \
    defined(CONFIG_BLE_LL_ROLE_PERIPHERAL)
/****************************************************************************
 * Name: ble_ll_acl_data_in
 *
 * Description:
 *   Called to put a packet on the Link Layer transmit packet queue.
 *
 * Input Parameters:
 *   txpdu - Pointer to transmit packet
 *
 ****************************************************************************/

void ble_ll_acl_data_in(struct iob_s *txpkt)
{
  irqstate_t sr;
  struct ble_ll_pkthdr_s *pkthdr;

  pkthdr = BLE_LL_PKTHDR(txpkt);
  sr = enter_critical_section();
  STAILQ_INSERT_TAIL(&g_ble_ll_data.ll_tx_pkt_q, pkthdr, next);
  leave_critical_section(sr);
  ble_ll_event_add(&g_ble_ll_data.ll_tx_pkt_ev);
}

/****************************************************************************
 * Name: ble_ll_data_buffer_overflow
 *
 * Description:
 *   Called to post event to Link Layer when a data buffer overflow has
 *   occurred.
 *
 *   Context: Interrupt
 *
 ****************************************************************************/

void ble_ll_data_buffer_overflow(void)
{
  ble_ll_event_add(&g_ble_ll_data.ll_dbuf_overflow_ev);
}

#endif

/****************************************************************************
 * Name: ble_ll_hw_error
 *
 * Description:
 *   Called when a HW error occurs.
 *
 *   Context: Interrupt
 *
 ****************************************************************************/

void ble_ll_hw_error(void)
{
  ble_ll_callout_reset(&g_ble_ll_data.ll_hw_err_timer, 0);
}

/****************************************************************************
 * Name: ble_ll_rx_start
 *
 * Description:
 *   Called upon start of received PDU
 *
 *   Context: Interrupt
 *
 * Input Parameters:
 *   rxpdu - chan
 *
 * Returned Value:
 *   int
 *   < 0: A frame we dont want to receive.
 *   = 0: Continue to receive frame. Dont go from rx to tx
 *   > 0: Continue to receive frame and go from rx to tx when done
 *
 ****************************************************************************/

int ble_ll_rx_start(uint8_t *rxbuf, uint8_t chan, struct ble_mbuf_hdr *rxhdr)
{
  int rc;
  uint8_t pdu_type;

  /* Advertising channel PDU */

  pdu_type = rxbuf[0] & BLE_ADV_PDU_HDR_TYPE_MASK;

  switch (g_ble_ll_data.ll_state)
    {
#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) || \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)
      case BLE_LL_STATE_CONNECTION:
        rc = ble_ll_conn_rx_isr_start(rxhdr, ble_phy_access_addr_get());
        break;
#endif
#ifdef CONFIG_BLE_LL_ROLE_BROADCASTER
      case BLE_LL_STATE_ADV:
        rc = ble_ll_adv_rx_isr_start(pdu_type);
        break;
#endif
#ifdef CONFIG_BLE_LL_ROLE_OBSERVER
      case BLE_LL_STATE_SCANNING:
        rc = ble_ll_scan_rx_isr_start(pdu_type, &rxhdr->rxinfo.flags);
        break;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV
      case BLE_LL_STATE_SYNC:
        rc = ble_ll_sync_rx_isr_start(pdu_type, rxhdr);
        break;
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV
      case BLE_LL_STATE_SCAN_AUX:
        rc = ble_ll_scan_aux_rx_isr_start(pdu_type, rxhdr);
        break;
#endif
#endif
      default:

        /* Should not be in this state! */

        rc = -1;
        BLE_LL_STATS_INC(ble_ll_stats, bad_ll_state);
        break;
    }

  return rc;
}

/****************************************************************************
 * Name: ble_ll_rx_end
 *
 * Description:
 *   Called by the PHY when a receive packet has ended.
 *
 *   NOTE: Called from interrupt context!
 *
 * Input Parameters:
 *   rxbuf - Pointer to received PDU data rxhdr Pointer to BLE header of
 * received mbuf
 *
 * Returned Value:
 *   int
 *   < 0: Disable the phy after reception.
 *   == 0: Success. Do not disable the PHY.
 *   > 0: Do not disable PHY as that has already been done.
 *
 ****************************************************************************/

int ble_ll_rx_end(uint8_t *rxbuf, struct ble_mbuf_hdr *rxhdr)
{
  int rc;
  int badpkt;
  uint8_t pdu_type;
  uint8_t len;
  uint8_t crcok;
  struct iob_s *rxpdu;

  /* Get CRC status from BLE header */

  crcok = BLE_MBUF_HDR_CRC_OK(rxhdr);

  /* Get advertising PDU type and length */

  pdu_type = rxbuf[0] & BLE_ADV_PDU_HDR_TYPE_MASK;
  len = rxbuf[1];

#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) || \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)
  if (BLE_MBUF_HDR_RX_STATE(rxhdr) == BLE_LL_STATE_CONNECTION)
    {
      rc = ble_ll_conn_rx_isr_end(rxbuf, rxhdr);
      return rc;
    }
#endif

#if defined(CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV) && \
    defined(CONFIG_BLE_LL_ROLE_OBSERVER)
  if (BLE_MBUF_HDR_RX_STATE(rxhdr) == BLE_LL_STATE_SYNC)
    {
      rc = ble_ll_sync_rx_isr_end(rxbuf, rxhdr);
      return rc;
    }
#endif

  /* If the CRC checks, make sure lengths check! */

  badpkt = 0;
  if (crcok)
    {
      switch (pdu_type)
        {
          case BLE_ADV_PDU_TYPE_SCAN_REQ:
          case BLE_ADV_PDU_TYPE_ADV_DIRECT_IND:
            if (len != BLE_SCAN_REQ_LEN)
              {
                badpkt = 1;
              }
            break;
          case BLE_ADV_PDU_TYPE_SCAN_RSP:
          case BLE_ADV_PDU_TYPE_ADV_IND:
          case BLE_ADV_PDU_TYPE_ADV_SCAN_IND:
          case BLE_ADV_PDU_TYPE_ADV_NONCONN_IND:
            if ((len < BLE_DEV_ADDR_LEN) || (len > BLE_ADV_SCAN_IND_MAX_LEN))
              {
                badpkt = 1;
              }
            break;
          case BLE_ADV_PDU_TYPE_AUX_CONNECT_RSP:
            break;
          case BLE_ADV_PDU_TYPE_ADV_EXT_IND:
            break;
          case BLE_ADV_PDU_TYPE_CONNECT_IND:
            if (len != BLE_CONNECT_REQ_LEN)
              {
                badpkt = 1;
              }
            break;
          default:
            badpkt = 1;
            break;
        }

      /* If this is a malformed packet, just kill it here */

      if (badpkt)
        {
          BLE_LL_STATS_INC(ble_ll_stats, rx_adv_malformed_pkts);
        }
    }

  /* Hand packet to the appropriate state machine (if crc ok) */

  rxpdu = NULL;
  switch (BLE_MBUF_HDR_RX_STATE(rxhdr))
    {
#ifdef CONFIG_BLE_LL_ROLE_BROADCASTER
      case BLE_LL_STATE_ADV:
        if (!badpkt)
          {
            rxpdu = ble_ll_rxpdu_alloc(len + BLE_LL_PDU_HDR_LEN);
            if (rxpdu)
              {
                ble_phy_rxpdu_copy(rxbuf, rxpdu);
              }
          }

        rc = ble_ll_adv_rx_isr_end(pdu_type, rxpdu, crcok);
        break;
#endif
#ifdef CONFIG_BLE_LL_ROLE_OBSERVER
      case BLE_LL_STATE_SCANNING:
        if (!badpkt)
          {
            rxpdu = ble_ll_rxpdu_alloc(len + BLE_LL_PDU_HDR_LEN);
            if (rxpdu)
              {
                ble_phy_rxpdu_copy(rxbuf, rxpdu);
              }
          }

        rc = ble_ll_scan_rx_isr_end(rxpdu, crcok);
        break;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV
      case BLE_LL_STATE_SCAN_AUX:
        if (!badpkt)
          {
            rxpdu = ble_ll_rxpdu_alloc(len + BLE_LL_PDU_HDR_LEN);
            if (rxpdu)
              {
                ble_phy_rxpdu_copy(rxbuf, rxpdu);
              }
          }

        rc = ble_ll_scan_aux_rx_isr_end(rxpdu, crcok);
        break;
#endif
#endif
      default:
        rc = -1;
        BLE_LL_STATS_INC(ble_ll_stats, bad_ll_state);
        break;
    }

  /* Hand packet up to higher layer (regardless of CRC failure) */

  if (rxpdu)
    {
      ble_ll_rx_pdu_in(rxpdu);
    }

  return rc;
}

/****************************************************************************
 * Name: ble_ll_tx_mbuf_pducb
 *
 ****************************************************************************/

uint8_t ble_ll_tx_mbuf_pducb(uint8_t *dptr, void *pducb_arg,
                             uint8_t *hdr_byte)
{
  struct iob_s *txpdu;
  struct ble_mbuf_hdr *ble_hdr;

  txpdu = pducb_arg;
  DEBUGASSERT(txpdu);
  ble_hdr = BLE_MBUF_HDR_PTR(txpdu);

  iob_copyout(dptr, txpdu, ble_hdr->txinfo.pyld_len, ble_hdr->txinfo.offset);

  *hdr_byte = ble_hdr->txinfo.hdr_byte;

  return ble_hdr->txinfo.pyld_len;
}

/****************************************************************************
 * Name: ble_ll_tx_flat_mbuf_pducb
 *
 ****************************************************************************/

uint8_t ble_ll_tx_flat_mbuf_pducb(uint8_t *dptr, void *pducb_arg,
                                  uint8_t *hdr_byte)
{
  struct iob_s *txpdu;
  struct ble_mbuf_hdr *ble_hdr;

  txpdu = pducb_arg;
  DEBUGASSERT(txpdu);
  ble_hdr = BLE_MBUF_HDR_PTR(txpdu);

  memcpy(dptr, IOB_DATA(txpdu), ble_hdr->txinfo.pyld_len);

  *hdr_byte = ble_hdr->txinfo.hdr_byte;

  return ble_hdr->txinfo.pyld_len;
}

/****************************************************************************
 * Name: ble_ll_state_set
 *
 * Description:
 *   ble ll state set
 *
 *   Called to set the current link layer state.
 *
 *   Context: Interrupt and Link Layer task
 *
 * Input Parameters:
 *   ll_state
 *
 ****************************************************************************/

void ble_ll_state_set(uint8_t ll_state)
{
  g_ble_ll_data.ll_state = ll_state;

  if (ll_state == BLE_LL_STATE_STANDBY)
    {
    }
}

/****************************************************************************
 * Name: ble_ll_state_get
 *
 * Description:
 *   ble ll state get
 *
 *   Called to get the current link layer state.
 *
 *   Context: Link Layer task (can be called from interrupt context though).
 *
 * Returned Value:
 *   ll_state
 *
 ****************************************************************************/

uint8_t ble_ll_state_get(void)
{
  return g_ble_ll_data.ll_state;
}

/****************************************************************************
 * Name: ble_ll_event_add
 *
 * Description:
 *   ble ll event send
 *
 *   Add an event to the Link Layer task
 *
 * Input Parameters:
 *   ev - Event to add to the Link Layer event queue.
 *
 ****************************************************************************/

void ble_ll_event_add(struct ble_ll_event_s *ev)
{
  /* An event already queued keeps its position */

  if (work_available(&ev->work))
    {
      work_queue_wq(g_ble_ll_wq, &ev->work, ev->fn, ev->arg, 0);
    }
}

/****************************************************************************
 * Name: ble_ll_event_remove
 *
 * Description:
 *   ble ll event remove
 *
 *   Remove an event from the Link Layer task
 *
 * Input Parameters:
 *   ev - Event to remove from the Link Layer event queue.
 *
 ****************************************************************************/

void ble_ll_event_remove(struct ble_ll_event_s *ev)
{
  work_cancel_wq(g_ble_ll_wq, &ev->work);
}

/****************************************************************************
 * Name: ble_ll_read_supp_states
 *
 * Description:
 *   Returns the features supported by the link layer
 *
 * Returned Value:
 *   uint8_t bitmask of supported features.
 *
 ****************************************************************************/

uint64_t ble_ll_read_supp_states(void)
{
  return BLE_LL_SUPPORTED_STATES;
}

/****************************************************************************
 * Name: ble_ll_read_supp_features
 *
 * Description:
 *   Returns the features supported by the link layer
 *
 * Returned Value:
 *   uint64_t bitmask of supported features.
 *
 ****************************************************************************/

uint64_t ble_ll_read_supp_features(void)
{
  return g_ble_ll_data.ll_supp_features;
}

#if BLE_LL_HOST_CONTROLLED_FEATURES
/****************************************************************************
 * Name: ble_ll_set_host_feat
 *
 * Description:
 *   Sets the features controlled by the host.
 *
 * Returned Value:
 *   HCI command status
 *
 ****************************************************************************/

int ble_ll_set_host_feat(const uint8_t *cmdbuf, uint8_t len)
{
  const struct ble_hci_le_set_host_feature_cp *cmd = (const void *)cmdbuf;
  uint64_t mask;

  if (len != sizeof(*cmd))
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

#if defined(CONFIG_BLE_LL_ROLE_CENTRAL) || \
    defined(CONFIG_BLE_LL_ROLE_PERIPHERAL)
  if (!SLIST_EMPTY(&g_ble_ll_conn_active_list))
    {
      return BLE_ERR_CMD_DISALLOWED;
    }
#endif

  if ((cmd->bit_num > 0x3f) || (cmd->bit_val > 1))
    {
      return BLE_ERR_INV_HCI_CMD_PARMS;
    }

  mask = (uint64_t)1 << (cmd->bit_num);
  if (!(mask & g_ble_ll_host_controlled_features))
    {
      return BLE_ERR_UNSUPPORTED;
    }

  if (cmd->bit_val == 0)
    {
      g_ble_ll_data.ll_supp_features &= ~(mask);
    }
  else
    {
      g_ble_ll_data.ll_supp_features |= mask;
    }

  return BLE_ERR_SUCCESS;
}

#endif /* BLE_LL_HOST_CONTROLLED_FEATURES */

/****************************************************************************
 * Name: ble_ll_mbuf_init
 *
 * Description:
 *   Called to initialize a mbuf used by the controller
 *
 *   NOTE: this is only used when the mbuf is created by the controller;
 *   it should not be used for data packets (ACL data packets) that come from
 *   the host. This routine assumes that the entire pdu length can fit in
 *   one mbuf contiguously.
 *
 * Input Parameters:
 *   m
 *   pdulen
 *   hdr
 *
 ****************************************************************************/

void ble_ll_mbuf_init(struct iob_s *m, uint8_t pdulen, uint8_t hdr)
{
  struct ble_mbuf_hdr *ble_hdr;

  /* Set mbuf length and packet length */

  m->io_len = pdulen;
  BLE_LL_PKTHDR(m)->iob->io_pktlen = pdulen;

  /* Set BLE transmit header */

  ble_hdr = BLE_MBUF_HDR_PTR(m);
  ble_hdr->txinfo.num_data_pkt = 0;
  ble_hdr->txinfo.offset = 0;
  ble_hdr->txinfo.pyld_len = pdulen;
  ble_hdr->txinfo.hdr_byte = hdr;
}

/****************************************************************************
 * Name: ble_ll_reset
 *
 * Description:
 *   Called to reset the controller. This performs a "software reset" of the
 * link layer; it does not perform a HW reset of the controller nor does it
 * reset the HCI interface.
 *
 *   Context: Link Layer task (HCI command)
 *
 * Returned Value:
 *   int The ble error code to place in the command complete event that
 *   is returned when this command is issued.
 *
 ****************************************************************************/

int ble_ll_reset(void)
{
  uint8_t phy_mask;
  int rc;
  irqstate_t sr;

  sr = enter_critical_section();
  ble_phy_disable();
  ble_ll_sched_stop();
#ifdef CONFIG_BLE_LL_ROLE_OBSERVER
  ble_ll_scan_reset();
#endif
  ble_ll_rfmgmt_reset();
  leave_critical_section(sr);

#ifdef CONFIG_BLE_LL_ROLE_BROADCASTER

  /* Stop any advertising */

  ble_ll_adv_reset();
#endif

#if defined(CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV) && \
    defined(CONFIG_BLE_LL_ROLE_OBSERVER)
  /* Stop sync */

  ble_ll_sync_reset();
#endif

  /* reset power compensation */

  g_ble_ll_tx_power_compensation = 0;
  g_ble_ll_rx_power_compensation = 0;

  /* Set output power to default */

  g_ble_ll_tx_power = ble_ll_tx_power_round(
      MIN(CONFIG_BLE_LL_TX_PWR_DBM, CONFIG_BLE_LL_TX_PWR_MAX_DBM));
  g_ble_ll_tx_power_phy_current = INT8_MAX;

  /* FLush all packets from Link layer queues */

  ble_ll_flush_pkt_queue(&g_ble_ll_data.ll_tx_pkt_q);
  ble_ll_flush_pkt_queue(&g_ble_ll_data.ll_rx_pkt_q);

  /* Reset LL stats */

  BLE_LL_STATS_RESET(ble_ll_stats);

  /* Reset any preferred PHYs */

  phy_mask = BLE_PHY_MASK_1M;
#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_2M_PHY
  phy_mask |= BLE_PHY_MASK_2M;
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_CODED_PHY
  phy_mask |= BLE_PHY_MASK_CODED;
#endif
  phy_mask &= CONFIG_BLE_LL_CONN_PHY_DEFAULT_PREF_MASK;
  DEBUGASSERT(phy_mask);
  g_ble_ll_data.ll_pref_tx_phys = phy_mask;
  g_ble_ll_data.ll_pref_rx_phys = phy_mask;

  /* Enable all channels in channel map */

  g_ble_ll_data.chan_map_used = BLE_PHY_NUM_DATA_CHANS;
  memset(g_ble_ll_data.chan_map, 0xff, BLE_LL_CHAN_MAP_LEN - 1);
  g_ble_ll_data.chan_map[4] = 0x1f;

#if defined(CONFIG_BLE_LL_ROLE_PERIPHERAL) || \
    defined(CONFIG_BLE_LL_ROLE_CENTRAL)

  /* Reset connection module */

  ble_ll_conn_module_reset();
#endif

  /* All this does is re-initialize the event masks so call the hci init */

  ble_ll_hci_init();

  /* Reset scheduler */

  ble_ll_sched_init();

  /* Set state to standby */

  ble_ll_state_set(BLE_LL_STATE_STANDBY);

  /* Reset our random address */

  memset(g_random_addr, 0, BLE_DEV_ADDR_LEN);

  /* Clear the whitelist */

  ble_ll_whitelist_clear();

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  /* Reset resolving list */

  ble_ll_resolv_list_reset();
#endif

  /* Re-initialize the PHY */

  rc = ble_phy_init();

  return rc;
}

/****************************************************************************
 * Name: ble_ll_pdu_max_tx_octets_get
 *
 ****************************************************************************/

uint16_t ble_ll_pdu_max_tx_octets_get(uint32_t usecs, int phy_mode)
{
  uint32_t header_tx_time;
  uint16_t octets = 0;

  DEBUGASSERT(phy_mode < BLE_PHY_NUM_MODE);

  header_tx_time = ble_ll_pdu_us(0, phy_mode);

  /* Current conn max tx time can be too short to even send a packet header
   * and this can happen if we changed connection form uncoded to coded phy.
   * However, the lower bound for conn max tx time (all of them) depends on
   * current phy (uncoded/coded) but it always allows to send at least 27
   * bytes of payload thus we always return at least 27 from here.
   *
   * Reference:
   * Core v5.0, Vol 6, Part B, section 4.5.10
   * see connEffectiveMaxTxTime and connEffectiveMaxRxTime definitions
   */

  if (usecs < header_tx_time)
    {
      return 27;
    }

  usecs -= header_tx_time;

  if (phy_mode == BLE_PHY_MODE_1M)
    {
      /* 8 usecs per byte */

      octets = usecs >> 3;
    }
  else if (phy_mode == BLE_PHY_MODE_2M)
    {
      /* 4 usecs per byte */

      octets = usecs >> 2;
    }
  else if (phy_mode == BLE_PHY_MODE_CODED_125KBPS)
    {
      /* S=8 => 8 * 8 = 64 usecs per byte */

      octets = usecs >> 6;
    }
  else if (phy_mode == BLE_PHY_MODE_CODED_500KBPS)
    {
      /* S=2 => 2 * 8 = 16 usecs per byte */

      octets = usecs >> 4;
    }
  else
    {
      DEBUGASSERT(0);
    }

  /* see comment at the beginning */

  return MAX(27, octets);
}

/****************************************************************************
 * Name: ble_ll_init
 *
 * Description:
 *   Initialize the Link Layer. Should be called only once
 *
 * Returned Value:
 *   int
 *
 ****************************************************************************/

void ble_ll_init(void)
{
  int rc;
  uint64_t features;
#if CONFIG_BLE_LL_PUBLIC_DEV_ADDR
  uint64_t pub_dev_addr;
  int i;
#endif
  ble_addr_t addr;
  struct ble_ll_obj *lldata;

  /* Set public device address if not already set */

  if (ble_ll_is_addr_empty(g_dev_addr))
    {
#if CONFIG_BLE_LL_PUBLIC_DEV_ADDR
      pub_dev_addr = CONFIG_BLE_LL_PUBLIC_DEV_ADDR;

      for (i = 0; i < BLE_DEV_ADDR_LEN; i++)
        {
          g_dev_addr[i] = pub_dev_addr & 0xff;
          pub_dev_addr >>= 8;
        }
#endif

      if (ble_ll_is_addr_empty(g_dev_addr))
        {
          rc = ble_hw_get_public_addr(&addr);
          if (!rc)
            {
              memcpy(g_dev_addr, &addr.val[0], BLE_DEV_ADDR_LEN);
            }
        }
    }

  ble_ll_rfmgmt_init();

  /* Get pointer to global data object */

  lldata = &g_ble_ll_data;

#if defined(CONFIG_BLE_LL_ROLE_CENTRAL) || \
    defined(CONFIG_BLE_LL_ROLE_PERIPHERAL)

  /* Set acl pkt size and number */

  lldata->ll_num_acl_pkts = CONFIG_BLE_LL_ACL_FROM_HS_COUNT;
  lldata->ll_acl_pkt_size = CONFIG_BLE_LL_ACL_SIZE;
#endif

  /* Initialize the transmit (from host) and receive (from phy) queues */

  STAILQ_INIT(&lldata->ll_tx_pkt_q);
  STAILQ_INIT(&lldata->ll_rx_pkt_q);

  /* Initialize transmit (from host) and receive packet (from phy) event */

  ble_ll_event_init(&lldata->ll_rx_pkt_ev, ble_ll_event_rx_pkt, NULL);
#if defined(CONFIG_BLE_LL_ROLE_CENTRAL) || \
    defined(CONFIG_BLE_LL_ROLE_PERIPHERAL)
  ble_ll_event_init(&lldata->ll_tx_pkt_ev, ble_ll_event_tx_pkt, NULL);
#endif

#if defined(CONFIG_BLE_LL_ROLE_CENTRAL) || \
    defined(CONFIG_BLE_LL_ROLE_PERIPHERAL)

  /* Initialize data buffer overflow event and completed packets */

  ble_ll_event_init(&lldata->ll_dbuf_overflow_ev,
                     ble_ll_event_dbuf_overflow, NULL);
  ble_ll_event_init(&lldata->ll_comp_pkt_ev, ble_ll_event_comp_pkts, NULL);
#endif

  /* Initialize the HW error timer */

  ble_ll_event_init(&g_ble_ll_data.ll_hw_err_timer, ble_ll_hw_err_timer_cb,
                    NULL);

  /* Initialize LL HCI */

  ble_ll_hci_init();

  /* Init the scheduler */

  ble_ll_sched_init();

#ifdef CONFIG_BLE_LL_ROLE_BROADCASTER

  /* Initialize advertiser */

  ble_ll_adv_init();
#endif

#ifdef CONFIG_BLE_LL_ROLE_OBSERVER

  /* Initialize a scanner */

  ble_ll_scan_init();
#endif

#if defined(CONFIG_BLE_LL_ROLE_CENTRAL) || \
    defined(CONFIG_BLE_LL_ROLE_PERIPHERAL)

  /* Initialize the connection module */

  ble_ll_conn_module_init();
#endif

  /* Set the supported features. NOTE: we always support extended reject. */

  features = BLE_LL_FEAT_EXTENDED_REJ;

#ifdef CONFIG_BLE_LL_CFG_FEAT_DATA_LEN_EXT
  features |= BLE_LL_FEAT_DATA_LEN_EXT;
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_CONN_PARAM_REQ
  features |= BLE_LL_FEAT_CONN_PARM_REQ;
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_PERIPH_INIT_FEAT_XCHG
  features |= BLE_LL_FEAT_PERIPH_INIT;
#endif
#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_ENCRYPTION
  features |= BLE_LL_FEAT_LE_ENCRYPTION;
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PRIVACY
  features |= (BLE_LL_FEAT_LL_PRIVACY | BLE_LL_FEAT_EXT_SCAN_FILT);
  ble_ll_resolv_init();
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_PING
  features |= BLE_LL_FEAT_LE_PING;
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_EXT_ADV
  features |= BLE_LL_FEAT_EXT_ADV;
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_CSA2

  /* CSA2 */

  features |= BLE_LL_FEAT_CSA2;
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_2M_PHY
  features |= BLE_LL_FEAT_LE_2M_PHY;
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LE_CODED_PHY
  features |= BLE_LL_FEAT_LE_CODED_PHY;
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV
  features |= BLE_LL_FEAT_PERIODIC_ADV;
#ifdef CONFIG_BLE_LL_ROLE_OBSERVER
  ble_ll_sync_init();
#endif
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV_SYNC_TRANSFER
  features |= BLE_LL_FEAT_SYNC_TRANS_RECV;
  features |= BLE_LL_FEAT_SYNC_TRANS_SEND;
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_SCA_UPDATE
  features |= BLE_LL_FEAT_SCA_UPDATE;
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_ENHANCED_CONN_UPDATE
  features |= BLE_LL_FEAT_CONN_SUBRATING;
#endif

#ifdef CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV_ADI_SUPPORT
  features |= BLE_LL_FEAT_PERIODIC_ADV_ADI;
#endif

#ifdef CONFIG_BLE_LL_ADV_CODING_SELECTION
  features |= BLE_LL_FEAT_ADV_CODING_SEL;
#endif

  lldata->ll_supp_features = features;

  /* Initialize random number generation */

  ble_ll_rand_init();

  /* Start the random number generator */

  ble_ll_rand_start();

  /* Init ble phy */

  ble_phy_init();

  /* Set output power to default */

  g_ble_ll_tx_power = ble_ll_tx_power_round(
      MIN(CONFIG_BLE_LL_TX_PWR_DBM, CONFIG_BLE_LL_TX_PWR_MAX_DBM));
  g_ble_ll_tx_power_phy_current = INT8_MAX;
}

/****************************************************************************
 * Name: ble_ll_tx_power_round
 *
 ****************************************************************************/

int ble_ll_tx_power_round(int tx_power)
{
  tx_power = ble_phy_tx_power_round(tx_power);

  return tx_power;
}

/****************************************************************************
 * Name: ble_ll_tx_power_set
 *
 ****************************************************************************/

void ble_ll_tx_power_set(int tx_power)
{
  /* If current TX power configuration matches requested one we don't need
   * to update PHY tx power.
   */

  if (g_ble_ll_tx_power_phy_current == tx_power)
    {
      return;
    }

  g_ble_ll_tx_power_phy_current = tx_power;
  ble_phy_tx_power_set(tx_power);
}

/****************************************************************************
 * Name: ble_ll_is_busy
 *
 ****************************************************************************/

int ble_ll_is_busy(unsigned int flags)
{
#if defined(CONFIG_BLE_LL_ROLE_CENTRAL) || \
    defined(CONFIG_BLE_LL_ROLE_PERIPHERAL)
  struct ble_ll_conn_sm *cur;
  int i = 0;
#endif

#if defined(CONFIG_BLE_LL_CFG_FEAT_LL_PERIODIC_ADV) && \
    defined(CONFIG_BLE_LL_ROLE_OBSERVER)
  if (ble_ll_sync_enabled())
    {
      return 1;
    }
#endif

#ifdef CONFIG_BLE_LL_ROLE_BROADCASTER
  if (ble_ll_adv_enabled())
    {
      return 1;
    }
#endif

#ifdef CONFIG_BLE_LL_ROLE_OBSERVER
  if (ble_ll_scan_enabled())
    {
      return 1;
    }
#endif

#ifdef CONFIG_BLE_LL_ROLE_CENTRAL
  if (g_ble_ll_conn_create_sm.connsm)
    {
      return 1;
    }
#endif

#if defined(CONFIG_BLE_LL_ROLE_CENTRAL) || \
    defined(CONFIG_BLE_LL_ROLE_PERIPHERAL)
  if (!(flags & BLE_LL_BUSY_EXCLUDE_CONNECTIONS))
    {
      STAILQ_FOREACH(cur, &g_ble_ll_conn_free_list, free_stqe)
      {
        i++;
      }

      /* check if all connection objects are free */

      if (i < CONFIG_BLE_LL_MAX_CONNECTIONS)
        {
          return 1;
        }
    }
#endif

  return 0;
}
