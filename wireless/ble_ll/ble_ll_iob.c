/****************************************************************************
 * wireless/ble_ll/ble_ll_iob.c
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

#include "ble_ll_os.h"
#include "ble_ll.h"
#include "ble_phy.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Control PDUs and extended advertising headers must be contiguous in the
 * first I/O buffer
 */

static_assert(CONFIG_IOB_BUFSIZE >= BLE_LL_IOB_HEADROOM + 66,
              "CONFIG_IOB_BUFSIZE too small for BLE Link Layer PDUs");

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* PDU metadata of the PDU being received, provided by the PHY */

FAR struct ble_mbuf_hdr *g_ble_ll_phy_rxhdr;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ble_ll_iob_alloc
 *
 * Description:
 *   Allocate a PDU buffer.  May be called from interrupt context.
 *
 ****************************************************************************/

struct iob_s *ble_ll_iob_alloc(void)
{
  struct ble_ll_pkthdr_s *pkthdr;
  struct iob_s *iob;

  iob = iob_tryalloc(false);
  if (iob == NULL)
    {
      return NULL;
    }

  iob->io_offset = BLE_LL_IOB_HEADROOM;
  iob->io_len    = 0;
  iob->io_pktlen = 0;

  pkthdr = BLE_LL_PKTHDR(iob);
  memset(pkthdr, 0, sizeof(*pkthdr));
  pkthdr->iob = iob;

  return iob;
}

/****************************************************************************
 * Name: ble_ll_iob_adj
 *
 * Description:
 *   Remove len bytes from the head of the PDU.
 *
 ****************************************************************************/

void ble_ll_iob_adj(struct iob_s *iob, unsigned int len)
{
  struct iob_s *cur;
  unsigned int n;

  DEBUGASSERT(len <= iob->io_pktlen);

  iob->io_pktlen -= len;

  for (cur = iob; cur != NULL && len > 0; cur = cur->io_flink)
    {
      n               = MIN(len, cur->io_len);
      cur->io_offset += n;
      cur->io_len    -= n;
      len            -= n;
    }
}

/****************************************************************************
 * Name: ble_ll_iob_prepend
 *
 * Description:
 *   Grow the PDU at the head into the headroom.
 *
 ****************************************************************************/

void ble_ll_iob_prepend(struct iob_s *iob, unsigned int len)
{
  DEBUGASSERT(BLE_LL_LEADINGSPACE(iob) >= len);

  iob->io_offset -= len;
  iob->io_len    += len;
  iob->io_pktlen += len;
}

/****************************************************************************
 * Name: ble_ll_rxpdu_alloc
 *
 * Description:
 *   Allocate a PDU buffer chain large enough for a received PDU.  The
 *   chain capacity is reserved but the data lengths are set when the PDU is
 *   copied.  Called from interrupt context.
 *
 * Input Parameters:
 *   len - Length of PDU, including PDU header and excluding MIC
 *
 * Returned Value:
 *   PDU buffer on success, NULL if out of buffers.
 *
 ****************************************************************************/

struct iob_s *ble_ll_rxpdu_alloc(uint16_t len)
{
  struct iob_s *head;
  struct iob_s *tail;
  struct iob_s *next;
  int rem;

  head = ble_ll_iob_alloc();
  if (head == NULL)
    {
      goto errout;
    }

  rem  = len - (IOB_BUFSIZE(head) - head->io_offset);
  tail = head;

  while (rem > 0)
    {
      next = iob_tryalloc(false);
      if (next == NULL)
        {
          iob_free_chain(head);
          goto errout;
        }

      next->io_offset = 0;
      next->io_len    = 0;
      tail->io_flink  = next;
      tail            = next;
      rem            -= IOB_BUFSIZE(next);
    }

  head->io_pktlen = len;
  return head;

errout:
  BLE_LL_STATS_INC(ble_ll_stats, no_bufs);
  return NULL;
}

/****************************************************************************
 * Name: ble_phy_rxpdu_copy
 *
 * Description:
 *   Copy a received PDU from the PHY buffer into a PDU buffer allocated by
 *   ble_ll_rxpdu_alloc() and attach the PDU metadata.
 *
 ****************************************************************************/

void ble_phy_rxpdu_copy(FAR uint8_t *dptr, FAR struct iob_s *rxpdu)
{
  struct iob_s *iob;
  unsigned int rem;
  unsigned int len;

  rem = rxpdu->io_pktlen;

  for (iob = rxpdu; iob != NULL && rem > 0; iob = iob->io_flink)
    {
      len = MIN(rem, IOB_BUFSIZE(iob) - iob->io_offset);
      memcpy(IOB_DATA(iob), dptr, len);
      iob->io_len = len;
      dptr       += len;
      rem        -= len;
    }

  DEBUGASSERT(rem == 0 && g_ble_ll_phy_rxhdr != NULL);

  memcpy(BLE_MBUF_HDR_PTR(rxpdu), g_ble_ll_phy_rxhdr,
         sizeof(struct ble_mbuf_hdr));
}
