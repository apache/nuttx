/****************************************************************************
 * wireless/ble_ll/ble_ll_os.h
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

#ifndef __WIRELESS_BLE_LL_BLE_LL_OS_H
#define __WIRELESS_BLE_LL_BLE_LL_OS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <endian.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/param.h>
#include <sys/queue.h>

#include <nuttx/clock.h>
#include <nuttx/compiler.h>
#include <nuttx/debug.h>
#include <nuttx/irq.h>
#include <nuttx/mm/iob.h>
#include <nuttx/nuttx.h>
#include <nuttx/wqueue.h>
#include <nuttx/wireless/bluetooth/ble_ll.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Statistics */

#ifdef CONFIG_BLE_LL_STATS
#  define BLE_LL_STATS_INC(s, f)     ((s).f++)
#  define BLE_LL_STATS_INCN(s, f, n) ((s).f += (n))
#  define BLE_LL_STATS_RESET(s)      memset(&(s), 0, sizeof(s))
#  define BLE_LL_STATS_CLEAR(s, f)   ((s).f = 0)
#else
#  define BLE_LL_STATS_INC(s, f)
#  define BLE_LL_STATS_INCN(s, f, n)
#  define BLE_LL_STATS_RESET(s)
#  define BLE_LL_STATS_CLEAR(s, f)
#endif

/* PDU buffers are I/O buffer chains.  The first I/O buffer of a PDU keeps
 * struct ble_ll_pkthdr_s at the start of io_data followed by a few bytes of
 * headroom for HCI headers.
 */

#define BLE_LL_IOB_PREPEND   4
#define BLE_LL_IOB_HEADROOM  (ALIGN_UP(sizeof(struct ble_ll_pkthdr_s), 4) + \
                              BLE_LL_IOB_PREPEND)

#define BLE_LL_PKTHDR(iob)   ((struct ble_ll_pkthdr_s *)(iob)->io_data)
#define BLE_MBUF_HDR_PTR(iob) (&BLE_LL_PKTHDR(iob)->hdr)
#define BLE_LL_LEADINGSPACE(iob) \
  ((iob)->io_offset - ALIGN_UP(sizeof(struct ble_ll_pkthdr_s), 4))

/* PDU metadata accessors */

#define BLE_MBUF_HDR_IGNORED(hdr) \
  (!!((hdr)->rxinfo.flags & BLE_MBUF_HDR_F_IGNORED))
#define BLE_MBUF_HDR_SCAN_REQ_TXD(hdr) \
  (!!((hdr)->rxinfo.flags & BLE_MBUF_HDR_F_SCAN_REQ_TXD))
#define BLE_MBUF_HDR_EXT_ADV_SEC(hdr) \
  (!!((hdr)->rxinfo.flags & BLE_MBUF_HDR_F_EXT_ADV_SEC))
#define BLE_MBUF_HDR_EXT_ADV(hdr) \
  (!!((hdr)->rxinfo.flags & BLE_MBUF_HDR_F_EXT_ADV))
#define BLE_MBUF_HDR_DEVMATCH(hdr) \
  (!!((hdr)->rxinfo.flags & BLE_MBUF_HDR_F_DEVMATCH))
#define BLE_MBUF_HDR_SCAN_RSP_RXD(hdr) \
  (!!((hdr)->rxinfo.flags & BLE_MBUF_HDR_F_SCAN_RSP_RXD))
#define BLE_MBUF_HDR_WAIT_AUX(hdr) \
  (!!((hdr)->rxinfo.flags & BLE_MBUF_HDR_F_AUX_PTR_WAIT))
#define BLE_MBUF_HDR_CRC_OK(hdr) \
  (!!((hdr)->rxinfo.flags & BLE_MBUF_HDR_F_CRC_OK))
#define BLE_MBUF_HDR_MIC_FAILURE(hdr) \
  (!!((hdr)->rxinfo.flags & BLE_MBUF_HDR_F_MIC_FAILURE))
#define BLE_MBUF_HDR_RESOLVED(hdr) \
  (!!((hdr)->rxinfo.flags & BLE_MBUF_HDR_F_RESOLVED))
#define BLE_MBUF_HDR_INITA_RESOLVED(hdr) \
  (!!((hdr)->rxinfo.flags & BLE_MBUF_HDR_F_INITA_RESOLVED))
#define BLE_MBUF_HDR_TARGETA_RESOLVED(hdr) \
  (!!((hdr)->rxinfo.flags & BLE_MBUF_HDR_F_TARGETA_RESOLVED))
#define BLE_MBUF_HDR_RX_STATE(hdr) \
  ((uint8_t)((hdr)->rxinfo.flags & BLE_MBUF_HDR_F_RXSTATE_MASK))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Signed system tick difference */

#ifdef CONFIG_SYSTEM_TIME64
typedef int64_t ble_ll_stime_t;
#else
typedef int32_t ble_ll_stime_t;
#endif

/* PDU packet header kept in the first I/O buffer of a PDU */

struct ble_ll_pkthdr_s
{
  STAILQ_ENTRY(ble_ll_pkthdr_s) next;  /* Packet queue link */
  struct iob_s *iob;                   /* Owning I/O buffer */
  struct ble_mbuf_hdr hdr;             /* PDU metadata */
};

STAILQ_HEAD(ble_ll_pkt_q, ble_ll_pkthdr_s);

/* Deferred work executed by the Link Layer work queue.  Used for both
 * immediate events and timeouts.
 */

struct ble_ll_event_s
{
  struct work_s work;
  worker_t      fn;
  FAR void     *arg;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern FAR struct kwork_wqueue_s *g_ble_ll_wq;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Allocate a PDU buffer with an initialized packet header */

struct iob_s *ble_ll_iob_alloc(void);

/* Remove len bytes from the head of the PDU (must fit in the first I/O
 * buffer).
 */

void ble_ll_iob_adj(struct iob_s *iob, unsigned int len);

/* Grow the PDU at the head into the headroom */

void ble_ll_iob_prepend(struct iob_s *iob, unsigned int len);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline void ble_ll_event_init(FAR struct ble_ll_event_s *ev,
                                     worker_t fn, FAR void *arg)
{
  if (!work_available(&ev->work))
    {
      work_cancel_wq(g_ble_ll_wq, &ev->work);
    }

  ev->fn  = fn;
  ev->arg = arg;
}

static inline void ble_ll_event_set_arg(FAR struct ble_ll_event_s *ev,
                                        FAR void *arg)
{
  ev->arg = arg;
}

static inline bool ble_ll_event_is_queued(FAR struct ble_ll_event_s *ev)
{
  return !work_available(&ev->work);
}

static inline void ble_ll_callout_reset(FAR struct ble_ll_event_s *ev,
                                        clock_t ticks)
{
  work_queue_wq(g_ble_ll_wq, &ev->work, ev->fn, ev->arg, ticks);
}

static inline void ble_ll_callout_stop(FAR struct ble_ll_event_s *ev)
{
  work_cancel_wq(g_ble_ll_wq, &ev->work);
}

static inline bool ble_ll_callout_is_active(FAR struct ble_ll_event_s *ev)
{
  return !work_available(&ev->work);
}

/* Little/big endian helpers for byte streams */

static inline void put_le16(FAR void *buf, uint16_t x)
{
  FAR uint8_t *u8ptr = buf;

  u8ptr[0] = (uint8_t)x;
  u8ptr[1] = (uint8_t)(x >> 8);
}

static inline void put_le24(FAR void *buf, uint32_t x)
{
  FAR uint8_t *u8ptr = buf;

  u8ptr[0] = (uint8_t)x;
  u8ptr[1] = (uint8_t)(x >> 8);
  u8ptr[2] = (uint8_t)(x >> 16);
}

static inline void put_le32(FAR void *buf, uint32_t x)
{
  FAR uint8_t *u8ptr = buf;

  u8ptr[0] = (uint8_t)x;
  u8ptr[1] = (uint8_t)(x >> 8);
  u8ptr[2] = (uint8_t)(x >> 16);
  u8ptr[3] = (uint8_t)(x >> 24);
}

static inline void put_le64(FAR void *buf, uint64_t x)
{
  put_le32(buf, (uint32_t)x);
  put_le32((FAR uint8_t *)buf + 4, (uint32_t)(x >> 32));
}

static inline uint16_t get_le16(FAR const void *buf)
{
  FAR const uint8_t *u8ptr = buf;

  return (uint16_t)u8ptr[0] | ((uint16_t)u8ptr[1] << 8);
}

static inline uint32_t get_le24(FAR const void *buf)
{
  FAR const uint8_t *u8ptr = buf;

  return (uint32_t)u8ptr[0] | ((uint32_t)u8ptr[1] << 8) |
         ((uint32_t)u8ptr[2] << 16);
}

static inline uint32_t get_le32(FAR const void *buf)
{
  FAR const uint8_t *u8ptr = buf;

  return (uint32_t)u8ptr[0] | ((uint32_t)u8ptr[1] << 8) |
         ((uint32_t)u8ptr[2] << 16) | ((uint32_t)u8ptr[3] << 24);
}

static inline uint64_t get_le64(FAR const void *buf)
{
  return (uint64_t)get_le32(buf) |
         ((uint64_t)get_le32((FAR const uint8_t *)buf + 4) << 32);
}

static inline void put_be16(FAR void *buf, uint16_t x)
{
  FAR uint8_t *u8ptr = buf;

  u8ptr[0] = (uint8_t)(x >> 8);
  u8ptr[1] = (uint8_t)x;
}

static inline void put_be32(FAR void *buf, uint32_t x)
{
  FAR uint8_t *u8ptr = buf;

  u8ptr[0] = (uint8_t)(x >> 24);
  u8ptr[1] = (uint8_t)(x >> 16);
  u8ptr[2] = (uint8_t)(x >> 8);
  u8ptr[3] = (uint8_t)x;
}

static inline uint16_t get_be16(FAR const void *buf)
{
  FAR const uint8_t *u8ptr = buf;

  return ((uint16_t)u8ptr[0] << 8) | (uint16_t)u8ptr[1];
}

static inline uint32_t get_be32(FAR const void *buf)
{
  FAR const uint8_t *u8ptr = buf;

  return ((uint32_t)u8ptr[0] << 24) | ((uint32_t)u8ptr[1] << 16) |
         ((uint32_t)u8ptr[2] << 8) | (uint32_t)u8ptr[3];
}

static inline void swap_buf(FAR uint8_t *dst, FAR const uint8_t *src,
                            int len)
{
  int i;

  for (i = 0; i < len; i++)
    {
      dst[len - 1 - i] = src[i];
    }
}

static inline void swap_in_place(FAR void *buf, int len)
{
  FAR uint8_t *u8ptr = buf;
  uint8_t tmp;
  int i;
  int j;

  for (i = 0, j = len - 1; i < j; i++, j--)
    {
      tmp      = u8ptr[i];
      u8ptr[i] = u8ptr[j];
      u8ptr[j] = tmp;
    }
}

#endif /* __WIRELESS_BLE_LL_BLE_LL_OS_H */
