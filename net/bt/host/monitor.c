/****************************************************************************
 * net/bt/host/monitor.c
 *
 * Copyright (c) ByteDance Ltd. and/or its affiliates.
 *
 * Author: chao an <anchao.archer@bytedance.com>
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

/** @file
 *  @brief Custom logging over UART
 */

/* Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2026 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#include <nuttx/net/bluetooth/addr.h>
#include <nuttx/net/bluetooth/buf.h>
#include <nuttx/net/bluetooth/compat/init.h>
#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/net/bluetooth/compat/sys/byteorder.h>
#include <nuttx/clock.h>
#include <nuttx/net/bluetooth/compat/sys/util.h>

#include "monitor.h"

/* TS resolution is 1/10th of a millisecond */
#define MONITOR_TS_FREQ 10000

/* This is the same default priority as for other console handlers,
 * except that we're not exporting it as a Kconfig variable until a clear
 * need arises.
 */
#define MONITOR_INIT_PRIORITY 60

struct bt_monitor_data
{
  const void *data;
  size_t len;
};

enum
{
  BT_LOG_BUSY,
};

static atomic_t flags;

static struct
{
  atomic_t cmd;
  atomic_t evt;
  atomic_t acl_tx;
  atomic_t acl_rx;
#if defined(CONFIG_BT_CLASSIC)
  atomic_t sco_tx;
  atomic_t sco_rx;
#endif
  atomic_t other;
} drops;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void drop_add(uint16_t opcode)
{
  switch (opcode)
    {
      case BT_MONITOR_COMMAND_PKT:
        atomic_add(&drops.cmd, 1);
        break;
      case BT_MONITOR_EVENT_PKT:
        atomic_add(&drops.evt, 1);
        break;
      case BT_MONITOR_ACL_TX_PKT:
        atomic_add(&drops.acl_tx, 1);
        break;
      case BT_MONITOR_ACL_RX_PKT:
        atomic_add(&drops.acl_rx, 1);
        break;
#if defined(CONFIG_BT_CLASSIC)
      case BT_MONITOR_SCO_TX_PKT:
        atomic_add(&drops.sco_tx, 1);
        break;
      case BT_MONITOR_SCO_RX_PKT:
        atomic_add(&drops.sco_rx, 1);
        break;
#endif
      default:
        atomic_add(&drops.other, 1);
        break;
    }
}

static bool monitor_send(const struct bt_monitor_data *frags, size_t count)
{
  ARG_UNUSED(frags);
  ARG_UNUSED(count);

  return false;
}

static void encode_drops(struct bt_monitor_hdr *hdr,
                         uint8_t type, atomic_t *val)
{
  int32_t count;

  count = atomic_xchg(val, 0);
  if (count)
    {
      hdr->ext[hdr->hdr_len++] = type;
      hdr->ext[hdr->hdr_len++] = MIN(count, 255);
      if (count > 255)
        {
          /* Keep the surplus for the next record */

          atomic_add(val, count - 255);
        }
    }
}

static atomic_t *drop_counter(uint8_t type)
{
  switch (type)
    {
      case BT_MONITOR_COMMAND_DROPS:
        return &drops.cmd;
      case BT_MONITOR_EVENT_DROPS:
        return &drops.evt;
      case BT_MONITOR_ACL_TX_DROPS:
        return &drops.acl_tx;
      case BT_MONITOR_ACL_RX_DROPS:
        return &drops.acl_rx;
#if defined(CONFIG_BT_CLASSIC)
      case BT_MONITOR_SCO_TX_DROPS:
        return &drops.sco_tx;
      case BT_MONITOR_SCO_RX_DROPS:
        return &drops.sco_rx;
#endif
      case BT_MONITOR_OTHER_DROPS:
        return &drops.other;
      default:
        return NULL;
    }
}

/* Restore drop counts that encode_hdr() consumed into a record which then
 * failed to be sent: without this, every record dropped back-to-back would
 * destroy the counts accumulated by its predecessors, understating drops
 * under sustained overload.
 */

static void restore_drops(const struct bt_monitor_hdr *hdr)
{
  /* The timestamp is always encoded first, drop pairs follow */

  for (uint8_t i = sizeof(struct bt_monitor_ts32); i +
          1U < hdr->hdr_len; i += 2U)
    {
      atomic_t *counter = drop_counter(hdr->ext[i]);

      if (counter != NULL)
        {
          atomic_add(counter, hdr->ext[i + 1]);
        }
    }
}

/* Convert microseconds to the monitor timestamp resolution */

static uint64_t monitor_ts_from_us(uint64_t us)
{
  return us / (USEC_PER_SEC / MONITOR_TS_FREQ);
}

/* Timestamp for HCI packets: the real-time clock when configured,
 * otherwise the system timer. Conversions go via microseconds rather than
 * dividing by hw_cycles / MONITOR_TS_FREQ: the latter truncates badly for
 * cycle rates that are not a multiple of MONITOR_TS_FREQ (e.g. 32768 Hz
 * yields a divisor of 3 instead of 3.2768, making the timestamps run 9.2%
 * fast).
 */

static uint64_t monitor_ts_get(void)
{
  struct timespec ts;
  uint64_t us;

  if (IS_ENABLED(CONFIG_LOG_TIMESTAMP_USE_REALTIME) &&
      clock_gettime(CLOCK_REALTIME, &ts) == 0)
    {
      us =
              ((uint64_t)ts.tv_sec * USEC_PER_SEC) +
              (ts.tv_nsec / NSEC_PER_USEC);
    }
  else
    {
      us = clock_systime_nsec() / NSEC_PER_USEC;
    }

  return monitor_ts_from_us(us);
}

static inline void encode_hdr(
  struct bt_monitor_hdr *hdr, uint64_t timestamp, uint16_t opcode,
  uint16_t len)
{
  struct bt_monitor_ts32 *ts;

  hdr->opcode = sys_cpu_to_le16(opcode);
  hdr->flags = 0U;

  ts = (void *)hdr->ext;
  ts->type = BT_MONITOR_TS32;
  /* The btsnoop protocol only supports 32-bit timestamps (in 1/10th ms).
   * This overflows after 4.97 days, which is acceptable as it only affects
   * rendering in btmon/wireshark. Recordings are usually not that long
   * anyway, and packet ordering is still preserved.
   */

  ts->ts32 = (uint32_t)timestamp;
  hdr->hdr_len = sizeof(*ts);

  encode_drops(hdr, BT_MONITOR_COMMAND_DROPS, &drops.cmd);
  encode_drops(hdr, BT_MONITOR_EVENT_DROPS, &drops.evt);
  encode_drops(hdr, BT_MONITOR_ACL_TX_DROPS, &drops.acl_tx);
  encode_drops(hdr, BT_MONITOR_ACL_RX_DROPS, &drops.acl_rx);
#if defined(CONFIG_BT_CLASSIC)
  encode_drops(hdr, BT_MONITOR_SCO_TX_DROPS, &drops.sco_tx);
  encode_drops(hdr, BT_MONITOR_SCO_RX_DROPS, &drops.sco_rx);
#endif
  encode_drops(hdr, BT_MONITOR_OTHER_DROPS, &drops.other);

  hdr->data_len = sys_cpu_to_le16(4 + hdr->hdr_len + len);
}

void bt_monitor_send(uint16_t opcode, const void *data, size_t len)
{
  struct bt_monitor_hdr hdr;
  struct bt_monitor_data frags[] =
    {
          {&hdr, 0}, /* Length known only after encode_hdr() */
          {
            data,
            len
          },
    };

  if (atomic_test_and_set_bit(&flags, BT_LOG_BUSY))
    {
      drop_add(opcode);
      return;
    }

  encode_hdr(&hdr, monitor_ts_get(), opcode, len);
  frags[0].len = BT_MONITOR_BASE_HDR_LEN + hdr.hdr_len;

  if (!monitor_send(frags, ARRAY_SIZE(frags)))
    {
      restore_drops(&hdr);
      drop_add(opcode);
    }

  atomic_clear_bit(&flags, BT_LOG_BUSY);
}

void bt_monitor_new_index(uint8_t type,
                          uint8_t bus, const bt_addr_t *addr,
                          const char *name)
{
  struct bt_monitor_new_index pkt;

  pkt.type = type;
  pkt.bus = bus;
  memcpy(pkt.bdaddr, addr, 6);
  strncpy(pkt.name, name, sizeof(pkt.name) - 1);
  pkt.name[sizeof(pkt.name) - 1] = '\0';

  bt_monitor_send(BT_MONITOR_NEW_INDEX, &pkt, sizeof(pkt));
}

static int bt_monitor_init(void)
{
  return 0;
}

SYS_INIT(bt_monitor_init, PRE_KERNEL_1, MONITOR_INIT_PRIORITY);
