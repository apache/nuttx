/****************************************************************************
 * net/bt/host/monitor_buffer.h
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

/* Copyright (c) 2026 Silicon Laboratories Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_BLUETOOTH_HOST_MONITOR_BUFFER_H_
#define ZEPHYR_SUBSYS_BLUETOOTH_HOST_MONITOR_BUFFER_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/circbuf.h>
#include <nuttx/arch.h>
#include <nuttx/net/bluetooth/compat/sys/util.h>

struct bt_monitor_data
{
  const void *data;
  size_t len;
};

/* Header-only so that the unit test in tests/bluetooth/host/monitor_buffer
 * can build this function without pulling in monitor.c and its dependencies.
 * monitor.c is the only in-tree includer, so the generated code is the same
 * as for a file-local static function.
 */

static inline bool bt_monitor_ring_buf_put(struct circbuf_s *buf,
  const struct bt_monitor_data *frags,
  size_t count)
{
  size_t space = circbuf_space(buf);
  size_t total = 0;

  /* total only grows after passing the check, so total <= space is
   * invariant: the subtraction cannot underflow, the sum cannot overflow,
   * and offset/remaining in the copy loop below stay bounded by the buffer
   * size.
   */

  for (size_t i = 0; i < count; i++)
    {
      if (frags[i].len > space - total)
        {
          return false;
        }

      total += frags[i].len;
    }

  /* Pairs with bt_monitor_ring_buf_consume(): space observed above may
   * have been freed by a consumer on another CPU, whose data reads must
   * complete before the writes below reuse it.
   */

  SMP_MB();

  /* The space check guarantees that the writes below always fit, and
   * the single commit makes the complete record visible to the consumer at
   * once.
   */

  for (size_t i = 0, offset = 0; i < count; i++)
    {
      const uint8_t *src = frags[i].data;
      size_t remaining = frags[i].len;

      while (remaining > 0)
        {
          uint8_t *dst;
          size_t len;
          size_t off;

          off = (buf->head + offset) % buf->size;
          dst = (uint8_t *)buf->base + off;
          len = MIN(remaining, buf->size - off);

          (void)memcpy(dst, src, len);
          offset += len;
          src += len;
          remaining -= len;
        }
    }

  /* Make the record bytes visible before the commit publishes them
   * (pairs with bt_monitor_ring_buf_get_ptr()).
   */

  SMP_MB();
  circbuf_writecommit(buf, total);

  return true;
}

/* Consumer-side wrappers adding the memory ordering that the ring buffer's
 * lock-free SPSC contract leaves to its users on SMP systems. The producer
 * side is handled in bt_monitor_ring_buf_put().
 */

static inline uint32_t bt_monitor_ring_buf_get_ptr(struct circbuf_s *buf,
                                                   uint8_t **data)
{
  size_t len;

  *data = circbuf_get_readptr(buf, &len);

  /* Order the index load against the data reads that follow */

  SMP_MB();

  return len;
}

static inline void bt_monitor_ring_buf_consume(struct circbuf_s *buf,
                                               uint32_t size)
{
  /* Order the data reads against the space becoming reusable */

  SMP_MB();
  circbuf_readcommit(buf, size);
}

#endif /* ZEPHYR_SUBSYS_BLUETOOTH_HOST_MONITOR_BUFFER_H_ */
