/****************************************************************************
 * net/bt/mesh/delayable_msg.c
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

/* Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/compat/kernel.h>
#include <errno.h>
#include <stdlib.h>

#include <nuttx/spinlock.h>

#include <nuttx/list.h>

#include <nuttx/mm/net_buf.h>
#include <nuttx/net/bluetooth/mesh.h>

#include "msg.h"
#include "access.h"
#include "net.h"

#define LOG_LEVEL CONFIG_BT_MESH_ACCESS_LOG_LEVEL
#include <nuttx/net/bluetooth/compat/logging/log.h>
LOG_MODULE_REGISTER(bt_mesh_delayable_msg);

static void delayable_msg_handler(struct k_work *w);
static bool push_msg_from_delayable_msgs(void);

static struct delayable_msg_chunk
{
  struct list_node node;
  uint8_t data[CONFIG_BT_MESH_ACCESS_DELAYABLE_MSG_CHUNK_SIZE];
}

delayable_msg_chunks[CONFIG_BT_MESH_ACCESS_DELAYABLE_MSG_CHUNK_COUNT];

static struct delayable_msg_ctx
{
  struct list_node node;
  struct list_node chunks;
  struct bt_mesh_msg_ctx ctx;
  uint16_t src_addr;
  const struct bt_mesh_send_cb *cb;
  void *cb_data;
  uint32_t fired_time;
  uint16_t len;
}

delayable_msgs_ctx[CONFIG_BT_MESH_ACCESS_DELAYABLE_MSG_COUNT];

static struct
{
  struct list_node busy_ctx;
  struct list_node free_ctx;
  struct list_node free_chunks;
  struct k_work_delayable random_delay;
}

access_delayable_msg =
{
    .random_delay = Z_WORK_DELAYABLE_INITIALIZER(delayable_msg_handler)
};

/* Serializes access to busy_ctx, free_ctx, free_chunks, and the fired_time
 * field of any ctx currently on busy_ctx. Must never be held across a call
 * that can block (bt_mesh_access_send(), bt_rand(), user callbacks).
 */

static spinlock_t lock;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Caller must hold `lock`. */

static void put_ctx_to_busy_list(struct delayable_msg_ctx *ctx)
{
  struct delayable_msg_ctx *curr_ctx;
  struct list_node *list = &access_delayable_msg.busy_ctx;
  struct list_node *head = list_peek_head(list);
  struct list_node *curr = head;
  struct list_node *prev = curr;

  if (!head)
    {
      list_add_tail(list, &ctx->node);
      return;
    }

  do
    {
      curr_ctx = CONTAINER_OF(curr, struct delayable_msg_ctx, node);
      if (ctx->fired_time < curr_ctx->fired_time)
        {
          if (curr == head)
            {
              list_add_head(list, &ctx->node);
            }
          else
            {
              list_insert_after(list, prev, &ctx->node);
            }

          return;
        }

      prev = curr;
    }
  while ((curr = list_next(list, curr)));

  list_add_tail(list, &ctx->node);
}

static struct delayable_msg_ctx *peek_pending_msg(void)
{
  struct delayable_msg_ctx *pending_msg = NULL;
  struct list_node *node = list_peek_head(&access_delayable_msg.busy_ctx);

  if (node)
    {
      pending_msg = CONTAINER_OF(node, struct delayable_msg_ctx, node);
    }

  return pending_msg;
}

static void reschedule_delayable_msg(struct delayable_msg_ctx *msg)
{
  int32_t remaining;
  clock_t delay = 0;
  struct delayable_msg_ctx *pending_msg;
  irqstate_t key = spin_lock_irqsave(&lock);

  if (msg)
    {
      put_ctx_to_busy_list(msg);
    }

  pending_msg = peek_pending_msg();

  if (!pending_msg)
    {
      spin_unlock_irqrestore(&lock, key);
      return;
    }

  remaining =
          (int32_t)(pending_msg->fired_time -
          (uint32_t)TICK2MSEC(clock_systime_ticks()));
  if (remaining > 0)
    {
      delay = MSEC2TICK(remaining);
    }

  /* k_work_reschedule() is safe to call under our spinlock: it uses its own
   * internal spinlock and does not block. Holding the lock here makes the
   * "select head + arm timer" step atomic w.r.t. concurrent insertions.
   */

  k_work_reschedule(&access_delayable_msg.random_delay, delay);
  spin_unlock_irqrestore(&lock, key);
}

static int allocate_delayable_msg_chunks(struct delayable_msg_ctx *msg,
                                         int number)
{
  struct list_node *node;
  irqstate_t key = spin_lock_irqsave(&lock);

  for (int i = 0; i < number; i++)
    {
      node = list_remove_head(&access_delayable_msg.free_chunks);
      if (!node)
        {
          spin_unlock_irqrestore(&lock, key);
          LOG_WRN("Unable allocate %u chunks, allocated %u", number, i);
          return i;
        }

      list_add_tail(&msg->chunks, node);
    }

  spin_unlock_irqrestore(&lock, key);
  return number;
}

static void release_delayable_msg_chunks(struct delayable_msg_ctx *msg)
{
  struct list_node *node;
  irqstate_t key = spin_lock_irqsave(&lock);

  while ((node = list_remove_head(&msg->chunks)))
    {
      list_add_tail(&access_delayable_msg.free_chunks, node);
    }

  spin_unlock_irqrestore(&lock, key);
}

static struct delayable_msg_ctx *allocate_delayable_msg_ctx(void)
{
  struct delayable_msg_ctx *msg;
  struct list_node *node;
  irqstate_t key;

  key = spin_lock_irqsave(&lock);
  node = list_remove_head(&access_delayable_msg.free_ctx);
  spin_unlock_irqrestore(&lock, key);

  if (!node)
    {
      LOG_WRN("Purge pending delayable message.");

      /* May block, so it must not be called with the lock held. */

      if (!push_msg_from_delayable_msgs())
        {
          return NULL;
        }

      key = spin_lock_irqsave(&lock);
      node = list_remove_head(&access_delayable_msg.free_ctx);
      spin_unlock_irqrestore(&lock, key);

      /* Another context may have taken the purged ctx before we retried. */

      if (!node)
        {
          return NULL;
        }
    }

  msg = CONTAINER_OF(node, struct delayable_msg_ctx, node);

  /* msg is off every list here, so it is private to this caller. */

  list_initialize(&msg->chunks);

  return msg;
}

static void release_delayable_msg_ctx(struct delayable_msg_ctx *ctx)
{
  irqstate_t key = spin_lock_irqsave(&lock);

  /* Not on busy_ctx when called from the manage() error path. */

  (void)list_find_and_remove(&access_delayable_msg.busy_ctx, &ctx->node);
  list_add_tail(&access_delayable_msg.free_ctx, &ctx->node);
  spin_unlock_irqrestore(&lock, key);
}

/* Releases `msg` and reports `err` to its sender, which is skipped for a
 * successful send because the transport owns the callback from then on. Must
 * not be called with the lock held.
 */

static void complete_delayable_msg(struct delayable_msg_ctx *msg, int err)
{
  /* Capture the callback before releasing the ctx; another caller may take
   * it immediately and overwrite msg->cb / msg->cb_data.
   */

  const struct bt_mesh_send_cb *cb = msg->cb;
  void *cb_data = msg->cb_data;

  release_delayable_msg_chunks(msg);
  release_delayable_msg_ctx(msg);

  if (err && cb && cb->start)
    {
      cb->start(0, err, cb_data);
    }
}

/* Signed delta keeps this correct across the 32-bit uptime wrap. */

static bool msg_expired(const struct delayable_msg_ctx *msg)
{
  return (int32_t)((uint32_t)TICK2MSEC(clock_systime_ticks()) -
          msg->fired_time) >= 0;
}

/* Takes the head off busy_ctx only if it is due. k_work_reschedule() cannot
 * withdraw a submission that already happened, so the handler can run for an
 * expiry another context has consumed; this leaves such an invocation with
 * nothing to send.
 */

static struct delayable_msg_ctx *take_expired_msg(void)
{
  struct delayable_msg_ctx *msg;
  irqstate_t key = spin_lock_irqsave(&lock);

  msg = peek_pending_msg();
  if (msg && !msg_expired(msg))
    {
      msg = NULL;
    }
  else if (msg)
    {
      (void)list_remove_head(&access_delayable_msg.busy_ctx);
    }

  spin_unlock_irqrestore(&lock, key);

  return msg;
}

/* Takes the head whatever its fired_time, so the purge path can force out
 * the earliest msg.
 */

static struct delayable_msg_ctx *take_earliest_msg(void)
{
  struct list_node *node;
  irqstate_t key = spin_lock_irqsave(&lock);

  /* Dequeueing under the lock stops concurrent callers from sending and
   * releasing the same msg twice.
   */

  node = list_remove_head(&access_delayable_msg.busy_ctx);
  spin_unlock_irqrestore(&lock, key);

  return node ? CONTAINER_OF(node, struct delayable_msg_ctx, node) : NULL;
}

/* Sends `msg` and releases it. Returns false if a transient failure put it
 * back on busy_ctx.
 */

static bool send_delayable_msg(struct delayable_msg_ctx *msg)
{
  struct list_node *node;
  struct delayable_msg_chunk *chunk;
  uint16_t len = msg->len;
  int err;
  irqstate_t key;

  /* bt_mesh_suspend() sets BT_MESH_SUSPENDED and bt_mesh_reset() clears
   * BT_MESH_VALID before the queue is drained, so either one means the stack
   * is down. Never hand a message to the transport in that state, however it
   * ended up on busy_ctx.
   */

  if (atomic_test_bit(bt_mesh.flags, BT_MESH_SUSPENDED) ||
      !atomic_test_bit(bt_mesh.flags, BT_MESH_VALID))
    {
      complete_delayable_msg(msg, -ENODEV);
      return true;
    }

  NET_BUF_SIMPLE_DEFINE(buf, BT_MESH_TX_SDU_MAX);

  /* msg is off busy_ctx, so it is unreachable from other contexts and its
   * chunks can be walked unlocked.
   */

  list_for_every(&msg->chunks, node)
  {
    uint16_t tmp =
            MIN((uint16_t)CONFIG_BT_MESH_ACCESS_DELAYABLE_MSG_CHUNK_SIZE,
            len);

    chunk = CONTAINER_OF(node, struct delayable_msg_chunk, node);
    memcpy(net_buf_simple_add(&buf, tmp), chunk->data, tmp);
    len -= tmp;
  }

  msg->ctx.rnd_delay = false;

  /* Blocking call: must NOT hold the spinlock. */

  err = bt_mesh_access_send(&msg->ctx, &buf, msg->src_addr, msg->cb,
          msg->cb_data);
  msg->ctx.rnd_delay = true;

  if (err == -EBUSY || err == -ENOBUFS)
    {
      /* Transient failure: retry the msg 10 ms later, re-sorted into
       * busy_ctx, and re-arm the timer for whatever is now the head.
       */

      key = spin_lock_irqsave(&lock);
      msg->fired_time += 10;
      put_ctx_to_busy_list(msg);
      spin_unlock_irqrestore(&lock, key);

      reschedule_delayable_msg(NULL);
      return false;
    }

  /* User callback: must NOT hold the spinlock. */

  complete_delayable_msg(msg, err);

  return true;
}

static bool push_msg_from_delayable_msgs(void)
{
  struct delayable_msg_ctx *msg = take_earliest_msg();

  if (!msg)
    {
      return false;
    }

  return send_delayable_msg(msg);
}

static void delayable_msg_handler(struct k_work *w)
{
  struct delayable_msg_ctx *msg = take_expired_msg();

  if (msg)
    {
      (void)send_delayable_msg(msg);
    }

  /* Re-arms for the next msg, immediately if one is already due. */

  reschedule_delayable_msg(NULL);
}

int bt_mesh_delayable_msg_manage(
  struct bt_mesh_msg_ctx *ctx, struct net_buf_simple *buf, uint16_t src_addr,
  const struct bt_mesh_send_cb *cb, void *cb_data)
{
  struct list_node *node;
  struct delayable_msg_ctx *msg;
  uint16_t random_delay;
  int total_number = DIV_ROUND_UP(buf->len,
          CONFIG_BT_MESH_ACCESS_DELAYABLE_MSG_CHUNK_SIZE);
  int allocated_number = 0;
  uint16_t len = buf->len;

  if (atomic_test_bit(bt_mesh.flags, BT_MESH_SUSPENDED))
    {
      LOG_WRN("Refusing to allocate message context while suspended");
      return -ENODEV;
    }

  if (total_number > CONFIG_BT_MESH_ACCESS_DELAYABLE_MSG_CHUNK_COUNT)
    {
      return -EINVAL;
    }

  msg = allocate_delayable_msg_ctx();
  if (!msg)
    {
      LOG_WRN("No available free delayable message context.");
      return -ENOMEM;
    }

  do
    {
      allocated_number += allocate_delayable_msg_chunks(msg,
              total_number - allocated_number);

      if (total_number > allocated_number)
        {
          LOG_DBG("Unable allocate %u chunks, allocated %u", total_number,
                  allocated_number);
          if (!push_msg_from_delayable_msgs())
            {
              LOG_WRN("No available chunk memory.");
              release_delayable_msg_chunks(msg);
              release_delayable_msg_ctx(msg);
              return -ENOMEM;
            }
        }
    }
  while (total_number > allocated_number);

  list_for_every(&msg->chunks, node)
  {
    uint16_t tmp = MIN(CONFIG_BT_MESH_ACCESS_DELAYABLE_MSG_CHUNK_SIZE,
            buf->len);

    struct delayable_msg_chunk *chunk = CONTAINER_OF(node,
            struct delayable_msg_chunk, node);

    memcpy(chunk->data, net_buf_simple_pull_mem(buf, tmp), tmp);
  }

  bt_rand(&random_delay, sizeof(uint16_t));
  random_delay =
          20 +
              random_delay % (BT_MESH_ADDR_IS_UNICAST(ctx->recv_dst) ?
                30 : 480);
  msg->fired_time =
      (uint32_t)TICK2MSEC(clock_systime_ticks()) + random_delay;
  msg->ctx = *ctx;
  msg->src_addr = src_addr;
  msg->cb = cb;
  msg->cb_data = cb_data;
  msg->len = len;

  reschedule_delayable_msg(msg);

  return 0;
}

void bt_mesh_delayable_msg_init(void)
{
  list_initialize(&access_delayable_msg.busy_ctx);
  list_initialize(&access_delayable_msg.free_ctx);
  list_initialize(&access_delayable_msg.free_chunks);

  for (int i = 0; i < CONFIG_BT_MESH_ACCESS_DELAYABLE_MSG_COUNT; i++)
    {
      list_add_tail(&access_delayable_msg.free_ctx,
              &delayable_msgs_ctx[i].node);
    }

  for (int i = 0; i < CONFIG_BT_MESH_ACCESS_DELAYABLE_MSG_CHUNK_COUNT; i++)
    {
      list_add_tail(&access_delayable_msg.free_chunks,
              &delayable_msg_chunks[i].node);
    }
}

void bt_mesh_delayable_msg_stop(void)
{
  struct list_node *node;
  irqstate_t key;

  k_work_cancel_delayable(&access_delayable_msg.random_delay);

  for (; ; )
    {
      key = spin_lock_irqsave(&lock);
      node = list_remove_head(&access_delayable_msg.busy_ctx);
      spin_unlock_irqrestore(&lock, key);

      if (!node)
        {
          break;
        }

      complete_delayable_msg(CONTAINER_OF(node, struct delayable_msg_ctx,
              node), -ENODEV);
    }
}
