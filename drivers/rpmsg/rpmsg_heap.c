/****************************************************************************
 * drivers/rpmsg/rpmsg_heap.c
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

#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/rpmsg/rpmsg.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define RPMSG_MM_ALLOC_REQ   1
#define RPMSG_MM_ALLOC_RESP  2
#define RPMSG_MM_FREE_REQ    3
#define RPMSG_MM_FREE_RESP   4

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rpmsg_heap_cookie_s
{
  sem_t    sem;
  int32_t  result;
  FAR void *data;
};

begin_packed_struct struct rpmsg_heap_msg_s
{
  uint32_t cmd;
  int32_t  result;
  uint32_t size;
  uint32_t align;
  uint64_t paddr;
  uint64_t cookie;
} end_packed_struct;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int rpmsg_heap_ept_cb(FAR struct rpmsg_endpoint *ept,
                             FAR void *data, size_t len, uint32_t src,
                             FAR void *priv)
{
  FAR struct rpmsg_heap_msg_s *msg = data;
  FAR struct rpmsg_heap_cookie_s *cookie =
          (FAR struct rpmsg_heap_cookie_s *)(uintptr_t)msg->cookie;
  int ret = -EINVAL;

  if (msg->cmd == RPMSG_MM_ALLOC_RESP)
    {
      cookie->data = up_addrenv_pa_to_va(msg->paddr);
      cookie->result = msg->result;
      ret = rpmsg_post(ept, &cookie->sem);
    }
  else if (msg->cmd == RPMSG_MM_FREE_RESP)
    {
      cookie->result = msg->result;
      ret = rpmsg_post(ept, &cookie->sem);
    }
  else if (msg->cmd == RPMSG_MM_ALLOC_REQ)
    {
      msg->paddr = (uintptr_t)up_addrenv_va_to_pa(
                    rpmsg_alloc_buf(ept->rdev, msg->size, msg->align));
      msg->result = msg->paddr ? 0 : -ENOMEM;
      msg->cmd = RPMSG_MM_ALLOC_RESP;
      ret = rpmsg_send(ept, msg, sizeof(struct rpmsg_heap_msg_s));
    }
  else if (msg->cmd == RPMSG_MM_FREE_REQ)
    {
      rpmsg_free_buf(ept->rdev, up_addrenv_pa_to_va(msg->paddr));
      msg->cmd = RPMSG_MM_FREE_RESP;
      msg->result = 0;
      ret = rpmsg_send(ept, msg, sizeof(struct rpmsg_heap_msg_s));
    }

  if (ret < 0)
    {
      rpmsgerr("rpmsg_heap_ept_cb failed %d cmd %" PRIu32 "\n",
               ret, msg->cmd);
    }

  return ret;
}

static FAR void *rpmsg_heap_get_tx_payload_buffer(
                 FAR struct rpmsg_endpoint *ept, FAR uint32_t *len)
{
  FAR sem_t *wait = ept->priv;
  int sval;

  nxsem_get_value(wait, &sval);
  if (sval <= 0)
    {
      rpmsg_wait(ept, wait);
      rpmsg_post(ept, wait);
    }

  return rpmsg_get_tx_payload_buffer(ept, len, true);
}

static FAR void *rpmsg_heap_send_recv(FAR struct rpmsg_endpoint *ept,
                                      uint32_t command, size_t size,
                                      size_t align, uint64_t paddr)
{
  FAR struct rpmsg_heap_cookie_s *cookie;
  FAR struct rpmsg_heap_msg_s *msg;
  FAR void *data = NULL;
  uint32_t len;
  int ret;

  msg = rpmsg_heap_get_tx_payload_buffer(ept, &len);
  if (!msg)
    {
      return NULL;
    }

  DEBUGASSERT(sizeof(struct rpmsg_heap_msg_s) <= len);

  msg->cmd = command;
  msg->result = -EINVAL;
  msg->size = size;
  msg->align = align;
  msg->paddr = paddr;

  cookie = rpmsg_pool_alloc(sizeof(*cookie));
  memset(cookie, 0, sizeof(*cookie));
  nxsem_init(&cookie->sem, 0, 0);
  msg->cookie = (uintptr_t)cookie;

  ret = rpmsg_send_nocopy(ept, msg, sizeof(struct rpmsg_heap_msg_s));
  if (ret >= 0)
    {
      rpmsg_wait(ept, &cookie->sem);
    }
  else
    {
      rpmsgerr("rpmsg_heap_send_recv send failed: %d\n", ret);
      rpmsg_release_tx_buffer(ept, msg);
      goto out;
    }

  if (cookie->result != 0)
    {
      rpmsgerr("rpmsg_heap_send_recv failed: %" PRIi32 "\n", cookie->result);
    }
  else
    {
      data = cookie->data;
    }

out:
  nxsem_destroy(&cookie->sem);
  rpmsg_pool_free(cookie);
  return data;
}

static void rpmsg_heap_ns_bound(struct rpmsg_endpoint *ept)
{
  FAR sem_t *wait = ept->priv;

  rpmsg_post(ept, wait);
}

static void rpmsg_heap_ns_unbind(FAR struct rpmsg_endpoint *ept)
{
  FAR sem_t *wait = ept->priv;

  rpmsg_wait(ept, wait);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR void *rpmsg_heap_alloc(FAR struct rpmsg_endpoint *ept,
                           size_t size, size_t align)
{
  return rpmsg_heap_send_recv(ept, RPMSG_MM_ALLOC_REQ, size, align, 0);
}

void rpmsg_heap_free(FAR struct rpmsg_endpoint *ept, FAR void *addr)
{
  rpmsg_heap_send_recv(ept, RPMSG_MM_FREE_REQ, 0, 0,
                       up_addrenv_va_to_pa(addr));
}

int rpmsg_heap_init(FAR struct rpmsg_device *rdev,
                    FAR struct rpmsg_endpoint *ept)
{
  FAR sem_t *wait;
  int ret = OK;

  wait = kmm_zalloc(sizeof(*wait));
  if (!wait)
    {
      rpmsgerr("rpmsg_heap_init alloc failed: %d\n", ret);
      return -ENOMEM;
    }

  nxsem_init(wait, 0, 0);
  ept->priv = wait;
  ept->ns_bound_cb = rpmsg_heap_ns_bound;
  ret = rpmsg_create_ept(ept, rdev, "rpmsg-heap",
                         RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                         rpmsg_heap_ept_cb, rpmsg_heap_ns_unbind);
  if (ret < 0)
    {
      rpmsgerr("rpmsg_heap_init failed: %d\n", ret);
      nxsem_destroy(wait);
    }

  return ret;
}

void rpmsg_heap_deinit(FAR struct rpmsg_endpoint *ept)
{
  FAR sem_t *wait = ept->priv;
  int sval;

  nxsem_get_value(wait, &sval);
  if (sval > 0)
    {
      rpmsg_wait(ept, wait);
    }

  rpmsg_destroy_ept(ept);
  nxsem_destroy(wait);
  kmm_free(wait);
}
