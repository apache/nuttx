/****************************************************************************
 * net/bt/mesh/sar_cfg_srv.c
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

/* Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/compat/kernel.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
#include <nuttx/net/bluetooth/compat/sys/util.h>
#include <nuttx/net/bluetooth/compat/sys/byteorder.h>

#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/conn.h>
#include <nuttx/net/bluetooth/mesh.h>

#include "net.h"
#include "access.h"
#include "foundation.h"
#include "mesh.h"
#include "sar_cfg_internal.h"
#include "settings.h"

#define LOG_LEVEL CONFIG_BT_MESH_MODEL_LOG_LEVEL
#include <nuttx/net/bluetooth/compat/logging/log.h>
LOG_MODULE_REGISTER(bt_mesh_sar_cfg_srv);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int sar_rx_store(const struct bt_mesh_model *model, bool delete)
{
  const void *data = delete ? NULL : &bt_mesh.sar_rx;
  size_t len = delete ? 0 : sizeof(struct bt_mesh_sar_rx);

  return bt_mesh_model_data_store(model, false, "sar_rx", data, len);
}

static int sar_tx_store(const struct bt_mesh_model *model, bool delete)
{
  const void *data = delete ? NULL : &bt_mesh.sar_tx;
  size_t len = delete ? 0 : sizeof(struct bt_mesh_sar_tx);

  return bt_mesh_model_data_store(model, false, "sar_tx", data, len);
}

static void transmitter_status(const struct bt_mesh_model *model,
                               struct bt_mesh_msg_ctx *ctx)
{
  BT_MESH_MODEL_BUF_DEFINE(msg, OP_SAR_CFG_TX_STATUS, BT_MESH_SAR_TX_LEN);
  const struct bt_mesh_sar_tx *tx = &bt_mesh.sar_tx;

  LOG_DBG("SAR TX {0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x}",
          tx->seg_int_step,
          tx->unicast_retrans_count, tx->unicast_retrans_without_prog_count,
          tx->unicast_retrans_int_step, tx->unicast_retrans_int_inc,
          tx->multicast_retrans_count,
          tx->multicast_retrans_int);

  bt_mesh_model_msg_init(&msg, OP_SAR_CFG_TX_STATUS);
  bt_mesh_sar_tx_encode(&msg, tx);

  if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL))
    {
      LOG_ERR("Unable to send Transmitter Status");
    }
}

static void receiver_status(const struct bt_mesh_model *model,
                            struct bt_mesh_msg_ctx *ctx)
{
  BT_MESH_MODEL_BUF_DEFINE(msg, OP_SAR_CFG_RX_STATUS, BT_MESH_SAR_RX_LEN);
  const struct bt_mesh_sar_rx *rx = &bt_mesh.sar_rx;

  LOG_DBG("SAR RX {0x%02x 0x%02x 0x%02x 0x%02x 0x%02x}", rx->seg_thresh,
          rx->ack_delay_inc,
          rx->discard_timeout, rx->rx_seg_int_step, rx->ack_retrans_count);

  bt_mesh_model_msg_init(&msg, OP_SAR_CFG_RX_STATUS);
  bt_mesh_sar_rx_encode(&msg, rx);

  if (bt_mesh_model_send(model, ctx, &msg, NULL, NULL))
    {
      LOG_ERR("Unable to send Receiver Status");
    }
}

static int transmitter_get(
  const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
  struct net_buf_simple *buf)
{
  LOG_DBG("src 0x%04x", ctx->addr);

  transmitter_status(model, ctx);

  return 0;
}

static int transmitter_set(
  const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
  struct net_buf_simple *buf)
{
  struct bt_mesh_sar_tx *tx = &bt_mesh.sar_tx;

  LOG_DBG("src 0x%04x", ctx->addr);

  bt_mesh_sar_tx_decode(buf, tx);
  transmitter_status(model, ctx);

  if (IS_ENABLED(CONFIG_BT_SETTINGS))
    {
      sar_tx_store(model, false);
    }

  return 0;
}

static int receiver_get(
  const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
  struct net_buf_simple *buf)
{
  LOG_DBG("src 0x%04x", ctx->addr);

  receiver_status(model, ctx);

  return 0;
}

static int receiver_set(
  const struct bt_mesh_model *model, struct bt_mesh_msg_ctx *ctx,
  struct net_buf_simple *buf)
{
  struct bt_mesh_sar_rx *rx = &bt_mesh.sar_rx;

  LOG_DBG("src 0x%04x", ctx->addr);

  bt_mesh_sar_rx_decode(buf, rx);
  receiver_status(model, ctx);

  if (IS_ENABLED(CONFIG_BT_SETTINGS))
    {
      sar_rx_store(model, false);
    }

  return 0;
}

const struct bt_mesh_model_op bt_mesh_sar_cfg_srv_op[] =
{
        {
          OP_SAR_CFG_TX_GET,
          BT_MESH_LEN_EXACT(0),
          transmitter_get
        },

        {
        OP_SAR_CFG_TX_SET,
        BT_MESH_LEN_EXACT(BT_MESH_SAR_TX_LEN),
        transmitter_set
        },

        {
          OP_SAR_CFG_RX_GET,
          BT_MESH_LEN_EXACT(0),
          receiver_get
        },

        {
          OP_SAR_CFG_RX_SET,
          BT_MESH_LEN_EXACT(BT_MESH_SAR_RX_LEN),
          receiver_set
        },

        BT_MESH_MODEL_OP_END,
};

static int sar_cfg_srv_init(const struct bt_mesh_model *model)
{
  if (!bt_mesh_model_in_primary(model))
    {
      LOG_ERR("Configuration Server only allowed in primary element");
      return -EINVAL;
    }

  /* SAR Configuration Model security is device-key based and only the local
   * device-key is allowed to access this model.
   */

  model->keys[0] = BT_MESH_KEY_DEV_LOCAL;
  model->rt->flags |= BT_MESH_MOD_DEVKEY_ONLY;

  return 0;
}

static void sar_cfg_srv_reset(const struct bt_mesh_model *model)
{
  struct bt_mesh_sar_tx sar_tx = BT_MESH_SAR_TX_INIT;
  struct bt_mesh_sar_rx sar_rx = BT_MESH_SAR_RX_INIT;

  bt_mesh.sar_tx = sar_tx;
  bt_mesh.sar_rx = sar_rx;

  if (IS_ENABLED(CONFIG_BT_SETTINGS))
    {
      sar_rx_store(model, true);
      sar_tx_store(model, true);
    }
}

#ifdef CONFIG_BT_SETTINGS
static int sar_cfg_srv_settings_set(
  const struct bt_mesh_model *model, const char *name, size_t len_rd,
  settings_read_cb read_cb, void *cb_data)
{
  if (!strncmp(name, "sar_rx", 5))
    {
      return bt_mesh_settings_set(read_cb, cb_data, &bt_mesh.sar_rx,
              sizeof(bt_mesh.sar_rx));
    }

  if (!strncmp(name, "sar_tx", 5))
    {
      return bt_mesh_settings_set(read_cb, cb_data, &bt_mesh.sar_tx,
              sizeof(bt_mesh.sar_tx));
    }

  return 0;
}
#endif

const struct bt_mesh_model_cb bt_mesh_sar_cfg_srv_cb =
{
  .init = sar_cfg_srv_init,
                                                        .reset =
                                                          sar_cfg_srv_reset,
#ifdef CONFIG_BT_SETTINGS
                                                        .settings_set =
    sar_cfg_srv_settings_set
#endif
};
