/****************************************************************************
 * net/bt/mesh/op_agg.c
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

/* Copyright (c) 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/mesh.h>

#include "foundation.h"
#include "op_agg.h"

#define LOG_LEVEL CONFIG_BT_MESH_MODEL_LOG_LEVEL
#include <nuttx/net/bluetooth/compat/logging/log.h>
LOG_MODULE_REGISTER(bt_mesh_op_agg);

#define IS_LENGTH_LONG(buf) ((buf)->data[0] & 1)
#define LENGTH_SHORT_MAX BIT_MASK(7)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bt_mesh_op_agg_encode_msg(struct net_buf_simple *msg,
                              struct net_buf_simple *buf)
{
  if (msg->len > LENGTH_SHORT_MAX)
    {
      if (net_buf_simple_tailroom(buf) < (msg->len + 2))
        {
          return -ENOMEM;
        }

      net_buf_simple_add_le16(buf, (msg->len << 1) | 1);
    }
  else
    {
      if (net_buf_simple_tailroom(buf) < (msg->len + 1))
        {
          return -ENOMEM;
        }

      net_buf_simple_add_u8(buf, msg->len << 1);
    }

  net_buf_simple_add_mem(buf, msg->data, msg->len);

  return 0;
}

int bt_mesh_op_agg_decode_msg(struct net_buf_simple *msg,
                              struct net_buf_simple *buf)
{
  uint16_t len;

  if (IS_LENGTH_LONG(buf))
    {
      if (buf->len < 2)
        {
          return -EINVAL;
        }

      len = net_buf_simple_pull_le16(buf) >> 1;
    }
  else
    {
      if (buf->len < 1)
        {
          return -EINVAL;
        }

      len = net_buf_simple_pull_u8(buf) >> 1;
    }

  if (buf->len < len)
    {
      return -EINVAL;
    }

  net_buf_simple_init_with_data(msg, net_buf_simple_pull_mem(buf, len), len);

  return 0;
}

bool bt_mesh_op_agg_is_op_agg_msg(struct net_buf_simple *buf)
{
  if (buf->len >= 2 && (buf->data[0] >> 6) == 2)
    {
      uint16_t opcode;
      struct net_buf_simple_state state;

      net_buf_simple_save(buf, &state);
      opcode = net_buf_simple_pull_be16(buf);
      net_buf_simple_restore(buf, &state);

      if ((opcode == OP_OPCODES_AGGREGATOR_STATUS) ||
          (opcode == OP_OPCODES_AGGREGATOR_SEQUENCE))
        {
          return true;
        }
    }

  return false;
}
