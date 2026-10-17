/****************************************************************************
 * net/bt/mesh/dfu_metadata.c
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

#include <nuttx/mm/net_buf.h>
#include <nuttx/net/bluetooth/compat/sys/byteorder.h>
#include <string.h>

#include <nuttx/net/bluetooth/mesh.h>
#include "crypto.h"
#include "access.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bt_mesh_dfu_metadata_decode(struct net_buf_simple *buf,
                                struct bt_mesh_dfu_metadata *metadata)
{
  if (buf->len < 12)
    {
      return -EMSGSIZE;
    }

  metadata->fw_ver.major = net_buf_simple_pull_u8(buf);
  metadata->fw_ver.minor = net_buf_simple_pull_u8(buf);
  metadata->fw_ver.revision = net_buf_simple_pull_le16(buf);
  metadata->fw_ver.build_num = net_buf_simple_pull_le32(buf);
  metadata->fw_size = net_buf_simple_pull_le24(buf);
  metadata->fw_core_type = net_buf_simple_pull_u8(buf);

  if (metadata->fw_core_type & BT_MESH_DFU_FW_CORE_TYPE_APP)
    {
      if (buf->len < 6)
        {
          return -EMSGSIZE;
        }

      metadata->comp_hash = net_buf_simple_pull_le32(buf);
      metadata->elems = net_buf_simple_pull_le16(buf);
    }

  metadata->user_data = buf->len > 0 ? buf->data : NULL;
  metadata->user_data_len = buf->len;

  return 0;
}

int bt_mesh_dfu_metadata_encode(const struct bt_mesh_dfu_metadata *metadata,
                                struct net_buf_simple *buf)
{
  size_t md_len_min = 12 + metadata->user_data_len;

  if (metadata->fw_core_type & BT_MESH_DFU_FW_CORE_TYPE_APP)
    {
      md_len_min += 6;
    }

  if (net_buf_simple_tailroom(buf) < md_len_min)
    {
      return -EMSGSIZE;
    }

  net_buf_simple_add_u8(buf, metadata->fw_ver.major);
  net_buf_simple_add_u8(buf, metadata->fw_ver.minor);
  net_buf_simple_add_le16(buf, metadata->fw_ver.revision);
  net_buf_simple_add_le32(buf, metadata->fw_ver.build_num);
  net_buf_simple_add_le24(buf, metadata->fw_size);
  net_buf_simple_add_u8(buf, metadata->fw_core_type);
  net_buf_simple_add_le32(buf, metadata->comp_hash);
  net_buf_simple_add_le16(buf, metadata->elems);

  if (metadata->user_data_len > 0)
    {
      net_buf_simple_add_mem(buf, metadata->user_data,
              metadata->user_data_len);
    }

  return 0;
}

int bt_mesh_dfu_metadata_comp_hash_get(struct net_buf_simple *buf,
                                       uint8_t *key, uint32_t *hash)
{
  uint8_t mac[16];
  int err;
  struct bt_mesh_sg sg =
    {
        .data = buf->data, .len = buf->len
    };

  err = bt_mesh_aes_cmac_raw_key(key, &sg, 1, mac);
  if (err)
    {
      return err;
    }

  *hash = sys_get_le32(mac);

  return 0;
}

int bt_mesh_dfu_metadata_comp_hash_local_get(uint8_t *key, uint32_t *hash)
{
  NET_BUF_SIMPLE_DEFINE(buf, BT_MESH_TX_SDU_MAX);
  int err;

  err = bt_mesh_comp_data_get_page_0(&buf, 0);
  if (err)
    {
      return err;
    }

  err = bt_mesh_dfu_metadata_comp_hash_get(&buf, key, hash);
  return err;
}
