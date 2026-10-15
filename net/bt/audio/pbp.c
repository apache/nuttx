/****************************************************************************
 * net/bt/audio/pbp.c
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

/* Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

#include <nuttx/config.h>
#include <nuttx/net/bluetooth/assigned_numbers.h>
#include <nuttx/net/bluetooth/audio/audio.h>
#include <nuttx/net/bluetooth/audio/pbp.h>
#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/data.h>
#include <nuttx/net/bluetooth/gap.h>
#include <nuttx/net/bluetooth/uuid.h>
#include <nuttx/net/bluetooth/compat/logging/log.h>
#include <nuttx/mm/net_buf.h>
#include <nuttx/net/bluetooth/compat/sys/byteorder.h>
#include <stdbool.h>
#include <sys/types.h>

LOG_MODULE_REGISTER(bt_pbp, CONFIG_BT_PBP_LOG_LEVEL);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bt_pbp_get_announcement(const uint8_t meta[], size_t meta_len,
                            enum bt_pbp_announcement_feature features,
                            struct net_buf_simple *pba_data_buf)
{
  if (pba_data_buf == NULL)
    {
      LOG_DBG("No buffer provided for advertising data!\n");

      return -EINVAL;
    }

  if ((meta == NULL && meta_len != 0U) || (meta != NULL && meta_len == 0U))
    {
      LOG_DBG("Invalid metadata combination: %p %zu", meta, meta_len);

      return -EINVAL;
    }

  if (pba_data_buf->size < (meta_len + BT_PBP_MIN_PBA_SIZE))
    {
      LOG_DBG("Buffer size needs to be at least %d!\n",
              meta_len + BT_PBP_MIN_PBA_SIZE);

      return -EINVAL;
    }

  /* Fill Announcement data */

  net_buf_simple_add_le16(pba_data_buf, BT_UUID_PBA_VAL);
  net_buf_simple_add_u8(pba_data_buf, features);
  net_buf_simple_add_u8(pba_data_buf, meta_len);
  net_buf_simple_add_mem(pba_data_buf, meta, meta_len);

  return 0;
}

int bt_pbp_parse_announcement(struct bt_data *data,
                              enum bt_pbp_announcement_feature *features,
                              uint8_t **meta)
{
  struct bt_uuid_16 adv_uuid;
  struct net_buf_simple buf;
  uint8_t meta_len = 0U;
  void *uuid;

  if (!data || !features || !meta)
    {
      return -EINVAL;
    }

  if (data->type != BT_DATA_SVC_DATA16)
    {
      return -ENOENT;
    }

  if (data->data_len < BT_PBP_MIN_PBA_SIZE)
    {
      return -EMSGSIZE;
    }

  net_buf_simple_init_with_data(&buf, (void *)data->data, data->data_len);
  uuid = net_buf_simple_pull_mem(&buf, BT_UUID_SIZE_16);

  (void)bt_uuid_create(&adv_uuid.uuid, uuid,
          BT_UUID_SIZE_16); /* cannot fail */

  if (bt_uuid_cmp(&adv_uuid.uuid, BT_UUID_PBA))
    {
      return -ENOENT;
    }

  /* Copy source features, metadata length and metadata from the Announcement
   */

  *features = net_buf_simple_pull_u8(&buf);
  meta_len = net_buf_simple_pull_u8(&buf);
  if (buf.len < meta_len)
    {
      return -EBADMSG;
    }

  *meta = (uint8_t *)net_buf_simple_pull_mem(&buf, meta_len);

  return meta_len;
}
