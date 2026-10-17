/****************************************************************************
 * net/bt/host/data.c
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

/* Copyright (c) 2017-2021 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
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

#include <nuttx/config.h>
#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/compat/logging/log.h>
#include <nuttx/mm/net_buf.h>

#define LOG_LEVEL CONFIG_BT_HCI_CORE_LOG_LEVEL
LOG_MODULE_REGISTER(bt_data);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void bt_data_parse(struct net_buf_simple *ad,
                   bool (*func)(struct bt_data *data, void *user_data),
                   void *user_data)
{
  while (ad->len > 1)
    {
      struct bt_data data;
      uint8_t len;

      len = net_buf_simple_pull_u8(ad);
      if (len == 0U)
        {
          /* Early termination */

          return;
        }

      if (len > ad->len)
        {
          LOG_WRN("malformed advertising data %u / %u", len, ad->len);
          return;
        }

      data.type = net_buf_simple_pull_u8(ad);
      data.data_len = len - 1;
      data.data = ad->data;

      if (!func(&data, user_data))
        {
          return;
        }

      net_buf_simple_pull(ad, len - 1);
    }
}

size_t bt_data_get_len(const struct bt_data data[], size_t data_count)
{
  size_t total_len = 0;

  for (size_t i = 0; i < data_count; i++)
    {
      total_len +=
              sizeof(data[i].data_len) + sizeof(data[i].type) +
              data[i].data_len;
    }

  return total_len;
}

size_t bt_data_serialize(const struct bt_data *input, uint8_t *output)
{
  if (input == NULL)
    {
      LOG_DBG("input is NULL");
      return 0;
    }

  if (output == NULL)
    {
      LOG_DBG("output is NULL");
      return 0;
    }

  uint8_t ad_data_len = input->data_len;
  uint8_t data_len = ad_data_len + 1;

  output[0] = data_len;
  output[1] = input->type;

  memcpy(&output[2], input->data, ad_data_len);

  return data_len + 1;
}
