/****************************************************************************
 * net/bt/audio/pacs_internal.h
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

/* @file
 * @brief Internal APIs for PACS handling
 *
 * Copyright (c) 2020 Intel Corporation Copyright (c) 2022 Nordic
 * Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/net/bluetooth/audio/audio.h>

#define BT_AUDIO_LOCATION_MASK BIT_MASK(28)

struct bt_pac_codec
{
  uint8_t id;   /* Codec ID */
  uint16_t cid; /* Company ID */
  uint16_t vid; /* Vendor specific Codec ID */
} __packed;

struct bt_pac_ltv
{
  uint8_t len;
  uint8_t type;
  uint8_t value[];
} __packed;

struct bt_pac_ltv_data
{
  uint8_t len;
  struct bt_pac_ltv data[];
} __packed;

struct bt_pacs_read_rsp
{
  uint8_t num_pac; /* Number of PAC Records */
} __packed;

struct bt_pacs_context
{
  uint16_t snk;
  uint16_t src;
} __packed;

const struct bt_audio_codec_cap *bt_pacs_get_codec_cap(enum bt_audio_dir dir,
  const struct bt_pac_codec *codec_id);
