/****************************************************************************
 * net/bt/audio/micp_internal.h
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
 *  @brief Internal APIs for Bluetooth MICP.
 */

/* Copyright (c) 2020 Bose Corporation
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_AUDIO_MICP_INTERNAL_
#define ZEPHYR_INCLUDE_BLUETOOTH_AUDIO_MICP_INTERNAL_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/config.h>
#include <nuttx/net/bluetooth/conn.h>
#include <nuttx/net/bluetooth/gatt.h>
#include <nuttx/net/bluetooth/compat/sys/util.h>

enum bt_micp_mic_ctlr_flag
{
  BT_MICP_MIC_CTLR_FLAG_BUSY,

  BT_MICP_MIC_CTLR_FLAG_NUM_FLAGS, /* keep as last */
};

struct bt_micp_mic_ctlr
{
  uint16_t start_handle;
  uint16_t end_handle;
  uint16_t mute_handle;
  struct bt_gatt_subscribe_params mute_sub_params;
  struct bt_gatt_discover_params mute_sub_disc_params;

  uint8_t mute_val_buf[1]; /* Mute value is a single octet */
  struct bt_gatt_write_params write_params;
  struct bt_gatt_read_params read_params;
  struct bt_gatt_discover_params discover_params;
  struct bt_conn *conn;

#if defined(CONFIG_BT_MICP_MIC_CTLR_AICS)
  uint8_t aics_inst_cnt;
  struct bt_aics *aics[CONFIG_BT_MICP_MIC_CTLR_MAX_AICS_INST];
#endif /* CONFIG_BT_MICP_MIC_CTLR_AICS */

  atomic_define(flags, BT_MICP_MIC_CTLR_FLAG_NUM_FLAGS);
};

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_AUDIO_MICP_INTERNAL_ */
