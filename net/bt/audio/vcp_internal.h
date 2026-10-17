/****************************************************************************
 * net/bt/audio/vcp_internal.h
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

/**
 * @file
 * @brief Internal Header for Bluetooth Volume Control Service (VCS).
 *
 * Copyright (c) 2020 Bose Corporation Copyright (c) 2020-2022 Nordic
 * Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_AUDIO_VCP_INTERNAL_
#define ZEPHYR_INCLUDE_BLUETOOTH_AUDIO_VCP_INTERNAL_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/config.h>
#include <nuttx/net/bluetooth/conn.h>
#include <nuttx/net/bluetooth/gatt.h>
#include <nuttx/net/bluetooth/uuid.h>
#include <nuttx/net/bluetooth/compat/sys/util.h>

/* VCS opcodes */
#define BT_VCP_OPCODE_REL_VOL_DOWN 0x00U
#define BT_VCP_OPCODE_REL_VOL_UP 0x01U
#define BT_VCP_OPCODE_UNMUTE_REL_VOL_DOWN 0x02U
#define BT_VCP_OPCODE_UNMUTE_REL_VOL_UP 0x03U
#define BT_VCP_OPCODE_SET_ABS_VOL 0x04U
#define BT_VCP_OPCODE_UNMUTE 0x05U
#define BT_VCP_OPCODE_MUTE 0x06U

struct vcs_state
{
  uint8_t volume;
  uint8_t mute;
  uint8_t change_counter;
} __packed;

struct vcs_control
{
  uint8_t opcode;
  uint8_t counter;
} __packed;

struct vcs_control_vol
{
  struct vcs_control cp;
  uint8_t volume;
} __packed;

enum bt_vcp_vol_ctlr_flag
{
  BT_VCP_VOL_CTLR_FLAG_BUSY,
  BT_VCP_VOL_CTLR_FLAG_CP_RETRIED,

  BT_VCP_VOL_CTLR_FLAG_NUM_FLAGS, /* keep as last */
};

struct bt_vcp_vol_ctlr
{
  struct vcs_state state;
  uint8_t vol_flags;

  uint16_t start_handle;
  uint16_t end_handle;
  uint16_t state_handle;
  uint16_t control_handle;
  uint16_t vol_flag_handle;
  struct bt_gatt_subscribe_params state_sub_params;
  struct bt_gatt_discover_params state_sub_disc_params;
  struct bt_gatt_subscribe_params vol_flag_sub_params;
  struct bt_gatt_discover_params vol_flag_sub_disc_params;

  struct vcs_control_vol cp_val;
  struct bt_gatt_write_params write_params;
  struct bt_gatt_read_params read_params;
  struct bt_gatt_discover_params discover_params;
  struct bt_uuid_16 uuid;
  struct bt_conn *conn;

#if defined(CONFIG_BT_VCP_VOL_CTLR_VOCS)
  uint8_t vocs_inst_cnt;
  struct bt_vocs *vocs[CONFIG_BT_VCP_VOL_CTLR_MAX_VOCS_INST];
#endif /* CONFIG_BT_VCP_VOL_CTLR_VOCS */
#if defined(CONFIG_BT_VCP_VOL_CTLR_AICS)
  uint8_t aics_inst_cnt;
  struct bt_aics *aics[CONFIG_BT_VCP_VOL_CTLR_MAX_AICS_INST];
#endif /* CONFIG_BT_VCP_VOL_CTLR_AICS */

  atomic_define(flags, BT_VCP_VOL_CTLR_FLAG_NUM_FLAGS);
};

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_AUDIO_VCP_INTERNAL_*/
