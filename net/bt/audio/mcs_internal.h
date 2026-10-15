/****************************************************************************
 * net/bt/audio/mcs_internal.h
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
 * @brief Media Control Service internal header file
 *
 * Copyright (c) 2020 - 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_SUBSYS_BLUETOOTH_HOST_AUDIO_MCS_INTERNAL_H_
#define ZEPHYR_SUBSYS_BLUETOOTH_HOST_AUDIO_MCS_INTERNAL_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/services/ots.h>
#include <nuttx/net/bluetooth/compat/sys/util.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* This differs from BT_OTS_VALID_OBJ_ID as MCS does not use the directory
 * list object
 */
#define BT_MCS_VALID_OBJ_ID(id) (IN_RANGE((id), BT_OTS_OBJ_ID_MIN, BT_OTS_OBJ_ID_MAX))

#define BT_MCS_VALID_OP(opcode)                                                                    \
(IN_RANGE((opcode), BT_MCS_OPC_PLAY, BT_MCS_OPC_STOP) || (opcode == BT_MCS_OPC_MOVE_RELATIVE) || \
 IN_RANGE((opcode), BT_MCS_OPC_PREV_SEGMENT, BT_MCS_OPC_GOTO_SEGMENT) ||                         \
 IN_RANGE((opcode), BT_MCS_OPC_PREV_TRACK, BT_MCS_OPC_GOTO_TRACK) ||                             \
 IN_RANGE((opcode), BT_MCS_OPC_PREV_GROUP, BT_MCS_OPC_GOTO_GROUP))

int bt_mcs_init(struct bt_ots_cb *ots_cbs);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SUBSYS_BLUETOOTH_HOST_AUDIO_MCS_INTERNAL_H_ */
