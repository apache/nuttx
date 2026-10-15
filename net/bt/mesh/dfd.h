/****************************************************************************
 * net/bt/mesh/dfd.h
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

/* Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define BT_MESH_DFD_OP_RECEIVERS_ADD BT_MESH_MODEL_OP_2(0x83, 0x11)
#define BT_MESH_DFD_OP_RECEIVERS_DELETE_ALL BT_MESH_MODEL_OP_2(0x83, 0x12)
#define BT_MESH_DFD_OP_RECEIVERS_STATUS BT_MESH_MODEL_OP_2(0x83, 0x13)
#define BT_MESH_DFD_OP_RECEIVERS_GET BT_MESH_MODEL_OP_2(0x83, 0x14)
#define BT_MESH_DFD_OP_RECEIVERS_LIST BT_MESH_MODEL_OP_2(0x83, 0x15)
#define BT_MESH_DFD_OP_CAPABILITIES_GET BT_MESH_MODEL_OP_2(0x83, 0x16)
#define BT_MESH_DFD_OP_CAPABILITIES_STATUS BT_MESH_MODEL_OP_2(0x83, 0x17)
#define BT_MESH_DFD_OP_GET BT_MESH_MODEL_OP_2(0x83, 0x18)
#define BT_MESH_DFD_OP_START BT_MESH_MODEL_OP_2(0x83, 0x19)
#define BT_MESH_DFD_OP_SUSPEND BT_MESH_MODEL_OP_2(0x83, 0x1a)
#define BT_MESH_DFD_OP_CANCEL BT_MESH_MODEL_OP_2(0x83, 0x1b)
#define BT_MESH_DFD_OP_APPLY BT_MESH_MODEL_OP_2(0x83, 0x1c)
#define BT_MESH_DFD_OP_STATUS BT_MESH_MODEL_OP_2(0x83, 0x1d)
#define BT_MESH_DFD_OP_UPLOAD_GET BT_MESH_MODEL_OP_2(0x83, 0x1e)
#define BT_MESH_DFD_OP_UPLOAD_START BT_MESH_MODEL_OP_2(0x83, 0x1f)
#define BT_MESH_DFD_OP_UPLOAD_START_OOB BT_MESH_MODEL_OP_2(0x83, 0x20)
#define BT_MESH_DFD_OP_UPLOAD_CANCEL BT_MESH_MODEL_OP_2(0x83, 0x21)
#define BT_MESH_DFD_OP_UPLOAD_STATUS BT_MESH_MODEL_OP_2(0x83, 0x22)
#define BT_MESH_DFD_OP_FW_GET BT_MESH_MODEL_OP_2(0x83, 0x23)
#define BT_MESH_DFD_OP_FW_GET_BY_INDEX BT_MESH_MODEL_OP_2(0x83, 0x24)
#define BT_MESH_DFD_OP_FW_DELETE BT_MESH_MODEL_OP_2(0x83, 0x25)
#define BT_MESH_DFD_OP_FW_DELETE_ALL BT_MESH_MODEL_OP_2(0x83, 0x26)
#define BT_MESH_DFD_OP_FW_STATUS BT_MESH_MODEL_OP_2(0x83, 0x27)
