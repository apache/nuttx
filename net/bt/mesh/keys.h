/****************************************************************************
 * net/bt/mesh/keys.h
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

/* Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define BT_MESH_KEY_PRIMARY 0x0000

enum bt_mesh_key_evt
{
  BT_MESH_KEY_ADDED,   /* New key added */
  BT_MESH_KEY_DELETED, /* Existing key deleted */
  BT_MESH_KEY_UPDATED, /* KR phase 1, second key added */
  BT_MESH_KEY_SWAPPED, /* KR phase 2, now sending on second key */
  BT_MESH_KEY_REVOKED, /* KR phase 3, old key removed */
};

enum bt_mesh_key_type
{
  BT_MESH_KEY_TYPE_ECB,
  BT_MESH_KEY_TYPE_CCM,
  BT_MESH_KEY_TYPE_CMAC,
  BT_MESH_KEY_TYPE_NET,
  BT_MESH_KEY_TYPE_APP,
  BT_MESH_KEY_TYPE_DEV
};

int bt_mesh_key_import(enum bt_mesh_key_type type,
                       const uint8_t in[16], struct bt_mesh_key *out);
int bt_mesh_key_export(uint8_t out[16], const struct bt_mesh_key *in);
void bt_mesh_key_assign(struct bt_mesh_key *dst,
                        const struct bt_mesh_key *src);
int bt_mesh_key_destroy(const struct bt_mesh_key *key);
int bt_mesh_key_compare(const uint8_t raw_key[16],
                        const struct bt_mesh_key *mesh_key);
