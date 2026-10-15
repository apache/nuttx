/****************************************************************************
 * net/bt/audio/bap_unicast_client_internal.h
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
 * @brief Internal APIs for BAP

 * Copyright (c) 2021-2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>

#include <nuttx/net/bluetooth/audio/audio.h>
#include <nuttx/net/bluetooth/audio/bap.h>
#include <nuttx/net/bluetooth/conn.h>
#include <nuttx/net/bluetooth/iso.h>
#include <nuttx/mm/net_buf.h>

int bt_bap_unicast_client_config(struct bt_bap_stream *stream,
                                 const struct bt_audio_codec_cfg *codec_cfg);

int bt_bap_unicast_client_qos(struct bt_conn *conn,
                              struct bt_bap_unicast_group *group);

int bt_bap_unicast_client_enable(
  struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len);

int bt_bap_unicast_client_metadata(
  struct bt_bap_stream *stream, const uint8_t meta[], size_t meta_len);

int bt_bap_unicast_client_disable(struct bt_bap_stream *stream);

int bt_bap_unicast_client_start(struct bt_bap_stream *stream);
int bt_bap_unicast_client_connect(struct bt_bap_stream *stream);

int bt_bap_unicast_client_stop(struct bt_bap_stream *stream);

int bt_bap_unicast_client_release(struct bt_bap_stream *stream);

struct net_buf_simple *bt_bap_unicast_client_ep_create_pdu(
  struct bt_conn *conn, uint8_t op);

int bt_bap_unicast_client_ep_send(struct bt_conn *conn, struct bt_bap_ep *ep,
                                  struct net_buf_simple *buf);

struct bt_bap_iso *bt_bap_unicast_client_new_audio_iso(void);
