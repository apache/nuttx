/****************************************************************************
 * net/bt/audio/bap_stream.h
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
 * @brief Internal APIs for Audio Stream handling

 * Copyright (c) 2020 Intel Corporation Copyright (c) 2021-2025 Nordic
 * Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/net/bluetooth/audio/ascs.h>
#include <nuttx/net/bluetooth/audio/audio.h>
#include <nuttx/net/bluetooth/audio/bap.h>
#include <nuttx/net/bluetooth/conn.h>
#include <nuttx/net/bluetooth/iso.h>

void bt_bap_stream_init(struct bt_bap_stream *stream);

/* Disconnect ISO channel */

int bt_bap_stream_disconnect(struct bt_bap_stream *stream);

void bt_bap_stream_reset(struct bt_bap_stream *stream);

void bt_bap_stream_attach(struct bt_conn *conn,
                          struct bt_bap_stream *stream, struct bt_bap_ep
                            *ep);

void bt_bap_qos_cfg_to_iso_qos(struct bt_iso_chan_io_qos *io,
                               const struct bt_bap_qos_cfg *qos_cfg);

void bt_bap_stream_detach(struct bt_bap_stream *stream);

enum bt_bap_ascs_reason
bt_audio_verify_qos(const struct bt_bap_qos_cfg *qos);
bool bt_audio_valid_codec_cfg(const struct bt_audio_codec_cfg *codec_cfg);
bool bt_bap_valid_qos_pref(const struct bt_bap_qos_cfg_pref *qos_pref);
bool bt_bap_stream_can_disconnect(const struct bt_bap_stream *stream);

enum bt_bap_ascs_reason bt_bap_stream_verify_qos(
  const struct bt_bap_stream *stream, const struct bt_bap_qos_cfg *qos);

struct bt_iso_chan *bt_bap_stream_iso_chan_get(struct bt_bap_stream *stream);
bool bt_bap_stream_valid_state_transition(const struct bt_bap_ep *ep,
                                          enum bt_bap_ep_state state);
