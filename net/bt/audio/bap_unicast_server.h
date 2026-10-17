/****************************************************************************
 * net/bt/audio/bap_unicast_server.h
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

/*  Bluetooth Audio Unicast Server */

/* Copyright (c) 2021-2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>
#include <stdint.h>

#include <nuttx/net/bluetooth/audio/audio.h>
#include <nuttx/net/bluetooth/audio/bap.h>

int bt_bap_unicast_server_reconfig(struct bt_bap_stream *stream,
  const struct bt_audio_codec_cfg *codec_cfg);
int bt_bap_unicast_server_start(struct bt_bap_stream *stream);
int bt_bap_unicast_server_metadata(struct bt_bap_stream *stream,
                                   const uint8_t meta[],
                                   size_t meta_len);
int bt_bap_unicast_server_disable(struct bt_bap_stream *stream);
int bt_bap_unicast_server_release(struct bt_bap_stream *stream);
