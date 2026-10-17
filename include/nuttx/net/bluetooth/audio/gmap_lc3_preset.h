/****************************************************************************
 * include/nuttx/net/bluetooth/audio/gmap_lc3_preset.h
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
 * @brief Header for Bluetooth GMAP LC3 presets.
 *
 * Copyright (c) 2023-2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_AUDIO_GMAP_LC3_PRESET_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_AUDIO_GMAP_LC3_PRESET_H

/**
 * @brief Gaming Audio Profile (GMAP) LC3 Presets
 *
 * @defgroup bt_gmap_lc3_preset Gaming Audio Profile (GMAP) LC3 Presets
 *
 * @since 3.5
 * @version 0.8.0
 *
 * @ingroup bluetooth
 * @{
 *
 * These APIs provide presets for codec configuration and QoS based on values
 * supplied by the codec configurations from table 3.16 in the GMAP v1.0
 * specification
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/audio/audio.h>
#include <nuttx/net/bluetooth/audio/bap.h>
#include <nuttx/net/bluetooth/audio/bap_lc3_preset.h>
#include <nuttx/net/bluetooth/audio/lc3.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Helper to declare LC3 32_1_gr codec configuration
 *
 * @param _loc             Audio channel location bitfield (@ref
 *                         bt_audio_location)
 * @param _stream_context  Stream context (``BT_AUDIO_CONTEXT_*``)
 */
#define BT_GMAP_LC3_PRESET_32_1_GR(_loc, _stream_context)                                          \
BT_BAP_LC3_PRESET(BT_AUDIO_CODEC_LC3_CONFIG(BT_AUDIO_CODEC_CFG_FREQ_32KHZ,                       \
                                            BT_AUDIO_CODEC_CFG_DURATION_7_5, _loc, 60U, 1,       \
                                            _stream_context),                                    \
                  BT_BAP_QOS_CFG_UNFRAMED(7500u, 60U, 1U, 15U, 10000U))

/**
 * @brief Helper to declare LC3 32_2_gr codec configuration
 *
 * @param _loc             Audio channel location bitfield (@ref
 *                         bt_audio_location)
 * @param _stream_context  Stream context (``BT_AUDIO_CONTEXT_*``)
 */
#define BT_GMAP_LC3_PRESET_32_2_GR(_loc, _stream_context)                                          \
BT_BAP_LC3_PRESET(BT_AUDIO_CODEC_LC3_CONFIG(BT_AUDIO_CODEC_CFG_FREQ_32KHZ,                       \
                                            BT_AUDIO_CODEC_CFG_DURATION_10, _loc, 80U, 1,        \
                                            _stream_context),                                    \
                  BT_BAP_QOS_CFG_UNFRAMED(10000u, 80U, 1U, 20U, 10000U))

/**
 * @brief Helper to declare LC3 48_1_gr codec configuration
 *
 * @param _loc             Audio channel location bitfield (@ref
 *                         bt_audio_location)
 * @param _stream_context  Stream context (``BT_AUDIO_CONTEXT_*``)
 */
#define BT_GMAP_LC3_PRESET_48_1_GR(_loc, _stream_context)                                          \
BT_BAP_LC3_PRESET(BT_AUDIO_CODEC_LC3_CONFIG(BT_AUDIO_CODEC_CFG_FREQ_48KHZ,                       \
                                            BT_AUDIO_CODEC_CFG_DURATION_7_5, _loc, 75U, 1,       \
                                            _stream_context),                                    \
                  BT_BAP_QOS_CFG_UNFRAMED(7500u, 75U, 1U, 15U, 10000U))

/**
 * @brief Helper to declare LC3 48_2_gr codec configuration
 *
 *  Mandatory to support as both unicast client and server
 *
 * @param _loc             Audio channel location bitfield (@ref
 *                         bt_audio_location)
 * @param _stream_context  Stream context (``BT_AUDIO_CONTEXT_*``)
 */
#define BT_GMAP_LC3_PRESET_48_2_GR(_loc, _stream_context)                                          \
BT_BAP_LC3_PRESET(BT_AUDIO_CODEC_LC3_CONFIG(BT_AUDIO_CODEC_CFG_FREQ_48KHZ,                       \
                                            BT_AUDIO_CODEC_CFG_DURATION_10, _loc, 100U, 1,       \
                                            _stream_context),                                    \
                  BT_BAP_QOS_CFG_UNFRAMED(10000u, 100U, 1U, 20U, 10000U))

/**
 * @brief Helper to declare LC3 48_3_gr codec configuration
 *
 * @param _loc             Audio channel location bitfield (@ref
 *                         bt_audio_location)
 * @param _stream_context  Stream context (``BT_AUDIO_CONTEXT_*``)
 */
#define BT_GMAP_LC3_PRESET_48_3_GR(_loc, _stream_context)                                          \
BT_BAP_LC3_PRESET(BT_AUDIO_CODEC_LC3_CONFIG(BT_AUDIO_CODEC_CFG_FREQ_48KHZ,                       \
                                            BT_AUDIO_CODEC_CFG_DURATION_7_5, _loc, 90U, 1,       \
                                            _stream_context),                                    \
                  BT_BAP_QOS_CFG_UNFRAMED(7500u, 90U, 1U, 15U, 10000U))

/**
 * @brief Helper to declare LC3 48_4_gr codec configuration
 *
 *  Mandatory to support as unicast server
 *
 * @param _loc             Audio channel location bitfield (@ref
 *                         bt_audio_location)
 * @param _stream_context  Stream context (``BT_AUDIO_CONTEXT_*``)
 */
#define BT_GMAP_LC3_PRESET_48_4_GR(_loc, _stream_context)                                          \
BT_BAP_LC3_PRESET(BT_AUDIO_CODEC_LC3_CONFIG(BT_AUDIO_CODEC_CFG_FREQ_48KHZ,                       \
                                            BT_AUDIO_CODEC_CFG_DURATION_10, _loc, 120u, 1,       \
                                            _stream_context),                                    \
                  BT_BAP_QOS_CFG_UNFRAMED(10000u, 120U, 1U, 20U, 10000U))

/**
 * @brief Helper to declare LC3 16_1_gs codec configuration
 *
 * @param _loc             Audio channel location bitfield (@ref
 *                         bt_audio_location)
 * @param _stream_context  Stream context (``BT_AUDIO_CONTEXT_*``)
 */
#define BT_GMAP_LC3_PRESET_16_1_GS(_loc, _stream_context)                                          \
BT_BAP_LC3_PRESET(BT_AUDIO_CODEC_LC3_CONFIG(BT_AUDIO_CODEC_CFG_FREQ_16KHZ,                       \
                                            BT_AUDIO_CODEC_CFG_DURATION_7_5, _loc, 30U, 1,       \
                                            _stream_context),                                    \
                  BT_BAP_QOS_CFG_UNFRAMED(7500u, 30U, 1U, 15U, 60000U))

/**
 * @brief Helper to declare LC3 16_2_gs codec configuration
 *
 * @param _loc             Audio channel location bitfield (@ref
 *                         bt_audio_location)
 * @param _stream_context  Stream context (``BT_AUDIO_CONTEXT_*``)
 */
#define BT_GMAP_LC3_PRESET_16_2_GS(_loc, _stream_context)                                          \
BT_BAP_LC3_PRESET(BT_AUDIO_CODEC_LC3_CONFIG(BT_AUDIO_CODEC_CFG_FREQ_16KHZ,                       \
                                            BT_AUDIO_CODEC_CFG_DURATION_10, _loc, 40U, 1,        \
                                            _stream_context),                                    \
                  BT_BAP_QOS_CFG_UNFRAMED(10000u, 40U, 1U, 20U, 60000U))

/**
 * @brief Helper to declare LC3 32_1_gs codec configuration
 *
 * @param _loc             Audio channel location bitfield (@ref
 *                         bt_audio_location)
 * @param _stream_context  Stream context (``BT_AUDIO_CONTEXT_*``)
 */
#define BT_GMAP_LC3_PRESET_32_1_GS(_loc, _stream_context)                                          \
BT_BAP_LC3_PRESET(BT_AUDIO_CODEC_LC3_CONFIG(BT_AUDIO_CODEC_CFG_FREQ_32KHZ,                       \
                                            BT_AUDIO_CODEC_CFG_DURATION_7_5, _loc, 60U, 1,       \
                                            _stream_context),                                    \
                  BT_BAP_QOS_CFG_UNFRAMED(7500u, 60U, 1U, 15U, 60000U))

/**
 * @brief Helper to declare LC3 32_2_gs codec configuration
 *
 * @param _loc             Audio channel location bitfield (@ref
 *                         bt_audio_location)
 * @param _stream_context  Stream context (``BT_AUDIO_CONTEXT_*``)
 */
#define BT_GMAP_LC3_PRESET_32_2_GS(_loc, _stream_context)                                          \
BT_BAP_LC3_PRESET(BT_AUDIO_CODEC_LC3_CONFIG(BT_AUDIO_CODEC_CFG_FREQ_32KHZ,                       \
                                            BT_AUDIO_CODEC_CFG_DURATION_10, _loc, 80U, 1,        \
                                            _stream_context),                                    \
                  BT_BAP_QOS_CFG_UNFRAMED(10000u, 80U, 1U, 20U, 60000U))

/**
 * @brief Helper to declare LC3 48_1_gs codec configuration
 *
 * @param _loc             Audio channel location bitfield (@ref
 *                         bt_audio_location)
 * @param _stream_context  Stream context (``BT_AUDIO_CONTEXT_*``)
 */
#define BT_GMAP_LC3_PRESET_48_1_GS(_loc, _stream_context)                                          \
BT_BAP_LC3_PRESET(BT_AUDIO_CODEC_LC3_CONFIG(BT_AUDIO_CODEC_CFG_FREQ_48KHZ,                       \
                                            BT_AUDIO_CODEC_CFG_DURATION_7_5, _loc, 75U, 1,       \
                                            _stream_context),                                    \
                  BT_BAP_QOS_CFG_UNFRAMED(7500u, 75U, 1U, 15U, 60000U))

/**
 * @brief Helper to declare LC3 48_2_gs codec configuration
 *
 * @param _loc             Audio channel location bitfield (@ref
 *                         bt_audio_location)
 * @param _stream_context  Stream context (``BT_AUDIO_CONTEXT_*``)
 */
#define BT_GMAP_LC3_PRESET_48_2_GS(_loc, _stream_context)                                          \
BT_BAP_LC3_PRESET(BT_AUDIO_CODEC_LC3_CONFIG(BT_AUDIO_CODEC_CFG_FREQ_48KHZ,                       \
                                            BT_AUDIO_CODEC_CFG_DURATION_10, _loc, 100U, 1,       \
                                            _stream_context),                                    \
                  BT_BAP_QOS_CFG_UNFRAMED(10000u, 100U, 1U, 20U, 60000U))

/* GMAP LC3 broadcast presets defined by table 3.22 in the GMAP v1.0
 * specification
 */

/**
 * @brief Helper to declare LC3 48_1_g codec configuration
 *
 * @param _loc             Audio channel location bitfield (@ref
 *                         bt_audio_location)
 * @param _stream_context  Stream context (``BT_AUDIO_CONTEXT_*``)
 */
#define BT_GMAP_LC3_PRESET_48_1_G(_loc, _stream_context)                                           \
BT_BAP_LC3_PRESET(BT_AUDIO_CODEC_LC3_CONFIG(BT_AUDIO_CODEC_CFG_FREQ_48KHZ,                       \
                                            BT_AUDIO_CODEC_CFG_DURATION_7_5, _loc, 75U, 1,       \
                                            _stream_context),                                    \
                  BT_BAP_QOS_CFG_UNFRAMED(7500u, 75U, 1U, 8U, 10000U))

/**
 * @brief Helper to declare LC3 48_2_g codec configuration
 *
 * @param _loc             Audio channel location bitfield (@ref
 *                         bt_audio_location)
 * @param _stream_context  Stream context (``BT_AUDIO_CONTEXT_*``)
 */
#define BT_GMAP_LC3_PRESET_48_2_G(_loc, _stream_context)                                           \
BT_BAP_LC3_PRESET(BT_AUDIO_CODEC_LC3_CONFIG(BT_AUDIO_CODEC_CFG_FREQ_48KHZ,                       \
                                            BT_AUDIO_CODEC_CFG_DURATION_10, _loc, 100U, 1,       \
                                            _stream_context),                                    \
                  BT_BAP_QOS_CFG_UNFRAMED(10000u, 100U, 1U, 10U, 10000U))

/**
 * @brief Helper to declare LC3 48_3_g codec configuration
 *
 * @param _loc             Audio channel location bitfield (@ref
 *                         bt_audio_location)
 * @param _stream_context  Stream context (``BT_AUDIO_CONTEXT_*``)
 */
#define BT_GMAP_LC3_PRESET_48_3_G(_loc, _stream_context)                                           \
BT_BAP_LC3_PRESET(BT_AUDIO_CODEC_LC3_CONFIG(BT_AUDIO_CODEC_CFG_FREQ_48KHZ,                       \
                                            BT_AUDIO_CODEC_CFG_DURATION_7_5, _loc, 90U, 1,       \
                                            _stream_context),                                    \
                  BT_BAP_QOS_CFG_UNFRAMED(7500u, 90U, 1U, 8U, 10000U))

/**
 * @brief Helper to declare LC3 48_4_g codec configuration
 *
 * @param _loc             Audio channel location bitfield (@ref
 *                         bt_audio_location)
 * @param _stream_context  Stream context (``BT_AUDIO_CONTEXT_*``)
 */
#define BT_GMAP_LC3_PRESET_48_4_G(_loc, _stream_context)                                           \
BT_BAP_LC3_PRESET(BT_AUDIO_CODEC_LC3_CONFIG(BT_AUDIO_CODEC_CFG_FREQ_48KHZ,                       \
                                            BT_AUDIO_CODEC_CFG_DURATION_10, _loc, 120u, 1,       \
                                            _stream_context),                                    \
                  BT_BAP_QOS_CFG_UNFRAMED(10000u, 120U, 1U, 10U, 10000U))

#ifdef __cplusplus
}
#endif
/** @} */

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_AUDIO_GMAP_LC3_PRESET_H */
