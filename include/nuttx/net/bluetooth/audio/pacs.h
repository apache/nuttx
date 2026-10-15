/****************************************************************************
 * include/nuttx/net/bluetooth/audio/pacs.h
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
 * @brief Bluetooth Published Audio Capabilities Service (PACS) APIs
 */

/* Copyright (c) 2021 Intel Corporation
 * Copyright (c) 2021-2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_AUDIO_PACS_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_AUDIO_PACS_H

/**
 * @brief Published Audio Capabilities Service (PACS)
 *
 * @defgroup bt_pacs Published Audio Capabilities Service (PACS)
 *
 * @since 3.0
 * @version 0.8.0
 *
 * @ingroup bluetooth
 * @{
 *
 * The Published Audio Capabilities Service (PACS) is used to expose
 * capabilities to remote devices.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

#include <nuttx/net/bluetooth/assigned_numbers.h>
#include <nuttx/net/bluetooth/audio/audio.h>
#include <nuttx/net/bluetooth/conn.h>
#include <nuttx/list.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Published Audio Capability structure. */

struct bt_pacs_cap
{
  /** Codec capability reference */

  const struct bt_audio_codec_cap *codec_cap;

  /** @cond INTERNAL_HIDDEN */

  /** Internally used field for list handling */

  struct list_node _node;

  /** @endcond */
};

/** Structure for registering PACS */

struct bt_pacs_register_param
{
#if defined(CONFIG_BT_PAC_SNK) || defined(__DOXYGEN__)
  /**
   * @brief Enables or disables registration of Sink PAC Characteristic.
   */

  bool snk_pac;
#endif /* CONFIG_BT_PAC_SNK */

#if defined(CONFIG_BT_PAC_SNK_LOC) || defined(__DOXYGEN__)
  /**
   * @brief Enables or disables registration of Sink Location Characteristic.
   *
   * Registration of Sink Location is dependent on @ref
   * bt_pacs_register_param.snk_pac also being set.
   */

  bool snk_loc;
#endif /* CONFIG_BT_PAC_SNK_LOC */

#if defined(CONFIG_BT_PAC_SRC) || defined(__DOXYGEN__)
  /**
   * @brief Enables or disables registration of Source PAC Characteristic.
   */

  bool src_pac;
#endif /* CONFIG_BT_PAC_SRC */

#if defined(CONFIG_BT_PAC_SRC_LOC) || defined(__DOXYGEN__)
  /**
   * @brief Enables or disables registration of Source Location
   * Characteristic.
   *
   * Registration of Source Location is dependent on @ref
   * bt_pacs_register_param.src_pac also being set.
   */

  bool src_loc;
#endif /* CONFIG_BT_PAC_SRC_LOC */
};

/**
 * @typedef bt_pacs_cap_foreach_func_t
 * @brief Published Audio Capability iterator callback.
 *
 * @param cap       Capability found.
 * @param user_data Data given.
 *
 * @return true to continue to the next capability
 * @return false to stop the iteration
 */

typedef bool (*bt_pacs_cap_foreach_func_t)(
  const struct bt_pacs_cap *cap, void *user_data);

/**
 * @brief Published Audio Capability iterator.
 *
 * Iterate capabilities with endpoint direction specified.
 *
 * @param dir       Direction of the endpoint to look capability for.
 * @param func      Callback function.
 * @param user_data Data to pass to the callback.
 */

void bt_pacs_cap_foreach(enum bt_audio_dir dir,
                         bt_pacs_cap_foreach_func_t func, void *user_data);

/**
 * @brief Register the Published Audio Capability Service instance.
 *
 * @param param PACS register parameters.
 *
 * @retval 0 Success
 * @retval -EINVAL @p param is NULL or bad combination of values in @p param
 * @retval -EALREADY Already registered
 * @retval -ENOEXEC Request was rejected by GATT
 */

int bt_pacs_register(const struct bt_pacs_register_param *param);

/**
 * @brief Unregister the Published Audio Capability Service instance.
 *
 * @return 0 in case of success or negative value in case of error.
 */

int bt_pacs_unregister(void);

/**
 * @brief Register Published Audio Capability.
 *
 * Register Audio Local Capability.
 *
 * @param dir Direction of the endpoint to register capability for.
 * @param cap Capability structure.
 *
 * @return 0 in case of success or negative value in case of error.
 */

int bt_pacs_cap_register(enum bt_audio_dir dir, struct bt_pacs_cap *cap);

/**
 * @brief Unregister Published Audio Capability.
 *
 * Unregister Audio Local Capability.
 *
 * @param dir Direction of the endpoint to unregister capability for.
 * @param cap Capability structure.
 *
 * @return 0 in case of success or negative value in case of error.
 */

int bt_pacs_cap_unregister(enum bt_audio_dir dir, struct bt_pacs_cap *cap);

/**
 * @brief Set the location for an endpoint type
 *
 * @param dir      Direction of the endpoints to change location for.
 * @param location The location to be set.
 *
 * @return 0 in case of success or negative value in case of error.
 */

int bt_pacs_set_location(enum bt_audio_dir dir,
                         enum bt_audio_location location);

/**
 * @brief Set the available contexts for an endpoint type
 *
 * @param dir      Direction of the endpoints to change available contexts
 *                 for.
 * @param contexts The contexts to be set.
 *
 * @return 0 in case of success or negative value in case of error.
 */

int bt_pacs_set_available_contexts(enum bt_audio_dir dir,
                                   enum bt_audio_context contexts);

/**
 * @brief Get the available contexts for an endpoint type
 *
 * @param dir      Direction of the endpoints to get contexts for.
 *
 * @return Bitmask of available contexts.
 */

enum bt_audio_context bt_pacs_get_available_contexts(enum bt_audio_dir dir);

/**
 * @brief Set the available contexts for a given connection
 *
 * This function sets the available contexts value for a given @p conn
 * connection object. If the @p contexts parameter is NULL the available
 * contexts value is reset to default. The default value of the available
 * contexts is set using @ref bt_pacs_set_available_contexts function. The
 * Available Context Value is reset to default on ACL disconnection.
 *
 * @param conn     Connection object.
 * @param dir      Direction of the endpoints to change available contexts
 *                 for.
 * @param contexts The contexts to be set or NULL to reset to default.
 *
 * @return 0 in case of success or negative value in case of error.
 */

int bt_pacs_conn_set_available_contexts_for_conn(
  struct bt_conn *conn, enum bt_audio_dir dir,
  enum bt_audio_context *contexts);

/**
 * @brief Get the available contexts for a given connection
 *
 * This server function returns the available contexts value for a given @p
 * conn connection object. The value returned is the one set with @ref
 * bt_pacs_conn_set_available_contexts_for_conn function or the default value
 * set with @ref bt_pacs_set_available_contexts function.
 *
 * @param conn     Connection object.
 * @param dir      Direction of the endpoints to get contexts for.
 *
 * @return Bitmask of available contexts.
 * @retval BT_AUDIO_CONTEXT_TYPE_NONE if @p conn or @p dir are invalid
 */

enum bt_audio_context bt_pacs_get_available_contexts_for_conn(
  struct bt_conn *conn, enum bt_audio_dir dir);

/**
 * @brief Set the supported contexts for an endpoint type
 *
 * @param dir      Direction of the endpoints to change available contexts
 *                 for.
 * @param contexts The contexts to be set.
 *
 * @return 0 in case of success or negative value in case of error.
 */

int bt_pacs_set_supported_contexts(enum bt_audio_dir dir,
                                   enum bt_audio_context contexts);

#ifdef __cplusplus
}
#endif

/** @} */

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_AUDIO_PACS_H */
