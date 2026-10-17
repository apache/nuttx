/****************************************************************************
 * include/nuttx/net/bluetooth/services/hrs.h
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

/* Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_SERVICES_HRS_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_SERVICES_HRS_H

/**
 * @brief Heart Rate Service (HRS)
 * @defgroup bt_hrs Heart Rate Service (HRS)
 * @ingroup bluetooth
 * @{
 *
 * [Experimental] Users should note that the APIs can change as a part of
 * ongoing development.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/list.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Server shall restart the accumulation of energy expended from zero
 */
#define BT_HRS_CONTROL_POINT_RESET_ENERGY_EXPANDED_REQ 0x01

/** @brief Heart rate service callback structure */

struct bt_hrs_cb
{
  /** @brief Heart rate notifications changed
   *
   * @param enabled Flag that is true if notifications were enabled, false if
   *                they were disabled.
   */

  void (*ntf_changed)(bool enabled);

  /* @brief Heart rate control point write callback
   *
   * @note if Server supports the Energy Expended feature then application
   * shall implement and support @ref
   * BT_HRS_CONTROL_POINT_RESET_ENERGY_EXPANDED_REQ request code
   *
   * @param request control point request code
   *
   * @return 0 on successful handling of control point request
   * @return -ENOTSUP if not supported. It can be used to pass handling to
   * other listeners in case of multiple listeners
   * @return other negative error codes will result in immediate error
   * response
   */

  int (*ctrl_point_write)(uint8_t request);

  /** Internal member to form a list of callbacks */

  struct list_node _node;
};

/** @brief Heart rate service callback register
 *
 * This function will register callbacks that will be called in certain
 * events related to Heart rate service.
 *
 * @param cb Pointer to callbacks structure. Must point to memory that
 *           remains valid until unregistered.
 *
 * @return 0 on success
 * @return -EINVAL in case @p cb is NULL
 */

int bt_hrs_cb_register(struct bt_hrs_cb *cb);

/** @brief Heart rate service callback unregister
 *
 * This function will unregister callback from Heart rate service.
 *
 * @param cb Pointer to callbacks structure
 *
 * @return 0 on success
 * @return -EINVAL in case @p cb is NULL
 * @return -ENOENT in case the @p cb was not found in registered callbacks
 */

int bt_hrs_cb_unregister(struct bt_hrs_cb *cb);

/** @brief Notify heart rate measurement.
 *
 * This will send a GATT notification to all current subscribers.
 *
 *  @param heartrate The heartrate measurement in beats per minute.
 *
 *  @return Zero in case of success and error code in case of error.
 */

int bt_hrs_notify(uint16_t heartrate);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_SERVICES_HRS_H */
