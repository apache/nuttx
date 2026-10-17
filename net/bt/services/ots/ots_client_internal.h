/****************************************************************************
 * net/bt/services/ots/ots_client_internal.h
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
 * @brief Object Transfer client internal header file
 *
 * For use with the Object Transfer Service Client (OTC)
 *
 * Copyright (c) 2020 - 2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef BT_GATT_OTC_INTERNAL_H_
#define BT_GATT_OTC_INTERNAL_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "ots_oacp_internal.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Translate OTS OLCP result code to "normal" error
 *
 * Replace the OTS SUCCESS value with the value 0.
 *
 * The OLCP result code has the value "1" for success. Elsewhere in the
 * le-audio host stack, "0" is used for no error.
 *
 * TODO: Update to also convert the non-SUCCESS values TODO: The macro is
 * called OLCP, but uses OACP success value
 * TODO: See https://github.com/zephyrproject-rtos/zephyr/issues/41184
 * OTS does not use the value "0".
 */
#define OLCP_RESULT_TO_ERROR(RESULT) (((RESULT) == BT_GATT_OTS_OACP_RES_SUCCESS) ? 0 : RESULT)

#define OTS_FEATURE_LEN (uint32_t)(2 * sizeof(uint32_t))
#define OTS_SIZE_LEN (uint32_t)(2 * sizeof(uint32_t))
#define OTS_PROPERTIES_LEN (uint32_t)(sizeof(uint32_t))
#define OTS_TYPE_MAX_LEN (uint32_t)(sizeof(struct bt_ots_obj_type))

#ifdef __cplusplus
}
#endif

#endif /* BT_GATT_OTC_INTERNAL_H_ */
