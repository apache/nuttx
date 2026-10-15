/****************************************************************************
 * include/nuttx/net/bluetooth/controller.h
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

/** @file
 *  @brief Bluetooth subsystem controller APIs.
 */

/* Copyright (c) 2018 Codecoup
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_CONTROLLER_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_CONTROLLER_H

/**
 * @brief Bluetooth Controller
 * @defgroup bt_ctrl Bluetooth Controller
 * @ingroup bluetooth
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Set public address for controller
 *
 *  Should be called before bt_enable().
 *
 *  @param addr Public address
 */

void bt_ctlr_set_public_addr(const uint8_t *addr);

#ifdef __cplusplus
}
#endif
/**
 * @}
 */

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_CONTROLLER_H */
