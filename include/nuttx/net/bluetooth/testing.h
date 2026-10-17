/****************************************************************************
 * include/nuttx/net/bluetooth/testing.h
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

/* Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

/** @brief Internal testing interfaces for Bluetooth
 *  @file @internal
 *
 *  The interfaces in this file are internal and not stable.
 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_TESTING_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_TESTING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/mm/net_buf.h>

/** @brief Hook for `acl_in_pool.destroy`
 *
 *  Weak-function interface. The user can simply define this function, and it
 *  will automatically become the event listener.
 *
 *  @kconfig_dep{CONFIG_BT_TESTING}
 */

void bt_testing_trace_event_acl_pool_destroy(struct net_buf *buf);

/** @brief Trace when extended advertising report reassembly timed out
 *
 *  Weak-function interface called when the host reassembly timer expires for
 *  a fragmented extended advertising report chain.
 *
 *  @kconfig_dep{CONFIG_BT_TESTING}
 */

void bt_testing_trace_ext_adv_reassembly_timeout(void);

/** @brief Trace when a fragmented extended advertising report is delivered
 *
 *  Weak-function interface called when a fragmented extended advertising
 *  report chain has been fully reassembled and delivered to the application.
 *
 *  @kconfig_dep{CONFIG_BT_TESTING}
 */

void bt_testing_trace_ext_adv_reassembly_complete(void);

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_TESTING_H */
