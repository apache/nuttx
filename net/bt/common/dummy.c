/****************************************************************************
 * net/bt/common/dummy.c
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
 * @file dummy.c Static compilation checks.
 */

/* Copyright (c) 2017 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdalign.h>

#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/buf.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* The unpacked structs below are used inside __packed structures that
 * reflect what goes on the wire. While alignment is not a problem for those,
 * since all their members are bytes or byte arrays, the size is. They must
 * not be padded by the compiler, otherwise the on-wire packet will not map
 * the packed structure correctly.
 *
 * The bt_addr structs are not marked __packed because it's considered ugly
 * by some. But here is a proof that the structs have all the properties of,
 * and can be safely used as packed structs.
 */

BUILD_ASSERT(sizeof(bt_addr_t) == BT_ADDR_SIZE);
BUILD_ASSERT(alignof(bt_addr_t) == 1);
BUILD_ASSERT(sizeof(bt_addr_le_t) == BT_ADDR_LE_SIZE);
BUILD_ASSERT(alignof(bt_addr_le_t) == 1);

#if defined(CONFIG_BT_HCI_HOST)
/* The Bluetooth subsystem requires that higher priority events shall be
 * given in a priority higher than the Bluetooth Host's Tx and the
 * Controller's receive thread priority. This is required in order to
 * dispatch Number of Completed Packets event before any new data arrives on
 * a connection to the Host threads.
 */

BUILD_ASSERT(CONFIG_BT_DRIVER_RX_HIGH_PRIO < CONFIG_BT_HCI_TX_PRIO);
#endif /* defined(CONFIG_BT_HCI_HOST) */

#if (defined(CONFIG_LOG_BACKEND_RTT) && (defined(CONFIG_SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL) ||     \
                                         defined(CONFIG_LOG_BACKEND_RTT_MODE_BLOCK))) ||           \
        defined(CONFIG_LOG_BACKEND_NET) || defined(CONFIG_LOG_IMMEDIATE_CLEAN_OUTPUT)
#define INCOMPATIBLE_IMMEDIATE_LOG_BACKEND 1
#endif

/* Immediate logging on most backend is not supported
 * with the software-based Link Layer since it introduces ISR latency due to
 * outputting log messages with interrupts disabled.
 */
#if !defined(CONFIG_TEST) && !defined(CONFIG_ARCH_POSIX) && defined(CONFIG_BT_LL_SW_SPLIT) &&      \
        defined(INCOMPATIBLE_IMMEDIATE_LOG_BACKEND)
BUILD_ASSERT(!IS_ENABLED(CONFIG_LOG_MODE_IMMEDIATE),
             "Immediate logging on selected backend(s) not supported with "
             "the software Link Layer");
#endif
