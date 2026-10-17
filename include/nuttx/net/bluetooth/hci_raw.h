/****************************************************************************
 * include/nuttx/net/bluetooth/hci_raw.h
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
 *  @brief Bluetooth HCI RAW channel handling
 */

/* Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_HCI_RAW_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_HCI_RAW_H

/**
 * @brief HCI RAW channel
 * @defgroup hci_raw HCI RAW channel
 * @since 2.3
 * @version 1.0.0
 * @ingroup bluetooth
 * @{
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stddef.h>

#include <nuttx/net/bluetooth/compat/kernel.h>
#include <nuttx/mm/net_buf.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief Send packet to the Bluetooth controller
 *
 * Send packet to the Bluetooth controller. The buffers should be allocated
 * using bt_buf_get_tx().
 *
 * @param buf HCI packet to be sent.
 *
 * @return Zero on success or (negative) error code otherwise.
 */

int bt_send(struct net_buf *buf);

/** @brief Enable Bluetooth RAW channel:
 *
 *  Enable Bluetooth RAW HCI channel.
 *
 *  @param rx_queue netbuf queue where HCI packets received from the
 *                  Bluetooth controller are to be queued. The queue is
 *                  defined in the caller while the available buffers pools
 *                  are handled in the stack.
 *
 *  @return Zero on success or (negative) error code otherwise.
 */

int bt_enable_raw(struct k_fifo *rx_queue);

#ifdef __cplusplus
}
#endif
/**
 * @}
 */

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_HCI_RAW_H */
