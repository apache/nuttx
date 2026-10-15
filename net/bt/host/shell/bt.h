/****************************************************************************
 * net/bt/host/shell/bt.h
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
 *  @brief Bluetooth shell functions
 *
 *  This is not to be included by the application.
 */

/* Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BT_H
#define __BT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>
#include <stdint.h>

#include <nuttx/config.h>
#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/mm/net_buf.h>
#include <sys/types.h>

extern struct bt_conn *default_conn;

bool passes_scan_filter(const struct bt_le_scan_recv_info *info,
                        const struct net_buf_simple *buf);

#if defined(CONFIG_BT_ISO_UNICAST)
extern struct bt_iso_chan iso_chan;
#endif /* CONFIG_BT_ISO_UNICAST */

#if defined(CONFIG_BT_EXT_ADV)
extern uint8_t selected_adv;
extern struct bt_le_ext_adv *adv_sets[CONFIG_BT_EXT_ADV_MAX_ADV_SET];
#if defined(CONFIG_BT_PER_ADV_SYNC)
extern size_t selected_per_adv_sync;
extern struct bt_le_per_adv_sync *per_adv_syncs[CONFIG_BT_PER_ADV_SYNC_MAX];
#endif /* CONFIG_BT_PER_ADV_SYNC */
#endif /* CONFIG_BT_EXT_ADV */

/**
 * @brief Compares two strings without case sensitivity
 *
 * @param substr The substring
 * @param str    The string to find the substring in
 *
 * @return true if @substr is a substring of @p, else false
 */

bool is_substring(const char *substr, const char *str);

#endif /* __BT_H */
