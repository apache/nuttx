/****************************************************************************
 * net/bt/common/rpa.h
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

/* rpa.h - Bluetooth Resolvable Private Addresses (RPA) generation and
 * resolution
 */

/* Copyright (c) 2017 Nordic Semiconductor ASA Copyright (c) 2015-2016 Intel
 * Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/hci.h>

bool bt_rpa_irk_matches(const uint8_t irk[16], const bt_addr_t *addr);
int bt_rpa_create(const uint8_t irk[16], bt_addr_t *rpa);
