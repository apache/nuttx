/****************************************************************************
 * include/nuttx/net/bluetooth/compat/psa/key_ids.h
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

/* Copyright (c) 2025 Nordic Semiconductor
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_PSA_KEY_IDS_H_
#define ZEPHYR_PSA_KEY_IDS_H_

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

typedef uint32_t psa_key_id_t;

#define ZEPHYR_PSA_OPENTHREAD_KEY_ID_RANGE_BEGIN (psa_key_id_t)0x20000
#define ZEPHYR_PSA_OPENTHREAD_KEY_ID_RANGE_SIZE  0x10000

#define ZEPHYR_PSA_MATTER_KEY_ID_RANGE_BEGIN (psa_key_id_t)0x30000
#define ZEPHYR_PSA_MATTER_KEY_ID_RANGE_SIZE  0x10000

#define ZEPHYR_PSA_BT_MESH_KEY_ID_RANGE_BEGIN (psa_key_id_t)0x20000000
#define ZEPHYR_PSA_BT_MESH_KEY_ID_RANGE_SIZE  0xc000

#define ZEPHYR_PSA_WIFI_CREDENTIALS_KEY_ID_RANGE_BEGIN (psa_key_id_t)0x20010000
#define ZEPHYR_PSA_WIFI_CREDENTIALS_KEY_ID_RANGE_SIZE  0x100

#define ZEPHYR_PSA_APPLICATION_KEY_ID_RANGE_BEGIN (psa_key_id_t)0x30000000
#define ZEPHYR_PSA_APPLICATION_KEY_ID_RANGE_SIZE  0x100000

#endif /* ZEPHYR_PSA_KEY_IDS_H_ */
