/****************************************************************************
 * net/bt/host/gatt_gap_svc_validate.h
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
 *  @brief Internal API GATT GAP service validation.
 */

/* Copyright (c) 2025 Koppel Electronic
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_BLUETOOTH_HOST_GATT_GAP_SVC_VALIDATE_H
#define ZEPHYR_SUBSYS_BLUETOOTH_HOST_GATT_GAP_SVC_VALIDATE_H

/** @brief Validate GATT database contains exactly one GAP service.
 *
 * This function iterates through the GATT attribute database and verifies
 * that exactly one Generic Access Profile (GAP) service is present. The
 * Bluetooth specification requires that a GATT server expose exactly one GAP
 * service.
 *
 * @return 0 on success (exactly one GAP service found).
 * @return -EINVAL if zero or multiple GAP services are found.
 */

int gatt_gap_svc_validate(void);

#endif /* ZEPHYR_SUBSYS_BLUETOOTH_HOST_GATT_GAP_SVC_VALIDATE_H */
