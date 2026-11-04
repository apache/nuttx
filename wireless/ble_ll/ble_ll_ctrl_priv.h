/****************************************************************************
 * wireless/ble_ll/ble_ll_ctrl_priv.h
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
 * Derived from Apache Mynewt NimBLE
 * (nimble/controller/src/ble_ll_ctrl_priv.h).
 *
 ****************************************************************************/

#ifndef __WIRELESS_BLE_LL_BLE_LL_CTRL_PRIV_H
#define __WIRELESS_BLE_LL_BLE_LL_CTRL_PRIV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "ble_ll_os.h"
#include "ble_ll_conn.h"

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void ble_ll_ctrl_init_conn_sm(struct ble_ll_conn_sm *connsm);

#endif /* __WIRELESS_BLE_LL_BLE_LL_CTRL_PRIV_H */
