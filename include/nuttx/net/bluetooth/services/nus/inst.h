/****************************************************************************
 * include/nuttx/net/bluetooth/services/nus/inst.h
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

/* Copyright (c) 2024 Croxel, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_SERVICES_NUS_INST_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_SERVICES_NUS_INST_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/gatt.h>
#include <nuttx/net/bluetooth/compat/sys/iterable_sections.h>
#include <nuttx/list.h>

#ifdef __cplusplus
extern "C"
{
#endif

struct bt_nus_inst
{
  /** Pointer to the NUS Service Instance  */

  const struct bt_gatt_service_static *svc;

  /** List of subscribers to invoke callbacks on asynchronous events */

  struct list_node *cbs;
};

#define BT_UUID_NUS_SERVICE BT_UUID_DECLARE_128(BT_UUID_NUS_SRV_VAL)
#define BT_UUID_NUS_TX_CHAR BT_UUID_DECLARE_128(BT_UUID_NUS_TX_CHAR_VAL)
#define BT_UUID_NUS_RX_CHAR BT_UUID_DECLARE_128(BT_UUID_NUS_RX_CHAR_VAL)

/** Required as the service may be instantiated outside of the module */

ssize_t nus_bt_chr_write(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr,
                         const void *buf,
                         uint16_t len,
                         uint16_t offset,
                         uint8_t flags);
void nus_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value);

#define Z_INTERNAL_BT_NUS_INST_DEFINE(_name)                                                       \
                                                                                                 \
BT_GATT_SERVICE_DEFINE(                                                                          \
        _name##_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_NUS_SERVICE),                               \
        BT_GATT_CHARACTERISTIC(BT_UUID_NUS_TX_CHAR, BT_GATT_CHRC_NOTIFY, BT_GATT_PERM_NONE,      \
                               NULL, NULL, NULL),                                                \
        BT_GATT_CCC(nus_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),                \
        BT_GATT_CHARACTERISTIC(BT_UUID_NUS_RX_CHAR,                                              \
                               BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP,             \
                               BT_GATT_PERM_WRITE, NULL, nus_bt_chr_write, NULL), );             \
                                                                                                 \
struct list_node _name##_cbs = LIST_INITIAL_VALUE(_name##_cbs);                                  \
                                                                                                 \
STRUCT_SECTION_ITERABLE(bt_nus_inst, _name) = {                                                  \
        .svc = &_name##_svc,                                                                     \
        .cbs = &_name##_cbs,                                                                     \
}

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_SERVICES_NUS_INST_H */
