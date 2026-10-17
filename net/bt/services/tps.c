/****************************************************************************
 * net/bt/services/tps.c
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
 *  @brief GATT TX Power Service
 */

/* Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/conn.h>
#include <nuttx/net/bluetooth/gatt.h>
#include <nuttx/net/bluetooth/uuid.h>
#include <nuttx/net/bluetooth/hci.h>

#define LOG_LEVEL CONFIG_BT_TPS_LOG_LEVEL
#include <nuttx/net/bluetooth/compat/logging/log.h>
LOG_MODULE_REGISTER(tps);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static ssize_t read_tx_power_level(struct bt_conn *conn,
                                   const struct bt_gatt_attr *attr,
                                   void *buf,
                                   uint16_t len,
                                   uint16_t offset)
{
  int err;
  struct bt_conn_le_tx_power tx_power_level =
    {
        0
    };

  if (offset)
    {
      return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
    }

  err = bt_conn_le_get_tx_power_level(conn, &tx_power_level);
  if (err)
    {
      LOG_ERR("Failed to read Tx Power Level over HCI: %d", err);
      return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

  LOG_INF("TPS Tx Power Level read %d", tx_power_level.current_level);

  return bt_gatt_attr_read(conn, attr, buf, len, offset,
          &tx_power_level.current_level,
                           sizeof(tx_power_level.current_level));
}

BT_GATT_SERVICE_DEFINE(tps_svc, BT_GATT_PRIMARY_SERVICE(BT_UUID_TPS),
                       BT_GATT_CHARACTERISTIC(BT_UUID_TPS_TX_POWER_LEVEL,
                           BT_GATT_CHRC_READ,

                                              BT_GATT_PERM_READ,
                                                  read_tx_power_level, NULL,
                                              NULL),
                                              );
