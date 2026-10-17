/****************************************************************************
 * net/bt/host/direction_internal.h
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

/* Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/bluetooth/bluetooth.h>
#include <nuttx/net/bluetooth/direction.h>
#include <nuttx/mm/net_buf.h>

/* Performs initialization of Direction Finding in Host */

int le_df_init(void);

int hci_df_prepare_connectionless_iq_report(struct net_buf *buf,
  struct bt_df_per_adv_sync_iq_samples_report *report,
  struct bt_le_per_adv_sync **per_adv_sync_to_report);
int hci_df_vs_prepare_connectionless_iq_report(struct net_buf *buf,
  struct bt_df_per_adv_sync_iq_samples_report *report,
  struct bt_le_per_adv_sync **per_adv_sync_to_report);
int hci_df_prepare_connection_iq_report(struct net_buf *buf,
  struct bt_df_conn_iq_samples_report *report,
  struct bt_conn **conn_to_report);
int hci_df_vs_prepare_connection_iq_report(struct net_buf *buf,
  struct bt_df_conn_iq_samples_report *report,
  struct bt_conn **conn_to_report);
int hci_df_prepare_conn_cte_req_failed(struct net_buf *buf,
  struct bt_df_conn_iq_samples_report *report,
  struct bt_conn **conn_to_report);
