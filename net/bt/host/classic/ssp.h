/****************************************************************************
 * net/bt/host/classic/ssp.h
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

/* Copyright (c) 2020 Nordic Semiconductor ASA Copyright (c) 2017 Intel
 * Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

int bt_ssp_start_security(struct bt_conn *conn);

int bt_ssp_auth_passkey_entry(struct bt_conn *conn, unsigned int passkey);
int bt_ssp_auth_passkey_confirm(struct bt_conn *conn);
int bt_ssp_auth_cancel(struct bt_conn *conn);
int bt_ssp_auth_pairing_confirm(struct bt_conn *conn);
