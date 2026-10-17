/****************************************************************************
 * include/nuttx/net/bluetooth/mesh.h
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
 *  @brief Bluetooth Mesh Profile APIs.
 */

/* Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>

#include <stdbool.h>
#include <stdint.h>
#include <sys/types.h>
#include <nuttx/mm/net_buf.h>

#include <nuttx/net/bluetooth/mesh/keys.h>
#include <nuttx/net/bluetooth/mesh/msg.h>
#include <nuttx/net/bluetooth/mesh/access.h>
#include <nuttx/net/bluetooth/mesh/main.h>
#include <nuttx/net/bluetooth/mesh/cfg.h>
#include <nuttx/net/bluetooth/mesh/cfg_srv.h>
#include <nuttx/net/bluetooth/mesh/health_srv.h>
#include <nuttx/net/bluetooth/mesh/blob_srv.h>
#include <nuttx/net/bluetooth/mesh/cfg_cli.h>
#include <nuttx/net/bluetooth/mesh/health_cli.h>
#include <nuttx/net/bluetooth/mesh/blob_cli.h>
#include <nuttx/net/bluetooth/mesh/blob_io_flash.h>
#include <nuttx/net/bluetooth/mesh/priv_beacon_srv.h>
#include <nuttx/net/bluetooth/mesh/priv_beacon_cli.h>
#include <nuttx/net/bluetooth/mesh/dfu_srv.h>
#include <nuttx/net/bluetooth/mesh/dfd_srv.h>
#include <nuttx/net/bluetooth/mesh/dfu_cli.h>
#include <nuttx/net/bluetooth/mesh/dfu_metadata.h>
#include <nuttx/net/bluetooth/mesh/proxy.h>
#include <nuttx/net/bluetooth/mesh/heartbeat.h>
#include <nuttx/net/bluetooth/mesh/cdb.h>
#include <nuttx/net/bluetooth/mesh/rpr_cli.h>
#include <nuttx/net/bluetooth/mesh/rpr_srv.h>
#include <nuttx/net/bluetooth/mesh/sar_cfg_srv.h>
#include <nuttx/net/bluetooth/mesh/sar_cfg_cli.h>
#include <nuttx/net/bluetooth/mesh/op_agg_srv.h>
#include <nuttx/net/bluetooth/mesh/op_agg_cli.h>
#include <nuttx/net/bluetooth/mesh/large_comp_data_srv.h>
#include <nuttx/net/bluetooth/mesh/large_comp_data_cli.h>
#include <nuttx/net/bluetooth/mesh/od_priv_proxy_srv.h>
#include <nuttx/net/bluetooth/mesh/od_priv_proxy_cli.h>
#include <nuttx/net/bluetooth/mesh/sol_pdu_rpl_srv.h>
#include <nuttx/net/bluetooth/mesh/sol_pdu_rpl_cli.h>
#include <nuttx/net/bluetooth/mesh/brg_cfg_cli.h>
#include <nuttx/net/bluetooth/mesh/brg_cfg_srv.h>
#include <nuttx/net/bluetooth/mesh/statistic.h>

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_H */
