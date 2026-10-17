/****************************************************************************
 * include/nuttx/net/bluetooth/mesh/sar_cfg.h
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

/* Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Header file for Bluetooth Mesh SAR Configuration common
 * definitions.
 * @ingroup bt_mesh_sar_cfg
 */

#ifndef __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_SAR_CFG_H
#define __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_SAR_CFG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @defgroup bt_mesh_sar_cfg SAR Configuration common header
 * @ingroup bt_mesh
 * @{
 */

/** SAR Transmitter Configuration state structure */

struct bt_mesh_sar_tx
{
  /** SAR Segment Interval Step state */

  uint8_t seg_int_step;

  /** SAR Unicast Retransmissions Count state */

  uint8_t unicast_retrans_count;

  /** SAR Unicast Retransmissions Without Progress Count state */

  uint8_t unicast_retrans_without_prog_count;

  /* SAR Unicast Retransmissions Interval Step state */

  uint8_t unicast_retrans_int_step;

  /** SAR Unicast Retransmissions Interval Increment state */

  uint8_t unicast_retrans_int_inc;

  /** SAR Multicast Retransmissions Count state */

  uint8_t multicast_retrans_count;

  /** SAR Multicast Retransmissions Interval state */

  uint8_t multicast_retrans_int;
};

/** SAR Receiver Configuration state structure */

struct bt_mesh_sar_rx
{
  /** SAR Segments Threshold state */

  uint8_t seg_thresh;

  /** SAR Acknowledgment Delay Increment state */

  uint8_t ack_delay_inc;

  /** SAR Discard Timeout state */

  uint8_t discard_timeout;

  /** SAR Receiver Segment Interval Step state */

  uint8_t rx_seg_int_step;

  /** SAR Acknowledgment Retransmissions Count state */

  uint8_t ack_retrans_count;
};

/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NET_BLUETOOTH_MESH_SAR_CFG_H */
