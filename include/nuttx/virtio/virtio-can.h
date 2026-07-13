/****************************************************************************
 * include/nuttx/virtio/virtio-can.h
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

#ifndef __INCLUDE_NUTTX_VIRTIO_VIRTIO_CAN_H
#define __INCLUDE_NUTTX_VIRTIO_VIRTIO_CAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/can.h>
#include <nuttx/compiler.h>

#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* OASIS VirtIO 1.4 CAN device (device id 36).  Virtqueues: txq (0), rxq (1),
 * controlq (2).
 */

#define VIRTIO_CAN_F_CAN_CLASSIC       0
#define VIRTIO_CAN_F_CAN_FD            1
#define VIRTIO_CAN_F_RTR_FRAMES        2
#define VIRTIO_CAN_F_LATE_TX_ACK       3

#define VIRTIO_CAN_RESULT_OK           0
#define VIRTIO_CAN_RESULT_NOT_OK       1

#define VIRTIO_CAN_FLAGS_EXTENDED      0x8000
#define VIRTIO_CAN_FLAGS_FD            0x4000
#define VIRTIO_CAN_FLAGS_RTR           0x2000

#define VIRTIO_CAN_IFNAME_SIZE         16

#define VIRTIO_CAN_TX                  0x0001
#define VIRTIO_CAN_RX                  0x0101

#define VIRTIO_CAN_SET_CTRL_MODE_START 0x0201
#define VIRTIO_CAN_SET_CTRL_MODE_STOP  0x0202

#define VIRTIO_CAN_S_CTRL_BUSOFF       (1u << 0)

/* Bytes of an RX frame message carrying an sdu of length payload bytes. */

#define VIRTIO_CAN_RX_SIZE(length) \
  (offsetof(struct virtio_can_rx_s, sdu) + (length))

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct virtio_can_config_s
{
  uint16_t status;
} end_packed_struct;

/* NuttX-private device config: the OASIS config followed by the name of
 * the local CAN interface the vhost backend bridges to.  A standard driver
 * reads only the leading spec config.
 */

begin_packed_struct struct vhost_can_config_s
{
  struct virtio_can_config_s config;
  char ifname[VIRTIO_CAN_IFNAME_SIZE];
} end_packed_struct;

begin_packed_struct struct virtio_can_tx_out_s
{
  uint16_t msg_type;
  uint16_t length;
  uint8_t  reserved_classic_dlc;
  uint8_t  padding;
  uint16_t reserved_xl_priority;
  uint32_t flags;
  uint32_t can_id;
  uint8_t  sdu[CANFD_MAX_DLEN];
} end_packed_struct;

begin_packed_struct struct virtio_can_tx_in_s
{
  uint8_t result;
} end_packed_struct;

begin_packed_struct struct virtio_can_rx_s
{
  uint16_t msg_type;
  uint16_t length;
  uint8_t  reserved_classic_dlc;
  uint8_t  padding;
  uint16_t reserved_xl_priority;
  uint32_t flags;
  uint32_t can_id;
  uint8_t  sdu[CANFD_MAX_DLEN];
} end_packed_struct;

begin_packed_struct struct virtio_can_control_out_s
{
  uint16_t msg_type;
} end_packed_struct;

begin_packed_struct struct virtio_can_control_in_s
{
  uint8_t result;
} end_packed_struct;

#endif /* __INCLUDE_NUTTX_VIRTIO_VIRTIO_CAN_H */
