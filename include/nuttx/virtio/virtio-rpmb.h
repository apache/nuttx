/****************************************************************************
 * include/nuttx/virtio/virtio-rpmb.h
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

#ifndef __INCLUDE_NUTTX_VIRTIO_VIRTIO_RPMB_H
#define __INCLUDE_NUTTX_VIRTIO_VIRTIO_RPMB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RPMB Request Types */

#define VIRTIO_RPMB_REQ_PROGRAM_KEY       0x0001
#define VIRTIO_RPMB_REQ_GET_WRITE_COUNTER 0x0002
#define VIRTIO_RPMB_REQ_DATA_WRITE        0x0003
#define VIRTIO_RPMB_REQ_DATA_READ         0x0004
#define VIRTIO_RPMB_REQ_RESULT_READ       0x0005

/* RPMB Response Types */

#define VIRTIO_RPMB_RESP_PROGRAM_KEY      0x0100
#define VIRTIO_RPMB_RESP_GET_COUNTER      0x0200
#define VIRTIO_RPMB_RESP_DATA_WRITE       0x0300
#define VIRTIO_RPMB_RESP_DATA_READ        0x0400

/* RPMB Operation Results
 * VIRTIO_RPMB_RES_OK              : Operation successful
 * VIRTIO_RPMB_RES_GENERAL_FAILURE : General failure
 * VIRTIO_RPMB_RES_AUTH_FAILURE    : Mac doesn't match or calculation
 * failure
 * VIRTIO_RPMB_RES_COUNT_FAILURE   : Counter doesn't match or counter
 * increment failure
 * VIRTIO_RPMB_RES_ADDR_FAILURE    : Address out of range or wrong
 * address alignment
 * VIRTIO_RPMB_RES_WRITE_FAILURE   : Data, counter, or result write failure
 * VIRTIO_RPMB_RES_READ_FAILURE    : Data, counter, or result read failure
 * VIRTIO_RPMB_RES_NO_AUTH_KEY     : Authentication key not yet programmed
 * VIRTIO_RPMB_RES_WRITE_COUNTER_EXPIRED:  Counter expired
 */

#define VIRTIO_RPMB_RES_OK                    0x0000
#define VIRTIO_RPMB_RES_GENERAL_FAILURE       0x0001
#define VIRTIO_RPMB_RES_AUTH_FAILURE          0x0002
#define VIRTIO_RPMB_RES_COUNT_FAILURE         0x0003
#define VIRTIO_RPMB_RES_ADDR_FAILURE          0x0004
#define VIRTIO_RPMB_RES_WRITE_FAILURE         0x0005
#define VIRTIO_RPMB_RES_READ_FAILURE          0x0006
#define VIRTIO_RPMB_RES_NO_AUTH_KEY           0x0007
#define VIRTIO_RPMB_RES_WRITE_COUNTER_EXPIRED 0x0080

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Device configuration layout (virtio spec 5.12.4) */

begin_packed_struct struct virtio_rpmb_config_s
{
  uint8_t capacity;
  uint8_t max_wr_cnt;
  uint8_t max_rd_cnt;
} end_packed_struct;

/* RPMB frame (virtio spec 5.12.6.1, 512 bytes) */

begin_packed_struct struct virtio_rpmb_frame
{
  uint8_t  stuff[196];
  uint8_t  key_mac[32];
  uint8_t  data[256];
  uint8_t  nonce[16];
  uint32_t write_counter;
  uint16_t addr;
  uint16_t block_count;
  uint16_t result;
  uint16_t req_resp;
} end_packed_struct;

#endif /* __INCLUDE_NUTTX_VIRTIO_VIRTIO_RPMB_H */
