/****************************************************************************
 * drivers/crypto/rpmsg_crypto.h
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

#ifndef __DRIVERS_CRYPTO_RPMSG_CRYPTO_H
#define __DRIVERS_CRYPTO_RPMSG_CRYPTO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/virtio/virtio-crypto.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

#define RPMSG_CRYPTO_NAME_PREFIX     "rpmsg-crypto"
#define RPMSG_CRYPTO_NAME_PREFIX_LEN sizeof(RPMSG_CRYPTO_NAME_PREFIX)

#define RPMSG_CRYPTO_NEWSESSION      0
#define RPMSG_CRYPTO_PROCESS         1
#define RPMSG_CRYPTO_FREESESSION     2
#define RPMSG_CRYPTO_KPROCESS        3
#define RPMSG_CRYPTO_KEYPROCESS      4
#define RPMSG_CRYPTO_CANCEL          5

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct rpmsg_crypto_header_s
{
  uint32_t command;
  int32_t  result;
  uint64_t tid;
  uint64_t cookie;
  uint32_t reqid;
} end_packed_struct;

begin_packed_struct struct rpmsg_crypto_newsession_s
{
  struct rpmsg_crypto_header_s header;
  struct virtio_crypto_op_ctrl_req_s ctrl;
  char buf[];
} end_packed_struct;

#define rpmsg_crypto_freesession_s rpmsg_crypto_newsession_s

begin_packed_struct struct rpmsg_crypto_process_s
{
  struct rpmsg_crypto_header_s header;
  struct virtio_crypto_op_data_req_s data;
  char buf[];
} end_packed_struct;

begin_packed_struct struct rpmsg_crypto_kprocess_s
{
  struct rpmsg_crypto_header_s header;
  struct virtio_crypto_op_ctrl_req_s ctrl;
  struct virtio_crypto_op_data_req_s data;
  char buf[];
} end_packed_struct;

begin_packed_struct struct rpmsg_crypto_keyprocess_s
{
  struct rpmsg_crypto_header_s header;
  struct virtio_key_op_req_s data;
  char buf[];
} end_packed_struct;

begin_packed_struct struct rpmsg_crypto_cancel_s
{
  struct rpmsg_crypto_header_s header;
  uint32_t op;
} end_packed_struct;

#endif /* __DRIVERS_CRYPTO_RPMSG_CRYPTO_H */
