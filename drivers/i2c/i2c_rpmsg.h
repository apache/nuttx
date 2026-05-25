/****************************************************************************
 * drivers/i2c/i2c_rpmsg.h
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

#ifndef __DRIVERS_I2C_I2C_RPMSG_H
#define __DRIVERS_I2C_I2C_RPMSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Native rpmsg wire protocol between i2c_rpmsg client and i2c_rpmsg_server.
 * This is a private NuttX-to-NuttX format (no VirtIO/spec constraints): each
 * message carries its own frequency, slave address and native I2C_M_* flags
 * in host byte order, and the whole sequence is handed to a single
 * I2C_TRANSFER() on the server.  Keep client and server in sync by defining
 * the command codes and message layout in this single private header.
 */

#define I2C_RPMSG_EPT_FORMAT "rpmsg-i2c-%s"

#define I2C_RPMSG_TRANSFER   0
#define I2C_RPMSG_RESET      1
#define I2C_RPMSG_SETUP      2
#define I2C_RPMSG_SHUTDOWN   3

/****************************************************************************
 * Public Types
 ****************************************************************************/

begin_packed_struct struct i2c_rpmsg_header_s
{
  uint32_t command;
  int32_t result;
  uint64_t cookie;
} end_packed_struct;

/* Per-message descriptor.  Data follows the descriptor array: the request
 * packs write payloads only; the response has one slot per message.
 */

begin_packed_struct struct i2c_rpmsg_msg_hdr_s
{
  uint32_t frequency;
  uint16_t addr;
  uint16_t flags;
  uint32_t length;
} end_packed_struct;

begin_packed_struct struct i2c_rpmsg_transfer_s
{
  struct i2c_rpmsg_header_s header;
  uint32_t count;
  struct i2c_rpmsg_msg_hdr_s msg[1];
} end_packed_struct;

begin_packed_struct struct i2c_rpmsg_simple_s
{
  struct i2c_rpmsg_header_s header;
} end_packed_struct;

#endif /* __DRIVERS_I2C_I2C_RPMSG_H */
