/****************************************************************************
 * boards/risc-v/ab32vg1/ab32vg1-generic/tools/target/stub.h
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

/* Protocol between the host tool (ab32vg1_dl.c) and the flash programming
 * stub (target/stub.c).  This header is shared by both.
 */

#ifndef __BOARDS_RISCV_AB32VG1_AB32VG1_GENERIC_TOOLS_TARGET_STUB_H
#define __BOARDS_RISCV_AB32VG1_AB32VG1_GENERIC_TOOLS_TARGET_STUB_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STUB_VERSION        1

#define STUB_REQ_SYNC       0x5a
#define STUB_RSP_SYNC       0xa5

#define STUB_MAXDATA        4096

/* Commands: field usage (addr = 32-bit header field, data = payload) */

#define STUB_CMD_INFO       0x01  /* -> id32, status8, version8 */
#define STUB_CMD_READ       0x02  /* addr = flash, data = size32 -> data */
#define STUB_CMD_ERASE4K    0x03  /* addr = flash address */
#define STUB_CMD_ERASE64K   0x04  /* addr = flash address */
#define STUB_CMD_WRITE      0x05  /* addr = flash address, data = bytes */
#define STUB_CMD_CRC32      0x06  /* addr = flash, data = size32 -> crc32 */
#define STUB_CMD_RESET      0x07  /* watchdog reset */
#define STUB_CMD_PEEK       0x08  /* addr = register address -> value32 */
#define STUB_CMD_MEMREAD    0x09  /* addr = memory, data = size32 -> data */

/* Status */

#define STUB_OK             0x00
#define STUB_ECRC           0x01
#define STUB_ECMD           0x02
#define STUB_EARG           0x03

#endif /* __BOARDS_RISCV_AB32VG1_AB32VG1_GENERIC_TOOLS_TARGET_STUB_H */
