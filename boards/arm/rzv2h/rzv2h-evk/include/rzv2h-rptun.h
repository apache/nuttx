/****************************************************************************
 * boards/arm/rzv2h/rzv2h-evk/include/rzv2h-rptun.h
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

#ifndef __BOARDS_ARM_RZV2H_RZV2H_EVK_INCLUDE_RZV2H_RPTUN_H
#define __BOARDS_ARM_RZV2H_RZV2H_EVK_INCLUDE_RZV2H_RPTUN_H

/* Multi-OS Package v3.3.0 CR8_0 service 0 UIO layout.  This header is also
 * consumed by the preprocessed linker script, so keep it macro-only.
 */

/* Multi-OS Package v3.3.0 CR8_0 UIO layout.  Linux physical addresses,
 * OpenAMP device addresses, and CR8 virtual addresses are identical.
 */

#define BOARD_RPTUN_RESOURCE_ADDR       0x42f00000
#define BOARD_RPTUN_RESOURCE_SIZE       0x00001000
#define BOARD_RPTUN_RESOURCE_ENTRY_SIZE 0x00000070
#define BOARD_RPTUN_RESOURCE_RPROC_MEM_OFFSET 0x00000018
#define BOARD_RPTUN_RESOURCE_VDEV_OFFSET 0x0000002c
#define BOARD_RPTUN_MHU_SHMEM_ADDR      0x42f01000
#define BOARD_RPTUN_MHU_SHMEM_SIZE      0x00001000
#define BOARD_RPTUN_MHU_SHMEM_SLOT_SIZE 0x00000008
#define BOARD_RPTUN_RPMSG_ADDR          0x43000000
#define BOARD_RPTUN_RPMSG_SIZE          0x00800000
#define BOARD_RPTUN_RPMSG_MPU_SIZE      0x01000000
#define BOARD_RPTUN_VRING_ALIGN         0x00000100
#define BOARD_RPTUN_VRING_NUM           512
#define BOARD_RPTUN_VDEV_NOTIFYID       0
#define BOARD_RPTUN_BUFFER_SIZE         0x00300000
#define BOARD_RPTUN_REMOTE_CPUNAME      "linux"
#define BOARD_RPTUN_EPT_ADDR            0

#define BOARD_RPTUN_CHANNEL0_VRING0_ADDR 0x43000000
#define BOARD_RPTUN_CHANNEL0_VRING1_ADDR 0x43050000
#define BOARD_RPTUN_CHANNEL0_BUFFER_ADDR 0x43200000
#define BOARD_RPTUN_CHANNEL0_NOTIFYID  0
#define BOARD_RPTUN_CHANNEL0_EPT_NAME  "rpmsg-service-0"

#define BOARD_RPTUN_CHANNEL1_VRING0_ADDR 0x43100000
#define BOARD_RPTUN_CHANNEL1_VRING1_ADDR 0x43150000
#define BOARD_RPTUN_CHANNEL1_BUFFER_ADDR 0x43500000
#define BOARD_RPTUN_CHANNEL1_NOTIFYID  1
#define BOARD_RPTUN_CHANNEL1_EPT_NAME  "rpmsg-service-1"

#ifdef CONFIG_RZV2H_RPTUN_CHANNEL1
#  define BOARD_RPTUN_CHANNEL          1
#  define BOARD_RPTUN_VRING0_ADDR      BOARD_RPTUN_CHANNEL1_VRING0_ADDR
#  define BOARD_RPTUN_VRING1_ADDR      BOARD_RPTUN_CHANNEL1_VRING1_ADDR
#  define BOARD_RPTUN_BUFFER_ADDR      BOARD_RPTUN_CHANNEL1_BUFFER_ADDR
#  define BOARD_RPTUN_NOTIFYID         BOARD_RPTUN_CHANNEL1_NOTIFYID
#  define BOARD_RPTUN_EPT_NAME         BOARD_RPTUN_CHANNEL1_EPT_NAME
#else
#  define BOARD_RPTUN_CHANNEL          0
#  define BOARD_RPTUN_VRING0_ADDR      BOARD_RPTUN_CHANNEL0_VRING0_ADDR
#  define BOARD_RPTUN_VRING1_ADDR      BOARD_RPTUN_CHANNEL0_VRING1_ADDR
#  define BOARD_RPTUN_BUFFER_ADDR      BOARD_RPTUN_CHANNEL0_BUFFER_ADDR
#  define BOARD_RPTUN_NOTIFYID         BOARD_RPTUN_CHANNEL0_NOTIFYID
#  define BOARD_RPTUN_EPT_NAME         BOARD_RPTUN_CHANNEL0_EPT_NAME
#endif

#endif /* __BOARDS_ARM_RZV2H_RZV2H_EVK_INCLUDE_RZV2H_RPTUN_H */
