/****************************************************************************
 * arch/arm/src/imxrt/hardware/rt118x/imxrt118x_dmamux.h
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

#ifndef __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_DMAMUX_H
#define __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_DMAMUX_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* eDMA channel source IDs for the LPUART instances (IMXRT1180RM Ch. 5).
 * Only the LPUART entries are needed by the shared serial driver.  DMA
 * support itself is disabled in the minimal RT1180 port.
 */

#define IMXRT_DMACHAN_LPUART1_TX   0
#define IMXRT_DMACHAN_LPUART1_RX   0
#define IMXRT_DMACHAN_LPUART2_TX   0
#define IMXRT_DMACHAN_LPUART2_RX   0
#define IMXRT_DMACHAN_LPUART3_TX   0
#define IMXRT_DMACHAN_LPUART3_RX   0
#define IMXRT_DMACHAN_LPUART4_TX   0
#define IMXRT_DMACHAN_LPUART4_RX   0
#define IMXRT_DMACHAN_LPUART5_TX   0
#define IMXRT_DMACHAN_LPUART5_RX   0
#define IMXRT_DMACHAN_LPUART6_TX   0
#define IMXRT_DMACHAN_LPUART6_RX   0
#define IMXRT_DMACHAN_LPUART7_TX   0
#define IMXRT_DMACHAN_LPUART7_RX   0
#define IMXRT_DMACHAN_LPUART8_TX   0
#define IMXRT_DMACHAN_LPUART8_RX   0
#define IMXRT_DMACHAN_LPUART9_TX   0
#define IMXRT_DMACHAN_LPUART9_RX   0
#define IMXRT_DMACHAN_LPUART10_TX  0
#define IMXRT_DMACHAN_LPUART10_RX  0
#define IMXRT_DMACHAN_LPUART11_TX  0
#define IMXRT_DMACHAN_LPUART11_RX  0
#define IMXRT_DMACHAN_LPUART12_TX  0
#define IMXRT_DMACHAN_LPUART12_RX  0

#endif /* __ARCH_ARM_SRC_IMXRT_HARDWARE_RT118X_IMXRT118X_DMAMUX_H */
