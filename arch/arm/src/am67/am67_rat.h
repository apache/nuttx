/****************************************************************************
 * arch/arm/src/am67/am67_rat.h
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

#ifndef __ARCH_ARM_SRC_AM67_AM67_RAT_H
#define __ARCH_ARM_SRC_AM67_AM67_RAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* 32-bit address range claimed by the RAT sliding window.  Must be mapped
 * Non-cacheable in the MPU (am67_mpuinit.c) and must not overlap anything
 * the firmware needs to address directly (NuttX RAM/IPC live in
 * 0xa2000000-0xa3000000).
 */

#define AM67_RAT_WIN_BASE    (0xfe000000ul)
#define AM67_RAT_WIN_SIZE    (0x01000000ul) /* 16 MB */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR void *am67_rat_map(uint64_t pa, FAR size_t *avail);

#endif /* __ARCH_ARM_SRC_AM67_AM67_RAT_H */
