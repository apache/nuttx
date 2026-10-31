/****************************************************************************
 * arch/rh850/include/u2c/chip.h
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

#ifndef __ARCH_RH850_INCLUDE_U2C_CHIP_H
#define __ARCH_RH850_INCLUDE_U2C_CHIP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RH850/U2C8 (R7F702600), CPU0 (G4KH) memory map */

#define RH850_CODEFLASH_BASE    0x00000000
#define RH850_CODEFLASH_SIZE    (8 * 1024 * 1024)
#define RH850_LRAM_BASE         0xfdc00000
#define RH850_LRAM_SIZE         (128 * 1024)

#endif /* __ARCH_RH850_INCLUDE_U2C_CHIP_H */
