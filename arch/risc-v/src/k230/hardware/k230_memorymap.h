/****************************************************************************
 * arch/risc-v/src/k230/hardware/k230_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_K230_HARDWARE_K230_MEMORYMAP_H
#define __ARCH_RISCV_SRC_K230_HARDWARE_K230_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Devices Base Address */

#define K230_PLIC_BASE    UINT64_C(0xF00000000)
#define K230_CLINT_BASE   (K230_PLIC_BASE + 0x04000000)
#define K230_CPU1_BOOTA   0x91102104UL
#define K230_CPU1_RESET   0x9110100cUL

/* T-Head c908 specific CSR */

#define CSR_MENVCFG      0x30a
#define CSR_MXSTATUS     0x7c0
#define CSR_MHCR         0x7c1
#define CSR_MCOR         0x7c2
#define CSR_MCCR2        0x7c3
#define CSR_MHINT        0x7c5
#define CSR_MSMPR        0x7f3
#define CSR_PLIC_BASE    0xfc1
#define CSR_MAGIC        0x7d9

/* Enable RV PBMT */

#define MENVCFG_PBMT     (UINT64_C(1) << 62)
#endif /* __ARCH_RISCV_SRC_K230_HARDWARE_K230_MEMORYMAP_H */
