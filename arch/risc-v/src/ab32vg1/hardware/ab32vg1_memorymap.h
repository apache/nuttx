/****************************************************************************
 * arch/risc-v/src/ab32vg1/hardware/ab32vg1_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_MEMORYMAP_H
#define __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_MEMORYMAP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* All peripheral registers live in the first 4 KB of the address space and
 * are 32 bits wide.  They are grouped in 16 "SFR banks" of 64 registers.
 */

#define AB32VG1_SFR0_BASE   0x00000000
#define AB32VG1_SFR1_BASE   0x00000100
#define AB32VG1_SFR2_BASE   0x00000200
#define AB32VG1_SFR3_BASE   0x00000300
#define AB32VG1_SFR4_BASE   0x00000400
#define AB32VG1_SFR5_BASE   0x00000500
#define AB32VG1_SFR6_BASE   0x00000600
#define AB32VG1_SFR7_BASE   0x00000700
#define AB32VG1_SFR8_BASE   0x00000800
#define AB32VG1_SFR9_BASE   0x00000900

#define AB32VG1_SFR(b, n)   (AB32VG1_SFR##b##_BASE + ((n) << 2))

#endif /* __ARCH_RISCV_SRC_AB32VG1_HARDWARE_AB32VG1_MEMORYMAP_H */
