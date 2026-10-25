/****************************************************************************
 * arch/risc-v/include/ab32vg1/chip.h
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

#ifndef __ARCH_RISCV_INCLUDE_AB32VG1_CHIP_H
#define __ARCH_RISCV_INCLUDE_AB32VG1_CHIP_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory map (AB32VG1 datasheet, "Memory mapping"):
 *
 *   0x00000000 - 0x00000fff  SFR space (accessed with x0-relative loads)
 *   0x00010000 - 0x0002ffff  SRAM0..3, 128 KB
 *   0x00050000 - 0x000579ff  AECRAM/DECRAM/ENCRAM, 30.5 KB
 *   0x00060000 - 0x00067fff  ICRAM (XIP cache lines), 32 KB
 *   0x00080000 - ...         Mask ROM
 *   0x10000000 - 0x100fffff  SPI flash window (software-cached XIP)
 */

#define AB32VG1_SRAM_BASE     0x00010000
#define AB32VG1_SRAM_SIZE     0x00020000
#define AB32VG1_AECRAM_BASE   0x00050000
#define AB32VG1_AECRAM_SIZE   0x00007a00
#define AB32VG1_ICRAM_BASE    0x00060000
#define AB32VG1_ICRAM_SIZE    0x00008000

#endif /* __ARCH_RISCV_INCLUDE_AB32VG1_CHIP_H */
