/****************************************************************************
 * arch/arm/src/mps/hardware/mps_memorymap.h
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

#ifndef __ARCH_ARM_SRC_MPS_HARDWARE_MPS_MEMORYMAP_H
#define __ARCH_ARM_SRC_MPS_HARDWARE_MPS_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* qemu/hw/arm/mps2.c
 *
 * The MPS2 and MPS2+ dev boards are FPGA based (the 2+ has a bigger
 * FPGA but is otherwise the same as the 2). Since the CPU itself
 * and most of the devices are in the FPGA, the details of the board
 * as seen by the guest depend significantly on the FPGA image.
 * We model the following FPGA images:
 *  "mps2-an385" -- Cortex-M3 as documented in ARM Application Note AN385
 *  "mps2-an386" -- Cortex-M4 as documented in ARM Application Note AN386
 *  "mps2-an500" -- Cortex-M7 as documented in ARM Application Note AN500
 *  "mps2-an511" -- Cortex-M3 'DesignStart' as documented in AN511
 *
 * Links to the TRM for the board itself and to the various Application
 *
 * System memory map
 *
 * The FPGA images have an odd combination of different RAMs,
 * because in hardware they are different implementations and
 * connected to different buses, giving varying performance/size
 * tradeoffs. For QEMU they're all just RAM, though. We arbitrarily
 * call the 16MB our "system memory", as it's the largest lump.
 *
 * AN385/AN386/AN511:
 *  0x21000000 .. 0x21ffffff : PSRAM (16MB)
 * AN385/AN386/AN500:
 *  0x00000000 .. 0x003fffff : ZBT SSRAM1 (4MB)
 *  0x00400000 .. 0x007fffff : mirror of ZBT SSRAM1
 *  0x20000000 .. 0x203fffff : ZBT SSRAM 2&3 (4MB)
 *  0x20400000 .. 0x207fffff : mirror of ZBT SSRAM 2&3
 * AN385/AN386 only:
 *  0x01000000 .. 0x01003fff : block RAM (16K)
 *  0x01004000 .. 0x01007fff : mirror of above
 *  0x01008000 .. 0x0100bfff : mirror of above
 *  0x0100c000 .. 0x0100ffff : mirror of above
 * AN511 only:
 *  0x00000000 .. 0x0003ffff : FPGA block RAM
 *  0x00400000 .. 0x007fffff : ZBT SSRAM1
 *  0x20000000 .. 0x2001ffff : SRAM
 *  0x20400000 .. 0x207fffff : ZBT SSRAM 2&3
 * AN500 only:
 *  0x60000000 .. 0x60ffffff : PSRAM (16MB)
 *
 * The AN385/AN386 has a feature where the lowest 16K can be mapped
 * either to the bottom of the ZBT SSRAM1 or to the block RAM.
 * This is of no use for QEMU so we don't implement it (as if
 * zbt_boot_ctrl is always zero).
 */

#if defined(CONFIG_ARCH_CHIP_MPS3_AN547)

/* flash (512KB) */
#define MPS_ITCM_START     0x00000000
#define MPS_ITCM_SIZE      0x00080000

/* SRAM1 (2MB) */
#define MPS_SRAM1_START    0x01000000
#define MPS_SRAM1_SIZE     0x00200000

/* SRAM2 (4MB) */
#define MPS_SRAM2_START    0x21000000
#define MPS_SRAM2_SIZE     0x00400000

/* External storage (2GB) */
#define MPS_EXTMEM_START   0x60000000
#define MPS_EXTMEM_SIZE    0x80000000

#define PRIMARY_RAM_START  MPS_SRAM2_START
#define PRIMARY_RAM_SIZE   MPS_SRAM2_SIZE

#if CONFIG_MM_REGIONS > 1
#define REGION1_RAM_START  MPS_EXTMEM_START
#define REGION1_RAM_SIZE   MPS_EXTMEM_SIZE

#endif /* CONFIG_MM_REGIONS > 1 */

#elif defined(CONFIG_ARCH_CHIP_MPS2_AN524)

/* Internal SRAM1 (16MB) */
#define MPS_SRAM1_START    0x20000000
#define MPS_SRAM1_SIZE     0x01000000

/* Internal SRAM2 (16MB) */
#define MPS_SRAM2_START    0x30000000
#define MPS_SRAM2_SIZE     0x01000000

#define PRIMARY_RAM_START  MPS_SRAM1_START
#define PRIMARY_RAM_SIZE   MPS_SRAM1_SIZE

#if CONFIG_MM_REGIONS > 1
#define REGION1_RAM_START  MPS_SRAM2_START
#define REGION1_RAM_SIZE   MPS_SRAM2_SIZE

#endif /* CONFIG_MM_REGIONS > 1 */

#elif defined(CONFIG_ARCH_CHIP_MPS2_AN521)

/* SSRAM1 (4MB) */
#define MPS_SRAM1_START    0x10000000
#define MPS_SRAM1_SIZE     0x00400000

/* SSRAM2_3 (4MB) */
#define MPS_SRAM23_START   0x38000000
#define MPS_SRAM23_SIZE    0x00400000

#define PRIMARY_RAM_START  MPS_SRAM23_START
#define PRIMARY_RAM_SIZE   MPS_SRAM23_SIZE

#if CONFIG_MM_REGIONS > 1
#define REGION1_RAM_START  MPS_SRAM23_START
#define REGION1_RAM_SIZE   MPS_SRAM23_SIZE

#endif /* CONFIG_MM_REGIONS > 1 */

#elif defined(CONFIG_ARCH_CHIP_MPS2_AN500) || \
      defined(CONFIG_ARCH_CHIP_MPS2_AN386)

/* ZBT SSRAM1 (4MB) */

#define MPS_SRAM1_START    0x00000000
#define MPS_SRAM1_SIZE     0x00400000

/* ZBT SSRAM23 (4MB) */

#define MPS_SRAM2_START    0x20000000
#define MPS_SRAM2_SIZE     0x00400000

/* PSRAM (16MB) */

#define MPS_PSRAM_START    0x60000000
#define MPS_PSRAM_SIZE     0x01000000

#define PRIMARY_RAM_START  MPS_PSRAM_START
#define PRIMARY_RAM_SIZE   MPS_PSRAM_SIZE

#if CONFIG_MM_REGIONS > 1
#define REGION1_RAM_START  MPS_SRAM2_START
#define REGION1_RAM_SIZE   MPS_SRAM2_SIZE

#endif /* CONFIG_MM_REGIONS > 1 */

#else
#  error Unrecognized Arm MPS2 and MPS3 architecture
#endif

#define PRIMARY_RAM_END    (PRIMARY_RAM_START + PRIMARY_RAM_SIZE)

#endif /* __ARCH_ARM_SRC_MPS_HARDWARE_MPS_MEMORYMAP_H */
