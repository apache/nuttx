/****************************************************************************
 * arch/arm64/include/rk3588/chip.h
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

#ifndef __ARCH_ARM64_INCLUDE_RK3588_CHIP_H
#define __ARCH_ARM64_INCLUDE_RK3588_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Rockchip RK3588 / RK3588S GICv3 base addresses.  The upstream device tree
 * places the distributor at 0xfe600000 and the redistributor window at
 * 0xfe680000.
 */

#define CONFIG_GICD_BASE          0xfe600000
#define CONFIG_GICD_SIZE          0x00010000
#define CONFIG_GICR_BASE          0xfe680000
#define CONFIG_GICR_SIZE          0x00100000
#define CONFIG_GICR_OFFSET        0x20000

/* U-Boot relocates the ARM64 Image from kernel_addr_r to 0x02080000.  The
 * initial port uses the 512 MiB window validated on the 4 GiB board.
 */

#define CONFIG_RAMBANK1_ADDR      CONFIG_RAM_START
#define CONFIG_RAMBANK1_SIZE      CONFIG_RAM_SIZE

/* The UART register window is 0x100 bytes in the upstream device tree.  The
 * MMU maps it as one 4 KiB page, its minimum mapping granularity.
 */

#define RK3588_UART_MMU_SIZE      0x1000

/* Address at which U-Boot enters NuttX after Image relocation. */

#define CONFIG_LOAD_BASE          CONFIG_RAM_START

#define MPID_TO_CLUSTER_ID(mpid)  ((mpid) & ~0xff)

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__

.macro  get_cpu_id xreg0
  mrs    \xreg0, mpidr_el1
  ubfx   \xreg0, \xreg0, #0, #8
.endm

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_INCLUDE_RK3588_CHIP_H */
