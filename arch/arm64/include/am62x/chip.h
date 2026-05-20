/****************************************************************************
 * arch/arm64/include/am62x/chip.h
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

/* Reference: AM62x TRM (SPRSP43)  §2.1 Memory Map  §9.2 GIC-600 */

#ifndef __ARCH_ARM64_INCLUDE_AM62X_CHIP_H
#define __ARCH_ARM64_INCLUDE_AM62X_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Size macros used in assembly and C */

#define KB(x)   ((x) << 10)
#define MB(x)   (KB(x) << 10)
#define GB(x)   (MB(UINT64_C(x)) << 10)

/* GIC-600 addresses (TRM §9.2) ********************************************/

/* Distributor — one per system */

#define CONFIG_GICD_BASE          0x01800000

/* Redistributor — one 128 KB frame per Cortex-A53 core */

#define CONFIG_GICR_BASE          0x01880000

/* Stride between consecutive per-core redistributor frames (128 KB) */

#define CONFIG_GICR_OFFSET        0x20000

/* Extract cluster ID from MPIDR_EL1.
 * AM62x has a single cluster (Aff1=0); mask off Aff0 (bits[7:0]).
 */

#define MPID_TO_CLUSTER_ID(mpid)  ((mpid) & ~0xffUL)

/* NuttX load address — must match U-Boot kernel_addr_r and the linker
 * script origin.  0x8200_0000 avoids U-Boot's reserved region at the
 * bottom of DDR (0x8000_0000–0x81FF_FFFF).
 */

#define CONFIG_LOAD_BASE          0x82000000

/* Memory map constants — used by am62x_boot.c MMU region table and by
 * the common arm64 linker/boot code.  These are NOT Kconfig symbols;
 * they are chip-level C macros, following the same pattern as a64/chip.h.
 * Board-specific DDR sizes are overridden in defconfig via CONFIG_RAM_SIZE
 * (decimal bytes) — these macros describe the default mapped window.
 */

/* DDR: 512 MB at 0x8000_0000 (common minimum for both boards) */

#define CONFIG_RAMBANK1_ADDR      0x80000000
#define CONFIG_RAMBANK1_SIZE      MB(512)

/* Device I/O flat-map: all peripherals in the lower 2 GB */

#define CONFIG_DEVICEIO_BASEADDR  0x00000000
#define CONFIG_DEVICEIO_SIZE      GB(2)

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__

/* get_cpu_id xreg0
 *
 * Read current CPU ID from MPIDR_EL1 Aff0 field [7:0].
 * On AM62x the four A53 cores are numbered 0-3 in Aff0.
 */

.macro  get_cpu_id xreg0
  mrs    \xreg0, mpidr_el1
  ubfx   \xreg0, \xreg0, #0, #8
.endm

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_INCLUDE_AM62X_CHIP_H */
