/****************************************************************************
 * arch/arm64/include/qemu/chip.h
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

#ifndef __ARCH_ARM64_INCLUDE_QEMU_CHIP_H
#define __ARCH_ARM64_INCLUDE_QEMU_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of bytes in x kibibytes/mebibytes/gibibytes */

#define KB(x)           ((x) << 10)
#define MB(x)           (KB(x) << 10)
#define GB(x)           (MB(UINT64_C(x)) << 10)

#if defined(CONFIG_ARCH_CHIP_QEMU)

#if CONFIG_ARM64_GIC_VERSION == 2

#define CONFIG_GICD_BASE           0x8000000
#define CONFIG_GICR_BASE           0x8010000
#define CONFIG_GICM_BASE           0x8020000

#elif CONFIG_ARM64_GIC_VERSION == 3 || CONFIG_ARM64_GIC_VERSION == 4

#define CONFIG_GICD_BASE           0x8000000
#define CONFIG_GICR_BASE           0x80a0000
#define CONFIG_GICR_OFFSET         0x20000
#else

#error CONFIG_ARM64_GIC_VERSION should be 2, 3 or 4

#endif /* CONFIG_ARM64_GIC_VERSION */

#define CONFIG_QEMU_DRAM_BASE      0x40000000

#define CONFIG_SECURE_MEM_BASEADDR 0x0e000000
#define CONFIG_SECURE_MEM_SIZE     MB(16)

#ifdef CONFIG_ARCH_TRUSTZONE_SECURE
#  define CONFIG_RAMBANK1_ADDR     CONFIG_SECURE_MEM_BASEADDR
#  define CONFIG_RAMBANK1_SIZE     CONFIG_SECURE_MEM_SIZE
#else
/* QEMU virt machine physical RAM always starts at 0x40000000.
 * RAMBANK1 must cover from physical RAM base through the end of
 * NuttX's configured RAM region (CONFIG_RAM_START + CONFIG_RAM_SIZE)
 * so that the MMU mapping is large enough.
 *
 * When CONFIG_RAM_START > 0x40000000 (e.g. running under a hypervisor),
 * the RAMBANK1 mapping still starts at 0x40000000 to keep the DTB
 * accessible, and extends to cover CONFIG_RAM_END.
 *
 * The user must ensure QEMU is launched with enough RAM via -m option:
 *   -m <size>  where size >=
 *   (CONFIG_RAM_START - 0x40000000 + CONFIG_RAM_SIZE)
 */

#  define CONFIG_RAMBANK1_ADDR     CONFIG_QEMU_DRAM_BASE
#  define CONFIG_RAMBANK1_SIZE     (CONFIG_RAM_START - CONFIG_RAMBANK1_ADDR + \
                                    CONFIG_RAM_SIZE)
#endif

#define CONFIG_DEVICEIO_BASEADDR   0x7000000
#define CONFIG_DEVICEIO_SIZE       MB(512)

#define CONFIG_PCI_CFG_BASEADDR    0x4010000000
#define CONFIG_PCI_CFG_SIZE        MB(256)

#define CONFIG_PCI_MEM_BASEADDR    0x8000000000
#define CONFIG_PCI_MEM_SIZE        GB(512)

#define CONFIG_PCI_IO_BASEADDR     0x3eff0000
#define CONFIG_PCI_IO_SIZE         KB(64)

#define CONFIG_SHMEM_BASEADDR      0x45000000
#define CONFIG_SHMEM_SIZE          MB(1)

#ifdef CONFIG_ARCH_TRUSTZONE_SECURE
/* When booting via TF-A (Trusted Firmware-A), BL2 and BL31 occupy the
 * secure SRAM region starting at 0x0e000000:
 *
 *   0x0e000000 - 0x0e001000 : Shared RAM (mailbox)
 *   0x0e05b000 - 0x0e090000 : BL2  (212KB)
 *   0x0e090000 - 0x0e100000 : BL31 (448KB)
 *
 * BL31 remains resident at runtime, so NuttX must be placed above
 * 0x0e100000 (secure DRAM) to avoid overwriting BL31 code and data.
 */

#  define CONFIG_LOAD_BASE         (CONFIG_SECURE_MEM_BASEADDR + 0x100000)
#else
/* Load base is 0x280000 (2.5MB) offset from CONFIG_RAM_START */

#  define CONFIG_LOAD_BASE         (CONFIG_RAM_START + 0x280000)
#endif

#define MPID_TO_CLUSTER_ID(mpid)   ((mpid) & ~0xff)

#endif

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__

.macro  get_cpu_id xreg0
  mrs    \xreg0, mpidr_el1
  ubfx   \xreg0, \xreg0, #0, #8
.endm

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_INCLUDE_QEMU_CHIP_H */
