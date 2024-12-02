/****************************************************************************
 * arch/arm64/include/imx9/chip.h
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

#ifndef __ARCH_ARM64_INCLUDE_IMX9_CHIP_H
#define __ARCH_ARM64_INCLUDE_IMX9_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Cache line sizes (in bytes)for the i.MX9 (Cortex-A55) */

#define ARMV8A_DCACHE_LINESIZE 64  /* 64 bytes (16 words) */
#define ARMV8A_ICACHE_LINESIZE 64  /* 64 bytes (16 words) */

/* Number of bytes in x kibibytes/mebibytes/gibibytes */

#define KB(x)   ((x) << 10)
#define MB(x)   (KB(x) << 10)
#define GB(x)   (MB(UINT64_C(x)) << 10)

#if defined(CONFIG_ARCH_CHIP_IMX93)

#if CONFIG_ARM64_GIC_VERSION == 3 || CONFIG_ARM64_GIC_VERSION == 4

#define CONFIG_GICD_BASE          0x48000000
#define CONFIG_GICR_BASE          0x48040000
#define CONFIG_GICR_OFFSET        0x20000

#else

#error CONFIG_ARM64_GIC_VERSION should be 3 or 4

#endif /* CONFIG_ARM64_GIC_VERSION */

#define CONFIG_RAMBANK1_ADDR      0x80000000
#define CONFIG_RAMBANK1_SIZE      MB(128)

#define CONFIG_DEVICEIO_BASEADDR  0x40000000
#define CONFIG_DEVICEIO_SIZE      MB(512)

#define CONFIG_OCRAM_BASE_ADDR    0x20480000
#define CONFIG_OCRAM_SIZE         KB(640)

#define CONFIG_FSPI_PER_BASEADDR  0x28000000
#define CONFIG_FSPI_PER_SIZE      MB(128)

#define MPID_TO_CLUSTER_ID(mpid)  ((mpid) & ~0xff)

#define IMX9_GPIO_NPORTS          4

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

#endif /* __ARCH_ARM64_INCLUDE_IMX9_CHIP_H */
