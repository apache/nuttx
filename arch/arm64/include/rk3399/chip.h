/****************************************************************************
 * arch/arm64/include/rk3399/chip.h
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

#ifndef __ARCH_ARM64_INCLUDE_RK3399_CHIP_H
#define __ARCH_ARM64_INCLUDE_RK3399_CHIP_H

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

/* Rockchip A64 Generic Interrupt Controller v2: Distributor and Redist */

#define CONFIG_GICD_BASE          0xfee00000
#define CONFIG_GICR_BASE          0xfef00000 
#define CONFIG_GICR_OFFSET        0x20000

/* Rockchip RK3399 Memory Map: RAM and Device I/O */

#define CONFIG_RAMBANK1_ADDR      0x02000000
#define CONFIG_RAMBANK1_SIZE      MB(512)
#define CONFIG_DEVICEIO_BASEADDR  0xF8000000
#define CONFIG_DEVICEIO_SIZE      MB(128)

/* U-Boot loads NuttX at this address (kernel_addr_r) */

#define CONFIG_LOAD_BASE          0x02080000

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

#endif /* __ARCH_ARM64_INCLUDE_RK3399_CHIP_H */
