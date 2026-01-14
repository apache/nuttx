/****************************************************************************
 * arch/arm64/include/bcm2711/chip.h
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

#ifndef __ARCH_ARM64_INCLUDE_BCM2711_CHIP_H
#define __ARCH_ARM64_INCLUDE_BCM2711_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of bytes in x kilobytes/megabytes/gigabytes */

#define KB(x) ((x) << 10)
#define MB(x) (KB(x) << 10)
#define GB(x) (MB(UINT64_C(x)) << 10)

/* TODO: config option for low peripheral mode */

/* TODO: config option for GIC400 interrupt controller or legacy one */

/* GIC-400 base address.
 * The GIC-400 uses GICv2 architecture.
 */

#if defined(CONFIG_BCM2711_LOW_PERIPHERAL)

/* Low peripheral GIC address */

#define BCM_GIC400_BASEADDR 0xff840000

#else

/* Used for both 35-bit addressing and legacy mode. */

#define BCM_GIC400_BASEADDR 0x4c0040000

#endif // defined(CONFIG_BCM2711_LOW_PERIPHERAL)

#define BCM_GIC400_DISTOFFSET 0x00001000  /* Distributor */
#define BCM_GIC400_RDISTOFFSET 0x00002000 /* CPU Interfaces */
#define CONFIG_GICD_BASE (BCM_GIC400_BASEADDR + BCM_GIC400_DISTOFFSET)
#define CONFIG_GICR_BASE (BCM_GIC400_BASEADDR + BCM_GIC400_RDISTOFFSET)
#define CONFIG_GICR_OFFSET BCM_GIC400_RDISTOFFSET

/* BCM2711 memory map: RAM and Device I/O
 * TODO: verify and test against all variants (1, 2, 4 & 8GB)
 */

#define CONFIG_RAMBANK1_ADDR (0x000000000)

/* Both the 4GB and 8GB ram variants use all the size in RAMBANK1 */

#if defined(CONFIG_RPI4B_RAM_4GB) || defined(CONFIG_RPI4B_RAM_8GB)
#define CONFIG_RAMBANK1_SIZE GB(4) - MB(64)
#endif /* defined(CONFIG_RPI4B_RAM_4GB) || defined(CONFIG_RPI4B_RAM_8GB) */

/* The 8GB version begins to use a second RAM bank.
 * TODO: verify this works on 8GB
 */

#if defined(CONFIG_RPI4B_RAM_8GB)
#define CONFIG_RAMBANK2_ADDR (0x100000000)
#define CONFIG_RAMBANK2_SIZE GB(4)
#endif /* defined(CONFIG_RPI4B_RAM_8GB) */

/* TODO: for low peripheral mode this is valid, otherwise it might change */
#define CONFIG_DEVICEIO_BASEADDR (0x0fc000000)
#define CONFIG_DEVICEIO_SIZE MB(64)

/* Raspberry Pi 4B loads NuttX at this address */

#define CONFIG_LOAD_BASE 0x480000

#define MPID_TO_CLUSTER_ID(mpid) ((mpid) & ~0xff)

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__

.macro  get_cpu_id xreg0
  mrs    \xreg0, mpidr_el1
  ubfx   \xreg0, \xreg0, #0, #8
.endm

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_INCLUDE_BCM2711_CHIP_H */
