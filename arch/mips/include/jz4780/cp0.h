/****************************************************************************
 * arch/mips/include/jz4780/cp0.h
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

#ifndef __ARCH_MIPS_INCLUDE_JZ4780_CP0_H
#define __ARCH_MIPS_INCLUDE_JZ4780_CP0_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/mips32/cp0.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CP0 Register Addresses ***************************************************/

/* CP0 Registers ************************************************************/

/* Register Number: 0 Sel: 0 Name: Index
 * Compliance Level: Required for TLB-based MMU; Optional otherwise.
 * To be provided
 */

/* Register Number: 1 Sel: 0 Name: Random
 * Compliance Level: Required for TLB-based MMU; Optional otherwise.
 * To be provided
 */

/* Register Number: 2 Sel: 0 Name: EntryLo0
 * Compliance Level: Required for TLB-based MMU; Optional otherwise.
 * To be provided
 */

/* Register Number: 3 Sel: 0 Name: EntryLo1
 * Compliance Level: Required for TLB-based MMU; Optional otherwise.
 * To be provided
 */

/* Register Number: 4 Sel: 0 Name: Context
 * Compliance Level: Required for TLB-based MMU; Optional otherwise.
 * To be provided
 */

/* Register Number: 4 Sel: 2 Name: UserLocal
 * Function: User read/write register
 * Compliance Level: Required for TLB-based MMU; Optional otherwise.
 * To be provided
 */

/* Register Number: 5 Sel: 0 Name: PageMask
 * Function: Used for reading from and writing to the TLB
 * Compliance Level: Required for TLB-based MMU; Optional otherwise.
 * To be provided
 */

/* Register Number: 5 Sel: 1 Name: PageGrain
 * Function: Enable or disable the read and execute inhibit bits in the
 *           EntryLo0 and EntryLo1 registers.
 * Compliance Level: Required for TLB-based MMU; Optional otherwise.
 * To be provided
 */

/* Register Number: 6 Sel: 0 Name: Wired
 * Compliance Level: Required for TLB-based MMU; Optional otherwise.
 * To be provided
 */

/* Register Number: 7 Sel: 0 Name: HWREna
 * Function: Enables access via the RDHWR instruction to selected hardware
 *   registers in non-privileged mode.
 * Compliance Level: (Reserved for future extensions)
 */

#define CP0_HWRENA_SHIFT            (0)       /* Bits 0-3: Enable access to a hardware resource */
#define CP0_HWRENA_MASK             (15 << CP0_HWRENA_SHIFT)
#  define CP0_HWRENA_BIT0           (1 << CP0_HWRENA_SHIFT)
#  define CP0_HWRENA_BIT1           (2 << CP0_HWRENA_SHIFT)
#  define CP0_HWRENA_BIT2           (4 << CP0_HWRENA_SHIFT)
#  define CP0_HWRENA_BIT3           (8 << CP0_HWRENA_SHIFT)
#define CP0_HWRENA_ULR              (1 << 29) /* Bit 29: User Local Register bit */

/* Register Number: 8 Sel: 0 Name: BadVAddr
 * Function: Reports the address for the most recent address-related
 *   exception
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 *
 * Register Number: 9 Sel: 0 Name: Count
 * Function: Processor cycle count
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 */

/* Register Number: 10 Sel: 0 Name: EntryHi
 * Compliance Level: Required for TLB-based MMU; Optional otherwise.
 */

/* Register Number: 11 Sel: 0 Name: Compare
 * Function: Timer interrupt control
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 */

/* Register Number: 12 Sel: 0 Name: Status
 * Function: Processor status and control
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 */

#undef CP0_STATUS_UX
#undef CP0_STATUS_SX
#undef CP0_STATUS_KX
#undef CP0_STATUS_IMPL
#undef CP0_STATUS_IMPL_SHIFT
#undef CP0_STATUS_IMPL_MASK
#undef CP0_STATUS_PX
#undef CP0_STATUS_FR
#undef CP0_STATUS_CU1
#undef CP0_STATUS_CU2
#undef CP0_STATUS_CU3

/*   2. The following field is of a different width.  Apparently, it
 *      excludes the software interrupt bits.
 *
 *      CP0_STATUS_IM   Bits 8-15: Interrupt Mask
 *      Vs.
 *      CP0_STATUS_IPL  Bits 10-16+18: Interrupt priority level
 *                      Bits 8-9 reserved
 */

#define CP0_STATUS_IPL_SHIFT        (10)   /*  Bits 10-16+18: Interrupt priority level */
#define CP0_STATUS_IPL_MASK         (0x17f << CP0_STATUS_IPL_SHIFT)

/*   3. Supervisor mode not supported
 *       CP0_STATUS_KSU Bits 3-4: Operating mode (with supervisor mode)
 */

#undef CP0_STATUS_KSU_SUPER

/* Register Number: 12 Sel: 1 Name: IntCtl */

#define CP0_INTCTL_VS_SHIFT         (5)       /* Bits 5-9: Vector spacing bits */
#define CP0_INTCTL_VS_MASK          (0x1f << CP0_INTCTL_VS_SHIFT)
#  define CP0_INTCTL_VS_0BYTES      (0x00 << CP0_INTCTL_VS_SHIFT)
#  define CP0_INTCTL_VS_32BYTES     (0x01 << CP0_INTCTL_VS_SHIFT)
#  define CP0_INTCTL_VS_64BYTES     (0x02 << CP0_INTCTL_VS_SHIFT)
#  define CP0_INTCTL_VS_128BYTES    (0x04 << CP0_INTCTL_VS_SHIFT)
#  define CP0_INTCTL_VS_256BYTES    (0x08 << CP0_INTCTL_VS_SHIFT)
#  define CP0_INTCTL_VS_512BYTES    (0x10 << CP0_INTCTL_VS_SHIFT)

/* Register Number: 12 Sel: 4 Name: View_Ipl
 * To be provided
 */

/* Register Number: 12 Sel: 5 Name: SRSMap2
 * To be provided
 */

/* Register Number: 13 Sel: 0 Name: Cause
 * Function: Cause of last general exception
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 */

/* Register Number: 13 Sel: 4 Name: View_RIPL
 * To be provided
 */

/* Register Number: 13 Sel: 5 Name: NestedExc
 * To be provided
 */

/* Register Number: 14 Sel: 0 Name: EPC
 * Function: Program counter at last exception
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 */

/* Register Number: 14 Sel: 2 Name: NestedEPC
 * To be provided
 */

/* Register Number: 15 Sel: 0 Name: PRId
 * Function: Processor identification and revision
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 */

#undef CP0_PRID_OPTIONS_SHIFT
#undef CP0_PRID_OPTIONS_MASK

/* Register Number: 15 Sel: 1 Name: EBASE */

/* Register Number: 15 Sel: 2 Name: CDMMBase
 * To be provided
 */

/* Register Number: 16 Sel: 0 Name: Config
 * Function: Configuration register
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 *   1. JZ4780 is always little-endian.
 *   2. Implementation specific bits defined.
 */

#undef CP0_CONFIG_MT_NONE
#undef CP0_CONFIG_MT_TLB
#undef CP0_CONFIG_MT_BAT

#undef CP0_CONFIG_IMPL_SHIFT
#undef CP0_CONFIG_IMPL_MASK

#define CP0_CONFIG_K0_SHIFT         (0)      /* Bits 0-2: KSEG0 cache */
#define CP0_CONFIG_K0_MASK          (7 << CP0_CONFIG_K0_SHIFT)
#define CP0_CONFIG_K0_WT            (0 << CP0_CONFIG_K0_SHIFT)
#define CP0_CONFIG_K0_UNCACHED      (2 << CP0_CONFIG_K0_SHIFT)
#define CP0_CONFIG_K0_WB            (3 << CP0_CONFIG_K0_SHIFT)

#define CP0_CONFIG_DS               (1 << 16) /* Dual SRAM bit */
#define CP0_CONFIG_BM               (1 << 16) /* Burst Mode bit */
#define CP0_CONFIG_MM_SHIFT         (17)      /* Bits 17-18: Merge Mode bits */
#define CP0_CONFIG_MM_MASK          (3 << CP0_CONFIG_MM_SHIFT)
#  define CP0_CONFIG_MM_PROHIBITED  (0 << CP0_CONFIG_MM_SHIFT)
#  define CP0_CONFIG_MM_ALLOWED     (2 << CP0_CONFIG_MM_SHIFT)
#define CP0_CONFIG_MDU              (1 << 20)
#define CP0_CONFIG_SB               (1 << 21)
#define CP0_CONFIG_UDI              (1 << 22)
#define CPO_CONFIG_DSP              (1 << 23)
#define CPO_CONFIG_ISP              (1 << 24)
#define CP0_CONFIG_KU_SHIFT         (25)
#define CP0_CONFIG_KU_MASK          (7 << CP0_CONFIG_KU_SHIFT)
#  define CP0_CONFIG_KU_UNCACHED    (2 << CP0_CONFIG_KU_SHIFT)
#  define CP0_CONFIG_KU_CACHEABLE   (3 << CP0_CONFIG_KU_SHIFT)
#define CP0_CONFIG_K23_SHIFT        (28)
#define CP0_CONFIG_K23_MASK         (7 << CP0_CONFIG_K23_SHIFT)
#  define CP0_CONFIG_K23_UNCACHED   (2 << CP0_CONFIG_K23_SHIFT)
#  define CP0_CONFIG_K23_CACHEABLE  (3 << CP0_CONFIG_K23_SHIFT)

/* Register Number: 16 Sel: 1 Name: Config1
 * Function: Configuration register 1
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 *
 * Register Number: 16 Sel: 2 Name: Config2
 * Function: Configuration register 2
 * Compliance Level: Optional.
 *
 *   See arch/mips/include/mips32/cp0.h
 */

#undef CP0_CONFIG2_TBS_SHIFT
#undef CP0_CONFIG2_TBS_MASK

/* Register Number: 16 Sel: 3 Name: Config3
 * Function: Configuration register 3
 * Compliance Level: Optional.
 *
 *   See arch/mips/include/mips32/cp0.h
 */

/* Register Number: 16 Sel: 4 Name: Config4
 * To be provided
 */

/* Register Number: 16 Sel: 5 Name: Config5
 * To be provided
 */

/* Register Number: 16 Sel: 7 Name: Config7
 * To be provided
 */

/* Register Number: 17 Sel: 0 Name: LLAddr
 * Compliance Level: Optional.
 * To be provided
 */

/* Register Number: 18 Sel: 0 Name: WatchLo
 * Compliance Level: Optional.
 * To be provided
 */

/* Register Number: 19 Sel: 0 Name: WatchHi
 * Compliance Level: Optional.
 * To be provided
 */

/* Register Number: 20-22 Reserved
 * Compliance Level: Optional.
 */

/* Register Number: 23 Sel: 0 Name: Debug
 * Function: EJTAG Debug register
 * Compliance Level: Optional.
 */

/* Register Number: 23 Sel: 1 Name: TraceControl
 * Function: EJTAG Debug register
 * Compliance Level: Optional.
 * To be provided
 */

/* Register Number: 23 Sel: 2 Name: TraceControl2
 * Function: EJTAG Debug register
 * Compliance Level: Optional.
 * To be provided
 */

/* Register Number: 23 Sel: 3 Name: UserTraceData1
 * Function: EJTAG Debug register
 * Compliance Level: Optional.
 * To be provided
 */

/* Register Number: 23 Sel: 4 Name: TraceBPC
 * Function: EJTAG Debug register
 * Compliance Level: Optional.
 * To be provided
 */

/* Register Number: 23 Sel: 5 Name: Debug2
 * Function: EJTAG Debug register
 * Compliance Level: Optional.
 * To be provided
 */

/* Register Number: 24 Sel: 0 Name: DEPC
 * Function: Program counter at last EJTAG debug exception
 * Compliance Level: Optional.
 *
 *   See arch/mips/include/mips32/cp0.h
 */

/* Register Number: 24 Sel: 3 Name: UserTraceData2
 * Function:  EJTAG user trace data 2 register
 * Compliance Level: Optional.
 * To be provided
 */

/* Register Number: 25 Sel: 0 Name: PerfCtl0
 * Function:  Performance counter 0 control
 * Compliance Level: Optional.
 * To be provided
 */

/* Register Number: 25 Sel: 1 Name: PerfCnt0
 * Function:  Performance counter 0
 * Compliance Level: Optional.
 * To be provided
 */

/* Register Number: 25 Sel: 2 Name: PerfCtl1
 * Function:  Performance counter 1 control
 * Compliance Level: Optional.
 * To be provided
 */

/* Register Number: 25 Sel: 3 Name: PerfCnt1
 * Function:  Performance counter 1
 * Compliance Level: Optional.
 * To be provided
 */

/* Register Number: 26 Sel: 0 Name: ErrCtl
 * Function:  Software test enable of way-select and data RAM arrays for
 *            I-Cache and D-Cache
 * Compliance Level: Optional.
 * To be provided
 */

/* Register Number: 27 Reserved
 * Compliance Level: Recommended/Optional.
 */

/* Register Number: 28 Sel: 0 Name: TagLo
 * Function:  Low-order portion of cache tag interface
 * Compliance Level: Optional.
 * To be provided
 */

/* Register Number: 28 Sel: 1 Name: DataLo
 * Function:  Low-order portion of cache tag interface
 * Compliance Level: Optional.
 * To be provided
 */

/* Register Number: 29 Reserved
 * Compliance Level: Recommended/Optional.
 */

/* Register Number: 30 Sel: 0 Name: ErrorEPC
 * Function: Program counter at last error
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 *
 * Register Number: 31 Sel: 0 Name: DeSAVE
 * Function: EJTAG debug exception save register
 * Compliance Level: Optional.
 *
 *   See arch/mips/include/mips32/cp0.h
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_INCLUDE_JZ4780_CP0_H */
