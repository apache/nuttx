/****************************************************************************
 * arch/mips/include/pic32mz/cp0.h
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

#ifndef __ARCH_MIPS_INCLUDE_PIC32MZ_CP0_H
#define __ARCH_MIPS_INCLUDE_PIC32MZ_CP0_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/mips32/cp0.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CP0 Register Addresses ***************************************************/

#ifdef __ASSEMBLY__

#  define PIC32MZ_CP0_INDEX      $0,0   /* Index into TLB array */
#  define PIC32MZ_CP0_RANDOM     $1,0   /* Randomly index into TLB array */
#  define PIC32MZ_CP0_ENTRYLO0   $2,0   /* Lower TLB entry, even pages */
#  define PIC32MZ_CP0_ENTRYLO1   $3,0   /* Lower TLB entry, odd pages */
#  define PIC32MZ_CP0_CONTEXT    $4,0   /* Address of page table entry in memory */
#  define PIC32MZ_CP0_USERLOCAL  $4,2   /* User read/write register */
#  define PIC32MZ_CP0_PAGEMASK   $5,0   /* Controls the variable page sizes */
#  define PIC32MZ_CP0_PAGEGRAIN  $5,1   /* Enables support of 1 KB pages */
#  define PIC32MZ_CP0_WIRED      $6,0   /* Number of fixed TLB entries */
#  define PIC32MZ_CP0_HWRENA     $7,0   /* Enables access via RDHWR hardware registers */
#  define PIC32MZ_CP0_BADVADDR   $8,0   /* Address of most recent exception */
#  define PIC32MZ_CP0_COUNT      $9,0   /* Processor cycle count */
#  define PIC32MZ_CP0_ENTRYHI    $10,0  /* Upper TLB entry */
#  define PIC32MZ_CP0_COMPARE    $11,0  /* Timer interrupt control */
#  define PIC32MZ_CP0_STATUS     $12,0  /* Processor status and control */
#  define PIC32MZ_CP0_INTCTL     $12,1  /* Interrupt system status and control */
#  define PIC32MZ_CP0_SRSCTL     $12,2  /* Shadow register set status and control */
#  define PIC32MZ_CP0_SRSMAP     $12,3  /* Maps from vectored interrupt to a shadow set */
#  define PIC32MZ_CP0_VIEWIPL    $12,4  /* Priority level access */
#  define PIC32MZ_CP0_SRSMAP2    $12,5  /* Vector number to shadow set mapping */
#  define PIC32MZ_CP0_CAUSE      $13,0  /* Cause of last general exception */
#  define PIC32MZ_CP0_VIEWRIPL   $13,4  /* RIPL access */
#  define PIC32MZ_CP0_NESTEDEXC  $13,5  /* Prior error and exception level */
#  define PIC32MZ_CP0_EPC        $14,0  /* Program counter at last exception */
#  define PIC32MZ_CP0_NESTEDEPC  $14,2  /* Prior EPC */
#  define PIC32MZ_CP0_PRID       $15,0  /* Processor identification and revision */
#  define PIC32MZ_CP0_EBASE      $15,1  /* Exception vector base register */
#  define PIC32MZ_CP0_CDMMBASE   $15,2  /* Common device memory map base */
#  define PIC32MZ_CP0_CONFIG     $16,0  /* Configuration register */
#  define PIC32MZ_CP0_CONFIG1    $16,1  /* Configuration register 1 */
#  define PIC32MZ_CP0_CONFIG2    $16,2  /* Configuration register 3 */
#  define PIC32MZ_CP0_CONFIG3    $16,3  /* Configuration register 3 */
#  define PIC32MZ_CP0_CONFIG4    $16,4  /* Configuration register 4 */
#  define PIC32MZ_CP0_CONFIG5    $16,5  /* Configuration register 5 */
#  define PIC32MZ_CP0_CONFIG7    $16,7  /* Configuration register 7 */
#  define PIC32MZ_CP0_LLADDR     $17,0  /* Load link address */
#  define PIC32MZ_CP0_WATCHLO    $18,0  /* Low-order watchpoint address */
#  define PIC32MZ_CP0_WATCHHI    $19,0  /* High-order watchpoint address */
#  define PIC32MZ_CP0_DEBUG      $23,0  /* Debug control and exception status */
#  define PIC32MZ_CP0_TRACECTRL  $23,1  /* EJTAG trace control */
#  define PIC32MZ_CP0_TRACECTRL2 $23,2  /* EJTAG trace control 2 */
#  define PIC32MZ_CP0_TRACEDATA1 $23,3  /* EJTAG user trace data 1 register */
#  define PIC32MZ_CP0_TRACEBPC   $23,4  /* EJTAG trace breakpoint register */
#  define PIC32MZ_CP0_DEBUG2     $23,5  /* Debug control/exception status 2 */
#  define PIC32MZ_CP0_DEPC       $24,0  /* Program counter at last debug exception */
#  define PIC32MZ_CP0_TRACEDATA2 $24,3  /* EJTAG user trace data 2 register */
#  define PIC32MZ_CP0_PERFCTL0   $25,0  /* Performance counter 0 control */
#  define PIC32MZ_CP0_PERFCNT0   $25,1  /* Performance counter 0 */
#  define PIC32MZ_CP0_PERFCTL1   $25,2  /* Performance counter 1 control */
#  define PIC32MZ_CP0_PERFCNT1   $25,3  /* Performance counter 1 */
#  define PIC32MZ_CP0_ERRCTL     $26,0  /* Test enable for cache RAM arrays */
#  define PIC32MZ_CP0_TAGLO      $28,0  /* Low-order portion of cache tag interface */
#  define PIC32MZ_CP0_DATALO     $28,1  /* Low-order portion of cache tag interface */
#  define PIC32MZ_CP0_ERREPC     $30,0  /* Program counter at last error */
#  define PIC32MZ_CP0_DESAVE     $31,0  /* Debug handler scratchpad register */
#endif

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
 *   NOTES:
 *   1. The following are reserved bits in the PIC32MZ:
 *      CP0_STATUS_UX   Bit 5: Enables 64-bit user address space (Not MIPS32)
 *      CP0_STATUS_SX   Bit 6: Enables 64-bit supervisor address space
 *                               (Not MIPS32)
 *      CP0_STATUS_KX   Bit 7: Enables 64-bit kernel address space
 *                               (Not MIPS32)
 *      CP0_STATUS_IMPL Bits 16-17: Implementation dependent
 *      CP0_STATUS_PX   Bit 23: Enables 64-bit operations (Not MIPS32)
 *      CP0_STATUS_FR   Bit 26: Controls the floating point register mode
 *                               (Not MIPS32)
 *      CP0_STATUS_MX   Bit 24: Enables MDMX (Not MIPS32)
 *      CP0_STATUS_CU1  Bit 29: Controls access to coprocessor 1
 *      CP0_STATUS_CU2  Bit 30: Controls access to coprocessor 2
 *      CP0_STATUS_CU3  Bit 31: Controls access to coprocessor 3
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

/* Register Number: 12 Sel: 2 Name: SRSCtl */

#define CP0_SRSCTL_CSS_SHIFT        (0)       /* Bits 0-3: Current shadow bit set */
#define CP0_SRSCTL_CSS_MASK         (15 << CP0_SRSCTL_CSS_SHIFT)
#define CP0_SRSCTL_PSS_SHIFT        (6)       /* Bits 6-9: Previous shadow set */
#define CP0_SRSCTL_PSS_MASK         (15 << CP0_SRSCTL_PSS_SHIFT)
#define CP0_SRSCTL_ESS_SHIFT        (12)      /* Bits 12-15: Exception shadow sets */
#define CP0_SRSCTL_ESS_MASK         (15 << CP0_SRSCTL_ESS_SHIFT)
#define CP0_SRSCTL_EICSS_SHIFT      (18)      /* Bits 18-21: External interrupt controller shadow sets */
#define CP0_SRSCTL_EICSS_MASK       (15 << CP0_SRSCTL_EICSS_SHIFT)
#define CP0_SRSCTL_HSS_SHIFT        (26)      /* Bits 26-29: High shadow sets */
#define CP0_SRSCTL_HSS_MASK         (15 << CP0_SRSCTL_HSS_SHIFT)
#  define CP0_SRSCTL_HSS_1SET       (0 << CP0_SRSCTL_HSS_SHIFT) /* One shadow set (normal GPR set) */
#  define CP0_SRSCTL_HSS_2SETS      (1 << CP0_SRSCTL_HSS_SHIFT) /* Two shadow sets */
#  define CP0_SRSCTL_HSS_4SETS      (3 << CP0_SRSCTL_HSS_SHIFT) /* Four shadow sets */
#  define CP0_SRSCTL_HSS_8SETS      (7 << CP0_SRSCTL_HSS_SHIFT) /* Eight shadow sets */

/* Register Number: 12 Sel: 3 Name: SRSMap */

#define CP0_SRSMAP_SSV0_SHIFT       (0)       /* Bits 0-3: Shadow set vector 0 */
#define CP0_SRSMAP_SSV0_MASK        (15 << CP0_SRSMAP_SSV0_SHIFT)
#define CP0_SRSMAP_SSV1_SHIFT       (4)       /* Bits 4-7: Shadow set vector 1 */
#define CP0_SRSMAP_SSV1_MASK        (15 << CP0_SRSMAP_SSV1_SHIFT)
#define CP0_SRSMAP_SSV2_SHIFT       (8)       /* Bits 8-11: Shadow set vector 2 */
#define CP0_SRSMAP_SSV2_MASK        (15 << CP0_SRSMAP_SSV2_SHIFT)
#define CP0_SRSMAP_SSV3_SHIFT       (12)      /* Bits 12-15: Shadow set vector 3 */
#define CP0_SRSMAP_SSV3_MASK        (15 << CP0_SRSMAP_SSV3_SHIFT)
#define CP0_SRSMAP_SSV4_SHIFT       (16)      /* Bits 16-19: Shadow set vector 4 */
#define CP0_SRSMAP_SSV4_MASK        (15 << CP0_SRSMAP_SSV4_SHIFT)
#define CP0_SRSMAP_SSV5_SHIFT       (20)      /* Bits 20-23: Shadow set vector 5 */
#define CP0_SRSMAP_SSV5_MASK        (15 << CP0_SRSMAP_SSV5_SHIFT)
#define CP0_SRSMAP_SSV6_SHIFT       (24)      /* Bits 24-27: Shadow set vector 6 */
#define CP0_SRSMAP_SSV6_MASK        (15 << CP0_SRSMAP_SSV6_SHIFT)
#define CP0_SRSMAP_SSV7_SHIFT       (28)      /* Bits 28-31: Shadow set vector 7 */
#define CP0_SRSMAP_SSV7_MASK        (15 << CP0_SRSMAP_SSV7_SHIFT)

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
 *   NOTES: The following bits are added in the PIC32:
 */

#define CP0_CAUSE_R                 (1 << 26) /* Bit 26: R bit */
#define CP0_CAUSE_DC                (1 << 27) /* Bit 27: Disable count */
#define CP0_CAUSE_TI                (1 << 30) /* Bit 30: Timer interrupt bit */

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
 *   NOTE: Slightly different bit interpretations of some fields:
 */

#define CP0_PRID_PATCH_SHIFT        (5)       /* Bits 0-1: Patch level */
#define CP0_PRID_PATCH_MASK         (3 << CP0_PRID_PATCH_SHIFT)
#define CP0_PRID_MINOR_SHIFT        (2)       /* Bits 2-4: Minor revision number */
#define CP0_PRID_MINOR_MASK         (7 << CP0_PRID_MINOR_SHIFT)
#define CP0_PRID_MAJOR_SHIFT        (5)       /* Bits 5-7: Major revision number */
#define CP0_PRID_MAJOR_MASK         (7 << CP0_PRID_MAJOR_SHIFT)

#undef CP0_PRID_OPTIONS_SHIFT
#undef CP0_PRID_OPTIONS_MASK

/* Register Number: 15 Sel: 1 Name: EBASE */

#define CP_EBASE_CPUNUM_SHIFT      (0)        /* Bits 0-9: CPU number */
#define CP_EBASE_CPUNUM_MASK       (0x3ff << CP_EBASE_CPUNUM_SHIFT)
#define CP_EBASE_SHIFT             (12)       /* Bits 30-31=10, Bits 12-29: Exception base */
#define CP_EBASE_MASK              (0x3ffff << CP_EBASE_SHIFT)

/* Register Number: 15 Sel: 2 Name: CDMMBase
 * To be provided
 */

/* Register Number: 16 Sel: 0 Name: Config
 * Function: Configuration register
 * Compliance Level: Required.
 *
 *   See arch/mips/include/mips32/cp0.h
 *   1. PIC32MZ is always little-endian.
 *   2. Implementation specific bits defined.
 */

#undef CP0_CONFIG_MT_NONE
#undef CP0_CONFIG_MT_TLB
#undef CP0_CONFIG_MT_BAT

#undef CP0_CONFIG_IMPL_SHIFT
#undef CP0_CONFIG_IMPL_MASK

#define CP0_CONFIG_DS               (1 << 16) /* Dual SRAM bit */
#define CP0_CONFIG_BM               (1 << 16) /* Burst Mode bit */
#define CP0_CONFIG_MM_SHIFT         (17)      /* Bits 17-18: Merge Mode bits */
#define CP0_CONFIG_MM_MASK          (3 << CP0_CONFIG_MM_SHIFT)
#  define CP0_CONFIG_MM_PROHIBITED  (0 << CP0_CONFIG_MM_SHIFT) /* Merging is not allowed */
#  define CP0_CONFIG_MM_ALLOWED     (2 << CP0_CONFIG_MM_SHIFT) /* Merging is allowed */

#define CP0_CONFIG_MDU              (1 << 20) /* Multipley/Divide unit bit */
#define CP0_CONFIG_SB               (1 << 21) /* Bit 32: Simple BE bus mode bit */
#define CP0_CONFIG_UDI              (1 << 22) /* Bit 22: User defined bit */
#define CPO_CONFIG_DSP              (1 << 23) /* Bit 24: Data Scratchpad RAM bit */
#define CPO_CONFIG_ISP              (1 << 24) /* Bit 24: Instruction Scratchpad RAM bit */
#define CP0_CONFIG_KU_SHIFT         (25)      /* Bits 25-27: KUSEG and USEG cacheability */
#define CP0_CONFIG_KU_MASK          (7 << CP0_CONFIG_KU_SHIFT)
#  define CP0_CONFIG_KU_UNCACHED    (2 << CP0_CONFIG_KU_SHIFT)
#  define CP0_CONFIG_KU_CACHEABLE   (3 << CP0_CONFIG_KU_SHIFT)
#define CP0_CONFIG_K23_SHIFT        (28)      /* Bits 28-30:  KSEG2 and KSEG3 cacheability */
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

#define CP0_DEBUG_DSS               (1 << 0) /* Bit 0: Debug single-step exception */
#define CP0_DEBUG_DBP               (1 << 1) /* Bit 1: Debug software breakpoint exception */
#define CP0_DEBUG_DDBL              (1 << 2) /* Bit 2: Debug data break exception on load */
#define CP0_DEBUG_DDBS              (1 << 3) /* Bit 3: Debug data break exception on store */
#define CP0_DEBUG_DIB               (1 << 4) /* Bit 4: Debug instruction break exception */
#define CP0_DEBUG_DINT              (1 << 5) /* Bit 5: Debug interrupt exception */
#define CP0_DEBUG_SST               (1 << 8) /* Bit 6: Enable debug single step exception */
#define CP0_DEBUG_NOSST             (1 << 9) /* Bit 7: No single step feature available */
#define CP0_DEBUG_DEXCCODE_SHIFT    (10)     /* Bits 10-14: Cause of latest exception in DEBUG mode */
#define CP0_DEBUG_DEXCCODE_MASK     (31 << CP0_DEBUG_DEXCCODE_SHIFT)
#define CP0_DEBUG_VER_SHIFT         (15)     /* Bits 15-17: EJTAG version */
#define CP0_DEBUG_VER_MASK          (7 << CP0_DEBUG_VER_SHIFT)
#define CP0_DEBUG_DDBLIMPR          (1 << 18) /* Bit 18: Imprecise debug data break load instruction */
#define CP0_DEBUG_DDBSIMPR          (1 << 19) /* Bit 19: Imprecise debug data break store instruction */
#define CP0_DEBUG_IEXI              (1 << 20) /* Bit 20: Imprecise error exception inhibit */
#define CP0_DEBUG_DBUSEP            (1 << 21) /* Bit 21: Data access bus error exception pending */
#define CP0_DEBUG_CACHEEP           (1 << 22) /* Bit 22: Imprecise cache error exception is pending */
#define CP0_DEBUG_MCHECKP           (1 << 23) /* Bit 23: Imprecise machine check exception is pending */
#define CP0_DEBUG_IBUSEP            (1 << 24) /* Bit 24: Bus error exception pending */
#define CP0_DEBUG_COUNTDM           (1 << 25) /* Bit 25: Count register behavior (1=running) */
#define CP0_DEBUG_HALT              (1 << 26) /* Bit 26: Internal system bus clock stopped */
#define CP0_DEBUG_DOZE              (1 << 27) /* Bit 27: Processor in low power mode */
#define CP0_DEBUG_LSNM              (1 << 28) /* Bit 28: Load/store in DSEG goes to main memory */
#define CP0_DEBUG_NODCR             (1 << 29) /* Bit 29: No DSEG preset */
#define CP0_DEBUG_DM                (1 << 30) /* Bit 30: Processor is operating in DEBUG mode */
#define CP0_DEBUG_DBD               (1 << 31) /* Bit 31: Last debug exception occurred in a dely slot */

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
#endif /* __ARCH_MIPS_INCLUDE_PIC32MZ_CP0_H */
