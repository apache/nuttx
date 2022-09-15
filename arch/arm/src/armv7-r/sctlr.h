/****************************************************************************
 * arch/arm/src/armv7-r/sctlr.h
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

/* References:
 *
 *  "ARM Architecture Reference Manual, ARMv7-A and ARMv7-R edition",
 *  Copyright 1996-1998, 2000, 2004-2012 ARM.
 *  All rights reserved. ARM DDI 0406C.c (ID051414)
 */

#ifndef __ARCH_ARM_SRC_ARMV7_R_SCTLR_H
#define __ARCH_ARM_SRC_ARMV7_R_SCTLR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "barriers.h"
#include "cp15.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CP15 c0 Registers ********************************************************/

/* Main ID Register (MIDR): CRn=c0, opc1=0, CRm=c0, opc2=0
 * TODO: To be provided
 */

/* Cache Type Register (CTR): CRn=c0, opc1=0, CRm=c0, opc2=1
 * TODO: To be provided
 */

/* TCM Type Register (TCMTR): CRn=c0, opc1=0, CRm=c0, opc2=2
 * Details implementation defined.
 */

/* Aliases of Main ID register (MIDR): CRn=c0, opc1=0, CRm=c0, opc2=3,7
 * TODO: To be provided
 */

/* MPU Type Register (MPUIR): CRn=c0, opc1=0, CRm=c0, opc2=4
 * TODO: To be provided
 */

/* Multiprocessor Affinity Register (MPIDR): CRn=c0, opc1=0, CRm=c0, opc2=5
 * TODO: To be provided
 */

/* Revision ID Register (REVIDR): CRn=c0, opc1=0, CRm=c0, opc2=6
 * TODO: To be provided
 */

/* Processor Feature Register 0 (ID_PFR0): CRn=c0, opc1=0, CRm=c1, opc2=0
 * Processor Feature Register 1 (ID_PFR1): CRn=c0, opc1=0, CRm=c1, opc2=1
 * TODO: To be provided
 */

/* Debug Feature Register 0 (ID_DFR0): CRn=c0, opc1=0, CRm=c1, opc2=2
 * TODO: To be provided
 */

/* Auxiliary Feature Register 0 (ID_AFR0): CRn=c0, opc1=0, CRm=c1, opc2=3
 * TODO: To be provided
 */

/* Memory Model Features Register 0 (ID_MMFR0):
 * CRn=c0, opc1=0, CRm=c1, opc2=4
 * Memory Model Features Register 1 (ID_MMFR1):
 * CRn=c0, opc1=0, CRm=c1, opc2=5
 * Memory Model Features Register 2 (ID_MMFR2):
 * CRn=c0, opc1=0, CRm=c1, opc2=6
 * Memory Model Features Register 3 (ID_MMFR3):
 * CRn=c0, opc1=0, CRm=c1, opc2=7
 * TODO: To be provided
 */

/* Instruction Set Attributes Register 0 (ID_ISAR0):
 * CRn=c0, opc1=0, CRm=c2, opc2=0
 * Instruction Set Attributes Register 1 (ID_ISAR1):
 * CRn=c0, opc1=0, CRm=c2, opc2=1
 * Instruction Set Attributes Register 2 (ID_ISAR2):
 * CRn=c0, opc1=0, CRm=c2, opc2=2
 * Instruction Set Attributes Register 3 (ID_ISAR3):
 * CRn=c0, opc1=0, CRm=c2, opc2=3
 * Instruction Set Attributes Register 4 (ID_ISAR4):
 * CRn=c0, opc1=0, CRm=c2, opc2=4
 * Instruction Set Attributes Register 5 (ID_ISAR5):
 * CRn=c0, opc1=0, CRm=c2, opc2=5
 * Instruction Set Attributes Register 6-7 (ID_ISAR6-7).
 *  Reserved.
 * TODO: Others to be provided
 */

/* Reserved: CRn=c0, opc1=0, CRm=c3-c7, opc2=* */

/* Cache Size Identification Register (CCSIDR):
 * CRn=c0, opc1=1, CRm=c0, opc2=0
 * TODO: To be provided
 */

/* Cache Level ID Register (CLIDR): CRn=c0, opc1=1, CRm=c0, opc2=1
 * TODO: To be provided
 */

/* Auxiliary ID Register (AIDR): CRn=c0, opc1=1, CRm=c0, opc2=7
 * TODO: To be provided
 */

/* Cache Size Selection Register (CSSELR): CRn=c0, opc1=2, CRm=c0, opc2=0
 * TODO: To be provided
 */

/* CP15 c1 Registers ********************************************************/

/* System Control Register (SCTLR): CRn=c1, opc1=0, CRm=c0, opc2=0
 */

#define SCTLR_M            (1 << 0)  /* Bit 0:  MPU enable bit */
#define SCTLR_A            (1 << 1)  /* Bit 1:  Enables strict alignment of data */
#define SCTLR_C            (1 << 2)  /* Bit 2:  Determines if data can be cached */
                                     /* Bits 3-4: Reserved */
#define SCTLR_CCP15BEN     (1 << 5)  /* Bit 5:  CP15 barrier enable */
                                     /* Bit 6:  Reserved */
#define SCTLR_B            (1 << 7)  /* Bit 7:  Should be zero on ARMv7-R */
                                     /* Bits 8-9: Reserved */
#define SCTLR_SW           (1 << 10) /* Bit 10: SWP/SWPB Enable bit */
#define SCTLR_Z            (1 << 11) /* Bit 11: Program flow prediction control */
#define SCTLR_I            (1 << 12) /* Bit 12: Determines if instructions can be cached */
#define SCTLR_V            (1 << 13) /* Bit 13: Vectors bit */
#define SCTLR_RR           (1 << 14) /* Bit 14: Cache replacement strategy */
                                     /* Bits 15-16: Reserved */
#define SCTLR_BR           (1 << 17) /* Bit 17: Background Region bit */
                                     /* Bit 18: Reserved */
#define SCTLR_DZ           (1 << 19) /* Bit 19: Divide by Zero fault enable bit */
                                     /* Bit 20: Reserved */
#define SCTLR_FI           (1 << 21) /* Bit 21: Fast interrupts configuration enable bit */
#define SCTLR_U            (1 << 22) /* Bit 22: Unaligned access model (always one) */
#define SCTLR_VE           (1 << 24) /* Bit 24: Interrupt Vectors Enable bit */
#define SCTLR_EE           (1 << 25) /* Bit 25: Determines the value the CPSR.E */
#define SCTLR_NMFI         (1 << 27) /* Bit 27: Non-maskable FIQ (NMFI) support */
                                     /* Bits 28-29: Reserved */
#define SCTLR_TE           (1 << 30) /* Bit 30: Thumb exception enable */
#define SCTLR_IE           (1 << 31) /* Bit 31: Instruction endian-ness */

/* Auxiliary Control Register (ACTLR): CRn=c1, opc1=0, CRm=c0, opc2=1
 * Implementation defined
 */

/* Coprocessor Access Control Register (CPACR):
 * CRn=c1, opc1=0, CRm=c0, opc2=2
 * TODO: To be provided
 */

/* CP15 c2-c4 Registers *****************************************************/

/* Not used on ARMv7-R */

/* CP15 c5 Registers ********************************************************/

/* Data Fault Status Register (DFSR): CRn=c5, opc1=0, CRm=c0, opc2=0
 * TODO: To be provided
 */

/* Instruction Fault Status Register (IFSR): CRn=c5, opc1=0, CRm=c0, opc2=1
 * TODO: To be provided
 */

/* Auxiliary DFSR (ADFSR): CRn=c5, opc1=0, CRm=c1, opc2=0
 * TODO: To be provided
 */

/* Auxiliary IFSR (AIFSR): CRn=c5, opc1=0, CRm=c1, opc2=1
 * TODO: To be provided
 */

/* CP15 c6 Registers ********************************************************/

/* Data Fault Address Register(DFAR): CRn=c6, opc1=0, CRm=c0, opc2=0
 *
 *   Holds the MVA of the faulting address when a synchronous fault occurs
 */

/* Instruction Fault Address Register(IFAR): CRn=c6, opc1=0, CRm=c0, opc2=1
 *
 *   Holds the MVA of the faulting address of the instruction that caused a
 *   prefetch abort.
 */

/* Data Region Base Address Register (DRBAR): CRn=c6, opc1=0, CRm=c1, opc2=0
 * TODO: To be provided
 */

/* Instruction Region Base Address Register (IRBAR):
 * CRn=c6, opc1=0, CRm=c1, opc2=1
 * TODO: To be provided
 */

/* Data Region Size and Enable Register (DRSR):
 * CRn=c6, opc1=0, CRm=c1, opc2=2
 * TODO: To be provided
 */

/* Instruction Region Size and Enable Register (IRSR):
 * CRn=c6, opc1=0, CRm=c1, opc2=3
 * TODO: To be provided
 */

/* Data Region Access Control Register (DRACR):
 * CRn=c6, opc1=0, CRm=c1, opc2=4
 * TODO: To be provided
 */

/* Instruction Region Access Control Register (IRACR):
 * CRn=c6, opc1=0, CRm=c1, opc2=5
 * TODO: To be provided
 */

/* MPU Region Number Register (RGNR):
 * CRn=c6, opc1=0, CRm=c2, opc2=0
 * TODO: To be provided
 */

/* CP15 c7 Registers ********************************************************/

/* See cp15_cacheops.h */

/* CP15 c8 Registers ********************************************************/

/* Not used on ARMv7-R */

/* CP15 c9 Registers ********************************************************/

/* 32-bit Performance Monitors Control Register (PMCR):
 * CRn=c9, opc1=0, CRm=c12, opc2=0
 */

#define PMCR_E             (1 << 0)  /* Enable all counters */
#define PMCR_P             (1 << 1)  /* Reset all counter events (except PMCCNTR) */
#define PMCR_C             (1 << 2)  /* Reset cycle counter (PMCCNTR) to zero */
#define PMCR_D             (1 << 3)  /* Enable cycle counter clock (PMCCNTR) divider */
#define PMCR_X             (1 << 4)  /* Export of events is enabled */
#define PMCR_DP            (1 << 5)  /* Disable PMCCNTR if event counting is prohibited */
#define PMCR_N_SHIFT       (11)      /* Bits 11-15:  Number of event counters */
#define PMCR_N_MASK        (0x1f << PMCR_N_SHIFT)
#define PMCR_IDCODE_SHIFT  (16)      /* Bits 16-23: Identification code */
#define PMCR_IDCODE_MASK   (0xff << PMCR_IDCODE_SHIFT)
#define PMCR_IMP_SHIFT     (24)      /* Bits 24-31: Implementer code */
#define PMCR_IMP_MASK      (0xff << PMCR_IMP_SHIFT)

/* 32-bit Performance Monitors Count Enable Set register (PMCNTENSET):
 * CRn=c9, opc1=0, CRm=c12, opc2=1
 */

#define PMCESR_CCES        (1 << 31) /* Bits 31: Count Enable Set Register */

/* 32-bit Performance Monitors Count Enable Clear register (PMCNTENCLR):
 * CRn=c9, opc1=0, CRm=c12, opc2=2
 */

#define PMCECR_CCEC        (1 << 31) /* Bits 31: Count Enable Clear Register */

/* 32-bit Performance Monitors Overflow Flag Status Register (PMOVSR):
 * CRn=c9, opc1=0, CRm=c12, opc2=3
 */

#define PMOFSR_COF0        (1 << 0)  /* Bits  0: Counter 0 overflow flag */
#define PMOFSR_COF1        (1 << 1)  /* Bits  1: Counter 1 overflow flag */
#define PMOFSR_COF2        (1 << 2)  /* Bits  2: Counter 2 overflow flag */
#define PMOFSR_COF3        (1 << 3)  /* Bits  3: Counter 3 overflow flag */
#define PMOFSR_COF4        (1 << 4)  /* Bits  4: Counter 4 overflow flag */
#define PMOFSR_COF5        (1 << 5)  /* Bits  5: Counter 5 overflow flag */
#define PMOFSR_CCOF        (1 << 31) /* Bits 31: Cycle counter overflow flag */

/* 32-bit Performance Monitors Software Increment register (PMSWINC):
 * CRn=c9, opc1=0, CRm=c12, opc2=4
 * TODO: To be provided
 */

/* 32-bit Performance Monitors Event Counter Selection Register (PMSELR):
 * CRn=c9, opc1=0, CRm=c12, opc2=5
 */

#define PMECSR_CS_MASK     (0x1f)    /* Bits 0-5: Counter select */

/* 32-bit Performance Monitors Common Event Identification (PMCEID0):
 * CRn=c9, opc1=0, CRm=c12, opc2=6
 * TODO: To be provided
 */

/* 32-bit Performance Monitors Common Event Identification (PMCEID1):
 * CRn=c9, opc1=0, CRm=c12, opc2=7
 * TODO: To be provided
 */

/* 32-bit Performance Monitors Cycle Count Register (PMCCNTR):
 * CRn=c9, opc1=0, CRm=c13, opc2=0
 * TODO: To be provided
 */

/* 32-bit Performance Monitors Event Type Select Register (PMETSR):
 * CRn=c9, opc1=0, CRm=c13, opc2=1
 */

#define PMETSR_L1_IC_FILL             (0x1)  /* Level 1 instruction cache refill */
#define PMETSR_L1_ITLB_FILL           (0x2)  /* Level 1 instruction TLB refill */
#define PMETSR_L1_DC_FILL             (0x3)  /* Level 1 data cache refill */
#define PMETSR_L1_DC_ACC              (0x4)  /* Level 1 data cache access */
#define PMETSR_L1_DTLB_FILL           (0x5)  /* Level 1 data TLB refill */
#define PMETSR_LOAD                   (0x6)  /* Data read architecturally executed */
#define PMETSR_STORE                  (0x7)  /* Data write architecturally executed */
#define PMETSR_INSTARCHEXEC           (0x8)  /* Instruction architecturally executed */
#define PMETSR_EXCEPETIONTAKEN        (0x9)  /* Exception taken */
#define PMETSR_EXCEPETIONRET          (0xa)  /* Exception return */
#define PMETSR_WRCONTEXTIDR           (0xb)  /* Write to CONTEXTIDR */
#define PMETSR_SOFTPCCHANGE           (0xc)  /* Software change of the PC */
#define PMETSR_IMMBR                  (0xd)  /* Immediate branch */
#define PMETSR_PROCRET                (0xe)  /* Procedure return */
#define PMETSR_UNALINGEDLDSTR         (0xf)  /* Unaligned load or store */
#define PMETSR_MISPREDICTEDBRANCHEXEC (0x10) /* Mispredicted or not predicted branch speculatively executed */
#define PMETSR_PREDICTEDBRANCHEXEC    (0x12) /* Predictable branch speculatively executed */
#define PMETSR_DATAMEMACC             (0x13) /* Data memory access. */
#define PMETSR_ICACC                  (0x14) /* Instruction Cache access. */
#define PMETSR_DCEVICTION             (0x15) /* Data cache eviction. */
#define PMETSR_IRQEXCEPTION           (0x86) /* IRQ exception taken. */
#define PMETSR_FIQEXCEPTION           (0x87) /* FIQ exception taken. */
#define PMETSR_EXTMEMREQ              (0xc0) /* External memory request. */
#define PMETSR_NCEXTMEMREQ            (0xc1) /* Non-cacheable external memory request */
#define PMETSR_PREFETCHLINEFILL       (0xc2) /* Linefill because of prefetch. */
#define PMETSR_PREFETCHLINEDROP       (0xc3) /* Prefetch linefill dropped. */
#define PMETSR_ENTERINGRAMODE         (0xc4) /* Entering read allocate mode. */
#define PMETSR_RAMODE                 (0xc5) /* Read allocate mode. */
#define PMETSR_RESERVED               (0xc6) /* Reserved, do not use */
#define PMETSR_DWSTALLSBFFULL         (0xc9) /* Data Write operation that stalls the pipeline because the store buffer is full. */
#define PMETSR_EVENT_MASK             (0xff)

/* 32-bit Performance Monitors Event Count Register (PMXEVCNTR):
 * CRn=c9, opc1=0, CRm=c13, opc2=2
 * TODO: To be provided
 */

/* 32-bit Performance Monitors User Enable Register (PMUSERENR):
 * CRn=c9, opc1=0, CRm=c14, opc2=0
 */

#define PMUER_UME_SHIFT      (0)       /* Bits 0: User mode enable */
#define PMUER_UME            (1 << PMUER_UME_SHIFT)

/* 32-bit Performance Monitors Interrupt Enable Set register (PMINTENSET):
 * CRn=c9, opc1=0, CRm=c14, opc2=1
 */

#define PMIESR_CCNTOIE_SHIFT (31)    /* Bits 31: CCNT overflow interrupt enable. */
#define PMIESR_CCNTOIE       (1 << PMIESR_CCNTOIE_SHIFT)

/* 32-bit Performance Monitors Interrupt Enable Clear register (PMINTENCLR):
 * CRn=c9, opc1=0, CRm=c14, opc2=2
 */

#define PMIECR_CCNTOIE_SHIFT (31)    /* Bits 31: CCNT overflow interrupt enable. */
#define PMIECR_CCNTOIE       (1 << PMIECR_CCNTOIE_SHIFT)

/* CP15 c10 Registers *******************************************************/

/* Not used on ARMv7-R */

/* CP15 c11 Registers *******************************************************/

/* Reserved for implementation defined DMA functions */

/* CP15 c12 Registers *******************************************************/

/* Not used on ARMv7-R */

/* CP15 c13 Registers *******************************************************/

/* Context ID Register (CONTEXTIDR): CRn=c13, opc1=0, CRm=c0, opc2=1
 * 32-Bit ContextID value.
 */

/* User Read/Write (TPIDRURW): CRn=c13, opc1=0, CRm=c0, opc2=2
 * TODO: To be provided
 */

/* User Read Only (TPIDRURO): CRn=c13, opc1=0, CRm=c0, opc2=3
 * TODO: To be provided
 */

/* PL1 only (TPIDRPRW): CRn=c13, opc1=0, CRm=c0, opc2=4
 * TODO: To be provided
 */

/* CP15 c14 Registers *******************************************************/

/* Counter Frequency register (CNTFRQ): CRn=c14, opc1=0, CRm=c0, opc2=0
 * TODO: To be provided
 */

/* Timer PL1 Control register (CNTKCTL): CRn=c14, opc1=0, CRm=c1, opc2=0
 * TODO: To be provided
 */

/* PL1 Physical TimerValue register (CNTP_TVAL):
 * CRn=c14, opc1=0, CRm=c2, opc2=0
 * TODO: To be provided
 */

/* PL1 Physical Timer Control register (CNTP_CTL):
 * CRn=c14, opc1=0, CRm=c2, opc2=0
 * TODO: To be provided
 */

/* Virtual TimerValue register (CNTV_TVAL): CRn=c14, opc1=0, CRm=c3, opc2=0
 * TODO: To be provided
 */

/* Virtual Timer Control register (CNTV_CTL): CRn=c14, opc1=0, CRm=c3, opc2=0
 * TODO: To be provided
 */

/* 64-bit Physical Count register (CNTPCT): CRn=c14, opc1=0, CRm=c14, opc2=n
 * TODO: To be provided
 */

/* Virtual Count register (CNTVCT): CRn=c14, opc1=1, CRm=c14, opc2=n
 * TODO: To be provided
 */

/* PL1 Physical Timer CompareValue register (CNTP_CVAL):
 * CRn=c14, opc1=2, CRm=c14, opc2=n
 * TODO: To be provided
 */

/* Virtual Timer CompareValue register (CNTV_CVAL):
 * CRn=c14, opc1=3, CRm=c14, opc2=n
 * TODO: To be provided
 */

/* CP15 c15 Registers *******************************************************/

/* Implementation defined */

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__

/* Get the device ID */

.macro cp15_rdid, id
  mrc p15, 0, \id, c0, c0, 0
.endm

/* Read/write the system control register (SCTLR) */

.macro cp15_rdsctlr, sctlr
  mrc p15, 0, \sctlr, c1, c0, 0
.endm

.macro cp15_wrsctlr, sctlr
  mcr p15, 0, \sctlr, c1, c0, 0
  nop
  nop
  nop
  nop
  nop
  nop
  nop
  nop
.endm
#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Get the device ID register */

static inline unsigned int cp15_rdid(void)
{
  return CP15_GET(MIDR);
}

/* Get the Multiprocessor Affinity Register (MPIDR) */

static inline unsigned int cp15_rdmpidr(void)
{
  return CP15_GET(MPIDR);
}

/* Read/write the system control register (SCTLR) */

static inline unsigned int cp15_rdsctlr(void)
{
  return CP15_GET(SCTLR);
}

static inline void cp15_wrsctlr(unsigned int sctlr)
{
  CP15_SET(SCTLR, sctlr);
  ARM_NOP();
  ARM_NOP();
  ARM_NOP();
  ARM_NOP();
  ARM_NOP();
  ARM_NOP();
  ARM_NOP();
  ARM_NOP();
}

/* Read/write the vector base address register (VBAR) */

static inline unsigned int cp15_rdvbar(void)
{
  return CP15_GET(VBAR);
}

static inline void cp15_wrvbar(unsigned int vbar)
{
  CP15_SET(VBAR, vbar);
}

/* Read/write the implementation defined Auxiliary Control Register (ACTLR) */

static inline unsigned int cp15_rdactlr(void)
{
  return CP15_GET(ACTLR);
}

static inline void cp15_wractlr(unsigned int actlr)
{
  CP15_SET(ACTLR, actlr);
}

/****************************************************************************
 * Name: cp15_pmu_rdpmcr/cp15_pmu_wrpmcr/cp15_pmu_pmcr
 *
 * Description:
 *   Read/Write the Performance Monitor Control Register (PMCR)
 *
 ****************************************************************************/

static inline unsigned int cp15_pmu_rdpmcr(void)
{
  return CP15_GET(PMCR);
}

static inline void cp15_pmu_wrpmcr(unsigned int pmcr)
{
  CP15_SET(PMCR, pmcr);
}

static inline void cp15_pmu_pmcr(unsigned int pmcr)
{
  cp15_pmu_wrpmcr(pmcr | cp15_pmu_rdpmcr());
}

/****************************************************************************
 * Name: cp15_pmu_rdcesr/cp15_pmu_wrcesr/cp15_pmu_cesr
 *
 * Description:
 *   Read/Write the Performance Monitors
 *   Count Enable Set register (PMCNTENSET)
 *
 ****************************************************************************/

static inline unsigned int cp15_pmu_rdcesr(void)
{
  return CP15_GET(PMCNTENSET);
}

static inline void cp15_pmu_wrcesr(unsigned int cesr)
{
  CP15_SET(PMCNTENSET, cesr);
}

static inline void cp15_pmu_cesr(unsigned int cesr)
{
  cp15_pmu_wrcesr(cesr | cp15_pmu_rdcesr());
}

/****************************************************************************
 * Name: cp15_pmu_rdcecr/cp15_pmu_wrcecr/cp15_pmu_cecr
 *
 * Description:
 *   Read/Write the Performance Monitors
 *   Count Enable Clear register (PMCNTENCLR)
 *
 ****************************************************************************/

static inline unsigned int cp15_pmu_rdcecr(void)
{
  return CP15_GET(PMCNTENCLR);
}

static inline void cp15_pmu_wrcecr(unsigned int cecr)
{
  CP15_SET(PMCNTENCLR, cecr);
}

static inline void cp15_pmu_cecr(unsigned int cecr)
{
  cp15_pmu_wrcecr(cecr | cp15_pmu_rdcecr());
}

/****************************************************************************
 * Name: cp15_pmu_rdcecr/cp15_pmu_wrcecr/cp15_pmu_cecr
 *
 * Description:
 *   Read/Write the Performance Monitors
 *   Overflow Flag Status Register (PMOVSR)
 *
 ****************************************************************************/

static inline unsigned int cp15_pmu_rdofsr(void)
{
  return CP15_GET(PMOVSR);
}

static inline void cp15_pmu_wrofsr(unsigned int ofsr)
{
  CP15_SET(PMOVSR, ofsr);
}

static inline void cp15_pmu_ofsr(unsigned int ofsr)
{
  cp15_pmu_wrofsr(ofsr | cp15_pmu_rdofsr());
}

/****************************************************************************
 * Name: cp15_pmu_rdsir/cp15_pmu_wrsir/cp15_pmu_sir
 *
 * Description:
 *   Read/Write the Performance Monitors
 *   Software Increment register (PMSWINC)
 *
 ****************************************************************************/

static inline unsigned int cp15_pmu_rdsir(void)
{
  return CP15_GET(PMSWINC);
}

static inline void cp15_pmu_wrsir(unsigned int sir)
{
  CP15_SET(PMSWINC, sir);
}

static inline void cp15_pmu_sir(unsigned int sir)
{
  cp15_pmu_wrsir(sir | cp15_pmu_rdsir());
}

/****************************************************************************
 * Name: cp15_pmu_wrecsr
 *
 * Description:
 *   Write the Performance Monitors Event Counter Selection Register (PMSELR)
 *
 ****************************************************************************/

static inline void cp15_pmu_wrecsr(unsigned int ecsr)
{
  CP15_SET(PMSELR, ecsr);
}

/****************************************************************************
 * Name: cp15_pmu_wretsr
 *
 * Description:
 *   Write the Performance Monitors Event Type Select Register (PMXEVTYPER)
 *
 ****************************************************************************/

static inline void cp15_pmu_wretsr(unsigned int etsr)
{
  CP15_SET(PMXEVTYPER, etsr);
}

/****************************************************************************
 * Name: cp15_pmu_wruer
 *
 * Description:
 *   Write the Performance Monitors User Enable Register (PMUSERENR)
 *
 ****************************************************************************/

static inline unsigned int cp15_pmu_rduer(void)
{
  return CP15_GET(PMUSERENR);
}

static inline void cp15_pmu_wruer(unsigned int uer)
{
  CP15_SET(PMUSERENR, uer);
}

static inline void cp15_pmu_uer(unsigned int uer)
{
  cp15_pmu_wruer(uer | cp15_pmu_rduer());
}

/****************************************************************************
 * Name: cp15_pmu_wriesr
 *
 * Description:
 *   Write the Performance Monitors
 *   Interrupt Enable Set register (PMINTENSET)
 *
 ****************************************************************************/

static inline void cp15_pmu_wriesr(unsigned int iesr)
{
  CP15_SET(PMINTENSET, iesr);
}

/****************************************************************************
 * Name: cp15_pmu_wriecr
 *
 * Description:
 *   Write the Performance Monitors
 *   Interrupt Enable Clear register (PMINTENCLR)
 *
 ****************************************************************************/

static inline void cp15_pmu_wriecr(unsigned int iecr)
{
  CP15_SET(PMINTENCLR, iecr);
}

/****************************************************************************
 * Name: cp15_pmu_rdccr
 *
 * Description:
 *   Read the Performance Monitors Cycle Count Register (PMCCNTR)
 *
 ****************************************************************************/

static inline unsigned int cp15_pmu_rdccr(void)
{
  return CP15_GET(PMCCNTR);
}

/****************************************************************************
 * Name: cp15_pmu_rdecr
 *
 * Description:
 *   Read the Performance Monitors Event Count Register (PMXEVCNTR)
 *
 ****************************************************************************/

static inline unsigned int cp15_pmu_rdecr(void)
{
  return CP15_GET(PMXEVCNTR);
}

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
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

#endif /* __ARCH_ARM_SRC_ARMV7_R_SCTLR_H */
