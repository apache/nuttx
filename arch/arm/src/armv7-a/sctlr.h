/****************************************************************************
 * arch/arm/src/armv7-a/sctlr.h
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
 *  "Cortex-A5 MPCore, Technical Reference Manual",
 *   Revision: r0p1, Copyright 2010 ARM.
 *   All rights reserved. ARM DDI 0434B (ID101810)
 *  "ARM Architecture Reference Manual, ARMv7-A and ARMv7-R edition",
 *   Copyright 1996-1998, 2000, 2004-2012 ARM.
 *  All rights reserved. ARM DDI 0406C.b (ID072512)
 */

#ifndef __ARCH_ARM_SRC_ARMV7_A_SCTLR_H
#define __ARCH_ARM_SRC_ARMV7_A_SCTLR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Reference: Cortex-A5 MPCore Paragraph 4.2, "Register summary." */

/* Main ID Register (MIDR) */

/* TODO: To be provided */

/* Cache Type Register (CTR) */

/* TODO: To be provided */

/* TCM Type Register
 *
 * The Cortex-A5 MPCore processor does not implement instruction or data
 * Tightly Coupled Memory (TCM), so this register always Reads-As-Zero (RAZ).
 *
 * TLB Type Register
 *
 * The Cortex-A5 MPCore processor does not implement instruction or data
 * Tightly CoupledMemory (TCM), so this register always Reads-As-Zero (RAZ).
 */

/* Multiprocessor Affinity Register (MPIDR) */

#define MPIDR_CPUID_SHIFT        (0)       /* Bits 0-1: CPU ID */
#define MPIDR_CPUID_MASK         (3 << MPIDR_CPUID_SHIFT)
#  define MPIDR_CPUID_CPU0       (0 << MPIDR_CPUID_SHIFT)
#  define MPIDR_CPUID_CPU1       (1 << MPIDR_CPUID_SHIFT)
#  define MPIDR_CPUID_CPU2       (2 << MPIDR_CPUID_SHIFT)
#  define MPIDR_CPUID_CPU3       (3 << MPIDR_CPUID_SHIFT)
                                           /* Bits 2-7: Reserved */
#define MPIDR_CLUSTID_SHIFT      (8)       /* Bits 8-11: Cluster ID value */
#define MPIDR_CLUSTID_MASK       (15 << MPIDR_CLUSTID_SHIFT)
                                           /* Bits 12-29: Reserved */
#define MPIDR_U                  (1 << 30) /* Bit 30: Multiprocessing Extensions. */

/* Processor Feature Register 0 (ID_PFR0) */

/* TODO: To be provided */

/* Processor Feature Register 1 (ID_PFR1) */

/* TODO: To be provided */

/* Debug Feature Register 0 (ID_DFR0) */

/* TODO: To be provided */

/* Auxiliary Feature Register 0 (ID_AFR0) */

/* TODO: To be provided */

/* Memory Model Features Register 0 (ID_MMFR0) */

/* Memory Model Features Register 1 (ID_MMFR1) */

/* Memory Model Features Register 2 (ID_MMFR2) */

/* Memory Model Features Register 3 (ID_MMFR3) */

/* TODO: To be provided */

/* Instruction Set Attributes Register 0 (ID_ISAR0) */

/* Instruction Set Attributes Register 1 (ID_ISAR1) */

/* Instruction Set Attributes Register 2 (ID_ISAR2) */

/* Instruction Set Attributes Register 3 (ID_ISAR3) */

/* Instruction Set Attributes Register 4 (ID_ISAR4) */

/* Instruction Set Attributes Register 5 (ID_ISAR5) */

/* Instruction Set Attributes Register 6-7 (ID_ISAR6-7).  Reserved. */

/* TODO: Others to be provided */

/* Cache Size Identification Register (CCSIDR) */

/* TODO: To be provided */

/* Cache Level ID Register (CLIDR) */

/* TODO: To be provided */

/* Auxiliary ID Register (AIDR) */

/* TODO: To be provided */

/* Cache Size Selection Register (CSSELR) */

/* TODO: To be provided */

/* System Control Register (SCTLR)
 *
 * NOTES:
 * (1) Always enabled on A5
 * (2) Not available on A5
 */

#define SCTLR_M                  (1 << 0)  /* Bit 0:  Enables the MMU */
#define SCTLR_A                  (1 << 1)  /* Bit 1:  Enables strict alignment of data */
#define SCTLR_C                  (1 << 2)  /* Bit 2:  Determines if data can be cached */
                                           /* Bits 3-9: Reserved */
#define SCTLR_SW                 (1 << 10) /* Bit 10: SWP/SWPB Enable bit */
#define SCTLR_Z                  (1 << 11) /* Bit 11: Program flow prediction control (1) */
#define SCTLR_I                  (1 << 12) /* Bit 12: Determines if instructions can be cached */
#define SCTLR_V                  (1 << 13) /* Bit 13: Vectors bit */
#define SCTLR_RR                 (1 << 14) /* Bit 14: Cache replacement strategy (2) */
                                           /* Bits 15-16: Reserved */
#define SCTLR_HA                 (1 << 17) /* Bit 17: Hardware management access disabled (2) */
                                           /* Bits 18-24: Reserved */
#define SCTLR_EE                 (1 << 25) /* Bit 25: Determines the value the CPSR.E */
                                           /* Bit 26: Reserved */
#define SCTLR_NMFI               (1 << 27) /* Bit 27: Non-maskable FIQ support (Cortex-A9) */
#define SCTLR_TRE                (1 << 28) /* Bit 28: TEX remap */
#define SCTLR_AFE                (1 << 29) /* Bit 29: Access Flag Enable bit */
#define SCTLR_TE                 (1 << 30) /* Bit 30: Thumb exception enable */
                                           /* Bit 31: Reserved */

/* Auxiliary Control Register (ACTLR) */

#define ACTLR_FW                 (1 << 0)  /* Bit 0: Enable Cache/TLB maintenance broadcast */
#define ACTLR_L2_PREFECTH        (1 << 1)  /* Bit 1: L2 pre-fetch hint enable */
#define ACTLR_L1_PREFETCH        (1 << 2)  /* Bit 2: L1 Dside pre-fetch enable */
#define ACTLR_LINE_ZERO          (1 << 3)  /* Bit 3: Enable write full line zero mode */
                                           /* Bits 4-5: Reserved */
#define ACTLR_SMP                (1 << 6)  /* Bit 6: Cortex-A9 taking part in coherency */
#define ACTLR_EXCL               (1 << 7)  /* Bit 7: Exclusive cache bit */
#define ACTLR_ALLOC_1WAY         (1 << 8)  /* Bit 8: Allocation in 1-way cache only */
#define ACTLR_PARITY             (1 << 9)  /* Bit 9: Parity ON */
                                           /* Bits 10-31: Reserved */

/* Coprocessor Access Control Register (CPACR) */

/* TODO: To be provided */

/* Secure Configuration Register (SCR) */

#define SCR_NS                   (1 << 0)  /* Bit 0: Non-secure */
#define SCR_IRQ                  (1 << 1)  /* Bit 1: IRQs taken in Monitor mode */
#define SCR_FIQ                  (1 << 2)  /* Bit 2: FIQs taken in Monitor mode */
#define SCR_EA                   (1 << 3)  /* Bit 3: External aborts taken in Monitor mode */
#define SCR_FW                   (1 << 4)  /* Bit 4: F bit writable */
#define SCR_AW                   (1 << 5)  /* Bit 5: A bit writable */
#define SCR_NET                  (1 << 6)  /* Bit 6: Not Early Termination */
#define SCR_SCD                  (1 << 7)  /* Bit 7: Secure Monitor Call disable */
#define SCR_HCE                  (1 << 8)  /* Bit 8: Hyp Call enable */
#define SCR_SIF                  (1 << 9)  /* Bit 9: Secure state instruction fetches from
                                            *        Non-secure memory are not permitted */
                                           /* Bits 10-31: Reserved */

/* Secure Debug Enable Register (SDER) */

/* TODO: To be provided */

/* Non-secure Access Control Register (NSACR) */

                                           /* Bits 0-9: Reserved */
#define NSACR_CP10               (1 << 10) /* Bit 10: Permission to access coprocessor 10 */
#define NSACR_CP11               (1 << 11) /* Bit 11: Permission to access coprocessor 11 */
                                           /* Bits 12-13: Reserved */
#define NSACR_NSD32DIS           (1 << 14) /* Bit 14: Disable the Non-secure use of VFP D16-D31 */
#define NSACR_NSASEDIS           (1 << 15) /* Bit 15: Disable Non-secure Advanced SIMD Extension */
                                           /* Bits 16-17: Reserved */
#define NSACR_NSSMP              (1 << 18) /* Bit 18: ACR SMP bit writable */
                                           /* Bits 19-31: Reserved */

/* Virtualization Control Register (VCR) */

/* TODO: To be provided */

/* Translation Table Base Register 0 (TTBR0).  See mmu.h */

/* Translation Table Base Register 1 (TTBR1).  See mmu.h */

/* Translation Table Base Control Register (TTBCR).  See mmu.h */

/* Domain Access Control Register (DACR).  See mmu.h */

/* Data Fault Status Register (DFSR).  See mmu.h */

/* Instruction Fault Status Register (IFSR).  See mmu.h */

/* Auxiliary Data Fault Status Register (ADFSR).
 * Not used in this implementation.
 */

/* Data Fault Address Register(DFAR)
 *
 *   Holds the MVA of the faulting address when a synchronous fault occurs
 *
 * Instruction Fault Address Register(IFAR)
 *
 *   Holds the MVA of the faulting address of the instruction that caused a
 *   prefetch abort.
 *
 * NOP Register
 *
 *   The use of this register is optional and deprecated.
 *   Use the NOP instruction instead.
 *
 * Physical Address Register (PAR)
 *
 *   Holds:
 *   - the PA after a successful translation
 *   - the source of the abort for an unsuccessful translation
 *
 * Instruction Synchronization Barrier
 *
 *   The use of ISB is optional and deprecated.
 *   Use the instruction ISB instead.
 *
 * Data Memory Barrier
 *   The use of DMB is deprecated and, on Cortex-A5 MPCore, behaves as NOP.
 *   Use the instruction DMB instead.
 */

/* Vector Base Address Register (VBAR) */

#define VBAR_MASK                (0xffffffe0)

/* Monitor Vector Base Address Register (MVBAR) */

/* TODO: To be provided */

/* Interrupt Status Register (ISR) */

/* TODO: To be provided */

/* Virtualization Interrupt Register (VIR) */

/* TODO: To be provided */

/* Context ID Register (CONTEXTIDR) */

#define CONTEXTIDR_ASID_SHIFT    (0)   /* Bits 0-7: Address Space Identifier */
#define CONTEXTIDR_ASID_MASK     (0xff << CONTEXTIDR_ASID_SHIFT)
#define CONTEXTIDR_PROCID_SHIFT  (8) /* Bits 8-31: Process Identifier */
#define CONTEXTIDR_PROCID_MASK   (0x00ffffff << CONTEXTIDR_PROCID_SHIFT)

/* Configuration Base Address Register (CBAR) */

/* TODO: To be provided */

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
  unsigned int id;
  __asm__ __volatile__
    (
      "\tmrc p15, 0, %0, c0, c0, 0\n"
      : "=r" (id)
      :
      : "memory"
    );

  return id;
}

/* Get the Multiprocessor Affinity Register (MPIDR) */

static inline unsigned int cp15_rdmpidr(void)
{
  unsigned int mpidr;
  __asm__ __volatile__
    (
      "\tmrc p15, 0, %0, c0, c0, 5\n"
      : "=r" (mpidr)
      :
      : "memory"
    );

  return mpidr;
}

/* Read/write the system control register (SCTLR) */

static inline unsigned int cp15_rdsctlr(void)
{
  unsigned int sctlr;
  __asm__ __volatile__
    (
      "\tmrc p15, 0, %0, c1, c0, 0\n"
      : "=r" (sctlr)
      :
      : "memory"
    );

  return sctlr;
}

static inline void cp15_wrsctlr(unsigned int sctlr)
{
  __asm__ __volatile__
    (
      "\tmcr p15, 0, %0, c1, c0, 0\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      :
      : "r" (sctlr)
      : "memory"
    );
}

/* Read/write the vector base address register (VBAR) */

static inline unsigned int cp15_rdvbar(void)
{
  unsigned int sctlr;
  __asm__ __volatile__
    (
      "\tmrc p15, 0, %0, c12, c0, 0\n"
      : "=r" (sctlr)
      :
      : "memory"
    );

  return sctlr;
}

static inline void cp15_wrvbar(unsigned int sctlr)
{
  __asm__ __volatile__
    (
      "\tmcr p15, 0, %0, c12, c0, 0\n"
      :
      : "r" (sctlr)
      : "memory"
    );
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
  unsigned int pmcr;
  __asm__ __volatile__
    (
      "\tmrc p15, 0, %0, c9, c12, 0\n"
      : "=r" (pmcr)
      :
      : "memory"
    );

  return pmcr;
}

static inline void cp15_pmu_wrpmcr(unsigned int pmcr)
{
  __asm__ __volatile__
    (
      "\tmcr p15, 0, %0, c9, c12, 0\n"
      :
      : "r" (pmcr)
      : "memory"
    );
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
  unsigned int cesr;
  __asm__ __volatile__
    (
      "\tmrc p15, 0, %0, c9, c12, 1\n"
      : "=r" (cesr)
      :
      : "memory"
    );

  return cesr;
}

static inline void cp15_pmu_wrcesr(unsigned int cesr)
{
  __asm__ __volatile__
    (
      "\tmcr p15, 0, %0, c9, c12, 1\n"
      :
      : "r" (cesr)
      : "memory"
    );
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
  unsigned int cecr;
  __asm__ __volatile__
    (
      "\tmrc p15, 0, %0, c9, c12, 2\n"
      : "=r" (cecr)
      :
      : "memory"
    );

  return cecr;
}

static inline void cp15_pmu_wrcecr(unsigned int cecr)
{
  __asm__ __volatile__
    (
      "\tmcr p15, 0, %0, c9, c12, 2\n"
      :
      : "r" (cecr)
      : "memory"
    );
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
  unsigned int ofsr;
  __asm__ __volatile__
    (
      "\tmrc p15, 0, %0, c9, c12, 3\n"
      : "=r" (ofsr)
      :
      : "memory"
    );

  return ofsr;
}

static inline void cp15_pmu_wrofsr(unsigned int ofsr)
{
  __asm__ __volatile__
    (
      "\tmcr p15, 0, %0, c9, c12, 3\n"
      :
      : "r" (ofsr)
      : "memory"
    );
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
  unsigned int sir;
  __asm__ __volatile__
    (
      "\tmrc p15, 0, %0, c9, c12, 4\n"
      : "=r" (sir)
      :
      : "memory"
    );

  return sir;
}

static inline void cp15_pmu_wrsir(unsigned int sir)
{
  __asm__ __volatile__
    (
      "\tmcr p15, 0, %0, c9, c12, 4\n"
      :
      : "r" (sir)
      : "memory"
    );
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
  __asm__ __volatile__
    (
      "\tmcr p15, 0, %0, c9, c12, 5\n"
      :
      : "r" (ecsr)
      : "memory"
    );
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
  __asm__ __volatile__
    (
      "\tmcr p15, 0, %0, c9, c13, 1\n"
      :
      : "r" (etsr)
      : "memory"
    );
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
  unsigned int uer;
  __asm__ __volatile__
    (
      "\tmrc p15, 0, %0, c9, c14, 0\n"
      : "=r" (uer)
      :
      : "memory"
    );

  return uer;
}

static inline void cp15_pmu_wruer(unsigned int uer)
{
  __asm__ __volatile__
    (
      "\tmcr p15, 0, %0, c9, c14, 0\n"
      :
      : "r" (uer)
      : "memory"
    );
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
  __asm__ __volatile__
    (
      "\tmcr p15, 0, %0, c9, c14, 1\n"
      :
      : "r" (iesr)
      : "memory"
    );
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
  __asm__ __volatile__
    (
      "\tmcr p15, 0, %0, c9, c14, 2\n"
      :
      : "r" (iecr)
      : "memory"
    );
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
  unsigned int ccr;
  __asm__ __volatile__
    (
      "\tmrc p15, 0, %0, c9, c13, 0\n"
      : "=r" (ccr)
      :
      : "memory"
    );

  return ccr;
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
  unsigned int ecr;
  __asm__ __volatile__
    (
      "\tmrc p15, 0, %0, c9, c13, 2"
      : "=r" (ecr)
      :
      : "memory"
    );

  return ecr;
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

#endif /* __ARCH_ARM_SRC_ARMV7_A_SCTLR_H */
