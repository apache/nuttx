/************************************************************************************
 * arch/arm/src/armv7-r/sctlr.h
 * CP15 System Control Registers
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *  "ARM Architecture Reference Manual, ARMv7-A and ARMv7-R edition", Copyright
 *   1996-1998, 2000, 2004-2012 ARM. All rights reserved. ARM DDI 0406C.c (ID051414)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_ARMV7_R_SCTLR_H
#define __ARCH_ARM_SRC_ARMV7_R_SCTLR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* CP15 c0 Registers ****************************************************************/

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

/* Memory Model Features Register 0 (ID_MMFR0): CRn=c0, opc1=0, CRm=c1, opc2=4
 * Memory Model Features Register 1 (ID_MMFR1): CRn=c0, opc1=0, CRm=c1, opc2=5
 * Memory Model Features Register 2 (ID_MMFR2): CRn=c0, opc1=0, CRm=c1, opc2=6
 * Memory Model Features Register 3 (ID_MMFR3): CRn=c0, opc1=0, CRm=c1, opc2=7
 * TODO: To be provided
 */

/* Instruction Set Attributes Register 0 (ID_ISAR0): CRn=c0, opc1=0, CRm=c2, opc2=0
 * Instruction Set Attributes Register 1 (ID_ISAR1): CRn=c0, opc1=0, CRm=c2, opc2=1
 * Instruction Set Attributes Register 2 (ID_ISAR2): CRn=c0, opc1=0, CRm=c2, opc2=2
 * Instruction Set Attributes Register 3 (ID_ISAR3): CRn=c0, opc1=0, CRm=c2, opc2=3
 * Instruction Set Attributes Register 4 (ID_ISAR4): CRn=c0, opc1=0, CRm=c2, opc2=4
 * Instruction Set Attributes Register 5 (ID_ISAR5): CRn=c0, opc1=0, CRm=c2, opc2=5
 * Instruction Set Attributes Register 6-7 (ID_ISAR6-7).  Reserved.
 * TODO: Others to be provided
 */

/* Reserved: CRn=c0, opc1=0, CRm=c3-c7, opc2=* */

/* Cache Size Identification Register (CCSIDR): CRn=c0, opc1=1, CRm=c0, opc2=0
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

/* CP15 c1 Registers ****************************************************************/
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

/* Coprocessor Access Control Register (CPACR): CRn=c1, opc1=0, CRm=c0, opc2=2
 * TODO: To be provided
 */

/* CP15 c2-c4 Registers *************************************************************/
/* Not used on ARMv7-R */

/* CP15 c5 Registers ****************************************************************/
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

/* CP15 c6 Registers ****************************************************************/

/* Data Fault Address Register(DFAR): CRn=c6, opc1=0, CRm=c0, opc2=0
 *
 *   Holds the MVA of the faulting address when a synchronous fault occurs
 */

/* Instruction Fault Address Register(IFAR): CRn=c6, opc1=0, CRm=c0, opc2=1
 *
 *   Holds the MVA of the faulting address of the instruction that caused a prefetch
 *   abort.
 */

/* Data Region Base Address Register (DRBAR): CRn=c6, opc1=0, CRm=c1, opc2=0
 * TODO: To be provided
 */

/* Instruction Region Base Address Register (IRBAR): CRn=c6, opc1=0, CRm=c1, opc2=1
 * TODO: To be provided
 */

/* Data Region Size and Enable Register (DRSR): CRn=c6, opc1=0, CRm=c1, opc2=2
 * TODO: To be provided
 */

/* Instruction Region Size and Enable Register (IRSR): CRn=c6, opc1=0, CRm=c1, opc2=3
 * TODO: To be provided
 */

/* Data Region Access Control Register (DRACR): CRn=c6, opc1=0, CRm=c1, opc2=4
 * TODO: To be provided
 */

/* Instruction Region Access Control Register (IRACR): CRn=c6, opc1=0, CRm=c1, opc2=5
 * TODO: To be provided
 */

/* MPU Region Number Register (RGNR): CRn=c6, opc1=0, CRm=c2, opc2=0
 * TODO: To be provided
 */

/* CP15 c7 Registers ****************************************************************/
/* See cp15_cacheops.h */

/* CP15 c8 Registers ****************************************************************/
/* Not used on ARMv7-R */

/* CP15 c9 Registers ****************************************************************/
/* 32-bit Performance Monitors Control Register (PMCR): CRn=c9, opc1=0, CRm=c12, opc2=0
 * TODO: To be provided
 */

#define PCMR_E             (1 << 0)  /* Enable all counters */
#define PCMR_P             (1 << 1)  /* Reset all counter eventts (except PMCCNTR) */
#define PCMR_C             (1 << 2)  /* Reset cycle counter (PMCCNTR) to zero */
#define PCMR_D             (1 << 3)  /* Enable cycle counter clock (PMCCNTR) divider */
#define PCMR_X             (1 << 4)  /* Export of events is enabled */
#define PCMR_DP            (1 << 5)  /* Disable PMCCNTR if event counting is prohibited */
#define PCMR_N_SHIFT       (11)      /* Bits 11-15:  Number of event counters */
#define PCMR_N_MASK        (0x1f << PCMR_N_SHIFT)
#define PCMR_IDCODE_SHIFT  (16)      /* Bits 16-23: Identification code */
#define PCMR_IDCODE_MASK   (0xff << PCMR_IDCODE_SHIFT)
#define PCMR_IMP_SHIFT     (24)      /* Bits 24-31: Implementer code */
#define PCMR_IMP_MASK      (0xff << PCMR_IMP_SHIFT)

/* 32-bit Performance Monitors Count Enable Set register (PMCNTENSET): CRn=c9, opc1=0, CRm=c12, opc2=1
 * TODO: To be provided
 */

/* 32-bit Performance Monitors Count Enable Clear register (PMCNTENCLR): CRn=c9, opc1=0, CRm=c12, opc2=2
 * TODO: To be provided
 */

/* 32-bit Performance Monitors Overflow Flag Status Register (PMOVSR): CRn=c9, opc1=0, CRm=c12, opc2=3
 * TODO: To be provided
 */

/* 32-bit Performance Monitors Software Increment register (PMSWINC): CRn=c9, opc1=0, CRm=c12, opc2=4
 * TODO: To be provided
 */

/* 32-bit Performance Monitors Event Counter Selection Register (PMSELR): CRn=c9, opc1=0, CRm=c12, opc2=5
 * TODO: To be provided
 */

/* 32-bit Performance Monitors Common Event Identification (PMCEID0): CRn=c9, opc1=0, CRm=c12, opc2=6
 * TODO: To be provided
 */

/* 32-bit Performance Monitors Common Event Identification (PMCEID1): CRn=c9, opc1=0, CRm=c12, opc2=7
 * TODO: To be provided
 */

/* 32-bit Performance Monitors Cycle Count Register (PMCCNTR): CRn=c9, opc1=0, CRm=c13, opc2=0
 * TODO: To be provided
 */

/* 32-bit Performance Monitors Event Type Select Register (PMXEVTYPER): CRn=c9, opc1=0, CRm=c13, opc2=1
 * TODO: To be provided
 */

/* 32-bit Performance Monitors Event Count Register (PMXEVCNTR): CRn=c9, opc1=0, CRm=c13, opc2=2
 * TODO: To be provided
 */

/* 32-bit Performance Monitors User Enable Register (PMUSERENR): CRn=c9, opc1=0, CRm=c14, opc2=0
 * TODO: To be provided
 */

/* 32-bit Performance Monitors Interrupt Enable Set register (PMINTENSET): CRn=c9, opc1=0, CRm=c14, opc2=1
 * TODO: To be provided
 */

/* 32-bit Performance Monitors Interrupt Enable Clear register (PMINTENCLR): CRn=c9, opc1=0, CRm=c14, opc2=2
 * TODO: To be provided
 */

/* CP15 c10 Registers ***************************************************************/
/* Not used on ARMv7-R */

/* CP15 c11 Registers ***************************************************************/
/* Reserved for implementation defined DMA functions */

/* CP15 c12 Registers ***************************************************************/
/* Not used on ARMv7-R */

/* CP15 c13 Registers ***************************************************************/

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

/* CP15 c14 Registers ***************************************************************/

/* Counter Frequency register (CNTFRQ): CRn=c14, opc1=0, CRm=c0, opc2=0
 * TODO: To be provided
 */

/* Timer PL1 Control register (CNTKCTL): CRn=c14, opc1=0, CRm=c1, opc2=0
 * TODO: To be provided
 */

/* PL1 Physical TimerValue register (CNTP_TVAL): CRn=c14, opc1=0, CRm=c2, opc2=0
 * TODO: To be provided
 */

/* PL1 Physical Timer Control register (CNTP_CTL): CRn=c14, opc1=0, CRm=c2, opc2=0
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

/* PL1 Physical Timer CompareValue register (CNTP_CVAL): CRn=c14, opc1=2, CRm=c14, opc2=n
 * TODO: To be provided
 */

/* Virtual Timer CompareValue register (CNTV_CVAL): CRn=c14, opc1=3, CRm=c14, opc2=n
 * TODO: To be provided
 */

/* CP15 c15 Registers ***************************************************************/
/* Implementation defined */

/************************************************************************************
 * Assemby Macros
 ************************************************************************************/

#ifdef __ASSEMBLY__

/* Get the device ID */

	.macro	cp15_rdid, id
	mrc		p15, 0, \id, c0, c0, 0
	.endm

/* Read/write the system control register (SCTLR) */

	.macro	cp15_rdsctlr, sctlr
	mrc		p15, 0, \sctlr, c1, c0, 0
	.endm

	.macro	cp15_wrsctlr, sctlr
	mcr		p15, 0, \sctlr, c1, c0, 0
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

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/* Get the device ID */

static inline unsigned int cp15_rdid(void)
{
  unsigned int id;
  __asm__ __volatile__
    (
      "\tmrc p15, 0, %0, c0, c0, 0"
      : "=r" (id)
      :
      : "memory"
    );

  return id;
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

/* Read/write the implementation defined Auxiliary Control Regster (ACTLR) */

static inline unsigned int cp15_rdactlr(void)
{
  unsigned int actlr;
  __asm__ __volatile__
    (
      "\tmrc p15, 0, %0, c1, c0, 1\n"
      : "=r" (actlr)
      :
      : "memory"
    );

  return actlr;
}

static inline void cp15_wractlr(unsigned int actlr)
{
  __asm__ __volatile__
    (
      "\tmcr p15, 0, %0, c1, c0, 1\n"
      :
      : "r" (actlr)
      : "memory"
    );
}

/* Read/write the Performance Monitor Control Register (PMCR) */

static inline unsigned int cp15_rdpmcr(void)
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

static inline void cp15_wrpmcr(unsigned int pmcr)
{
  __asm__ __volatile__
    (
      "\tmcr p15, 0, %0, c9, c12, 0\n"
      :
      : "r" (pmcr)
      : "memory"
    );
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

#endif  /* __ARCH_ARM_SRC_ARMV7_R_SCTLR_H */
