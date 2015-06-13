/************************************************************************************
 * arch/arm/src/armv7-a/sctlr.h
 * CP15 System Control Registers
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *  "Cortex-A5™ MPCore, Technical Reference Manual", Revision: r0p1, Copyright © 2010
 *   ARM. All rights reserved. ARM DDI 0434B (ID101810)
 *  "ARM® Architecture Reference Manual, ARMv7-A and ARMv7-R edition", Copyright ©
 *   1996-1998, 2000, 2004-2012 ARM. All rights reserved. ARM DDI 0406C.b (ID072512)
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

#ifndef __ARCH_ARM_SRC_ARMV7_A_SCTLR_H
#define __ARCH_ARM_SRC_ARMV7_A_SCTLR_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Reference: Cortex-A5™ MPCore Paragraph 4.2, "Register summary." */

/* Main ID Register (MIDR) */
/* TODO: To be provided */

/* Cache Type Register (CTR) */
/* TODO: To be provided */

/* TCM Type Register
 *
 * The Cortex-A5 MPCore processor does not implement instruction or data Tightly
 * Coupled Memory (TCM), so this register always Reads-As-Zero (RAZ).
 *
 * TLB Type Register
 *
 * The Cortex-A5 MPCore processor does not implement instruction or data Tightly
 * CoupledMemory (TCM), so this register always Reads-As-Zero (RAZ).
 */

/* Multiprocessor Affinity Register (MPIDR) */
/* TODO: To be provided */

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

#define SCTLR_M            (1 << 0)  /* Bit 0:  Enables the MMU */
#define SCTLR_A            (1 << 1)  /* Bit 1:  Enables strict alignment of data */
#define SCTLR_C            (1 << 2)  /* Bit 2:  Determines if data can be cached */
                                     /* Bits 3-9: Reserved */
#define SCTLR_SW           (1 << 10) /* Bit 10: SWP/SWPB Enable bit */
#define SCTLR_Z            (1 << 11) /* Bit 11: Program flow prediction control (1) */
#define SCTLR_I            (1 << 12) /* Bit 12: Determines if instructions can be cached */
#define SCTLR_V            (1 << 13) /* Bit 13: Vectors bit */
#define SCTLR_RR           (1 << 14) /* Bit 14: Cache replacement strategy (2) */
                                     /* Bits 15-16: Reserved */
#define SCTLR_HA           (1 << 17) /* Bit 17: Hardware management access disabled (2) */
                                     /* Bits 18-24: Reserved */
#define SCTLR_EE           (1 << 25) /* Bit 25: Determines the value the CPSR.E */
                                     /* Bits 26-27: Reserved */
#define SCTLR_TRE          (1 << 28) /* Bit 28: TEX remap */
#define SCTLR_AFE          (1 << 29) /* Bit 29: Access Flag Enable bit */
#define SCTLR_TE           (1 << 30) /* Bit 30: Thumb exception enable */
                                     /* Bit 31: Reserved */

/* Auxiliary Control Register (ACTLR) */
/* TODO: To be provided */

/* Coprocessor Access Control Register (CPACR) */
/* TODO: To be provided */

/* Secure Configuration Register (SCR) */
/* TODO: To be provided */

/* Secure Debug Enable Register (SDER) */
/* TODO: To be provided */

/* Non-secure Access Control Register (NSACR) */

                                     /* Bits 0-9: Reserved */
#define NSACR_CP10         (1 << 10) /* Bit 10: Permission to access coprocessor 10 */
#define NSACR_CP11         (1 << 11) /* Bit 11: Permission to access coprocessor 11 */
                                     /* Bits 12-13: Reserved */
#define NSACR_NSD32DIS     (1 << 14) /* Bit 14: Disable the Non-secure use of VFP D16-D31 */
#define NSACR_NSASEDIS     (1 << 15) /* Bit 15: Disable Non-secure Advanced SIMD Extension */
                                     /* Bits 16-17: Reserved */
#define NSACR_NSSMP        (1 << 18) /* Bit 18: ACR SMP bit writable */
                                     /* Bits 19-31: Reserved */

/* Virtualization Control Register (VCR) */
/* TODO: To be provided */

/* Translation Table Base Register 0 (TTBR0).  See mmu.h */
/* Translation Table Base Register 1 (TTBR1).  See mmu.h */
/* Translation Table Base Control Register (TTBCR).  See mmu.h */
/* Domain Access Control Register (DACR).  See mmu.h */
/* Data Fault Status Register (DFSR).  See mmu.h */
/* Instruction Fault Status Register (IFSR).  See mmu.h */

/* Auxiliary Data Fault Status Register (ADFSR).  Not used in this implementation. */

/* Data Fault Address Register(DFAR)
 *
 *   Holds the MVA of the faulting address when a synchronous fault occurs
 *
 * Instruction Fault Address Register(IFAR)
 *
 *   Holds the MVA of the faulting address of the instruction that caused a prefetch
 *   abort.
 *
 * NOP Register
 *
 *   The use of this register is optional and deprecated. Use the NOP instruction
 *   instead.
 *
 * Physical Address Register (PAR)
 *
 *   Holds:
 *   - the PA after a successful translation
 *   - the source of the abort for an unsuccessful translation
 *
 * Instruction Synchronization Barrier
 *
 *   The use of ISB is optional and deprecated. Use the instruction ISB instead.
 *
 * Data Memory Barrier
 *   The use of DMB is deprecated and, on Cortex-A5 MPCore, behaves as NOP. Use the
 *   instruction DMB instead.
 */

/* Vector Base Address Register (VBAR) */

#define VBAR_MASK               (0xffffffe0)

/* Monitor Vector Base Address Register (MVBAR) */
/* TODO: To be provided */

/* Interrupt Status Register (ISR) */
/* TODO: To be provided */

/* Virtualization Interrupt Register (VIR) */
/* TODO: To be provided */

/* Context ID Register (CONTEXTIDR) */

#define CONTEXTIDR_ASID_SHIFT   (0)   /* Bits 0-7: Address Space Identifier */
#define CONTEXTIDR_ASID_MASK    (0xff << CONTEXTIDR_ASID_SHIFT)
#define CONTEXTIDR_PROCID_SHIFT (8) /* Bits 8-31: Process Identifier */
#define CONTEXTIDR_PROCID_MASK  (0x00ffffff << CONTEXTIDR_PROCID_SHIFT)

/* Configuration Base Address Register (CBAR) */
/* TODO: To be provided */

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

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Variables
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

#endif  /* __ARCH_ARM_SRC_ARMV7_A_SCTLR_H */
