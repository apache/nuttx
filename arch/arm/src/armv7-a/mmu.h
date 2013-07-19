/************************************************************************************
 * arch/arm/src/armv7-a/mmu.h
 * CP15 MMU register definitions
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

#ifndef __ARCH_ARM_SRC_ARMV7_A_MMU_H
#define __ARCH_ARM_SRC_ARMV7_A_MMU_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* The Cortex-A5 supports two translation table base address registers.  In this,
 * implementation, only Translation Table Base Register 0 (TTBR0) is used.  The
 * TTBR0 contains the upper bits of the address a a page table in physical memory.
 * If 4KB page sizes are used, then TTBR0 registers holds bits 14-31 of the page
 * table address;  A full 30-bit address is formed by ORing in bits 2-13 or the
 * virtual address (MVA).  As a consequence, the page table must be aligned to a
 * 16Kb address in physical memory and could require up to 16Kb of memory.
 */

#define PGTABLE_SIZE       0x00004000

/* Reference: Cortex-A5™ MPCore Paragraph 6.7, "MMU software accessible registers." */

/* TLB Type Register TLB Type Register
 *
 * The Translation Lookaside Buffer (TLB) Type Register, TLBTR, returns the number
 * of lockable entries for the TLB. The Cortex-A5 MPCore processor does not
 * implement this feature, so this register always RAZ.
 */

/* System Control Register (SCTLR). see cstlr.h */
/* Non-secure Access Control Register (NSACR).  See cstlr.h */

/* Translation Table Base Register 0 (TTBR0)*/

#define TTBR0_IRGN1        (1 << 0)  /* Bit 0:  Inner cacheability for table walk */
#define TTBR0_S            (1 << 1)  /* Bit 1:  Translation table walk */
                                     /* Bit 2:  Reserved */
#define TTBR0_RGN_SHIFT    (3)       /* Bits 3-4: Outer cacheable attributes for table walk */
#define TTBR0_RGN_MASK     (3 << TTBR0_RGN_SHIFT)
#  define TTBR0_RGN_NONE   (0 << TTBR0_RGN_SHIFT) /* Non-cacheable */
#  define TTBR0_RGN_WBWA   (1 << TTBR0_RGN_SHIFT) /* Write-Back cached + Write-Allocate */
#  define TTBR0_RGN_WT     (2 << TTBR0_RGN_SHIFT) /* Write-Through */
#  define TTBR0_RGN_WB     (3 << TTBR0_RGN_SHIFT) /* Write-Back */
                                     /* Bit 5:  Reserved */
#define TTBR0_IRGN0        (1 << 6)  /* Bit 6:  Inner cacheability (with IRGN0) */
                                     /* Bits 7-n: Reserved, n=7-13 */
#define _TTBR0_LOWER(n)    (0xffffffff << (n))
                                     /* Bits (n+1)-31: Translation table base 0 */
#define TTBR0_BASE_MASK(n) (~_TTBR0_LOWER(n))

/* Translation Table Base Register 1 (TTBR1) */

#define TTBR1_IRGN1        (1 << 0)  /* Bit 0:  Inner cacheability for table walk */
#define TTBR1_S            (1 << 1)  /* Bit 1:  Translation table walk */
                                     /* Bit 2:  Reserved */
#define TTBR1_RGN_SHIFT    (3)       /* Bits 3-4: Outer cacheable attributes for table walk */
#define TTBR1_RGN_MASK     (3 << TTBR1_RGN_SHIFT)
#  define TTBR1_RGN_NONE   (0 << TTBR1_RGN_SHIFT) /* Non-cacheable */
#  define TTBR1_RGN_WBWA   (1 << TTBR1_RGN_SHIFT) /* Write-Back cached + Write-Allocate */
#  define TTBR1_RGN_WT     (2 << TTBR1_RGN_SHIFT) /* Write-Through */
#  define TTBR1_RGN_WB     (3 << TTBR1_RGN_SHIFT) /* Write-Back */
                                     /* Bit 5:  Reserved */
#define TTBR1_IRGN0        (1 << 6)  /* Bit 6:  Inner cacheability (with IRGN0) */
                                     /* Bits 7-13: Reserved */
#define TTBR1_BASE_SHIFT   (14)      /* Bits 14-31: Translation table base 1 */
#define TTBR1_BASE_MASK    (0xffffc000)

/* Translation Table Base Control Register (TTBCR) */

#define TTBCR_N_SHIFT      (0)       /* Bits 0-2: Boundary size of TTBR0 */
#define TTBCR_N_MASK       (7 << TTBCR_N_SHIFT)
#  define TTBCR_N_16KB     (0 << TTBCR_N_SHIFT) /* Reset value */
#  define TTBCR_N_8KB      (1 << TTBCR_N_SHIFT)
#  define TTBCR_N_4KB      (2 << TTBCR_N_SHIFT)
#  define TTBCR_N_2KB      (3 << TTBCR_N_SHIFT)
#  define TTBCR_N_1KB      (4 << TTBCR_N_SHIFT)
#  define TTBCR_N_512B     (5 << TTBCR_N_SHIFT)
#  define TTBCR_N_256B     (6 << TTBCR_N_SHIFT)
#  define TTBCR_N_128B     (7 << TTBCR_N_SHIFT)
                                     /* Bit 3:  Reserved */
#define TTBCR_PD0          (1 << 4)  /* Bit 4:  Translation table walk on a TLB miss w/TTBR0 */
#define TTBCR_PD1          (1 << 5)  /* Bit 5:  Translation table walk on a TLB miss w/TTBR1 */
                                     /* Bits 6-31: Reserved */

/* Domain Access Control Register (DACR) */

#define DACR_SHIFT(n)      (1 << ((n) << 1)) /* Domain n, n=0-31 */
#define DACR_MASK(n)       (3 << DACR_SHIFT(n))
#  define DACR_NONE(n)     (0 << DACR_SHIFT(n)) /* Any access generates a domain fault */
#  define DACR_CLIENT(n)   (1 << DACR_SHIFT(n)) /* Accesses checked against permissions TLB */
#  define DACR_MANAGER(n)  (3 << DACR_SHIFT(n)) /* Accesses are not checked */

/* Data Fault Status Register (DFSR) */

#define DFSR_STATUS_SHIFT  (0)       /* Bits 0-3: Type of exception generated (w/EXT and FS) */
#define DFSR_STATUS_MASK   (15 << DFSR_STATUS_SHIFT)
#define DFSR_DOMAIN_SHIFT  (4)       /* Bits 4-7: Domain accessed when a data fault occurred */
#define DFSR_DOMAIN_MASK   (15 << DFSR_STATUS_MASK)
                                     /* Bits 8-9: Reserved */
#define DFSR_FS            (1 << 10) /* Bit 10: Part of the STATUS field */
#define DFSR_WNR           (1 << 11) /* Bit 11: Not read and write */
#define DFSR_EXT           (1 << 12) /* Bit 12: External Abort Qualifier */
                                     /* Bits 13-31: Reserved */

/* Instruction Fault Status Register (IFSR) */

#define IFSR_STATUS_SHIFT  (0)       /* Bits 0-3: Type of fault generated (w/EXT and FS) */
#define IFSR_STATUS_MASK   (15 << IFSR_STATUS_SHIFT)
                                     /* Bits 4-9: Reserved */
#define IFSR_S             (1 << 10) /* Bit 10: Part of the STATUS field */
                                     /* Bits 11: Reserved */
#define IFSR_EXT           (1 << 12) /* Bit 12: External Abort Qualifier */
                                     /* Bits 13-31: Reserved */

/* Data Fault Address Register(DFAR).  Holds the MVA of the faulting address when a
 * synchronous fault occurs
 *
 * Instruction Fault Address Register(IFAR).  Holds the MVA of the faulting address
 * of the instruction that caused a prefetch abort.
 */

/* TLB operations.
 *
 * CP15 Register: TLBIALLIS
 *   Description:     Invalidate entire Unified TLB Inner Shareable
 *   Register Format: SBZ
 *   Instruction:     MCR p15, 0, <Rd>, c8, c3, 0
 * CP15 Register: TLBIMVAIS
 *   Description:     Invalidate Unified TLB entry by VA Inner Shareable
 *   Register Format: VA/ASID
 *   Instruction:     MCR p15, 0, <Rd>, c8, c3, 1
 * CP15 Register: TLBIASIDIS
 *   Description:     Invalidate Unified TLB entry by ASID match Inner Shareable
 *   Register Format: ASID
 *   Instruction:     MCR p15, 0, <Rd>, c8, c3, 2
 * CP15 Register: TLBIMVAAIS
 *   Description:     Invalidate Unified TLB entry by VA all ASID Inner Shareable
 *   Register Format: VA
 *   Instruction:     MCR p15, 0, <Rd>, c8, c3, 3
 * CP15 Register: TLBIALL
 *   Description:     Invalidate entire Unified TLB  
 *   Register Format: Ignored
 *   Instruction:     MCR p15, 0, <Rd>, c8, c7, 0
 * CP15 Register: TLBIMVA
 *   Description:     Invalidate Unified TLB by VA
 *   Register Format: VA/ASID
 *   Instruction:     MCR p15, 0, <Rd>, c8, c7, 1
 * CP15 Register: TLBIASID
 *   Description:     Invalidate TLB entries by ASID Match 
 *   Register Format: ASID
 *   MCR p15, 0, <Rd>, c8, c7, 2
 * CP15 Register: TLBIMVAA
 *   Description:     Invalidate TLB entries by VA All ASID
 *   Register Format: VA
 *   Instruction:     MCR p15, 0, <Rd>, c8, c7, 3
 */

#define TLB_ASID_SHIFT     (0)       /* Bits 0-7:  Address Space Identifier */
#define TLB_ASID_MASK      (0xff << TLB_ASID_SHIFT)
#define TLB_SBZ_SHIFT      (8)       /* Bits 8-11:  SBZ */
#define TLB_SBZ_MASK       (15 << TLB_SBZ_SHIFT)
#define TLB_VA_MASK        (0xfffff000) /* Bits 12-31: Virtual address */

/* Primary Region Remap Register (PRRR) */
/* Normal Memory Remap Register (NMRR) */

/* TLB Hitmap Register (TLBHR) */

#define TLBHR_4KB          (1 << 0)  /* Bit 0:  4KB pages are present in the TLB */
#define TLBHR_16KB         (1 << 1)  /* Bit 1:  16KB pages are present in the TLB */
#define TLBHR_1MB          (1 << 2)  /* Bit 2:  1MB sections are present in the TLB */
#define TLBHR_16MB         (1 << 3)  /* Bit 3:  16MB supersections are present in the TLB */
                                     /* Bits 4-31: Reserved */

/* Context ID Register (CONTEXTIDR).  See cstlr.h */

/************************************************************************************
 * Assemby Macros
 ************************************************************************************/

#ifdef __ASSEMBLY__

/* Write the Domain Access Control Register (DACR) */

	.macro	cp15_wrdacr, dacr
	mcr		p15, 0, \dacr, c3, c0, 0
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	.endm

/* The  ARMv7-aA architecture supports two translation tables.  This
 * implementation, however, uses only translation table 0.  This
 * functions clears the TTB control register (TTBCR), indicating that
 * we are using TTB 0.  This is it writes the value of the page table
 * to Translation Table Base Register 0 (TTBR0).
 */

	.macro	cp14_wrttb, ttb, scratch
	mcr		p15, 0, \ttb, c2, c0, 0
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	mov		\scratch, #0x0
	mcr		p15, 0, \scratch, c2, c0, 2
	.endm

#endif /* __ASSEMBLY__ */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/* Write the Domain Access Control Register (DACR) */

static inline void cp15_wrdacr(unsigned int dacr)
{
  __asm__ __volatile__
    (
      "\tmcr p15, 0, $0, c3, c0, 0\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      :
      : "r" (dacr)
      : "memory"
    );
}

/* The  ARMv7-aA architecture supports two translation tables.  This
 * implementation, however, uses only translation table 0.  This
 * functions clears the TTB control register (TTBCR), indicating that
 * we are using TTB 0.  This is it writes the value of the page table
 * to Translation Table Base Register 0 (TTBR0).
 */

static inline void cp14_wrttb(unsigned int ttb)
{
  __asm__ __volatile__
    (
      "\tmcr p15, 0, $0, c2, c0, 0\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tnop\n"
      "\tmov r1, #0\n"
      "\tmcr p15, 0, r1, c2, c0, 2\n"
      :
      : "r" (ttb)
      : "r1", "memory"
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
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif  /* __ARCH_ARM_SRC_ARMV7_A_MMU_H */
