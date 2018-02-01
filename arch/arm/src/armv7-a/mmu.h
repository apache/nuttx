/************************************************************************************
 * arch/arm/src/armv7-a/mmu.h
 * CP15 MMU register definitions
 *
 *   Copyright (C) 2013-2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *
 *  "Cortex-A5™ MPCore, Technical Reference Manual", Revision: r0p1, Copyright ©
 *   2010 ARM. All rights reserved. ARM DDI 0434B (ID101810)
 *  "ARM® Architecture Reference Manual, ARMv7-A and ARMv7-R edition", Copyright ©
 *   1996-1998, 2000, 2004-2012 ARM. All rights reserved. ARM
 *   DDI 0406C.b (ID072512)
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

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <sys/types.h>
#  include <stdint.h>
#  include "chip.h"
#endif /* __ASSEMBLY__ */

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#if defined(CONFIG_PAGING) || defined(CONFIG_ARCH_ADDRENV)

/* Sanity check -- we cannot be using a ROM page table and supporting on-
 * demand paging.
 */

#ifdef CONFIG_ARCH_ROMPGTABLE
#  error "Cannot support both CONFIG_PAGING/CONFIG_ARCH_ADDRENV and CONFIG_ARCH_ROMPGTABLE"
#endif
#endif /* CONFIG_PAGING */

/* MMU CP15 Register Bit Definitions ************************************************/
/* Reference: Cortex-A5™ MPCore Paragraph 6.7, "MMU software accessible registers." */

/* TLB Type Register TLB Type Register
 *
 * The Translation Lookaside Buffer (TLB) Type Register, TLBTR, returns the number of
 * lockable entries for the TLB. The Cortex-A5 MPCore processor does not implement
 * this feature, so this register always RAZ.
 */

/* System Control Register (SCTLR). see cstlr.h */
/* Non-secure Access Control Register (NSACR).  See cstlr.h */

/* Translation Table Base Register 0 (TTBR0)*/

#define TTBR0_IRGN1          (1 << 0)  /* Bit 0:  Inner cacheability IRGN[1] (MP extensions) */
#define TTBR0_C              (1 << 0)  /* Bit 0:  Inner cacheability for table walk */
#define TTBR0_S              (1 << 1)  /* Bit 1:  Translation table walk */
                                       /* Bit 2:  Reserved */
#define TTBR0_RGN_SHIFT      (3)       /* Bits 3-4: Outer cacheable attributes for table walk */
#define TTBR0_RGN_MASK       (3 << TTBR0_RGN_SHIFT)
#  define TTBR0_RGN_NONE     (0 << TTBR0_RGN_SHIFT) /* Non-cacheable */
#  define TTBR0_RGN_WBWA     (1 << TTBR0_RGN_SHIFT) /* Write-Back cached + Write-Allocate */
#  define TTBR0_RGN_WT       (2 << TTBR0_RGN_SHIFT) /* Write-Through */
#  define TTBR0_RGN_WB       (3 << TTBR0_RGN_SHIFT) /* Write-Back */
#define TTBR0_NOS            (1 << 5)  /* Bit 5:  Not Outer Shareable bit */
#define TTBR0_IRGN0          (1 << 6)  /* Bit 6:  Inner cacheability IRGN[0] (MP extensions) */
                                       /* Bits 7-n: Reserved, n=7-13 */
#define _TTBR0_LOWER(n)      (0xffffffff << (n))
                                       /* Bits (n+1)-31: Translation table base 0 */
#define TTBR0_BASE_MASK(n)   (~_TTBR0_LOWER(n))

/* Translation Table Base Register 1 (TTBR1) */

#define TTBR1_IRGN1          (1 << 0)  /* Bit 0:  Inner cacheability IRGN[1] (MP extensions) */
#define TTBR1_C              (1 << 0)  /* Bit 0:  Inner cacheability for table walk */
#define TTBR1_S              (1 << 1)  /* Bit 1:  Translation table walk */
                                       /* Bit 2:  Reserved */
#define TTBR1_RGN_SHIFT      (3)       /* Bits 3-4: Outer cacheable attributes for table walk */
#define TTBR1_RGN_MASK       (3 << TTBR1_RGN_SHIFT)
#  define TTBR1_RGN_NONE     (0 << TTBR1_RGN_SHIFT) /* Non-cacheable */
#  define TTBR1_RGN_WBWA     (1 << TTBR1_RGN_SHIFT) /* Write-Back cached + Write-Allocate */
#  define TTBR1_RGN_WT       (2 << TTBR1_RGN_SHIFT) /* Write-Through */
#  define TTBR1_RGN_WB       (3 << TTBR1_RGN_SHIFT) /* Write-Back */
#define TTBR1_NOS            (1 << 5)  /* Bit 5:  Not Outer Shareable bit */
#define TTBR1_IRGN0          (1 << 6)  /* Bit 6:  Inner cacheability IRGN[0] (MP extensions) */
                                       /* Bits 7-13: Reserved */
#define TTBR1_BASE_SHIFT     (14)      /* Bits 14-31: Translation table base 1 */
#define TTBR1_BASE_MASK      (0xffffc000)

/* Translation Table Base Control Register (TTBCR) */

#define TTBCR_N_SHIFT        (0)       /* Bits 0-2: Boundary size of TTBR0 */
#define TTBCR_N_MASK         (7 << TTBCR_N_SHIFT)
#  define TTBCR_N_16KB       (0 << TTBCR_N_SHIFT) /* Reset value */
#  define TTBCR_N_8KB        (1 << TTBCR_N_SHIFT)
#  define TTBCR_N_4KB        (2 << TTBCR_N_SHIFT)
#  define TTBCR_N_2KB        (3 << TTBCR_N_SHIFT)
#  define TTBCR_N_1KB        (4 << TTBCR_N_SHIFT)
#  define TTBCR_N_512B       (5 << TTBCR_N_SHIFT)
#  define TTBCR_N_256B       (6 << TTBCR_N_SHIFT)
#  define TTBCR_N_128B       (7 << TTBCR_N_SHIFT)
                                       /* Bit 3:  Reserved */
#define TTBCR_PD0            (1 << 4)  /* Bit 4:  Translation table walk on a TLB miss w/TTBR0 */
#define TTBCR_PD1            (1 << 5)  /* Bit 5:  Translation table walk on a TLB miss w/TTBR1 */
                                       /* Bits 6-31: Reserved */

/* Domain Access Control Register (DACR) */

#define DACR_SHIFT(n)        ((n) << 1) /* Domain n, n=0-15 */
#define DACR_MASK(n)         (3 << DACR_SHIFT(n))
#  define DACR_NONE(n)       (0 << DACR_SHIFT(n)) /* Any access generates a domain fault */
#  define DACR_CLIENT(n)     (1 << DACR_SHIFT(n)) /* Accesses checked against permissions TLB */
#  define DACR_MANAGER(n)    (3 << DACR_SHIFT(n)) /* Accesses are not checked */

/* Data Fault Status Register (DFSR) */

#define DFSR_STATUS_SHIFT    (0)       /* Bits 0-3: Type of exception generated (w/EXT and FS) */
#define DFSR_STATUS_MASK     (15 << DFSR_STATUS_SHIFT)
#define DFSR_DOMAIN_SHIFT    (4)       /* Bits 4-7: Domain accessed when a data fault occurred */
#define DFSR_DOMAIN_MASK     (15 << DFSR_STATUS_MASK)
                                       /* Bits 8-9: Reserved */
#define DFSR_FS              (1 << 10) /* Bit 10: Part of the STATUS field */
#define DFSR_WNR             (1 << 11) /* Bit 11: Not read and write */
#define DFSR_EXT             (1 << 12) /* Bit 12: External Abort Qualifier */
                                       /* Bits 13-31: Reserved */

/* Instruction Fault Status Register (IFSR) */

#define IFSR_STATUS_SHIFT    (0)       /* Bits 0-3: Type of fault generated (w/EXT and FS) */
#define IFSR_STATUS_MASK     (15 << IFSR_STATUS_SHIFT)
                                       /* Bits 4-9: Reserved */
#define IFSR_S               (1 << 10) /* Bit 10: Part of the STATUS field */
                                       /* Bits 11: Reserved */
#define IFSR_EXT             (1 << 12) /* Bit 12: External Abort Qualifier */
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
 *   Description:     Invalidate Unified TLB entry by ASID match Inner
 *                    Shareable
 *   Register Format: ASID
 *   Instruction:     MCR p15, 0, <Rd>, c8, c3, 2
 * CP15 Register: TLBIMVAAIS
 *   Description:     Invalidate Unified TLB entry by VA all ASID Inner
 *                    Shareable
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

#define TLB_ASID_SHIFT       (0)       /* Bits 0-7:  Address Space Identifier */
#define TLB_ASID_MASK        (0xff << TLB_ASID_SHIFT)
#define TLB_SBZ_SHIFT        (8)       /* Bits 8-11:  SBZ */
#define TLB_SBZ_MASK         (15 << TLB_SBZ_SHIFT)
#define TLB_VA_MASK          (0xfffff000) /* Bits 12-31: Virtual address */

/* Primary Region Remap Register (PRRR) */
/* Normal Memory Remap Register (NMRR) */

/* TLB Hitmap Register (TLBHR) */

#define TLBHR_4KB            (1 << 0)  /* Bit 0:  4KB pages are present in the TLB */
#define TLBHR_16KB           (1 << 1)  /* Bit 1:  16KB pages are present in the TLB */
#define TLBHR_1MB            (1 << 2)  /* Bit 2:  1MB sections are present in the TLB */
#define TLBHR_16MB           (1 << 3)  /* Bit 3:  16MB supersections are present in the TLB */
                                       /* Bits 4-31: Reserved */

/* Context ID Register (CONTEXTIDR).  See cstlr.h */

/* Translation Table Definitions ****************************************************/
/* Hardware translation table definitions.  Only the "short descriptor format" is
 * supported.
 *
 * Level 1 Descriptor (PMD)
 *
 * Common definitions that apply to all L1 table entry types
 */

#define PMD_TYPE_SHIFT       (0)         /* Bits: 1:0:  Type of mapping */
#define PMD_TYPE_MASK        (3 << PMD_TYPE_SHIFT)
#  define PMD_TYPE_FAULT     (0 << PMD_TYPE_SHIFT) /* None */
#  define PMD_TYPE_PTE       (1 << PMD_TYPE_SHIFT) /* Page table */
#  define PMD_TYPE_SECT      (2 << PMD_TYPE_SHIFT) /* Section or supersection */
#  define PMD_TYPE_PXN       (3 << PMD_TYPE_SHIFT) /* PXN Section or supersection */
                                        /* Bits 2-31: Depend on the mapping type */

/* Level 1 Fault Translation Table Format.
 *
 * Invalid or fault entry.  "The associated VA is unmapped, and any attempt to
 *   access it generates a Translation fault.  Software can use bits[31:2] of the
 *   descriptor for its own purposes, because the hardware ignores
 *   these bits."
 */

/* Level 1 Page Table Translation Table Format.
 *
 * Page table. "The descriptor gives the address of a second-level translation
 *   table, that specifies the mapping of the associated 1MByte VA range."
 */

                                          /* Bits 0-1:   Type of mapping */
#define PMD_PTE_PXN          (1 << 2)     /* Bit 2:  Privileged execute-never bit */
#define PMD_PTE_NS           (1 << 3)     /* Bit 3:  Non-secure bit */
                                          /* Bit 4:  Should be zero (SBZ) */
#define PMD_PTE_DOM_SHIFT    (5)          /* Bits 5-8: Domain */
#define PMD_PTE_DOM_MASK     (15 << PMD_PTE_DOM_SHIFT)
#  define PMD_PTE_DOM(n)     ((n) << PMD_PTE_DOM_SHIFT)
                                          /* Bit 9:  Not implemented */
#define PMD_PTE_PADDR_MASK   (0xfffffc00) /* Bits 10-31: Page table base address */

/* Level 1 Section/Supersection Descriptor.
 *
 * Section or Supersection.  "The descriptor gives the base address of the
 *   Section or Supersection. Bit[18] determines whether the entry describes a
 *   Section or a Supersection.  If the implementation supports the PXN
 *   attribute, this encoding also defines the PXN bit as 0. Section descriptors
 *   allow fast, single level mapping between 1Mb address regions."

 * PXN Section or Supersection.  "If an implementation supports the PXN attribute,
 *   this encoding is identical..., except that it defines the PXN bit as 1.
 *
 *  "If the implementation does not support the PXN attribute, an attempt to access
 *   the associated VA generates a Translation fault.  On an implementation that
 *   does not support the PXN attribute, this encoding must not be used."
 */

/* Section */

#define PMD_SECT_PXN         (1 << 0)     /* Bit 0:  Privileged execute-never bit */
                                          /* Bits 0-1: Type of mapping */
#define PMD_SECT_B           (1 << 2)     /* Bit 2:  Bufferable bit */
#define PMD_SECT_C           (1 << 3)     /* Bit 3:  Cacheable bit */
#define PMD_SECT_XN          (1 << 4)     /* Bit 4:  Execute-never bit */
#define PMD_SECT_DOM_SHIFT   (5)          /* Bits 5-8: Domain */
#define PMD_SECT_DOM_MASK    (15 << PMD_SECT_DOM_SHIFT)
#  define PMD_SECT_DOM(n)    ((n) << PMD_SECT_DOM_SHIFT)
                                          /* Bit 9:  Implementation defined */
#define PMD_SECT_AP_SHIFT    (10)         /* Bits 10-11: Access Permissions bits AP[0:1] */
#define PMD_SECT_AP_MASK     (3 << PMD_SECT_AP_SHIFT)
#  define PMD_SECT_AP0       (1 << PMD_SECT_AP_SHIFT) /* AP[0]:  Access permission bit 0 */
#  define PMD_SECT_AP1       (2 << PMD_SECT_AP_SHIFT) /* AP[1]:  Access permission bit 1 */
#define PMD_SECT_TEX_SHIFT   (12)         /* Bits 12-14: Memory region attribute bits */
#define PMD_SECT_TEX_MASK    (7 << PMD_SECT_TEX_SHIFT)
#define PMD_SECT_AP2         (1 << 15)    /* Bit 15: AP[2]:  Access permission bit 2 */
#define PMD_SECT_S           (1 << 16)    /* Bit 16: Shareable bit */
#define PMD_SECT_NG          (1 << 17)    /* Bit 17: Not global bit. */
#define PMD_SECT_PADDR_MASK  (0xfff00000) /* Bits 20-31: Section base address, PA[31:20] */

/* Super Section (differences only) */

#define PMD_SSECT_XBA3_SHIFT  (5)          /* Bits 24-31: Extended base address, PA[39:36] */
#define PMD_SSECT_XBA3_MASK   (15 << PMD_SSECT_XBA3_SHIFT)
#define PMD_SSECT_XBA2_SHIFT  (5)          /* Bits 20-23: Extended base address, PA[35:32] */
#define PMD_SSECT_XBA2_MASK   (15 << PMD_SSECT_XBA2_SHIFT)
#define PMD_SSECT_XBA1_SHIFT  (5)          /* Bits 24-31: Extended base address, PA[31:24] */
#define PMD_SSECT_XBA1_MASK   (15 << PMD_SSECT_XBA1_SHIFT)

/* Level 1 Section/Supersection Access Permissions.
 *
 * Paragraph B3.7.1, Access permissions: "If address translation is using
 * the Short-descriptor translation table format, it must set SCTLR.AFE to
 * 1 to enable use of the Access flag.... Setting this bit to 1 redefines
 * the AP[0] bit in the translation table descriptors as an Access flag, and
 * limits the access permissions information in the translation table
 * descriptors to AP[2:1]...
 *
 * Key:
 *
 *   WR    - Read/write address allowed
 *   R     - Read-only access allowed
 *   0,1,2 - At PL0, PL1, and/or PL2
 *
 *   PL0   - User privilege level
 *   PL1   - Privileged mode
 *   PL2   - Software executing in Hyp mode
 */

#ifdef CPU_AFE_ENABLE
/* AP[2:1] access permissions model.  AP[0] is used as an access flag:
 *
 * AP[2] AP[1]   PL1        PL0        Description
 * ----- ----- ----------- ---------- --------------------------------
 *   0     0   Read/write  No access  Access only at PL1
 *   0     1   Read/write  Read/write Full access
 *   1     0   Read-only   No access  Read-only for PL1
 *   1     1   Read-only   Read-only  Read-only at any privilege level
 */

#  define PMD_SECT_AP_RW1     (0)
#  define PMD_SECT_AP_RW01    (PMD_SECT_AP1)
#  define PMD_SECT_AP_R1      (PMD_SECT_AP2)
#  define PMD_SECT_AP_R01     (PMD_SECT_AP1 | PMD_SECT_AP2)

#else
/* AP[2:0] access permissions control, Short-descriptor format only:
 *
 * AP[2] AP[1] AP[0]  PL1/2       PL0        Description
 * ----- ----- ----- ----------- ---------- --------------------------------
 *   0     0     0   No access   No access  All accesses generate faults
 *   0     0     1   Read/write  No access  Access only at PL1 and higher
 *   0     1     0   Read/write  Read-only  Writes at PL0 generate faults
 *   0     1     1   Read/write  Read/write Full access
 *   1     0     0     ----        ---      Reserved
 *   1     0     1   Read-only   No access  Read-only for PL1 and higher
 *   1     1     0   Read-only   Read-only  (deprecated)
 *   1     1     1   Read-only   Read-only  Read-only at any privilege level
 */

#  define PMD_SECT_AP_NONE    (0)
#  define PMD_SECT_AP_RW12    (PMD_SECT_AP0)
#  define PMD_SECT_AP_RW12_R0 (PMD_SECT_AP1)
#  define PMD_SECT_AP_RW012   (PMD_SECT_AP0 | PMD_SECT_AP1)
#  define PMD_SECT_AP_R12     (PMD_SECT_AP0 | PMD_SECT_AP2)
#  define PMD_SECT_AP_R012    (PMD_SECT_AP0 | PMD_SECT_AP1 | PMD_SECT_AP2)

/* Some mode-independent aliases */

#  define PMD_SECT_AP_RW1     PMD_SECT_AP_RW12
#  define PMD_SECT_AP_RW01    PMD_SECT_AP_RW012
#  define PMD_SECT_AP_R1      PMD_SECT_AP_R12
#  define PMD_SECT_AP_R01     PMD_SECT_AP_R012

#endif

/* Short-descriptor translation table second-level descriptor formats
 *
 * A PMD_TYPE_PTE level-one table entry provides the base address of the beginning
 * of a second-level page table. There are two types of page table entries:
 *
 *   - Large page table entries support mapping of 64KB memory regions.
 *   - Small page table entries support mapping of 4KB memory regions.
 *
 * The following definitions apply to all L2 tables:
 */

#define PTE_TYPE_SHIFT       (0)          /* Bits: 1:0:  Type of mapping */
#define PTE_TYPE_MASK        (3 << PTE_TYPE_SHIFT)
#  define PTE_TYPE_FAULT     (0 << PTE_TYPE_SHIFT) /* None */
#  define PTE_TYPE_LARGE     (1 << PTE_TYPE_SHIFT) /* 64Kb of memory */
#  define PTE_TYPE_SMALL     (2 << PTE_TYPE_SHIFT) /*  4Kb of memory */
#define PTE_B                (1 << 2)     /* Bit 2:  Bufferable bit */
#define PTE_C                (1 << 3)     /* Bit 3:  Cacheable bit */
#define PTE_AP_SHIFT         (4)          /* Bits 4-5: Access Permissions bits AP[0:1] */
#define PTE_AP_MASK          (3 << PTE_AP_SHIFT)
#  define PTE_AP0            (1 << PTE_AP_SHIFT)   /* AP[0]:  Access permission bit 0 */
#  define PTE_AP1            (2 << PTE_AP_SHIFT)   /* AP[1]:  Access permission bit 1 */
                                          /* Bits 6-8: Depend on entry type */
#define PTE_AP2              (1 << 9)     /* Bit 9: AP[2]:  Access permission bit 2 */
#define PTE_S                (1 << 10)    /* Bit 10: Shareable bit */
#define PTE_NG               (1 << 11)    /* Bit 11: Not global bit. */
                                          /* Bits 12-31:Depend on entry type */

/* Large page -- 64Kb */
                                          /* Bits: 1:0:  Type of mapping */
                                          /* Bit 2:  Bufferable bit */
                                          /* Bit 3:  Cacheable bit */
                                          /* Bits 4-5: Access Permissions bits AP[0:1] */
#define PTE_LARGE_TEX_SHIFT  (12)         /* Bits 12-14: Memory region attribute bits */
#define PTE_LARGE_TEX_MASK   (7 << PTE_LARGE_TEX_SHIFT)
#define PTE_LARGE_XN         (1 << 15)    /* Bit 15: Execute-never bit */
#define PTE_LARGE_FLAG_MASK  (0x0000f03f) /* Bits 0-15: MMU flags (mostly) */
#define PTE_LARGE_PADDR_MASK (0xffff0000) /* Bits 16-31: Large page base address, PA[31:16] */

/* Small page -- 4Kb */

                                          /* Bits: 1:0:  Type of mapping */
                                          /* Bit 2:  Bufferable bit */
                                          /* Bit 3:  Cacheable bit */
                                          /* Bits 4-5: Access Permissions bits AP[0:1] */
#define PTE_SMALL_FLAG_MASK  (0x0000003f) /* Bits 0-11: MMU flags (mostly) */
#define PTE_SMALL_PADDR_MASK (0xfffff000) /* Bits 12-31: Small page base address, PA[31:12] */

/* Level 2 Translation Table Access Permissions:
 *
 * WR    - Read/write access allowed
 * R     - Read-only access allowed
 * 0,1,2 - At PL0, PL1, and/or PL2
 *
 * PL0   - User privilege level
 * PL1   - Privileged mode
 * PL2   - Software executing in Hyp mode
 */

#ifdef CONFIG_AFE_ENABLE
/* AP[2:1] access permissions model.  AP[0] is used as an access flag:
 *
 * AP[2] AP[1]   PL1        PL0        Description
 * ----- ----- ----------- ---------- --------------------------------
 *   0     0   Read/write  No access  Access only at PL1
 *   0     1   Read/write  Read/write Full access
 *   1     0   Read-only   No access  Read-only for PL1
 *   1     1   Read-only   Read-only  Read-only at any privilege level
 */

#  define PTE_AP_RW1         (0)
#  define PTE_AP_RW01        (PTE_AP1)
#  define PTE_AP_R1          (PTE_AP2)
#  define PTE_AP_R01         (PTE_AP1 | PTE_AP2)

#else
/* AP[2:0] access permissions control, Short-descriptor format only:
 *
 * AP[2] AP[1] AP[0]  PL1/2       PL0        Description
 * ----- ----- ----- ----------- ---------- --------------------------------
 *   0     0     0   No access   No access  All accesses generate faults
 *   0     0     1   Read/write  No access  Access only at PL1 and higher
 *   0     1     0   Read/write  Read-only  Writes at PL0 generate faults
 *   0     1     1   Read/write  Read/write Full access
 *   1     0     0     ----        ---      Reserved
 *   1     0     1   Read-only   No access  Read-only for PL1 and higher
 *   1     1     0   Read-only   Read-only  (deprecated)
 *   1     1     1   Read-only   Read-only  Read-only at any privilege level
 */

#  define PTE_AP_NONE        (0)
#  define PTE_AP_RW12        (PTE_AP0)
#  define PTE_AP_RW12_R0     (PTE_AP1)
#  define PTE_AP_RW012       (PTE_AP0 | PTE_AP1)
#  define PTE_AP_R12         (PTE_AP0 | PTE_AP2)
#  define PTE_AP_R012        (PTE_AP0 | PTE_AP1 | PTE_AP2)

/* Some mode-independent aliases */

#  define PTE_AP_RW1         PTE_AP_RW12
#  define PTE_AP_RW01        PTE_AP_RW012
#  define PTE_AP_R1          PTE_AP_R12
#  define PTE_AP_R01         PTE_AP_R012

#endif

/* Memory types
 *
 * When TEX[2] == 1, the memory region is cacheable memory, and TEX[1:0]
 * describe inner and outer cache attributes.  In this implementation,
 * however, TEX[2:0] are always zero.  In this case, the cacheability is
 * described simply as:
 *
 *  C B Memory Type
 *  - - ---------------------------------------------------------------
 *  0 0 Strongly-ordered. Strongly-ordered Shareable
 *  0 1 Shareable Device. Device Shareable
 *  1 0 Outer and Inner Write-Through, no Write-Allocate. Normal S bit
 *  1 1 Outer and Inner Write-Back, no Write-Allocate. Normal S bit
 *
 * The memory type is actually controlled by the contents of the PRRR and
 * NMRR registers.  For the simple case where TEX[2:0] = 0b000, the control
 * is as follows:
 *
 *       MEMORY     INNER         OUTER        OUTER SHAREABLE
 *   C B TYPE       CACHEABILITY  CACHEABILITY ATTRIBUTE
 *   - - ---------- ------------- ------------ -----------------
 *   0 0 PRRR[1:0]  NMRR[1:0]     NMRR[17:16]  NOT(PRRR[24])
 *   0 1 PRRR[3:2]  NMRR[3:2]     NMRR[19:18]  NOT(PRRR[25])
 *   1 0 PRRR[5:4]  NMRR[5:4]     NMRR[21:20]  NOT(PRRR[26])
 *   1 1 PRRR[7:6]  NMRR[7:6]     NMRR[23:22]  NOT(PRRR[27])
 *
 * But on reset I see the following in PRRR:
 *
 *   PRRR[1:0]   = 0b00, Strongly ordered memory
 *   PRRR[3:2]   = 0b01, Device memory
 *   PRRR[5:4]   = 0b10, Normal memory
 *   PRRR[7:6]   = 0b10, Normal memory
 *   PRRR[14:27] = 0b10, Outer shareable
 *
 * And the following in NMRR:
 *
 *   NMRR[1:0]   = 0b00, Region is Non-cacheable
 *   NMRR[3:2]   = 0b00, Region is Non-cacheable
 *   NMRR[5:4]   = 0b10, Region is Write-Through, no Write-Allocate
 *   NMRR[7:6]   = 0b11, Region is Write-Back, no Write-Allocate
 *   NMRR[17:16] = 0b00, Region is Non-cacheable
 *   NMRR[19:18] = 0b00, Region is Non-cacheable
 *   NMRR[21:20] = 0b10, Region is Write-Through, no Write-Allocate
 *   NMRR[23:22] = 0b11, Region is Write-Back, no Write-Allocate
 *
 * Interpretation of Cacheable (C) and Bufferable (B) Bits:
 *
 *         Write-Through  Write-Back    Write-Through/Write-Back
 *  C   B  Cache          Only Cache    Cache
 * --- --- -------------- ------------- -------------------------
 *  0   0  Uncached/      Uncached/     Uncached/
 *         Unbuffered     Unbuffered    Unbuffered
 *  0   1  Uncached/      Uncached/     Uncached/
 *         Buffered       Buffered      Buffered
 *  1   0  Cached/        UNPREDICTABLE Write-Through cached
 *         Unbuffered                   Buffered
 *  1   1  Cached/        Cached/       Write-Back cached
 *         Buffered       Buffered      Buffered
 */

#define PMD_STRONGLY_ORDERED (0)
#define PMD_DEVICE           (PMD_SECT_B)
#define PMD_CACHEABLE        (PMD_SECT_B | PMD_SECT_C)

#define PTE_STRONGLY_ORDER   (0)
#define PTE_DEVICE           (PTE_B)
#define PTE_WRITE_THROUGH    (PTE_C)
#define PTE_WRITE_BACK       (PTE_B | PTE_C)

/* Default MMU flags for RAM memory, IO, vector sections (level 1)
 *
 * REVISIT:  Here we expect all threads to be running at PL1
 */

#define MMU_ROMFLAGS         (PMD_TYPE_SECT | PMD_SECT_AP_R1 | PMD_CACHEABLE | \
                              PMD_SECT_DOM(0))
#define MMU_MEMFLAGS         (PMD_TYPE_SECT | PMD_SECT_AP_RW1 | PMD_CACHEABLE | \
                              PMD_SECT_DOM(0))
#define MMU_IOFLAGS          (PMD_TYPE_SECT | PMD_SECT_AP_RW1 | PMD_DEVICE | \
                              PMD_SECT_DOM(0) | PMD_SECT_XN)
#define MMU_STRONGLY_ORDERED (PMD_TYPE_SECT | PMD_SECT_AP_RW1 | \
                              PMD_STRONGLY_ORDERED | PMD_SECT_DOM(0) | \
                              PMD_SECT_XN)

/* MMU Flags for each type memory region (level 1 and 2) */

#define MMU_L1_TEXTFLAGS      (PMD_TYPE_PTE | PMD_PTE_DOM(0))

#define MMU_L2_KTEXTFLAGS     (PTE_TYPE_SMALL | PTE_WRITE_BACK | PTE_AP_R1)
#ifdef CONFIG_AFE_ENABLE
#  define MMU_L2_UTEXTFLAGS   (PTE_TYPE_SMALL | PTE_WRITE_BACK | PTE_AP_RW01)
#else
#  define MMU_L2_UTEXTFLAGS   (PTE_TYPE_SMALL | PTE_WRITE_BACK | PTE_AP_RW12_R0)
#endif

#define MMU_L1_DATAFLAGS      (PMD_TYPE_PTE | PMD_PTE_PXN | PMD_PTE_DOM(0))
#define MMU_L2_UDATAFLAGS     (PTE_TYPE_SMALL | PTE_WRITE_BACK | PTE_AP_RW01)
#define MMU_L2_KDATAFLAGS     (PTE_TYPE_SMALL | PTE_WRITE_BACK | PTE_AP_RW1)
#define MMU_L2_UALLOCFLAGS    (PTE_TYPE_SMALL | PTE_WRITE_BACK | PTE_AP_RW01)
#define MMU_L2_KALLOCFLAGS    (PTE_TYPE_SMALL | PTE_WRITE_BACK | PTE_AP_RW1)

#define MMU_L1_PGTABFLAGS     (PMD_TYPE_PTE | PMD_PTE_PXN | PTE_WRITE_THROUGH | \
                               PMD_PTE_DOM(0))
#define MMU_L2_PGTABFLAGS     (PTE_TYPE_SMALL | PTE_WRITE_THROUGH | PTE_AP_RW1)

#define MMU_L1_VECTORFLAGS    (PMD_TYPE_PTE | PMD_PTE_PXN | PMD_PTE_DOM(0))
#define MMU_L2_VECTRWFLAGS    (PTE_TYPE_SMALL | PTE_WRITE_THROUGH | PTE_AP_RW1)
#define MMU_L2_VECTROFLAGS    (PTE_TYPE_SMALL | PTE_WRITE_THROUGH | PTE_AP_R1)
#define MMU_L2_VECTORFLAGS    MMU_L2_VECTRWFLAGS

/* Mapped section size */

#define SECTION_SHIFT         (20)
#define SECTION_SIZE          (1 << SECTION_SHIFT)   /* 1Mb */
#define SECTION_MASK          (SECTION_SIZE - 1)

/* The Cortex-A5 supports two translation table base address registers.  In
 * this, implementation, only Translation Table Base Register 0 (TTBR0) is
 * used.  The TTBR0 contains the upper bits of the address a a page table in
 * physical memory. If 4KB page sizes are used, then TTBR0 registers holds
 * bits 14-31 of the page table address;  A full 30-bit address is formed by
 * ORing in bits 2-13 or the virtual address (MVA).  As a consequence, the
 * page table must be aligned to a 16Kb address in physical memory and could
 * require up to 16Kb of memory.
 */

#define PGTABLE_SIZE       0x00004000

/* Virtual Page Table Location ******************************************************/

#ifdef CONFIG_PAGING
/* Check if the virtual address of the page table has been defined. It
 * should not be defined:  architecture specific logic should suppress
 * defining PGTABLE_BASE_VADDR unless:  (1) it is defined in the NuttX
 * configuration file, or (2) the page table is position in low memory
 * (because the vectors are in high memory).
 */

#ifndef PGTABLE_BASE_VADDR
#  define PGTABLE_BASE_VADDR      (PG_LOCKED_VBASE + PG_TEXT_VSIZE + PG_DATA_SIZE)

  /* Virtual base of the address of the L2 page tables need to recalculates
   * using this new virtual base address of the L2 page table.
   */

#  undef  PGTABLE_L2_VBASE
#  define PGTABLE_L2_VBASE (PGTABLE_BASE_VADDR+PGTABLE_L2_OFFSET)

#endif /* PGTABLE_BASE_VADDR */

/* MMU flags ************************************************************************/

/* Create some friendly definitions to handle page table entries */

#if CONFIG_PAGING_PAGESIZE != 4096
#  error "Unsupported value for CONFIG_PAGING_PAGESIZE"
#endif

/* Base of the L2 page table (aligned to 1Kb byte boundaries) */

#define PGTABLE_L2_BASE_PADDR PGTABLE_L2_PBASE
#define PGTABLE_L2_BASE_VADDR PGTABLE_L2_VBASE

/* Number of pages in an L2 table per L1 entry */

#define PTE_NPAGES            PTE_SMALL_NPAGES
#define PT_SIZE               (4*PTE_NPAGES)

/* Mask to get the page table physical address from an L1 entry */

#define PG_L1_PADDRMASK       PMD_SECT_PADDR_MASK

/* Addresses of Memory Regions ******************************************************/

/* We position the locked region PTEs at an offset into the first
 * L2 page table.  The L1 entry points to an 1Mb aligned virtual
 * address.  The actual L2 entry will be offset into the aligned
 * L2 table.  For 4KB, "small" pages:
 *
 *   PG_L1_PADDRMASK=0xfffff000
 *   OFFSET=(((a) & 0x000fffff) >> 10) << 2)
 */

#define PG_L1_LOCKED_PADDR      (PGTABLE_BASE_PADDR + ((PG_LOCKED_VBASE >> 20) << 2))
#define PG_L1_LOCKED_VADDR      (PGTABLE_BASE_VADDR + ((PG_LOCKED_VBASE >> 20) << 2))

#define PG_L2_LOCKED_OFFSET     (((PG_LOCKED_VBASE & 0x000fffff) >> PAGESHIFT) << 2)
#define PG_L2_LOCKED_PADDR      (PGTABLE_L2_BASE_PADDR + PG_L2_LOCKED_OFFSET)
#define PG_L2_LOCKED_VADDR      (PGTABLE_L2_BASE_VADDR + PG_L2_LOCKED_OFFSET)
#define PG_L2_LOCKED_SIZE       (4*CONFIG_PAGING_NLOCKED)

/* We position the paged region PTEs immediately after the locked
 * region PTEs.  NOTE that the size of the paged regions is much
 * larger than the size of the physical paged region.  That is the
 * core of what the On-Demanding Paging feature provides.
 */

#define PG_L1_PAGED_PADDR       (PGTABLE_BASE_PADDR + ((PG_PAGED_VBASE >> 20) << 2))
#define PG_L1_PAGED_VADDR       (PGTABLE_BASE_VADDR + ((PG_PAGED_VBASE >> 20) << 2))

#define PG_L2_PAGED_PADDR       (PG_L2_LOCKED_PADDR + PG_L2_LOCKED_SIZE)
#define PG_L2_PAGED_VADDR       (PG_L2_LOCKED_VADDR + PG_L2_LOCKED_SIZE)
#define PG_L2_PAGED_SIZE        (4*CONFIG_PAGING_NVPAGED)

/* This describes the overall text region */

#define PG_L1_TEXT_PADDR        PG_L1_LOCKED_PADDR
#define PG_L1_TEXT_VADDR        PG_L1_LOCKED_VADDR

#define PG_L2_TEXT_PADDR        PG_L2_LOCKED_PADDR
#define PG_L2_TEXT_VADDR        PG_L2_LOCKED_VADDR
#define PG_L2_TEXT_SIZE         (PG_L2_LOCKED_SIZE + PG_L2_PAGED_SIZE)

/* We position the data section PTEs just after the text region PTE's */

#define PG_L1_DATA_PADDR        (PGTABLE_BASE_PADDR + ((PG_DATA_VBASE >> 20) << 2))
#define PG_L1_DATA_VADDR        (PGTABLE_BASE_VADDR + ((PG_DATA_VBASE >> 20) << 2))

#define PG_L2_DATA_PADDR        (PG_L2_LOCKED_PADDR + PG_L2_TEXT_SIZE)
#define PG_L2_DATA_VADDR        (PG_L2_LOCKED_VADDR + PG_L2_TEXT_SIZE)
#define PG_L2_DATA_SIZE         (4*PG_DATA_NPAGES)

/* Page Table Info ******************************************************************/

/* The number of pages in the in the page table (PG_PGTABLE_NPAGES).  We
 * position the page table PTEs just after the data section PTEs.
 */

#define PG_PGTABLE_NPAGES       (PGTABLE_SIZE >> PAGESHIFT)
#define PG_L1_PGTABLE_PADDR     (PGTABLE_BASE_PADDR + ((PGTABLE_BASE_VADDR >> 20) << 2))
#define PG_L1_PGTABLE_VADDR     (PGTABLE_BASE_VADDR + ((PGTABLE_BASE_VADDR >> 20) << 2))

#define PG_L2_PGTABLE_PADDR     (PG_L2_DATA_PADDR + PG_L2_DATA_SIZE)
#define PG_L2_PGTABLE_VADDR     (PG_L2_DATA_VADDR + PG_L2_DATA_SIZE)
#define PG_L2_PGTABLE_SIZE      (4*PG_DATA_NPAGES)

/* Vector Mapping *******************************************************************/

/* One page is required to map the vector table.  The vector table could lie
 * at virtual address zero (or at the start of RAM which is aliased to address
 * zero on the ea3131) or at virtual address 0xfff00000.  We only have logic
 * here to support the former case.
 *
 * NOTE:  If the vectors are at address zero, the page table will be
 * forced to the highest RAM addresses.  If the vectors are at 0xfff0000,
 * then the page table is forced to the beginning of RAM.
 *
 * When the vectors are at the beginning of RAM, they will probably overlap
 * the first page of the locked text region.  In any other case, the
 * configuration must set CONFIG_PAGING_VECPPAGE to provide the physical
 * address of the page to use for the vectors.
 *
 * When the vectors overlap the first page of the locked text region (the
 * only case in use so far), then the text page will be temporarily be made
 * writable in order to copy the vectors.
 *
 * PG_VECT_PBASE - This the physical address of the page in memory to be
 *   mapped to the vector address.
 * PG_L2_VECT_PADDR - This is the physical address of the L2 page table
 *   entry to use for the vector mapping.
 * PG_L2_VECT_VADDR - This is the virtual address of the L2 page table
 *   entry to use for the vector mapping.
 */

/* Case 1: The configuration tells us everything */

#if defined(CONFIG_PAGING_VECPPAGE)
#  define PG_VECT_PBASE         CONFIG_PAGING_VECPPAGE
#  define PG_L2_VECT_PADDR      CONFIG_PAGING_VECL2PADDR
#  define PG_L2_VECT_VADDR      CONFIG_PAGING_VECL2VADDR

/* Case 2: Vectors are in low memory and the locked text region starts at
 * the beginning of SRAM (which will be aliased to address 0x00000000).
 * However, the beginning of SRAM may not be aligned to the beginning
 * of the L2 page table (because the beginning of RAM is offset into
 * the table.
 */

#elif defined(CONFIG_ARCH_LOWVECTORS) && !defined(CONFIG_PAGING_LOCKED_PBASE)
#  define PG_VECT_PBASE         PG_LOCKED_PBASE
#  define PG_L2_VECT_OFFSET     (((PG_LOCKED_VBASE & 0x000fffff) >> PAGESHIFT) << 2)
#  define PG_L2_VECT_PADDR      (PGTABLE_L2_BASE_PADDR + PG_L2_VECT_OFFSET)
#  define PG_L2_VECT_VADDR      (PGTABLE_L2_BASE_VADDR + PG_L2_VECT_OFFSET)

/* Case 3: High vectors or the locked region is not at the beginning or SRAM */

#else
#  error "Logic missing for high vectors in this case"
#endif

/* Page Usage ***********************************************************************/

/* This is the total number of pages used in the text/data mapping: */

#define PG_TOTAL_NPPAGES        (PG_TEXT_NPPAGES + PG_DATA_NPAGES + PG_PGTABLE_NPAGES)
#define PG_TOTAL_NVPAGES        (PG_TEXT_NVPAGES + PG_DATA_NPAGES + PG_PGTABLE_NPAGES)
#define PG_TOTAL_PSIZE          (PG_TOTAL_NPPAGES << PAGESHIFT)
#define PG_TOTAL_VSIZE          (PG_TOTAL_NVPAGES << PAGESHIFT)

/* Sanity check: */

#if PG_TOTAL_NPPAGES > PG_RAM_PAGES
#  error "Total pages required exceeds RAM size"
#endif

/* Page Management ******************************************************************/

/* For page management purposes, the following summarize the "heap" of
 * free pages, operations on free pages and the L2 page table.
 *
 * PG_POOL_VA2L1OFFSET(va)  - Given a virtual address, return the L1 table
 *                            offset (in bytes).
 * PG_POOL_VA2L1VADDR(va)   - Given a virtual address, return the virtual
 *                            address of the L1 table entry
 * PG_POOL_L12PPTABLE(L1)   - Given the value of an L1 table entry return
 *                            the physical address of the start of the L2
 *                            page table
 * PG_POOL_L12PPTABLE(L1)   - Given the value of an L1 table entry return
 *                            the virtual address of the start of the L2
 *                            page table.
 *
 * PG_POOL_L1VBASE          - The virtual address of the start of the L1
 *                            page table range corresponding to the first
 *                            virtual address of the paged text region.
 * PG_POOL_L1VEND           - The virtual address of the end+1 of the L1
 *                            page table range corresponding to the last
 *                            virtual address+1 of the paged text region.
 *
 * PG_POOL_VA2L2NDX(va)     - Converts a virtual address within the paged
 *                            text region to the most compact possible
 *                            representation. Each PAGESIZE of address
 *                            corresponds to 1 index in the L2 page table;
 *                            Index 0 corresponds to the first L2 page table
 *                            entry for the first page in the virtual paged
 *                            text address space.
 * PG_POOL_NDX2VA(ndx)      - Performs the opposite conversion.. converts
 *                            an index into a virtual address in the paged
 *                            text region (the address at the beginning of
 *                            the page).
 * PG_POOL_MAXL2NDX         - This is the maximum value+1 of such an index.
 *
 * PG_POOL_PGPADDR(ndx)     - Converts an page index into the corresponding
 *                            (physical) address of the backing page memory.
 * PG_POOL_PGVADDR(ndx)     - Converts an page index into the corresponding
 *                            (virtual)address of the backing page memory.
 *
 * These are used as follows:  If a miss occurs at some virtual address, va,
 * A new page index, ndx, is allocated.  PG_POOL_PGPADDR(i) converts the index
 * into the physical address of the page memory; PG_POOL_L2VADDR(va) converts
 * the virtual address in the L2 page table there the new mapping will be
 * written.
 */

#define PG_POOL_VA2L1OFFSET(va) (((va) >> 20) << 2)
#define PG_POOL_VA2L1VADDR(va)  (PGTABLE_BASE_VADDR + PG_POOL_VA2L1OFFSET(va))
#define PG_POOL_L12PPTABLE(L1)  ((L1) & PG_L1_PADDRMASK)
#define PG_POOL_L12VPTABLE(L1)  (PG_POOL_L12PPTABLE(L1) - PGTABLE_BASE_PADDR + PGTABLE_BASE_VADDR)

#define PG_POOL_L1VBASE         (PGTABLE_BASE_VADDR + ((PG_PAGED_VBASE >> 20) << 2))
#define PG_POOL_L1VEND          (PG_POOL_L1VBASE + (CONFIG_PAGING_NVPAGED << 2))

#define PG_POOL_VA2L2NDX(va)    (((va) -  PG_PAGED_VBASE) >> PAGESHIFT)
#define PG_POOL_NDX2VA(ndx)     (((ndx) << PAGESHIFT) + PG_PAGED_VBASE)
#define PG_POOL_MAXL2NDX        PG_POOL_VA2L2NDX(PG_PAGED_VEND)

#define PG_POOL_PGPADDR(ndx)    (PG_PAGED_PBASE + ((ndx) << PAGESHIFT))
#define PG_POOL_PGVADDR(ndx)    (PG_PAGED_VBASE + ((ndx) << PAGESHIFT))

#endif /* CONFIG_PAGING */

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__
/* struct section_mapping_s describes the L1 mapping of a large region of memory
 * consisting of one or more 1MB sections (nsections).
 *
 * All addresses must be aligned to 1MB address boundaries.
 */

struct section_mapping_s
{
  uint32_t physbase;   /* Physical address of the region to be mapped */
  uint32_t virtbase;   /* Virtual address of the region to be mapped */
  uint32_t mmuflags;   /* MMU settings for the region (e.g., cache-able) */
  uint32_t nsections;  /* Number of mappings in the region */
};
#endif

/************************************************************************************
 * Assemby Macros
 ************************************************************************************/

#ifdef __ASSEMBLY__

/************************************************************************************
 * Name: cp15_disable_mmu
 *
 * Description:
 *   Disable the MMU
 *
 * Input Parameters:
 *   None
 *
 ************************************************************************************/

	.macro	cp15_disable_mmu, scratch
	mrc		p15, 0, \scratch, c1, c0, 0
	bic		\scratch, \scratch, #1
	mcr		p15, 0, \scratch, c1, c0, 0
	.endm

/************************************************************************************
 * Name: cp15_invalidate_tlbs
 *
 * Description:
 *   Invalidate entire unified TLB
 *
 *   The Invalidate entire TLB operations invalidate all unlocked entries in the
 *   TLB. The operation ignores the value in the register Rt specified by the MCR
 *   instruction that performs the operation. Software does not have to write a
 *   value to the register before issuing the MCR instruction.
 *
 * Input Parameters:
 *   None
 *
 ************************************************************************************/

	.macro	cp15_invalidate_tlbs, scratch
	mcr		p15, 0, \scratch, c8, c7, 0	/* TLBIALL */
	.endm

/************************************************************************************
 * Name: cp15_invalidate_tlb_bymva
 *
 * Description:
 *   Invalidate unified TLB entry by MVA all ASID Inner Shareable
 *
 * Input Parameters:
 *   vaddr - The virtual address to be invalidated
 *
 ************************************************************************************/

	.macro	cp15_invalidate_tlb_bymva, vaddr
	dsb
	mcr		p15, 0, \vaddr, c8, c3, 3	/* TLBIMVAAIS */
	dsb
	isb
	.endm

/************************************************************************************
 * Name: cp15_wrdacr
 *
 * Description:
 *   Write the Domain Access Control Register (DACR)
 *
 * Input Parameters:
 *   dacr - The new value of the DACR
 *
 ************************************************************************************/

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

/************************************************************************************
 * Name: cp14_wrttb
 *
 * Description:
 *   The ARMv7-aA architecture supports two translation tables.  This
 *   implementation, however, uses only translation table 0.  This
 *   function  writes the address of the page table to the Translation
 *   Table Base Register 0 (TTBR0).  Then it clears the TTB control
 *   register (TTBCR), indicating that we are using TTBR0.
 *
 * Input Parameters:
 *   ttb - The new value of the TTBR0 register
 *
 ************************************************************************************/

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

/************************************************************************************
 * Name: pg_l2map
 *
 * Description:
 *   Write several, contiguous L2 page table entries.  npages entries will be
 *   written. This macro is used when CONFIG_PAGING is enable.  This case,
 *   it is used as follows:
 *
 *	ldr	r0, =PGTABLE_L2_BASE_PADDR	<-- Address in L2 table
 *	ldr	r1, =PG_LOCKED_PBASE		<-- Physical page memory address
 *	ldr	r2, =CONFIG_PAGING_NLOCKED	<-- number of pages
 *      ldr	r3, =MMUFLAGS			<-- L2 MMU flags
 *	pg_l2map r0, r1, r2, r3, r4
 *
 * Input Parameters:
 *   l2 - Physical or virtual start address in the L2 page table, depending
 *        upon the context. (modified)
 *   ppage - The physical address of the start of the region to span. Must
 *           be aligned to 1Mb section boundaries (modified)
 *   npages - Number of pages to write in the section (modified)
 *   mmuflags - L2 MMU FLAGS
 *
 * Scratch registers (modified): tmp
 *   l2  - Next address in the L2 page table.
 *   ppage - Start of next physical page
 *   npages - Loop counter
 *   tmp - scratch
 *
 * Assumptions:
 * - The MMU is not yet enabled
 * - The L2 page tables have been zeroed prior to calling this function
 * - pg_l1span has been called to initialize the L1 table.
 *
 ************************************************************************************/

#ifdef CONFIG_PAGING
	.macro	pg_l2map, l2, ppage, npages, mmuflags, tmp
	b		2f
1:
	/* Write the one L2 entries.  First,  get tmp = (ppage | mmuflags),
	 * the value to write into the L2 PTE
	 */

	orr		\tmp, \ppage, \mmuflags

	/* Write value into table at the current table address
	 * (and increment the L2 page table address by 4)
	 */

	str		\tmp, [\l2], #4

	/* Update the physical address that will correspond to the next
	 * table entry.
	 */

	add		\ppage, \ppage, #CONFIG_PAGING_PAGESIZE

	/* Decrement the number of pages written */

	sub		\npages, \npages, #1
2:
	/* Check if all of the pages have been written.  If not, then
	 * loop and write the next PTE.
	 */

	cmp		\npages, #0
	bgt		1b
	.endm
#endif /* CONFIG_PAGING */

/************************************************************************************
 * Name: pg_l1span
 *
 * Description:
 *   Write several, contiguous, unmapped, small L1 page table entries.  As many
 *   entries will be written as  many as needed to span npages.  This macro is
 *   used when CONFIG_PAGING is enable.  In this case, it is used as follows:
 *
 *	ldr	r0, =PG_L1_PGTABLE_PADDR	<-- Address in the L1 table
 *	ldr	r1, =PG_L2_PGTABLE_PADDR	<-- Physical address of L2 page table
 *	ldr	r2, =PG_PGTABLE_NPAGES		<-- Total number of pages
 *	ldr	r3, =PG_PGTABLE_NPAGE1		<-- Number of pages in the first PTE
 *	ldr	r4, =MMU_L1_PGTABFLAGS		<-- L1 MMU flags
 *	pg_l1span r0, r1, r2, r3, r4, r4
 *
 * Input Parameters (unmodified unless noted):
 *   l1 - Physical or virtual address in the L1 table to begin writing (modified)
 *   l2 - Physical start address in the L2 page table (modified)
 *   npages - Number of pages to required to span that memory region (modified)
 *   ppage - The number of pages in page 1 (modified)
 *   mmuflags - L1 MMU flags to use
 *
 * Scratch registers (modified): l1, l2, npages, tmp
 *   l1 - Next L1 table address
 *   l2 - Physical start address of the next L2 page table
 *   npages - Loop counter
 *   ppage - After the first page, this will be the full number of pages.
 *   tmp - scratch
 *
 * Returned Value:
 *   Nothing of interest.
 *
 * Assumptions:
 * - The MMU is not yet enabled
 * - The L2 page tables have been zeroed prior to calling this function
 *
 ************************************************************************************/

#ifdef CONFIG_PAGING
	.macro	pg_l1span, l1, l2, npages, ppage, mmuflags, tmp
	b		2f
1:
	/* Write the L1 table entry that refers to this (unmapped) small page
	 * table.
	 *
	 * tmp = (l2table | mmuflags), the value to write into the page table
	 */

	orr		\tmp, \l2, \mmuflags

	/* Write the value into the L1 table at the correct offset.
	 * (and increment the L1 table address by 4)
	 */

	str		\tmp, [\l1], #4

	/* Update the L2 page table address for the next L1 table entry. */

	add		\l2, \l2, #PT_SIZE  /* Next L2 page table start address */

	/* Update the number of pages that we have account for (with
	 * non-mappings).  NOTE that the first page may have fewer than
	 * the maximum entries per page table.
	 */

	sub		\npages, \npages, \ppage
	mov		\ppage, #PTE_NPAGES
2:
	/* Check if all of the pages have been written.  If not, then
	 * loop and write the next L1 entry.
	 */

	cmp		\npages, #0
	bgt		1b
	.endm

#endif /* CONFIG_PAGING */
#endif /* __ASSEMBLY__ */

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Name: cp15_disable_mmu
 *
 * Description:
 *   Disable the MMU
 *
 * Input Parameters:
 *   None
 *
 ************************************************************************************/

static inline void cp15_disable_mmu(void)
{
  __asm__ __volatile__
    (
      "\tmrc p15, 0, r0, c1, c0, 0\n"
      "\tbic r0, r0, #1\n"
      "\tmcr p15, 0, r0, c1, c0, 0\n"
      :
      :
      : "r0", "memory"
    );
}

/************************************************************************************
 * Name: cp15_invalidate_tlbs
 *
 * Description:
 *   Invalidate entire unified TLB
 *
 *   The Invalidate entire TLB operations invalidate all unlocked entries in the
 *   TLB. The operation ignores the value in the register Rt specified by the MCR
 *   instruction that performs the operation. Software does not have to write a
 *   value to the register before issuing the MCR instruction.
 *
 * Input Parameters:
 *   None
 *
 ************************************************************************************/

static inline void cp15_invalidate_tlbs(void)
{
  __asm__ __volatile__
    (
      "\tmcr p15, 0, r0, c8, c7, 0\n" /* TLBIALL */
      :
      :
      : "r0", "memory"
    );
}

/************************************************************************************
 * Name: cp15_invalidate_tlb_bymva
 *
 * Description:
 *   Invalidate unified TLB entry by MVA all ASID Inner Shareable
 *
 * Input Parameters:
 *   vaddr - The virtual address to be invalidated
 *
 ************************************************************************************/

static inline void cp15_invalidate_tlb_bymva(uint32_t vaddr)
{
  __asm__ __volatile__
    (
      "\tdsb\n"
      "\tmcr p15, 0, %0, c8, c3, 3\n" /* TLBIMVAAIS */
      "\tdsb\n"
      "\tisb\n"
      :
      : "r" (vaddr)
      : "r1", "memory"
    );
}

/************************************************************************************
 * Name: cp15_wrdacr
 *
 * Description:
 *   Write the Domain Access Control Register (DACR)
 *
 * Input Parameters:
 *   dacr - The new value of the DACR
 *
 ************************************************************************************/

static inline void cp15_wrdacr(unsigned int dacr)
{
  __asm__ __volatile__
    (
      "\tmcr p15, 0,0, c3, c0, 0\n"
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

/************************************************************************************
 * Name: cp14_wrttb
 *
 * Description:
 *   The ARMv7-aA architecture supports two translation tables.  This
 *   implementation, however, uses only translation table 0.  This
 *   function  writes the address of the page table to the Translation
 *   Table Base Register 0 (TTBR0).  Then it clears the TTB control
 *   register (TTBCR), indicating that we are using TTBR0.
 *
 * Input Parameters:
 *   ttb - The new value of the TTBR0 register
 *
 ************************************************************************************/

static inline void cp14_wrttb(unsigned int ttb)
{
  __asm__ __volatile__
    (
      "\tmcr p15, 0,0, c2, c0, 0\n"
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

/*************************************************************************************
 * Name: mmu_l1_getentry
 *
 * Description:
 *   Given a virtual address, return the value of the corresponding L1 table entry.
 *
 * Input Parameters:
 *   vaddr - The virtual address to be mapped.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
static inline uint32_t mmu_l1_getentry(uint32_t vaddr)
{
  uint32_t *l1table = (uint32_t*)PGTABLE_BASE_VADDR;
  uint32_t  index   = vaddr >> 20;

  /* Return the address of the page table entry */

  return l1table[index];
}
#endif

/*************************************************************************************
 * Name: mmu_l2_getentry
 *
 * Description:
 *   Given a address of the beginning of an L2 page table and a virtual address,
 *   return the value of the corresponding L2 page table entry.
 *
 * Input Parameters:
 *   l2vaddr - The virtual address of the beginning of the L2 page table
 *   vaddr - The virtual address to be mapped.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
static inline uint32_t mmu_l2_getentry(uint32_t l2vaddr, uint32_t vaddr)
{
  uint32_t *l2table  = (uint32_t*)l2vaddr;
  uint32_t  index;

  /* The table divides a 1Mb address space up into 256 entries, each
   * corresponding to 4Kb of address space.  The page table index is
   * related to the offset from the beginning of 1Mb region.
   */

  index = (vaddr & 0x000ff000) >> 12;

  /* Return the address of the page table entry */

  return l2table[index];
}
#endif

#endif /* __ASSEMBLY__ */

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Name: mmu_l1_setentry
 *
 * Description:
 *   Set a one level 1 translation table entry.  Only a single L1 page table is
 *   supported.
 *
 * Input Parameters:
 *   paddr - The physical address to be mapped.  Must be aligned to a 1MB address
 *     boundary
 *   vaddr - The virtual address to be mapped.  Must be aligned to a 1MB address
 *     boundary
 *   mmuflags - The MMU flags to use in the mapping.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l1_setentry(uint32_t paddr, uint32_t vaddr, uint32_t mmuflags);
#endif

/****************************************************************************
 * Name: mmu_l1_restore
 *
 * Description:
 *   Restore one L1 table entry previously returned by mmu_l1_getentry() (or
 *   any other encoded L1 page table value).
 *
 * Input Parameters:
 *   vaddr - A virtual address to be mapped
 *   l1entry - The value to write into the page table entry
 *
 ****************************************************************************/

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_ADDRENV)
void mmu_l1_restore(uintptr_t vaddr, uint32_t l1entry);
#endif

/************************************************************************************
 * Name: mmu_l1_clrentry
 *
 * Description:
 *   Unmap one L1 region by writing zero into the L1 page table entry and by
 *   flushing caches and TLBs appropriately.
 *
 * Input Parameters:
 *   vaddr - A virtual address within the L1 address region to be unmapped.
 *
 ************************************************************************************/

#if !defined (CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_ADDRENV)
#  define mmu_l1_clrentry(v) mmu_l1_restore(v,0)
#endif

/****************************************************************************
 * Name: mmu_l2_setentry
 *
 * Description:
 *   Set one small (4096B) entry in a level2 translation table.
 *
 * Input Parameters:
 *   l2vaddr - the virtual address of the beginning of the L2 translation
 *     table.
 *   paddr - The physical address to be mapped.  Must be aligned to a 4KB
 *     address boundary
 *   vaddr - The virtual address to be mapped.  Must be aligned to a 4KB
 *     address boundary
 *   mmuflags - The MMU flags to use in the mapping.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l2_setentry(uint32_t l2vaddr, uint32_t paddr, uint32_t vaddr,
                     uint32_t mmuflags);
#endif

/************************************************************************************
 * Name: mmu_l1_map_region
 *
 * Description:
 *   Set multiple level 1 translation table entries in order to map a region of
 *   memory.
 *
 * Input Parameters:
 *   mapping - Describes the mapping to be performed.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l1_map_region(const struct section_mapping_s *mapping);
#endif

/****************************************************************************
 * Name: mmu_invalidate_region
 *
 * Description:
 *   Invalidate TLBs for a range of addresses (all 4KB aligned).
 *
 * Input Parameters:
 *   vaddr - The beginning of the region to invalidate.
 *   size  - The size of the region in bytes to be invalidated.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_invalidate_region(uint32_t vstart, size_t size);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif  /* __ARCH_ARM_SRC_ARMV7_A_MMU_H */
