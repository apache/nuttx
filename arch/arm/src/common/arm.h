/************************************************************************************
 * arch/arm/src/common/arm.h
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_COMMON_ARM_H
#define __ARCH_ARM_SRC_COMMON_ARM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#ifndef __ASSEMBLY__
# include <sys/types.h>
#endif

/**************************************************************************
 * Conditional Compilation
 **************************************************************************/

#undef CONFIG_ALIGNMENT_TRAP
#undef CONFIG_DCACHE_WRITETHROUGH
#undef CONFIG_CACHE_ROUND_ROBIN
#undef CONFIG_DCACHE_DISABLE
#undef CONFIG_ICACHE_DISABLE

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* ARM9EJS **************************************************************************/

/* PSR bits */

#define MODE_MASK       0x0000001f
#define USR26_MODE      0x00000000
#define FIQ26_MODE      0x00000001
#define IRQ26_MODE      0x00000002
#define SVC26_MODE      0x00000003
#define USR_MODE        0x00000010
#define FIQ_MODE        0x00000011
#define IRQ_MODE        0x00000012
#define SVC_MODE        0x00000013
#define ABT_MODE        0x00000017
#define UND_MODE        0x0000001b
#define MODE32_BIT      0x00000010
#define SYSTEM_MODE     0x0000001f
#define PSR_T_BIT       0x00000020
#define PSR_F_BIT       0x00000040
#define PSR_I_BIT       0x00000080
#define PSR_J_BIT       0x01000000
#define PSR_Q_BIT       0x08000000
#define PSR_V_BIT       0x10000000
#define PSR_C_BIT       0x20000000
#define PSR_Z_BIT       0x40000000
#define PSR_N_BIT       0x80000000

/* CR1 bits (CP#15 CR1) */

#define CR_M            0x00000001 /* MMU enable                          */
#define CR_A            0x00000002 /* Alignment abort enable              */
#define CR_C            0x00000004 /* Dcache enable                       */
#define CR_W            0x00000008 /* Write buffer enable                 */
#define CR_P            0x00000010 /* 32-bit exception handler            */
#define CR_D            0x00000020 /* 32-bit data address range           */
#define CR_L            0x00000040 /* Implementation defined              */
#define CR_B            0x00000080 /* Big endian                          */
#define CR_S            0x00000100 /* System MMU protection               */
#define CR_R            0x00000200 /* ROM MMU protection                  */
#define CR_F            0x00000400 /* Implementation defined              */
#define CR_Z            0x00000800 /* Implementation defined              */
#define CR_I            0x00001000 /* Icache enable                       */
#define CR_V            0x00002000 /* Vectors relocated to 0xffff0000     */
#define CR_RR           0x00004000 /* Round Robin cache replacement       */
#define CR_L4           0x00008000 /* LDR pc can set T bit                */
#define CR_DT           0x00010000
#define CR_IT           0x00040000
#define CR_ST           0x00080000
#define CR_FI           0x00200000 /* Fast interrupt (lower latency mode) */
#define CR_U            0x00400000 /* Unaligned access operation          */
#define CR_XP           0x00800000 /* Extended page tables                */
#define CR_VE           0x01000000 /* Vectored interrupts                 */

/* Hardware page table definitions.
 *
 * Level 1 Descriptor (PMD)
 *
 * Common definitions.
 */

#define PMD_TYPE_MASK       0x00000003  /* Bits 1:0:   Type of descriptor */
#define PMD_TYPE_FAULT      0x00000000
#define PMD_TYPE_COARSE     0x00000001
#define PMD_TYPE_SECT       0x00000002
#define PMD_TYPE_FINE       0x00000003
                                    /* Bits 3:2:   Depends on descriptor */
#define PMD_BIT4            0x00000010  /* Bit  4:     Must be one */
#define PMD_DOMAIN_MASK     0x000001e0  /* Bits 8:5:   Domain control bits */
#define PMD_DOMAIN(x)       ((x) << 5)
#define PMD_PROTECTION      0x00000200  /* Bit 9:      v5 only */
                                    /* Bits 31:10: Depend on descriptor */

/* Level 1 Section Descriptor.  Section descriptors allow fast, single
 * level mapping between 1Mb address regions.
 */
                                        /* Bits 1:0:   Type of mapping */
#define PMD_SECT_BUFFERABLE 0x00000004  /* Bit  2:     1=bufferable */
#define PMD_SECT_CACHEABLE  0x00000008  /* Bit  3:     1=cacheable */
                                        /* Bit  4:     Common, must be one */
                                        /* Bits 8:5:   Common domain control */
                                        /* Bit  9:     Common protection */
#define PMD_SECT_AP_MASK    0x00000c00  /* Bits 11:10: Access permission */
#define PMD_SECT_AP_WRITE   0x00000400
#define PMD_SECT_AP_READ    0x00000800
                                        /* Bits 19:20: Should be zero */
#define PMD_SECT_TEX_MASK   0xfff00000  /* Bits 31:20: v5, Physical page */
#define PMD_SECT_APX        0x00008000  /* Bit  15:    v6 only */
#define PMD_SECT_S          0x00010000  /* Bit  16:    v6 only */
#define PMD_SECT_nG         0x00020000  /* Bit  17:    v6 only */

#define PMD_SECT_UNCACHED   (0)
#define PMD_SECT_BUFFERED   (PMD_SECT_BUFFERABLE)
#define PMD_SECT_WT         (PMD_SECT_CACHEABLE)
#define PMD_SECT_WB         (PMD_SECT_CACHEABLE|PMD_SECT_BUFFERABLE)
#define PMD_SECT_MINICACHE  (PMD_SECT_TEX(1)|PMD_SECT_CACHEABLE)
#define PMD_SECT_WBWA       (PMD_SECT_TEX(1)|PMD_SECT_CACHEABLE|PMD_SECT_BUFFERABLE)

/* Level 1 Coarse Table Descriptor.  Coarse Table Descriptors support
 * two level mapping between 16Kb memory regions.
 */
                                        /* Bits 1:0:   Type of mapping */
                                        /* Bits 3:2:   Should be zero */
                                        /* Bit  4:     Common, must be one */
                                        /* Bits 8:5:   Common domain control */
                                        /* Bits 9:     Should be zero */
#define PMD_COARSE_TEX_MASK 0xfffffc00  /* Bits 31:10: v5, Physical page */

/* Level 1 Fine Table Descriptor.  Coarse Table Descriptors support
 * two level mapping between 4Kb memory regions.
 */

                                        /* Bits 1:0:   Type of mapping */
                                        /* Bits 3:2:   Should be zero */
                                        /* Bit  4:     Common, must be one */
                                        /* Bits 8:5:   Common domain control */
                                        /* Bits 11:9:  Should be zero */
#define PMD_FINE_TEX_MASK   0xfffff000  /* Bits 31:12: v5, Physical page */

/* Level 2 Table Descriptor (PTE).  -- All tables */

#define PTE_TYPE_MASK       (3 << 0)    /* Bits: 1:0:  Type of mapping */
#define PTE_TYPE_FAULT      (0 << 0)    /*   None */
#define PTE_TYPE_LARGE      (1 << 0)    /*   64Kb of memory */
#define PTE_TYPE_SMALL      (2 << 0)    /*    4Kb of memory */
#define PTE_TYPE_TINY       (3 << 0)    /*    1Kb of memory (v5)*/
#define PTE_BUFFERABLE      (1 << 2)    /* Bit  2:     1=bufferable */
#define PTE_CACHEABLE       (1 << 3)    /* Bit  3:     1=cacheable */
                                        /* Bits 31:4:  Depend on type */

/* Large page -- 64Kb */
                                         /* Bits: 1:0:  Type of mapping */
                                         /* Bits: 3:2:  Bufferable/cacheable */
#define PTE_LARGE_AP_MASK    (0xff << 4) /* Bits 11:4   Access permissions */
#define PTE_LARGE_AP_UNO_SRO (0x00 << 4)
#define PTE_LARGE_AP_UNO_SRW (0x55 << 4)
#define PTE_LARGE_AP_URO_SRW (0xaa << 4)
#define PTE_LARGE_AP_URW_SRW (0xff << 4)
                                         /* Bits 15:12: Should be zero */
#define PTE_LARGE_TEX_MASK   0xffff0000  /* Bits 31:16: v5, Physical page */

/* Small page -- 4Kb */

                                         /* Bits: 1:0:  Type of mapping */
                                         /* Bits: 3:2:  Bufferable/cacheable */
#define PTE_SMALL_AP_MASK    (0xff << 4) /* Bits: 11:4: Access permissions */
#define PTE_SMALL_AP_UNO_SRO (0x00 << 4)
#define PTE_SMALL_AP_UNO_SRW (0x55 << 4)
#define PTE_SMALL_AP_URO_SRW (0xaa << 4)
#define PTE_SMALL_AP_URW_SRW (0xff << 4)
#define PTE_SMALL_TEX_MASK   0xfffff000  /* Bits: 31:12: Physical page */

/* Tiny page -- 1Kb */

                                        /* Bits: 1:0:  Type of mapping */
                                        /* Bits: 3:2:  Bufferable/cacheable */
#define PTE_EXT_AP_MASK      (3 << 4)   /* Bits: 5:4:  Access persions */
#define PTE_EXT_AP_UNO_SRO   (0 << 4)
#define PTE_EXT_AP_UNO_SRW   (1 << 4)
#define PTE_EXT_AP_URO_SRW   (2 << 4)
#define PTE_EXT_AP_URW_SRW   (3 << 4)
                                        /* Bits: 9:6:  Should be zero */
#define PTE_TINY_TEX_MASK    0xfffffc00 /* Bits: 31:10: Physical page */

/* Default MMU flags for memory and IO */

#define MMU_MEMFLAGS \
  (PMD_TYPE_SECT|PMD_SECT_WB|PMD_BIT4|PMD_SECT_AP_WRITE|PMD_SECT_AP_READ)

#define MMU_IOFLAGS \
  (PMD_TYPE_SECT|PMD_BIT4|PMD_SECT_AP_WRITE|PMD_SECT_AP_READ)

#define MMU_L1_VECTORFLAGS   (PMD_TYPE_COARSE|PMD_BIT4)
#define MMU_L2_VECTORFLAGS   (PTE_TYPE_SMALL|PTE_SMALL_AP_UNO_SRW)

/* Mapped section size */

#define SECTION_SIZE          (1 << 20)   /* 1Mb */

/* We place the page tables 16K below the beginning of .text.  The
 * following value is assume to be the (virtual) start address of
 * .text.
 */

#define PGTABLE_SIZE          0x00004000

/************************************************************************************
 * Inline Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

#endif /* __ASSEMBLY__ */

#endif  /* __ARCH_ARM_SRC_COMMON_ARM_H */
