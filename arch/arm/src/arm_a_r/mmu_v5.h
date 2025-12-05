/****************************************************************************
 * arch/arm/src/arm_a_r/mmu_v5.h
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

#ifndef __ARCH_ARM_SRC_ARM_A_R_MMU_V5_H
#define __ARCH_ARM_SRC_ARM_A_R_MMU_V5_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Fault status register */

#define FSR_MASK                 15
#define FSR_FAULT(fsr)           ((fsr) & FSR_MASK)
#define FSR_FAULT_DEBUG          2
#define FSR_FAULT_TRANSLATION_L1 5
#define FSR_FAULT_TRANSLATION_L2 7

/* System control register */

#define SCTLR_M                  (1 << 0)
#define SCTLR_A                  (1 << 1)
#define SCTLR_C                  (1 << 2)
#define SCTLR_W                  (1 << 3)
#define SCTLR_P                  (1 << 4)
#define SCTLR_CCP15BEN           (1 << 5)
#define SCTLR_S                  (1 << 8)
#define SCTLR_R                  (1 << 9)
#define SCTLR_SW                 (1 << 10)
#define SCTLR_Z                  (1 << 11)
#define SCTLR_I                  (1 << 12)
#define SCTLR_V                  (1 << 13)
#define SCTLR_RR                 (1 << 14)

/* Level 1 descriptors */

#define PMD_TYPE_MASK            0x00000003
#define PMD_TYPE_FAULT           0x00000000
#define PMD_TYPE_COARSE          0x00000001
#define PMD_TYPE_SECT            0x00000002
#define PMD_TYPE_FINE            0x00000003
#define PMD_BIT4                 0x00000010
#define PMD_DOMAIN_MASK          0x000001e0
#define PMD_DOMAIN(x)            ((x) << 5)
#define PMD_PROTECTION           0x00000200

#define PMD_SECT_BUFFERABLE      0x00000004
#define PMD_SECT_CACHEABLE       0x00000008
#define PMD_SECT_AP_MASK         0x00000c00
#define PMD_SECT_AP_WRITE        0x00000400
#define PMD_SECT_AP_READ         0x00000800
#define PMD_SECT_TEX_MASK        0xfff00000
#define PMD_SECT_PADDR_MASK      PMD_SECT_TEX_MASK
#define PMD_SECT_APX             0x00008000
#define PMD_SECT_S               0x00010000
#define PMD_SECT_NG              0x00020000

#define PMD_SECT_UNCACHED        0
#define PMD_SECT_BUFFERED        PMD_SECT_BUFFERABLE
#define PMD_SECT_WT              PMD_SECT_CACHEABLE
#define PMD_SECT_WB              (PMD_SECT_CACHEABLE | PMD_SECT_BUFFERABLE)

#define PMD_COARSE_TEX_MASK      0xfffffc00
#define PMD_FINE_TEX_MASK        0xfffff000

/* Level 2 descriptors */

#define PTE_TYPE_MASK            (3 << 0)
#define PTE_TYPE_FAULT           (0 << 0)
#define PTE_TYPE_LARGE           (1 << 0)
#define PTE_TYPE_SMALL           (2 << 0)
#define PTE_TYPE_TINY            (3 << 0)
#define PTE_B                    (1 << 2)
#define PTE_C                    (1 << 3)

#define PTE_LARGE_AP_MASK        (0xff << 4)
#define PTE_LARGE_AP_UNO_SRO     (0x00 << 4)
#define PTE_LARGE_AP_UNO_SRW     (0x55 << 4)
#define PTE_LARGE_AP_URO_SRW     (0xaa << 4)
#define PTE_LARGE_AP_URW_SRW     (0xff << 4)
#define PTE_LARGE_TEX_MASK       0xffff0000

#define PTE_SMALL_AP_MASK        (0xff << 4)
#define PTE_SMALL_AP_UNO_SRO     (0x00 << 4)
#define PTE_SMALL_AP_UNO_SRW     (0x55 << 4)
#define PTE_SMALL_AP_URO_SRW     (0xaa << 4)
#define PTE_SMALL_AP_URW_SRW     (0xff << 4)
#define PTE_SMALL_TEX_MASK       0xfffff000
#define PTE_SMALL_NPAGES         256

#define PTE_EXT_AP_MASK          (3 << 4)
#define PTE_EXT_AP_UNO_SRO       (0 << 4)
#define PTE_EXT_AP_UNO_SRW       (1 << 4)
#define PTE_EXT_AP_URO_SRW       (2 << 4)
#define PTE_EXT_AP_URW_SRW       (3 << 4)
#define PTE_TINY_TEX_MASK        0xfffffc00
#define PTE_TINY_NPAGES          1024

/* Default mappings */

#define MMU_ROMFLAGS \
  (PMD_TYPE_SECT | PMD_BIT4 | PMD_SECT_AP_READ)
#define MMU_MEMFLAGS \
  (PMD_TYPE_SECT | PMD_SECT_WB | PMD_BIT4 | PMD_SECT_AP_WRITE | \
   PMD_SECT_AP_READ)
#define MMU_IOFLAGS \
  (PMD_TYPE_SECT | PMD_BIT4 | PMD_SECT_AP_WRITE | PMD_SECT_AP_READ)

#define MMU_L1_VECTORFLAGS       (PMD_TYPE_COARSE | PMD_BIT4)
#define MMU_L2_VECTORFLAGS       (PTE_TYPE_SMALL | PTE_SMALL_AP_UNO_SRW)

#define SECTION_SHIFT            20
#define SECTION_SIZE             (1 << SECTION_SHIFT)
#define SECTION_MASK             (SECTION_SIZE - 1)
#define PGTABLE_SIZE             0x00004000

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct section_mapping_s
{
  uintptr_t physbase;
  uintptr_t virtbase;
  uint32_t  mmuflags;
  uint32_t  nsections;
};

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline void cp15_invalidate_tlb_bymva(uint32_t mva)
{
  mva &= PTE_TINY_TEX_MASK;
  __asm__ __volatile__
    (
      "mcr p15, 0, %0, c8, c7, 1"
      :
      : "r" (mva)
      : "memory"
    );
}

  #define tlb_inst_invalidate_single(mva) cp15_invalidate_tlb_bymva(mva)

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_ARM_A_R_MMU_V5_H */