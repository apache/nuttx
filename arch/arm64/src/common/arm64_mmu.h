/****************************************************************************
 * arch/arm64/src/common/arm64_mmu.h
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
 *
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_COMMON_ARM64_MMU_H
#define __ARCH_ARM64_SRC_COMMON_ARM64_MMU_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "arm64_arch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Following Memory types supported through MAIR encodings can be passed
 * by user through "attrs"(attributes) field of specified memory region.
 * As MAIR supports such 8 encodings, we will reserve attrs[2:0];
 * so that we can provide encodings upto 7 if needed in future.
 *
 * See Arm® Architecture Reference Manual, ARM DDI 0487E.a, B2.7.2
 */

#define MT_TYPE_MASK                0x7U
#define MT_TYPE(attr)               ((attr) & MT_TYPE_MASK)
#define MT_DEVICE_NGNRNE            0U
#define MT_DEVICE_NGNRE             1U
#define MT_DEVICE_GRE               2U
#define MT_NORMAL_NC                3U
#define MT_NORMAL                   4U

#ifdef CONFIG_ARM64_MTE
#define MT_NORMAL_VAL               0xf0UL
#else
#define MT_NORMAL_VAL               0xffUL
#endif

#define MEMORY_ATTRIBUTES           ((0x00 << (MT_DEVICE_NGNRNE * 8)) |  \
                                     (0x04 << (MT_DEVICE_NGNRE * 8))   | \
                                     (0x0c << (MT_DEVICE_GRE * 8))     | \
                                     (0x44 << (MT_NORMAL_NC * 8))      | \
                                     (MT_NORMAL_VAL << (MT_NORMAL * 8)))

/* More flags from user's perpective are supported using remaining bits
 * of "attrs" field, i.e. attrs[31:3], underlying code will take care
 * of setting PTE fields correctly.
 *
 * current usage of attrs[31:3] is:
 * attrs[3] : Access Permissions
 * attrs[4] : Memory access from secure/ns state
 * attrs[5] : Execute Permissions
 *
 * See Arm® Architecture Reference Manual, ARM DDI 0487E.a
 */

#define MT_PERM_SHIFT               3U
#define MT_SEC_SHIFT                4U
#define MT_EXECUTE_SHIFT            5U

#define MT_RO                       (0U << MT_PERM_SHIFT)
#define MT_RW                       (1U << MT_PERM_SHIFT)

#define MT_SECURE                   (0U << MT_SEC_SHIFT)
#define MT_NS                       (1U << MT_SEC_SHIFT)

#define MT_EXECUTE                  (0U << MT_EXECUTE_SHIFT)
#define MT_EXECUTE_NEVER            (1U << MT_EXECUTE_SHIFT)

/* Some compound attributes for most common usages */

#define MT_CODE                     (MT_NORMAL | MT_RO | MT_EXECUTE)
#define MT_RODATA                   (MT_NORMAL | MT_RO | MT_EXECUTE_NEVER)

/* PTE descriptor can be Block descriptor or Table descriptor
 * or Page descriptor.
 */

#define PTE_DESC_TYPE_MASK          3U
#define PTE_BLOCK_DESC              1U
#define PTE_TABLE_DESC              3U
#define PTE_PAGE_DESC               3U
#define PTE_INVALID_DESC            0U

/* Block and Page descriptor attributes fields */

#define PTE_BLOCK_DESC_MEMTYPE(x)   ((x) << 2)
#define PTE_BLOCK_DESC_NS           (1ULL << 5) /* Non-secure */
#define PTE_BLOCK_DESC_AP_USER      (1ULL << 6) /* User */
#define PTE_BLOCK_DESC_AP_RO        (1ULL << 7) /* Read-only */
#define PTE_BLOCK_DESC_AP_RW        (0ULL << 7) /* Read-write */
#define PTE_BLOCK_DESC_AP_MASK      (3ULL << 6)
#define PTE_BLOCK_DESC_NON_SHARE    (0ULL << 8)
#define PTE_BLOCK_DESC_OUTER_SHARE  (2ULL << 8)
#define PTE_BLOCK_DESC_INNER_SHARE  (3ULL << 8)
#define PTE_BLOCK_DESC_AF           (1ULL << 10) /* A-flag */
#define PTE_BLOCK_DESC_NG           (1ULL << 11) /* Non-global */
#define PTE_BLOCK_DESC_DIRTY        (1ULL << 51) /* D-flag */
#define PTE_BLOCK_DESC_PXN          (1ULL << 53) /* Kernel execute never */
#define PTE_BLOCK_DESC_UXN          (1ULL << 54) /* User execute never */

/* PTE address field */

#define PTE_PADDR_SHIFT             (12U)
#define PTE_PADDR_WIDTH             (36U)
#define PTE_PADDR_MASK              (((1UL << PTE_PADDR_WIDTH) - 1) << PTE_PADDR_SHIFT)

/* TCR definitions.
 *
 * See Arm® Architecture Reference Manual, ARM DDI 0487E.a D13.2.112~114
 *
 */

#define TCR_EL1_IPS_SHIFT           32U
#define TCR_EL2_PS_SHIFT            16U
#define TCR_EL3_PS_SHIFT            16U

#define TCR_T0SZ_SHIFT              0U
#define TCR_T0SZ(x)                 ((64 - (x)) << TCR_T0SZ_SHIFT)

#define TCR_IRGN_NC                 (0ULL << 8)
#define TCR_IRGN_WBWA               (1ULL << 8)
#define TCR_IRGN_WT                 (2ULL << 8)
#define TCR_IRGN_WBNWA              (3ULL << 8)
#define TCR_IRGN_MASK               (3ULL << 8)
#define TCR_ORGN_NC                 (0ULL << 10)
#define TCR_ORGN_WBWA               (1ULL << 10)
#define TCR_ORGN_WT                 (2ULL << 10)
#define TCR_ORGN_WBNWA              (3ULL << 10)
#define TCR_ORGN_MASK               (3ULL << 10)
#define TCR_SHARED_NON              (0ULL << 12)
#define TCR_SHARED_OUTER            (2ULL << 12)
#define TCR_SHARED_INNER            (3ULL << 12)
#define TCR_TG0_4K                  (0ULL << 14)
#define TCR_TG0_64K                 (1ULL << 14)
#define TCR_TG0_16K                 (2ULL << 14)
#define TCR_EPD1_DISABLE            (1ULL << 23)

#define TCR_AS_SHIFT                36U
#define TCR_ASID_8                  (0ULL << TCR_AS_SHIFT)
#define TCR_ASID_16                 (1ULL << TCR_AS_SHIFT)
#define TCR_TBI0                    (1ULL << 37)
#define TCR_TBI1                    (1ULL << 38)

/* TCMA1 (bit [58]) controls whether memory accesses
 * in the address range [59:55] = 0b11111 are unchecked accesses.
 *
 * TCMA0 (bit [57]) controls whether memory accesses
 * in the address range [59:55] = 0b00000 are unchecked accesses.
 */

#define TCR_TCMA0                   (1ULL << 57)
#define TCR_TCMA1                   (1ULL << 58)

#define TCR_PS_BITS_4GB             0x0ULL
#define TCR_PS_BITS_64GB            0x1ULL
#define TCR_PS_BITS_1TB             0x2ULL
#define TCR_PS_BITS_4TB             0x3ULL
#define TCR_PS_BITS_16TB            0x4ULL
#define TCR_PS_BITS_256TB           0x5ULL

#define CTR_EL0_DMINLINE_SHIFT      16
#define CTR_EL0_DMINLINE_MASK       BIT_MASK(4)
#define CTR_EL0_CWG_SHIFT           24
#define CTR_EL0_CWG_MASK            BIT_MASK(4)

/* clidr_el1 */

#define CLIDR_EL1_LOC_SHIFT          24
#define CLIDR_EL1_LOC_MASK           BIT_MASK(3)
#define CLIDR_EL1_CTYPE_SHIFT(level) ((level) * 3)
#define CLIDR_EL1_CTYPE_MASK         BIT_MASK(3)

/* ccsidr_el1 */

#define CCSIDR_EL1_LN_SZ_SHIFT       0
#define CCSIDR_EL1_LN_SZ_MASK        BIT_MASK(3)
#define CCSIDR_EL1_WAYS_SHIFT        3
#define CCSIDR_EL1_WAYS_MASK         BIT_MASK(10)
#define CCSIDR_EL1_SETS_SHIFT        13
#define CCSIDR_EL1_SETS_MASK         BIT_MASK(15)

/* ttbr0/1_el1 */

#define TTBR_BADDR_SHIFT            (0)
#define TTBR_BADDR_WIDTH            (48)
#define TTBR_BADDR_MASK             (((1UL << TTBR_BADDR_WIDTH) - 1) << TTBR_BADDR_SHIFT)
#define TTBR_ASID_SHIFT             (48)
#define TTBR_ASID_WIDTH             (16)
#define TTBR_ASID_MASK              (((1UL << TTBR_ASID_WIDTH) - 1) << TTBR_ASID_SHIFT)

/* TLBI instruction */

#define TLBI_VADDR_SHIFT            (0)
#define TLBI_VADDR_INPUT_SHIFT      (12) /* From input vaddr to TLBI vaddr field */
#define TLBI_VADDR_WIDTH            (43)
#define TLBI_VADDR_MASK             (((1UL << TLBI_VADDR_WIDTH) - 1) << TLBI_VADDR_SHIFT)
#define TLBI_ASID_SHIFT             (48)
#define TLBI_ASID_WIDTH             (16)
#define TLBI_ASID_MASK              (((1UL << TLBI_ASID_WIDTH) - 1) << TLBI_ASID_SHIFT)

/* Create an argument suitable for TLBI, with vaddr and asid as inputs */

#define TLBI_ARG(vaddr, asid)                                       \
  ({                                                                \
    uintptr_t __arg;                                                \
    __arg  = ((vaddr) >> TLBI_VADDR_INPUT_SHIFT) & TLBI_VADDR_MASK; \
    __arg |= (uintptr_t)(asid) << TLBI_ASID_SHIFT;                  \
    __arg;                                                          \
  })

/* csselr_el1 */

#define CSSELR_EL1_IND_SHIFT         0
#define CSSELR_EL1_IND_MASK          BIT_MASK(1)
#define CSSELR_EL1_LEVEL_SHIFT       1
#define CSSELR_EL1_LEVEL_MASK        BIT_MASK(3)

/* Convenience macros to represent the ARMv8-A-specific
 * configuration for memory access permission and
 * cache-ability attribution.
 */

#define MMU_REGION_ENTRY(_name, _base_pa, _base_va, _size, _attrs) \
  {                                                                \
    .name       = (_name),                                         \
    .base_pa    = (_base_pa),                                      \
    .base_va    = (_base_va),                                      \
    .size       = (_size),                                         \
    .attrs      = (_attrs),                                        \
  }

#define MMU_REGION_FLAT_ENTRY(name, adr, sz, attrs) \
  MMU_REGION_ENTRY(name, adr, adr, sz, attrs)

#define MMU_PAGE_SHIFT              (12U)
#define MMU_PAGE_SIZE               (1U << MMU_PAGE_SHIFT) /* 4K pages */
#define MMU_PAGE_MASK               (MMU_PAGE_SIZE - 1)

/* Entries per PGT */

#define MMU_PAGE_ENTRIES            (MMU_PAGE_SIZE / sizeof(uintptr_t))

/* Amount of page table levels */

#define MMU_PGT_LEVELS              (4U)
#define MMU_PGT_LEVEL_MAX           (3U) /* Levels go from 0-3 */

/* Page sizes per page table level */

#define MMU_L0_PAGE_SIZE            (0x8000000000) /* 512G */
#define MMU_L1_PAGE_SIZE            (0x40000000)   /* 1G */
#define MMU_L2_PAGE_SIZE            (0x200000)     /* 2M */
#define MMU_L3_PAGE_SIZE            (0x1000)       /* 4K */

/* Flags for user page tables */

#define MMU_UPGT_FLAGS              (PTE_TABLE_DESC)

/* Flags for normal memory region */

#define MMU_MT_NORMAL_FLAGS         (PTE_PAGE_DESC | PTE_BLOCK_DESC_AF | PTE_BLOCK_DESC_INNER_SHARE | PTE_BLOCK_DESC_MEMTYPE(MT_NORMAL))

/* Flags for user FLASH (RX) and user RAM (RW) */

#define MMU_UTEXT_FLAGS             (PTE_BLOCK_DESC_AP_RO | PTE_BLOCK_DESC_PXN | PTE_BLOCK_DESC_AP_USER | PTE_BLOCK_DESC_NG | MMU_MT_NORMAL_FLAGS)
#define MMU_UDATA_FLAGS             (PTE_BLOCK_DESC_AP_RW | PTE_BLOCK_DESC_PXN | PTE_BLOCK_DESC_AP_USER | PTE_BLOCK_DESC_NG | PTE_BLOCK_DESC_UXN | MMU_MT_NORMAL_FLAGS)

/* I/O region flags */

#define MMU_IO_FLAGS                (PTE_BLOCK_DESC_AP_RW | PTE_BLOCK_DESC_UXN | PTE_BLOCK_DESC_PXN)

/* Flags for kernel page tables */

#define MMU_KPGT_FLAGS              (PTE_TABLE_DESC)

/* Kernel FLASH and RAM are mapped globally */

#define MMU_KTEXT_FLAGS             (PTE_BLOCK_DESC_AP_RO | PTE_BLOCK_DESC_UXN | MMU_MT_NORMAL_FLAGS)
#define MMU_KDATA_FLAGS             (PTE_BLOCK_DESC_AP_RW | PTE_BLOCK_DESC_UXN | MMU_MT_NORMAL_FLAGS)

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Region definition data structure */

struct arm_mmu_region
{
  /* Region Base Physical Address */

  uint64_t base_pa;

  /* Region Base Virtual Address */

  uint64_t base_va;

  /* Region size */

  uint64_t size;

  /* Region Name */

  const char *name;

  /* Region Attributes */

  unsigned int attrs;
};

/* MMU configuration data structure */

struct arm_mmu_config
{
  /* Number of regions */

  uint32_t num_regions;

  /* Regions */

  const struct arm_mmu_region *mmu_regions;
};

struct arm_mmu_ptables
{
  uint64_t *base_xlat_table;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Reference to the MMU configuration.
 *
 * This struct is defined and populated for each SoC,
 * and holds the build-time configuration information for the fixed MMU
 * regions enabled during kernel initialization.
 */

extern const struct arm_mmu_config g_mmu_config;

/* Kernel page directory root */

extern uintptr_t g_kernel_pgt_pbase;

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmu_ttbr_reg
 *
 * Description:
 *   Utility function to build ttbr register value for input parameters
 *
 * Input Parameters:
 *   pgbase - The physical base address of the translation table base
 *   asid - Address space identifier. This can be used to identify different
 *     address spaces.
 *
 ****************************************************************************/

static inline uintptr_t mmu_ttbr_reg(uintptr_t pgbase, uint16_t asid)
{
  uintptr_t reg;
  reg  = ((pgbase << TTBR_BADDR_SHIFT) & TTBR_BADDR_MASK);
  reg |= (((uintptr_t)asid << TTBR_ASID_SHIFT) & TTBR_ASID_MASK);
  return reg;
}

/****************************************************************************
 * Name: mmu_invalidate_tlb_by_vaddr
 *
 * Description:
 *   Flush the TLB for vaddr entry
 *
 * Input Parameters:
 *   vaddr - The virtual address to flush
 *
 ****************************************************************************/

static inline void mmu_invalidate_tlb_by_vaddr(uintptr_t vaddr)
{
  __asm__ __volatile__
    (
      "dsb ishst\n"
      "tlbi vale1is, %0\n"
      "dsb ish\n"
      "isb"
      :
      : "r" (TLBI_ARG(vaddr, 0))
      : "memory"
    );
}

/****************************************************************************
 * Name: mmu_invalidate_tlbs
 *
 * Description:
 *   Flush the entire TLB
 *
 ****************************************************************************/

static inline void mmu_invalidate_tlbs(void)
{
  __asm__ __volatile__
    (
      "dsb nshst\n"
      "tlbi vmalle1\n"
      "dsb nsh\n"
      "isb"
      :
      :
      : "memory"
    );
}

/****************************************************************************
 * Name: mmu_write_ttbr0
 *
 * Description:
 *   Write ttbr0
 *
 * Input Parameters:
 *   reg - ttbr0 value
 *
 ****************************************************************************/

static inline void mmu_write_ttbr0(uintptr_t reg)
{
  write_sysreg(reg, ttbr0_el1);
  mmu_invalidate_tlbs();
}

/****************************************************************************
 * Name: mmu_read_ttbr0
 *
 * Description:
 *   Read ttbr0
 *
 * Returned Value:
 *   ttbr0 register value
 *
 ****************************************************************************/

static inline uintptr_t mmu_read_ttbr0(void)
{
  return read_sysreg(ttbr0_el1);
}

/****************************************************************************
 * Name: mmu_enable
 *
 * Description:
 *   Enable MMU and set the base page table address
 *
 * Input Parameters:
 *   pgbase - The physical base address of the translation table base
 *   asid - Address space identifier. This can be used to identify different
 *     address spaces.
 *
 ****************************************************************************/

static inline void mmu_enable(uintptr_t pgbase, uint16_t asid)
{
  uintptr_t reg = mmu_ttbr_reg(pgbase, asid);

  /* Commit to ttbr0 and synchronize */

  mmu_write_ttbr0(reg);
}

/****************************************************************************
 * Name: mmu_pte_to_paddr
 *
 * Description:
 *   Extract physical address from PTE
 *
 * Input Parameters:
 *   pte - Page table entry
 *
 * Returned Value:
 *   Physical address from PTE
 *
 ****************************************************************************/

static inline uintptr_t mmu_pte_to_paddr(uintptr_t pte)
{
  uintptr_t paddr = pte;
  paddr &= PTE_PADDR_MASK;  /* Remove flags */
  return paddr;
}

/****************************************************************************
 * Name: mmu_ttbr_to_paddr
 *
 * Description:
 *   Extract physical address from TTBR0/1
 *
 * Returned Value:
 *   Physical address from TTBR0/1 value
 *
 ****************************************************************************/

static inline uintptr_t mmu_ttbr_to_paddr(uintptr_t ttbr)
{
  uintptr_t baddr;
  baddr = ttbr;
  baddr = ((baddr >> TTBR_BADDR_SHIFT) & TTBR_BADDR_MASK);
  return baddr;
}

/****************************************************************************
 * Name: mmu_get_ttbr0_pgbase
 *
 * Description:
 *   Utility function to read the current base page table physical address
 *
 * Returned Value:
 *   Physical address of the current base page table
 *
 ****************************************************************************/

static inline uintptr_t mmu_get_ttbr0_pgbase(void)
{
  return mmu_ttbr_to_paddr(read_sysreg(ttbr0_el1));
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int arm64_mmu_init(bool is_primary_core);
int arm64_mmu_set_memregion(const struct arm_mmu_region *region);

/****************************************************************************
 * Name: mmu_ln_setentry
 *
 * Description:
 *   Set a level n translation table entry.
 *
 * Input Parameters:
 *   ptlevel - The translation table level, amount of levels is
 *     MMU implementation specific
 *   lnvaddr - The virtual address of the beginning of the page table at
 *     level n
 *   paddr - The physical address to be mapped. Must be aligned to a PPN
 *     address boundary which is dependent on the level of the entry
 *   vaddr - The virtual address to be mapped. Must be aligned to a PPN
 *     address boundary which is dependent on the level of the entry
 *   mmuflags - The MMU flags to use in the mapping.
 *
 ****************************************************************************/

void mmu_ln_setentry(uint32_t ptlevel, uintptr_t lnvaddr, uintptr_t paddr,
                     uintptr_t vaddr, uint64_t mmuflags);

/****************************************************************************
 * Name: mmu_ln_getentry
 *
 * Description:
 *   Get a level n translation table entry.
 *
 * Input Parameters:
 *   ptlevel - The translation table level, amount of levels is
 *     MMU implementation specific
 *   lnvaddr - The virtual address of the beginning of the page table at
 *     level n
 *   vaddr - The virtual address to get pte for. Must be aligned to a PPN
 *     address boundary which is dependent on the level of the entry
 *
 ****************************************************************************/

uintptr_t mmu_ln_getentry(uint32_t ptlevel, uintptr_t lnvaddr,
                          uintptr_t vaddr);

/****************************************************************************
 * Name: mmu_ln_restore
 *
 * Description:
 *   Restore a level n translation table entry.
 *
 * Input Parameters:
 *   ptlevel - The translation table level, amount of levels is
 *     MMU implementation specific
 *   lnvaddr - The virtual address of the beginning of the page table at
 *     level n
 *   vaddr - The virtual address to get pte for. Must be aligned to a PPN
 *     address boundary which is dependent on the level of the entry
 *   entry - Entry to restore, previously obtained by mmu_ln_getentry
 *
 ****************************************************************************/

void mmu_ln_restore(uint32_t ptlevel, uintptr_t lnvaddr, uintptr_t vaddr,
                    uintptr_t entry);

/****************************************************************************
 * Name: mmu_ln_clear
 *
 * Description:
 *   Unmap a level n translation table entry.
 *
 * Input Parameters:
 *   ptlevel - The translation table level, amount of levels is
 *     MMU implementation specific
 *   lnvaddr - The virtual address of the beginning of the page table at
 *     level n
 *   vaddr - The virtual address to get pte for. Must be aligned to a PPN
 *     address boundary which is dependent on the level of the entry
 *
 ****************************************************************************/

#define mmu_ln_clear(ptlevel, lnvaddr, vaddr) \
  mmu_ln_restore(ptlevel, lnvaddr, vaddr, 0)

/****************************************************************************
 * Name: mmu_get_region_size
 *
 * Description:
 *   Get (giga/mega) page size for level n.
 *
 * Input Parameters:
 *   ptlevel - The translation table level, amount of levels is
 *     MMU implementation specific
 *
 * Returned Value:
 *   Region size for one page at level n.
 *
 ****************************************************************************/

size_t mmu_get_region_size(uint32_t ptlevel);

/****************************************************************************
 * Name: mmu_get_base_pgt_level
 *
 * Description:
 *   Get the base translation table level. The ARM64 MMU implementation
 *   optimizes the amount of translation table levels in use, based on the
 *   configured virtual address range (CONFIG_ARM64_VA_BITS).
 *
 *   Table indices range from 0...3 and the lowest table indices are dropped
 *   as needed. If CONFIG_ARM64_VA_BITS >= 40, all 4 translation table levels
 *   are needed.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   The base translation table level.
 *
 ****************************************************************************/

uintptr_t mmu_get_base_pgt_level(void);

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_COMMON_ARM64_MMU_H */
