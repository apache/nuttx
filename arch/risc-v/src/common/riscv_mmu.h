/****************************************************************************
 * arch/risc-v/src/common/riscv_mmu.h
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

#ifndef ___ARCH_RISC_V_SRC_COMMON_RISCV_MMU_H_
#define ___ARCH_RISC_V_SRC_COMMON_RISCV_MMU_H_

/* RV32/64 page size */

#define RV_MMU_PAGE_SHIFT       (12)
#define RV_MMU_PAGE_SIZE        (1 << RV_MMU_PAGE_SHIFT) /* 4K pages */
#define RV_MMU_PAGE_MASK        (RV_MMU_PAGE_SIZE - 1)

/* Entries per PGT */

#define RV_MMU_PAGE_ENTRIES     (RV_MMU_PAGE_SIZE / sizeof(uintptr_t))

/* Supervisor Address Translation and Protection (satp) */

#define SATP_PPN_SHIFT          (0)
#define SATP_PPN_MASK           (((1ul << 44) - 1) << SATP_PPN_SHIFT)
#define SATP_ASID_SHIFT         (44)
#define SATP_ASID_MASK          (((1ul << 16) - 1) << SATP_ASID_SHIFT)
#define SATP_MODE_SHIFT         (60)
#define SATP_MODE_MASK          (((1ul << 4) - 1) << SATP_MODE_SHIFT)

/* Modes, for RV32 only 1 is valid, for RV64 1-7 and 10-15 are reserved */

#define SATP_MODE_BARE          (0ul)
#define SATP_MODE_SV32          (1ul)
#define SATP_MODE_SV39          (8ul)
#define SATP_MODE_SV48          (9ul)

/* satp address to PPN translation */

#define SATP_ADDR_TO_PPN(_addr) ((_addr) >> RV_MMU_PAGE_SHIFT)
#define SATP_PPN_TO_ADDR(_ppn)  ((_ppn)  << RV_MMU_PAGE_SHIFT)

/* Common Page Table Entry (PTE) bits */

#define PTE_VALID               (1 << 0) /* PTE is valid */
#define PTE_R                   (1 << 1) /* Page is readable */
#define PTE_W                   (1 << 2) /* Page is writable */
#define PTE_X                   (1 << 3) /* Page is executable */
#define PTE_U                   (1 << 4) /* Page is a user mode page */
#define PTE_G                   (1 << 5) /* Page is a global mapping */
#define PTE_A                   (1 << 6) /* Page has been accessed */
#define PTE_D                   (1 << 7) /* Page is dirty */

/* Check if leaf PTE entry or not (if X/W/R are set it is) */

#define PTE_LEAF_MASK           (7 << 1)

/* Flags for user page tables */

#define MMU_UPGT_FLAGS          (0)

/* Flags for user FLASH (RX) and user RAM (RW) */

#define MMU_UTEXT_FLAGS         (PTE_R | PTE_X | PTE_U)
#define MMU_UDATA_FLAGS         (PTE_R | PTE_W | PTE_U)

/* I/O region flags */

#define MMU_IO_FLAGS            (PTE_R | PTE_W | PTE_G)

/* Kernel FLASH and RAM are mapped globally */

#define MMU_KTEXT_FLAGS         (PTE_R | PTE_X | PTE_G)
#define MMU_KDATA_FLAGS         (PTE_R | PTE_W | PTE_G)

/* SvX definitions, only Sv39 is currently supported, but it should be
 * trivial to extend the driver to support other SvX implementations
 *
 * Sv39 has:
 * - 4K page size
 * - 3 page table levels
 * - 9-bit VPN width
 */

#ifdef CONFIG_ARCH_MMU_TYPE_SV39
#define RV_MMU_PTE_PADDR_SHIFT  (10)
#define RV_MMU_PTE_PPN_MASK     (((1ul << 44) - 1) << RV_MMU_PTE_PADDR_SHIFT)
#define RV_MMU_PTE_PPN_SHIFT    (2)
#define RV_MMU_VPN_WIDTH        (9)
#define RV_MMU_VPN_MASK         ((1ul << RV_MMU_VPN_WIDTH) - 1)
#define RV_MMU_PT_LEVELS        (3)
#define RV_MMU_VADDR_SHIFT(_n)  (RV_MMU_PAGE_SHIFT + RV_MMU_VPN_WIDTH * \
                                 (RV_MMU_PT_LEVELS - (_n)))
#define RV_MMU_SATP_MODE        (SATP_MODE_SV39)
#define RV_MMU_L1_PAGE_SIZE     (0x40000000) /* 1G */
#define RV_MMU_L2_PAGE_SIZE     (0x200000)   /* 2M */
#define RV_MMU_L3_PAGE_SIZE     (0x1000)     /* 4K */

/* Minimum alignment requirement for any section of memory is 2MB */

#define RV_MMU_SECTION_ALIGN    (RV_MMU_L2_PAGE_SIZE)
#else
#error "Unsupported RISC-V MMU implementation selected"
#endif /* CONFIG_ARCH_MMU_TYPE_SV39 */

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uintptr_t g_kernel_pgt_pbase;

/****************************************************************************
 * Name: mmu_satp_reg
 *
 * Description:
 *   Utility function to build satp register value for input parameters
 *
 * Input Parameters:
 *   pgbase - The physical base address of the translation table base
 *   asid - Address space identifier. This can be used to identify different
 *     address spaces. It is not necessary to use this, nor is it necessary
 *     for the RISC-V implementation to implement such bits. This means in
 *     practice that the value should not be used in this generic driver.
 *
 ****************************************************************************/

static inline uintptr_t mmu_satp_reg(uintptr_t pgbase, uint16_t asid)
{
  uintptr_t reg;
  reg  = ((RV_MMU_SATP_MODE << SATP_MODE_SHIFT) & SATP_MODE_MASK);
  reg |= (((uintptr_t)asid << SATP_ASID_SHIFT) & SATP_ASID_MASK);
  reg |= ((SATP_ADDR_TO_PPN(pgbase) << SATP_PPN_SHIFT) & SATP_PPN_MASK);
  return reg;
}

/****************************************************************************
 * Name: mmu_write_satp
 *
 * Description:
 *   Write satp
 *
 * Input Parameters:
 *   reg - satp value
 *
 ****************************************************************************/

static inline void mmu_write_satp(uintptr_t reg)
{
  __asm__ __volatile__
    (
      "csrw satp, %0\n"
      "sfence.vma x0, x0\n"
      "fence\n"
      :
      : "rK" (reg)
      : "memory"
    );
}

/****************************************************************************
 * Name: mmu_read_satp
 *
 * Description:
 *   Read satp
 *
 * Returned Value:
 *   satp register value
 *
 ****************************************************************************/

static inline uintptr_t mmu_read_satp(void)
{
  uintptr_t reg;

  __asm__ __volatile__
    (
      "csrr %0, satp\n"
      : "=r" (reg)
      :
      : "memory"
    );

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
      "sfence.vma x0, %0\n"
      :
      : "rK" (vaddr)
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
      "sfence.vma x0, x0\n"
      :
      :
      : "memory"
    );
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
 *     address spaces. It is not necessary to use this, nor is it necessary
 *     for the RISC-V implementation to implement such bits. This means in
 *     practice that the value should not be used in this generic driver.
 *
 ****************************************************************************/

static inline void mmu_enable(uintptr_t pgbase, uint16_t asid)
{
  uintptr_t reg = mmu_satp_reg(pgbase, asid);

  /* Commit to satp and synchronize */

  mmu_write_satp(reg);
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
  paddr  &= RV_MMU_PTE_PPN_MASK;  /* Remove flags */
  paddr <<= RV_MMU_PTE_PPN_SHIFT; /* Move to correct position */
  return paddr;
}

/****************************************************************************
 * Name: mmu_get_satp_pgbase
 *
 * Description:
 *   Utility function to read the base page table physical address
 *
 * Returned Value:
 *   Physical address of the current base page table
 *
 ****************************************************************************/

static inline uintptr_t mmu_get_satp_pgbase(void)
{
  uintptr_t ppn;
  ppn = mmu_read_satp();
  ppn = ((ppn >> SATP_PPN_SHIFT) & SATP_PPN_MASK);
  return SATP_PPN_TO_ADDR(ppn);
}

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
                     uintptr_t vaddr, uint32_t mmuflags);

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
 * Name: mmu_ln_map_region
 *
 * Description:
 *   Set a translation table region for level n
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
 *   size - The size of the region in bytes
 *   mmuflags - The MMU flags to use in the mapping.
 *
 ****************************************************************************/

void mmu_ln_map_region(uint32_t ptlevel, uintptr_t lnvaddr, uintptr_t paddr,
                       uintptr_t vaddr, size_t size, uint32_t mmuflags);

#endif /* ___ARCH_RISC_V_SRC_COMMON_RISCV_MMU_H_ */
