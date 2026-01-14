/****************************************************************************
 * arch/risc-v/src/common/riscv_aia.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_RISCV_AIA_H
#define __ARCH_RISCV_SRC_COMMON_RISCV_AIA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/bits.h>
#include <arch/csr.h>
#include "riscv_internal.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* AIA IMSIC */

#define RISCV_IMSIC_MAX_REGS                    16

#define RISCV_IMSIC_MMIO_PAGE_LE                0x00
#define RISCV_IMSIC_MMIO_PAGE_BE                0x04
#define RISCV_IMSIC_MMIO_PAGE_BIT               12

#define RISCV_IMSIC_TOPEI_ID_BIT                16

#define RISCV_IMSIC_EIP_BITS                    32
#define RISCV_IMSIC_EIE_BITS                    32

#define RISCV_IMSIC_DISABLE_EIDELIVERY          0
#define RISCV_IMSIC_ENABLE_EIDELIVERY           1
#define RISCV_IMSIC_DISABLE_EITHRESHOLD         1
#define RISCV_IMSIC_ENABLE_EITHRESHOLD          0

#define RISCV_IMSIC_IPI_ID                      1

/* AIA APLIC */

#define RISCV_APLIC_MAX_DELEGATE                16

#define RISCV_APLIC_MAX_IDC                     (1UL << 14)
#define RISCV_APLIC_MAX_SOURCE                  1024

#define RISCV_APLIC_DOMAINCFG                   0x0000
#define RISCV_APLIC_DOMAINCFG_IE                (1 << 8)
#define RISCV_APLIC_DOMAINCFG_DM                (1 << 2)
#define RISCV_APLIC_DOMAINCFG_BE                (1 << 0)

#define RISCV_APLIC_SOURCECFG_BASE              0x0004
#define RISCV_APLIC_SOURCECFG_D                 (1 << 10)
#define RISCV_APLIC_SOURCECFG_CHILDIDX_MASK     0x000003ff
#define RISCV_APLIC_SOURCECFG_SM_MASK           0x00000007
#define RISCV_APLIC_SOURCECFG_SM_INACTIVE       0x0
#define RISCV_APLIC_SOURCECFG_SM_DETACH         0x1
#define RISCV_APLIC_SOURCECFG_SM_EDGE_RISE      0x4
#define RISCV_APLIC_SOURCECFG_SM_EDGE_FALL      0x5
#define RISCV_APLIC_SOURCECFG_SM_LEVEL_HIGH     0x6
#define RISCV_APLIC_SOURCECFG_SM_LEVEL_LOW      0x7

#define RISCV_APLIC_MMSICFGADDR                 0x1bc0
#define RISCV_APLIC_MMSICFGADDRH                0x1bc4
#define RISCV_APLIC_SMSICFGADDR                 0x1bc8
#define RISCV_APLIC_SMSICFGADDRH                0x1bcc

#define RISCV_APLIC_MSICFGADDRH_L               (1UL << 31)
#define RISCV_APLIC_MSICFGADDRH_HHXS_SHIFT      24
#define RISCV_APLIC_MSICFGADDRH_LHXS_SHIFT      20
#define RISCV_APLIC_MSICFGADDRH_HHXW_SHIFT      16
#define RISCV_APLIC_MSICFGADDRH_LHXW_SHIFT      12

#define RISCV_APLIC_MSICFGADDR_PPN_SHIFT        12

#define RISCV_APLIC_SETIP_BASE                  0x1c00
#define RISCV_APLIC_SETIPNUM                    0x1cdc

#define RISCV_APLIC_CLRIP_BASE                  0x1d00
#define RISCV_APLIC_CLRIPNUM                    0x1ddc

#define RISCV_APLIC_SETIE_BASE                  0x1e00
#define RISCV_APLIC_SETIENUM                    0x1edc

#define RISCV_APLIC_CLRIE_BASE                  0x1f00
#define RISCV_APLIC_CLRIENUM                    0x1fdc

#define RISCV_APLIC_SETIPNUM_LE                 0x2000
#define RISCV_APLIC_SETIPNUM_BE                 0x2004

#define RISCV_APLIC_TARGET_BASE                 0x3004
#define RISCV_APLIC_TARGET_HART_IDX_SHIFT       18
#define RISCV_APLIC_TARGET_GUEST_IDX_SHIFT      12

#define RISCV_APLIC_IDC_BASE                    0x4000
#define RISCV_APLIC_IDC_SIZE                    32

#define RISCV_APLIC_IDC_IDELIVERY               0x00

#define RISCV_APLIC_IDC_IFORCE                  0x04

#define RISCV_APLIC_IDC_ITHRESHOLD              0x08

#define RISCV_APLIC_IDC_TOPI                    0x18
#define RISCV_APLIC_IDC_TOPI_ID_SHIFT           16

#define RISCV_APLIC_IDC_CLAIMI                  0x1c

#define RISCV_APLIC_DEFAULT_PRIORITY            1
#define RISCV_APLIC_DISABLE_IDELIVERY           0
#define RISCV_APLIC_ENABLE_IDELIVERY            1
#define RISCV_APLIC_DISABLE_ITHRESHOLD          1
#define RISCV_APLIC_ENABLE_ITHRESHOLD           0

#define RISCV_APLIC_IDC(base, i)  \
  ((base) + RISCV_APLIC_IDC_BASE + RISCV_APLIC_IDC_SIZE * (i))

/****************************************************************************
 * Public Types
 ****************************************************************************/

union aplic_mode_arg_u
{
  struct
  {
    uint32_t idelivery;
    uint32_t ithreshold;
  } direct;

  struct
  {
    uint64_t imsic_ppn;
    uint32_t lhxs;
    uint32_t lhxw;
    uint32_t hhxs;
    uint32_t hhxw;
  } msi;
};

/****************************************************************************
 * Public Function
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* IMSIC */
#define riscv_imsic_csr_write(reg, val)   WRITE_INDIRECT_CSR_REG0(reg, val)
#define riscv_imsic_csr_read(reg, val)    READ_INDIRECT_CSR_REG0(reg, val)
#define riscv_imsic_csr_set(reg, val)     SET_INDIRECT_CSR_REG0(reg, val)
#define riscv_imsic_csr_clear(reg, val)   CLEAR_INDIRECT_CSR_REG0(reg, val)

void riscv_imsic_local_eix_update(unsigned long base_id,
                                  unsigned long num_id,
                                  bool pend, bool val);

static inline void riscv_imsic_local_eie_update(unsigned long base_id,
           unsigned long num_id, bool val)
{
  return riscv_imsic_local_eix_update(base_id, num_id, false, val);
}

static inline void riscv_imsic_local_eip_update(unsigned long base_id,
           unsigned long num_id, bool val)
{
  return riscv_imsic_local_eix_update(base_id, num_id, true, val);
}

static inline void riscv_imsic_local_eie_enable(unsigned long base_id)
{
  return riscv_imsic_local_eie_update(base_id, 1, true);
}

static inline void riscv_imsic_local_eie_disable(unsigned long base_id)
{
  return riscv_imsic_local_eie_update(base_id, 1, false);
}

void riscv_imsic_send_ipi(int cpu);

/* APLIC */

static inline int riscv_aplic_set_delegate(uintptr_t base,
                                           uint32_t first_irq,
                                           uint32_t last_irq,
                                           uint32_t child_id)
{
  uint32_t j;
  uintptr_t sourcecfg_base = base + RISCV_APLIC_SOURCECFG_BASE;

  if (!first_irq || !last_irq)
    return 0;

  if (RISCV_APLIC_SOURCECFG_CHILDIDX_MASK < child_id)
    return -EINVAL;

  for (j = first_irq; j <= last_irq; j++)
  {
    putreg32(RISCV_APLIC_SOURCECFG_D | child_id,
             sourcecfg_base + (j - 1) * sizeof(uint32_t));
  }

  return 0;
}

static inline void riscv_aplic_enable_irq(uintptr_t base, uint32_t irq)
{
  putreg32(irq, base + RISCV_APLIC_SETIENUM);
}

static inline void riscv_aplic_disable_irq(uintptr_t base, uint32_t irq)
{
  putreg32(irq, base + RISCV_APLIC_CLRIENUM);
}

static inline void riscv_aplic_configure_irq(uintptr_t base, uint32_t irq,
                                             uint32_t mode, uint32_t hartid)
{
  uint32_t val = (hartid << RISCV_APLIC_TARGET_HART_IDX_SHIFT) | irq;
  putreg32(mode, base + RISCV_APLIC_SOURCECFG_BASE
                      + (irq - 1) * sizeof(uint32_t));
  putreg32(val, base + RISCV_APLIC_TARGET_BASE
                     + (irq - 1) * sizeof(uint32_t));
}

static inline void riscv_aplic_disable_irqs(uintptr_t base,
                                            uint32_t num_source)
{
  uint32_t i;

  putreg32(0, base + RISCV_APLIC_DOMAINCFG);

  /* Disable all interrupts */

  for (i = 0; i < num_source; i += 32)
    {
      putreg32(-1U, base + RISCV_APLIC_CLRIE_BASE
                         + (i / 32) * sizeof(uint32_t));
    }
}

/****************************************************************************
 * Name: riscv_aplic_init
 *
 * Description:
 *   Init APLIC to direct mode
 *
 ****************************************************************************/

void riscv_aplic_init(uintptr_t base,
                      uint32_t idelivery, uint32_t ithreshold);

/****************************************************************************
 * Name: riscv_aplic_init
 *
 * Description:
 *   Init APLIC to msi mode
 *
 ****************************************************************************/

void riscv_aplic_init_msi(uintptr_t base, uint64_t imsic_addr,
                          uint32_t lhxs, uint32_t lhxw,
                          uint32_t hhxs, uint32_t hhxw);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_COMMON_RISCV_AIA_H */
