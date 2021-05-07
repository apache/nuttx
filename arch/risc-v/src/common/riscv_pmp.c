/****************************************************************************
 * arch/risc-v/src/common/riscv_pmp.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <arch/csr.h>

#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PMP_CFG_BITS_CNT        (8)
#define PMP_CFG_FLAG_MASK       (0xFF)

#define PMP_CFG_CNT_IN_REG      (__riscv_xlen / PMP_CFG_BITS_CNT)

#define PMP_MASK_SET_ONE_REGION(region, attr, reg) \
  do { \
      uintptr_t offset = region % PMP_CFG_CNT_IN_REG; \
      reg &= ~(PMP_CFG_FLAG_MASK << (offset * PMP_CFG_BITS_CNT)); \
      reg |= attr << (offset * PMP_CFG_BITS_CNT); \
    } while(0);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_config_pmp_region
 *
 * Description:
 *   This function will set the specific PMP region with the desired cfg.
 *
 * Input Parameters:
 *   region - The region index number.
 *   attr - The region configurations.
 *   base - The base address of the region.
 *   size - The memory length of the region.
 *   For the NAPOT mode, the base address must aligned to the size boundary,
 *   and the size must be power-of-two according to the the PMP spec.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void riscv_config_pmp_region(uintptr_t region, uintptr_t attr,
                             uintptr_t base, uintptr_t size)
{
  uintptr_t addr = 0;
  uintptr_t cfg = 0;

  /* TODO: check the base address alignment and size */

  addr = base >> 2;
  if (PMPCFG_A_NAPOT == (attr & PMPCFG_A_MASK))
    {
      addr |= (size - 1) >> 3;
    }

  switch (region)
    {
      case 0:
        WRITE_CSR(pmpaddr0, addr);
        break;

      case 1:
        WRITE_CSR(pmpaddr1, addr);
        break;

      case 2:
        WRITE_CSR(pmpaddr2, addr);
        break;

      case 3:
        WRITE_CSR(pmpaddr3, addr);
        break;

      case 4:
        WRITE_CSR(pmpaddr4, addr);
        break;

      case 5:
        WRITE_CSR(pmpaddr5, addr);
        break;

      case 6:
        WRITE_CSR(pmpaddr6, addr);
        break;

      case 7:
        WRITE_CSR(pmpaddr7, addr);
        break;

      case 8:
        WRITE_CSR(pmpaddr8, addr);
        break;

      case 9:
        WRITE_CSR(pmpaddr9, addr);
        break;

      case 10:
        WRITE_CSR(pmpaddr10, addr);
        break;

      case 11:
        WRITE_CSR(pmpaddr11, addr);
        break;

      case 12:
        WRITE_CSR(pmpaddr12, addr);
        break;

      case 13:
        WRITE_CSR(pmpaddr13, addr);
        break;

      case 14:
        WRITE_CSR(pmpaddr14, addr);
        break;

      case 15:
        WRITE_CSR(pmpaddr15, addr);
        break;

      default:
        break;
    }

# if (__riscv_xlen == 32)
  switch (region)
    {
      case 0 ... 3:
        cfg = READ_CSR(pmpcfg0);
        PMP_MASK_SET_ONE_REGION(region, attr, cfg);
        WRITE_CSR(pmpcfg0, cfg);
        break;

      case 4 ... 7:
        cfg = READ_CSR(pmpcfg1);
        PMP_MASK_SET_ONE_REGION(region, attr, cfg);
        WRITE_CSR(pmpcfg1, cfg);
        break;

      case 8 ... 11:
        cfg = READ_CSR(pmpcfg2);
        PMP_MASK_SET_ONE_REGION(region, attr, cfg);
        WRITE_CSR(pmpcfg2, cfg);
        break;

      case 12 ... 15:
        cfg = READ_CSR(pmpcfg3);
        PMP_MASK_SET_ONE_REGION(region, attr, cfg);
        WRITE_CSR(pmpcfg3, cfg);
        break;

      default:
        break;
    }
# elif (__riscv_xlen == 64)
  switch (region)
    {
      case 0 ... 7:
        cfg = READ_CSR(pmpcfg0);
        PMP_MASK_SET_ONE_REGION(region, attr, cfg);
        WRITE_CSR(pmpcfg0, cfg);
        break;

      case 8 ... 15:
        cfg = READ_CSR(pmpcfg2);
        PMP_MASK_SET_ONE_REGION(region, attr, cfg);
        WRITE_CSR(pmpcfg2, cfg);
        break;

      default:
        break;
    }
# else
#   error "XLEN of risc-v not supported"
# endif

  /* fence is needed when page-based virtual memory is implemented */

  __asm volatile("sfence.vma x0, x0" : : : "memory");
}
