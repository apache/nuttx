/****************************************************************************
 * arch/risc-v/src/common/riscv_aplic.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "riscv_internal.h"
#include "riscv_aia.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void riscv_aplic_init(uintptr_t base,
                      uint32_t idelivery, uint32_t ithreshold)
{
  int i;

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      uintptr_t idc_base = RISCV_APLIC_IDC(base, i);

      putreg32(idelivery, idc_base + RISCV_APLIC_IDC_IDELIVERY);
      putreg32(0, idc_base + RISCV_APLIC_IDC_IFORCE);
      putreg32(ithreshold, idc_base + RISCV_APLIC_IDC_ITHRESHOLD);
    }
}

void riscv_aplic_init_msi(uintptr_t base, uint64_t imsic_addr,
                          uint32_t lhxs, uint32_t lhxw,
                          uint32_t hhxs, uint32_t hhxw)
{
    uint32_t tmp;

    tmp = imsic_addr >> RISCV_IMSIC_MMIO_PAGE_BIT;
    putreg32(tmp, base + RISCV_APLIC_MMSICFGADDR);
    tmp = (uint32_t)(imsic_addr >> 32) |
            (lhxw << RISCV_APLIC_MSICFGADDRH_LHXW_SHIFT) |
            (lhxs << RISCV_APLIC_MSICFGADDRH_LHXS_SHIFT) |
            (hhxw << RISCV_APLIC_MSICFGADDRH_HHXW_SHIFT) |
            (hhxs << RISCV_APLIC_MSICFGADDRH_HHXS_SHIFT);
    putreg32(tmp, base + RISCV_APLIC_MMSICFGADDRH);
}
