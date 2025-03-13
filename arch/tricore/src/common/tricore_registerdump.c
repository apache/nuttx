/****************************************************************************
 * arch/tricore/src/common/tricore_registerdump.c
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

#include <nuttx/config.h>

#include <stdio.h>
#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "tricore_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tricore_dump_upcsa
 ****************************************************************************/

void tricore_dump_upcsa(volatile uint32_t *regs)
{
  _alert("UPCXI:%-13.8" PRIX32 "PSW:%-15.8" PRIX32
         "SP:%-16.8" PRIX32 "A11:%-15.8" PRIX32 "\n",
         regs[REG_UPCXI], regs[REG_PSW], regs[REG_A10], regs[REG_UA11]);
  _alert("D8:%-16.8" PRIX32 "D9:%-16.8" PRIX32
         "D10:%-15.8" PRIX32 "D11:%-15.8" PRIX32 "\n",
         regs[REG_D8], regs[REG_D9], regs[REG_D10], regs[REG_D11]);
  _alert("A12:%-15.8" PRIX32 "A13:%-15.8" PRIX32
         "A14:%-15.8" PRIX32 "A15:%-15.8" PRIX32 "\n",
         regs[REG_A12], regs[REG_A13], regs[REG_A14], regs[REG_A15]);
  _alert("D12:%-15.8" PRIX32 "D13:%-15.8" PRIX32
         "D14:%-15.8" PRIX32 "D15:%-15.8" PRIX32 "\n\n",
         regs[REG_D12], regs[REG_D13], regs[REG_D14], regs[REG_D15]);
}

/****************************************************************************
 * Name: tricore_dump_lowcsa
 ****************************************************************************/

void tricore_dump_lowcsa(volatile uint32_t *regs)
{
  _alert("LPCXI:%-13.8" PRIX32 "PC:%-16.8" PRIX32
         "A2:%-16.8" PRIX32 "A3:%-16.8" PRIX32 "\n",
         regs[REG_LPCXI] | PCXI_UL, regs[REG_LA11],
         regs[REG_A2], regs[REG_A3]);
  _alert("D0:%-16.8" PRIX32 "D1:%-16.8" PRIX32
         "D2:%-16.8" PRIX32 "D3:%-16.8" PRIX32 "\n",
         regs[REG_D0], regs[REG_D1], regs[REG_D2], regs[REG_D3]);
  _alert("A4:%-16.8" PRIX32 "A5:%-16.8" PRIX32
         "A6:%-16.8" PRIX32 "A7:%-16.8" PRIX32 "\n",
         regs[REG_A4], regs[REG_A5], regs[REG_A6], regs[REG_A7]);
  _alert("D4:%-16.8" PRIX32 "D5:%-16.8" PRIX32
         "D6:%-16.8" PRIX32 "D7:%-16.8" PRIX32 "\n\n",
         regs[REG_D4], regs[REG_D5], regs[REG_D6], regs[REG_D7]);
}

/****************************************************************************
 * Name: tricore_dump_trapctrl
 ****************************************************************************/

void tricore_dump_trapctrl(void)
{
  _alert("PSTR:%-14.8" PRIX32 "DSTR:%-14.8" PRIX32
         "DATR:%-14.8" PRIX32 "DEADD:%-13.8" PRIX32 "\n\n",
         (uint32_t)__mfcr(CPU_PSTR), (uint32_t)__mfcr(CPU_DSTR),
         (uint32_t)__mfcr(CPU_DATR), (uint32_t)__mfcr(CPU_DEADD));
}

/****************************************************************************
 * Name: tricore_dump_csachain
 ****************************************************************************/

void tricore_dump_csachain(uintptr_t pcxi)
{
  while (pcxi & FCX_FREE)
    {
      if (pcxi & PCXI_UL)
        {
          tricore_dump_upcsa((uint32_t *)tricore_csa2addr(pcxi));
        }
      else
        {
          tricore_dump_lowcsa((uint32_t *)tricore_csa2addr(pcxi));
        }

        pcxi = tricore_csa2addr(pcxi)[0];
    }
}

/****************************************************************************
 * Name: up_dump_register
 ****************************************************************************/

void up_dump_register(void *dumpregs)
{
  volatile uint32_t *regs = dumpregs;

  tricore_dump_lowcsa(regs);

  tricore_dump_upcsa(regs + TC_CONTEXT_REGS);

  tricore_dump_trapctrl();
}

/****************************************************************************
 * Name: up_regs_memcpy
 ****************************************************************************/

void up_regs_memcpy(FAR void *dest, FAR void *src, size_t count)
{
  int csa_size = TC_CONTEXT_REGS * sizeof(uintptr_t);
  int csa_num = count / csa_size;

  while (csa_num--)
    {
      memcpy(dest, src, csa_size);
      dest = (char *)dest + csa_size;
      src = tricore_csa2addr(((uintptr_t *)src)[REG_LPCXI]);
    }
}
