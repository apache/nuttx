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
 * Name: tricore_upcsa_register
 ****************************************************************************/

void tricore_upcsa_register(volatile uint32_t *regs)
{
  _alert("UPCXI:%08x  PSW:%08x  SP:%08x   PC:%08x\n",
         regs[REG_UPCXI], regs[REG_PSW], regs[REG_A10], regs[REG_UA11]);
  _alert("D8:%08x     D9:%08x   D10:%08x  D11:%08x\n",
         regs[REG_D8], regs[REG_D9], regs[REG_D10], regs[REG_D11]);
  _alert("A12:%08x    A13:%08x  A14:%08x  A15:%08x\n",
         regs[REG_A12], regs[REG_A13], regs[REG_A14], regs[REG_A15]);
  _alert("D12:%08x    D13:%08x  D14:%08x  D15:%08x\n\n",
         regs[REG_D12], regs[REG_D13], regs[REG_D14], regs[REG_D15]);
}

/****************************************************************************
 * Name: tricore_lowcsa_register
 ****************************************************************************/

void tricore_lowcsa_register(volatile uint32_t *regs)
{
  _alert("LPCXI:%08x  A11:%08x   A2:%08x  A3:%08x\n",
         regs[REG_LPCXI] | PCXI_UL, regs[REG_LA11],
         regs[REG_A2], regs[REG_A3]);
  _alert("D0:%08x     D1:%08x    D2:%08x  D3:%08x\n",
         regs[REG_D0], regs[REG_D1], regs[REG_D2], regs[REG_D3]);
  _alert("A4:%08x     A5:%08x    A6:%08x  A7:%08x\n",
         regs[REG_A4], regs[REG_A5], regs[REG_A6], regs[REG_A7]);
  _alert("D4:%08x     D5:%08x    D6:%08x  D7:%08x\n\n",
         regs[REG_D4], regs[REG_D5], regs[REG_D6], regs[REG_D7]);
}

/****************************************************************************
 * Name: tricore_csachain_dump
 ****************************************************************************/

void tricore_csachain_dump(uintptr_t pcxi)
{
  while (pcxi & FCX_FREE)
    {
      if (pcxi & PCXI_UL)
        {
          tricore_upcsa_register(tricore_csa2addr(pcxi));
        }
      else
        {
          tricore_lowcsa_register(tricore_csa2addr(pcxi));
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

  tricore_lowcsa_register(regs);

  tricore_upcsa_register(regs + TC_CONTEXT_REGS);
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
