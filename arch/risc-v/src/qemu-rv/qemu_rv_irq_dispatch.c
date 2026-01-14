/****************************************************************************
 * arch/risc-v/src/qemu-rv/qemu_rv_irq_dispatch.c
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

#include <stdint.h>
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <sys/types.h>

#include "riscv_internal.h"
#include "riscv_aia.h"
#include "hardware/qemu_rv_memorymap.h"
#include "hardware/qemu_rv_plic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ARCH_RV_HAVE_IMSIC
static void *riscv_dispatch_irq_ext(uintreg_t irq, uintreg_t *regs)
{
  int extirq;

  while ((extirq = SWAP_CSR(CSR_TOPEI, 0)) != 0)
    {
      extirq = (extirq >> TOPI_IID_SHIFT) + irq;
      regs = riscv_doirq(extirq, regs);
    }

  return regs;
}
#elif defined(CONFIG_ARCH_RV_HAVE_APLIC)
static void *riscv_dispatch_irq_ext(uintreg_t irq, uintreg_t *regs)
{
  int extirq;
  int hartid = up_cpu_index();
  uintptr_t aplic_base = RISCV_APLIC_IDC(QEMU_RV_APLIC_BASE, hartid) +
                            RISCV_APLIC_IDC_CLAIMI;

  while ((extirq = getreg32(aplic_base)) != 0)
    {
      extirq = (extirq >> RISCV_APLIC_IDC_TOPI_ID_SHIFT) + irq;
      regs = riscv_doirq(extirq, regs);
    }

  return regs;
}
#else
static void *riscv_dispatch_irq_ext(uintreg_t irq, uintreg_t *regs)
{
  int extirq;

  while ((extirq = getreg32(QEMU_RV_PLIC_CLAIM)) != 0)
    {
      regs = riscv_doirq(irq + extirq, regs);
      putreg32(extirq, QEMU_RV_PLIC_CLAIM);
    }

  return regs;
}
#endif

#ifdef CONFIG_ARCH_RV_EXT_AIA
static void *riscv_dispatch_async_irq(uintreg_t irq, uintreg_t *regs)
{
  while ((irq = READ_CSR(CSR_TOPI)) != 0)
    {
      irq = (irq >> TOPI_IID_SHIFT) + RISCV_IRQ_ASYNC;

      if (RISCV_IRQ_EXT == irq)
        {
          regs = riscv_dispatch_irq_ext(irq, regs);
        }
      else
        {
          regs = riscv_doirq(irq, regs);
        }
    }

  return regs;
}
#else
static void *riscv_dispatch_async_irq(uintreg_t irq, uintreg_t *regs)
{
  irq += RISCV_IRQ_ASYNC;

  if (irq == RISCV_IRQ_EXT)
    {
      regs = riscv_dispatch_irq_ext(irq, regs);
    }
  else
    {
      regs = riscv_doirq(irq, regs);
    }

  return regs;
}
#endif

void *riscv_dispatch_irq(uintreg_t vector, uintreg_t *regs)
{
  int irq = vector & (~RISCV_IRQ_BIT);

  if ((vector & RISCV_IRQ_BIT) != 0)
    {
      regs = riscv_dispatch_async_irq(irq, regs);
    }
  else
    {
      regs = riscv_doirq(irq, regs);
    }

  return regs;
}
